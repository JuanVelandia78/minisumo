// Robot minisumo teleoperado - ESP32 DevKit V1
// Lazo a 200 Hz: lectura de IMU y encoders, filtrado, PID de velocidad por rueda,
// odometría y telemetría por UDP hacia el PC con ROS2.
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "soc/soc_caps.h"
#if SOC_TEMP_SENSOR_SUPPORTED
#include "driver/temperature_sensor.h"
#endif

#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "mpu6050.h"
#include "filters.h"
#include "filter_coefs.h"
#include "pid.h"
#include "odometry.h"
#include "net.h"

static const char *TAG = "minisumo";

#define DEG2RAD ((float)M_PI / 180.0f)

static const char TELEM_HEADER[] =
    "H,t_ms,ax,ay,az,gx,gy,gz,ax_f,ay_f,az_f,gx_f,gy_f,gz_f,"
    "wl,wr,wl_f,wr_f,wl_ref,wr_ref,duty_l,duty_r,ticks_l,ticks_r,"
    "x,y,theta,theta_enc,theta_gyro,filt_us,mode\n";

// ---------------------------------------------------------------------------
// Estado compartido entre la tarea de red (comandos) y el lazo de control
// ---------------------------------------------------------------------------
typedef enum { MODE_STOP = 0, MODE_VEL = 1, MODE_OPEN = 2 } drive_mode_t;

typedef struct {
    drive_mode_t mode;
    float ref_l, ref_r;        // rad/s (modo velocidad)
    float duty_l, duty_r;      // ciclo útil (modo lazo abierto)
    int64_t last_cmd_us;
    float kp, ki, kd, kff;
    float dmin_l, dmin_r, trim;  // zona muerta de cada motor y corrección máxima del PID
    float yaw_kp, yaw_ki;        // control de rumbo con el giroscopio
    bool gains_dirty, reset_odom, recal, send_header;
} cmd_state_t;

static cmd_state_t s_cmd;
static portMUX_TYPE s_cmd_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    int len;
    char data[1472];   // máximo de un datagrama UDP por WiFi sin fragmentar (6 líneas de ~240 bytes)
} telem_pkt_t;

// Muestra en binario: el lazo solo la copia a la cola; el texto (snprintf con ~30 floats,
// lento en el C3 que no tiene FPU) se arma en la tarea de telemetría, fuera del lazo.
typedef struct {
    int64_t t_ms;
    imu_sample_t imu, imf;
    float wl, wr, wl_f, wr_f, ref_l, ref_r, al, ar;
    int32_t ticks_l, ticks_r;
    odom_t od;
    int32_t filt_us;
    int mode;
} telem_sample_t;

static QueueHandle_t s_telem_q;
static volatile bool s_hdr_req;
static TaskHandle_t s_ctrl_task;
static bool s_imu_ok;

static struct {
    volatile uint32_t overruns, imu_errors, telem_drops;
    volatile int32_t max_loop_us;
    volatile drive_mode_t mode;
} s_stats;

// ---------------------------------------------------------------------------
// Comandos UDP (texto, una línea por datagrama)
//   W,wl,wr        velocidad angular de cada rueda en rad/s (PID)
//   M,dl,dr        ciclo útil directo en [-1, 1] (lazo abierto, para identificar el motor)
//   S              detener
//   P              mantener vivo sin cambiar la orden
//   G,kp,ki,kd[,kff]  ganancias del PID
//   F,dmin_l,dmin_r[,trim]  zona muerta de cada motor y corrección máxima del PID
//   Y,kp,ki        control de rumbo con el giroscopio (Y,0,0 lo desactiva)
//   R              reiniciar odometría y contadores
//   C              recalibrar giroscopio (robot quieto, 2 s)
//   H              pedir la cabecera de la telemetría
// ---------------------------------------------------------------------------
static void on_cmd(const char *m, int len)
{
    int64_t now = esp_timer_get_time();
    float a = 0, b = 0, c = 0, d = 0;
    int n = 0;

    switch (m[0]) {
    case 'W':
        if (sscanf(m, "W,%f,%f", &a, &b) == 2) {
            portENTER_CRITICAL(&s_cmd_mux);
            s_cmd.mode = MODE_VEL;
            s_cmd.ref_l = a;
            s_cmd.ref_r = b;
            s_cmd.last_cmd_us = now;
            portEXIT_CRITICAL(&s_cmd_mux);
        }
        break;
    case 'M':
        if (sscanf(m, "M,%f,%f", &a, &b) == 2) {
            portENTER_CRITICAL(&s_cmd_mux);
            s_cmd.mode = MODE_OPEN;
            s_cmd.duty_l = a;
            s_cmd.duty_r = b;
            s_cmd.last_cmd_us = now;
            portEXIT_CRITICAL(&s_cmd_mux);
        }
        break;
    case 'S':
        portENTER_CRITICAL(&s_cmd_mux);
        s_cmd.mode = MODE_STOP;
        s_cmd.last_cmd_us = now;
        portEXIT_CRITICAL(&s_cmd_mux);
        break;
    case 'P':
        portENTER_CRITICAL(&s_cmd_mux);
        s_cmd.last_cmd_us = now;
        portEXIT_CRITICAL(&s_cmd_mux);
        break;
    case 'G':
        n = sscanf(m, "G,%f,%f,%f,%f", &a, &b, &c, &d);
        if (n >= 3) {
            portENTER_CRITICAL(&s_cmd_mux);
            s_cmd.kp = a;
            s_cmd.ki = b;
            s_cmd.kd = c;
            if (n == 4) s_cmd.kff = d;
            s_cmd.gains_dirty = true;
            portEXIT_CRITICAL(&s_cmd_mux);
        }
        break;
    case 'F':
        n = sscanf(m, "F,%f,%f,%f", &a, &b, &c);
        if (n >= 2) {
            portENTER_CRITICAL(&s_cmd_mux);
            s_cmd.dmin_l = a;
            s_cmd.dmin_r = b;
            if (n == 3) s_cmd.trim = c;
            s_cmd.gains_dirty = true;
            portEXIT_CRITICAL(&s_cmd_mux);
        }
        break;
    case 'Y':
        if (sscanf(m, "Y,%f,%f", &a, &b) == 2) {
            portENTER_CRITICAL(&s_cmd_mux);
            s_cmd.yaw_kp = a;
            s_cmd.yaw_ki = b;
            s_cmd.gains_dirty = true;
            portEXIT_CRITICAL(&s_cmd_mux);
        }
        break;
    case 'R':
        portENTER_CRITICAL(&s_cmd_mux);
        s_cmd.reset_odom = true;
        portEXIT_CRITICAL(&s_cmd_mux);
        break;
    case 'C':
        portENTER_CRITICAL(&s_cmd_mux);
        s_cmd.recal = true;
        portEXIT_CRITICAL(&s_cmd_mux);
        break;
    case 'H':
        portENTER_CRITICAL(&s_cmd_mux);
        s_cmd.send_header = true;
        portEXIT_CRITICAL(&s_cmd_mux);
        break;
    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Lazo de control
// ---------------------------------------------------------------------------
static iir_t s_f_imu[6];        // IIR: ax, ay, az, gx, gy, gz
static movavg_t s_f_wheel[2];    // media móvil: rueda izquierda, derecha

static void filters_setup(void)
{
    for (int i = 0; i < 6; i++) iir_init(&s_f_imu[i], IIR_IMU_SOS, IIR_IMU_SECTIONS);
    for (int i = 0; i < 2; i++) movavg_init(&s_f_wheel[i], MOVAVG_N);
}

static void control_tick(void *arg)
{
    xTaskNotifyGive(s_ctrl_task);
}

static void control_task(void *arg)
{
    pid_ctrl_t pid_l, pid_r;
    pid_init(&pid_l, PID_KP, PID_KI, PID_KD, PID_KFF, PID_DUTY_MIN_L, PID_TRIM, PWM_MAX);
    pid_init(&pid_r, PID_KP, PID_KI, PID_KD, PID_KFF, PID_DUTY_MIN_R, PID_TRIM, PWM_MAX);
    filters_setup();

    odom_t od;
    odom_reset(&od);
    imu_sample_t imu = {0}, imf = {0};
    float yaw_int = 0.0f;     // integral del control de rumbo
    float ramp_l = 0.0f, ramp_r = 0.0f;   // velocidad pedida con rampa (REF_ACCEL_MAX)
    float slew_l = 0.0f, slew_r = 0.0f;   // último ciclo útil aplicado (arranque suave)
    esp_task_wdt_add(NULL);   // si el lazo se traba, el watchdog reinicia el ESP y apaga los motores

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        esp_task_wdt_reset();
        int64_t now = esp_timer_get_time();

        // Copia de la orden actual y limpieza de las peticiones de una sola vez
        cmd_state_t cmd;
        portENTER_CRITICAL(&s_cmd_mux);
        cmd = s_cmd;
        s_cmd.gains_dirty = s_cmd.reset_odom = s_cmd.recal = s_cmd.send_header = false;
        portEXIT_CRITICAL(&s_cmd_mux);

        if (cmd.gains_dirty) {
            pid_init(&pid_l, cmd.kp, cmd.ki, cmd.kd, cmd.kff, cmd.dmin_l, cmd.trim, PWM_MAX);
            pid_init(&pid_r, cmd.kp, cmd.ki, cmd.kd, cmd.kff, cmd.dmin_r, cmd.trim, PWM_MAX);
            ESP_LOGI(TAG, "PID: kp=%.4f ki=%.4f kd=%.4f kff=%.4f | zona muerta izq=%.3f der=%.3f | trim=%.3f | rumbo kp=%.3f ki=%.3f",
                     cmd.kp, cmd.ki, cmd.kd, cmd.kff, cmd.dmin_l, cmd.dmin_r, cmd.trim, cmd.yaw_kp, cmd.yaw_ki);
        }
        if (cmd.reset_odom) {
            odom_reset(&od);
            enc_reset();
        }
        if (cmd.send_header) s_hdr_req = true;
        if (cmd.recal && s_imu_ok) {
            motors_stop();
            pid_reset(&pid_l);
            pid_reset(&pid_r);
            ESP_LOGI(TAG, "Recalibrando giroscopio: no mover el robot");
            mpu_calibrate_gyro(GYRO_CAL_SAMPLES);
            filters_setup();
            continue;
        }

        bool timeout = (now - cmd.last_cmd_us) > (int64_t)CMD_TIMEOUT_MS * 1000;
        drive_mode_t mode = timeout ? MODE_STOP : cmd.mode;
        s_stats.mode = mode;

        // --- Sensores ---
        bool imu_fresh = s_imu_ok && mpu_read(&imu) == ESP_OK;
        if (s_imu_ok && !imu_fresh) s_stats.imu_errors++;   // conserva la última lectura
        enc_reading_t el, er;
        enc_update(ENC_LEFT, now, &el);
        enc_update(ENC_RIGHT, now, &er);

        // --- Filtrado: IIR en la IMU, media móvil en las ruedas (se mide su tiempo de cómputo) ---
        int64_t tf0 = esp_timer_get_time();
        imf.ax = iir_update(&s_f_imu[0], imu.ax);
        imf.ay = iir_update(&s_f_imu[1], imu.ay);
        imf.az = iir_update(&s_f_imu[2], imu.az);
        imf.gx = iir_update(&s_f_imu[3], imu.gx);
        imf.gy = iir_update(&s_f_imu[4], imu.gy);
        imf.gz = iir_update(&s_f_imu[5], imu.gz);
        float wl_f = movavg_update(&s_f_wheel[0], el.omega);
        float wr_f = movavg_update(&s_f_wheel[1], er.omega);
        int32_t filt_us = (int32_t)(esp_timer_get_time() - tf0);

        // --- Control ---
        float ref_l = 0, ref_r = 0, dl = 0, dr = 0;
        if (mode == MODE_VEL) {
            // Rampa: la referencia sube a lo sumo REF_ACCEL_MAX, igual para las dos ruedas
            float step = REF_ACCEL_MAX * CONTROL_DT;
            ramp_l += fmaxf(-step, fminf(step, cmd.ref_l - ramp_l));
            ramp_r += fmaxf(-step, fminf(step, cmd.ref_r - ramp_r));
            ref_l = ramp_l;
            ref_r = ramp_r;

            // Rumbo (lazo externo): giro pedido por las ruedas frente al giro medido por el giroscopio.
            // La corrección se suma a la velocidad pedida a cada rueda (rad/s), no al PWM, así el PID
            // de cada rueda la cumple en vez de deshacerla (control en cascada).
            // Si el robot gira de más hacia la izquierda, baja la rueda derecha y sube la izquierda.
            float wl_ref = ref_l, wr_ref = ref_r;
            if (s_imu_ok && (ref_l != 0.0f || ref_r != 0.0f)) {
                float yaw_ref = WHEEL_RADIUS_M * (ref_r - ref_l) / WHEEL_BASE_M;
                float yaw_meas = GYRO_Z_SIGN * imf.gz * DEG2RAD;
                float e_yaw = yaw_ref - yaw_meas;
                yaw_int += cmd.yaw_ki * e_yaw * CONTROL_DT;
                if (yaw_int > YAW_CORR_MAX) yaw_int = YAW_CORR_MAX;
                if (yaw_int < -YAW_CORR_MAX) yaw_int = -YAW_CORR_MAX;
                float corr = cmd.yaw_kp * e_yaw + yaw_int;
                if (corr > YAW_CORR_MAX) corr = YAW_CORR_MAX;
                if (corr < -YAW_CORR_MAX) corr = -YAW_CORR_MAX;
                wl_ref -= corr;
                wr_ref += corr;
                // La corrección no invierte el sentido pedido a ninguna rueda
                if ((ref_l > 0 && wl_ref < 0) || (ref_l < 0 && wl_ref > 0)) wl_ref = 0;
                if ((ref_r > 0 && wr_ref < 0) || (ref_r < 0 && wr_ref > 0)) wr_ref = 0;
            } else {
                yaw_int = 0.0f;
            }
            // Lazo interno: PID de velocidad de cada rueda
            dl = pid_update(&pid_l, wl_ref, wl_f, CONTROL_DT);
            dr = pid_update(&pid_r, wr_ref, wr_f, CONTROL_DT);
        } else {
            pid_reset(&pid_l);
            pid_reset(&pid_r);
            yaw_int = 0.0f;
            ramp_l = ramp_r = 0.0f;
            if (mode == MODE_OPEN) {
                dl = cmd.duty_l;
                dr = cmd.duty_r;
            }
        }
        float al, ar;
        // Arranque suave: limita qué tan rápido sube el ciclo útil para evitar el pico de
        // corriente que hace caer el voltaje y reinicia el ESP. Frenar sí es inmediato.
        {
            const float step = MOTOR_SLEW_PER_S * CONTROL_DT;
            if (fabsf(dl) > fabsf(slew_l) && dl - slew_l > step) dl = slew_l + step;
            if (fabsf(dl) > fabsf(slew_l) && slew_l - dl > step) dl = slew_l - step;
            if (fabsf(dr) > fabsf(slew_r) && dr - slew_r > step) dr = slew_r + step;
            if (fabsf(dr) > fabsf(slew_r) && slew_r - dr > step) dr = slew_r - step;
            slew_l = dl;
            slew_r = dr;
        }
        motors_set(dl, dr, &al, &ar);
        // Sentido del encoder: en modo velocidad, el de la referencia; en lazo abierto, el del PWM
        float sl = (mode == MODE_VEL) ? ref_l : al;
        float sr = (mode == MODE_VEL) ? ref_r : ar;
        if (sl != 0.0f) enc_set_dir(ENC_LEFT, sl > 0 ? 1 : -1);
        if (sr != 0.0f) enc_set_dir(ENC_RIGHT, sr > 0 ? 1 : -1);

        // --- Estimación de posición ---
        bool moving = al != 0.0f || ar != 0.0f || el.dtheta != 0.0f || er.dtheta != 0.0f;
        odom_update(&od, el.dtheta, er.dtheta, GYRO_Z_SIGN * imu.gz * DEG2RAD, CONTROL_DT, moving);
        // Robot detenido y sin girar: reajusta el bias del giroscopio (evita que se desvíe con el tiempo)
        if (imu_fresh && mode == MODE_STOP && !moving && fabsf(imu.gz) < 3.0f && fabsf(imu.gx) < 3.0f && fabsf(imu.gy) < 3.0f) {
            mpu_track_bias(&imu, GYRO_BIAS_TRACK);
        }

        // --- Telemetría: se encola la muestra en binario (el texto lo arma telem_task) ---
        telem_sample_t ts = {
            .t_ms = now / 1000, .imu = imu, .imf = imf,
            .wl = el.omega, .wr = er.omega, .wl_f = wl_f, .wr_f = wr_f,
            .ref_l = ref_l, .ref_r = ref_r, .al = al, .ar = ar,
            .ticks_l = el.ticks, .ticks_r = er.ticks, .od = od,
            .filt_us = filt_us, .mode = (int)mode,
        };
        if (xQueueSend(s_telem_q, &ts, 0) != pdTRUE) s_stats.telem_drops++;

        int32_t dur = (int32_t)(esp_timer_get_time() - now);
        if (dur > s_stats.max_loop_us) s_stats.max_loop_us = dur;
        if (dur > 1000000 / CONTROL_HZ) s_stats.overruns++;
    }
}

// ---------------------------------------------------------------------------
// Envío de telemetría y estado por consola
// ---------------------------------------------------------------------------
static void telem_task(void *arg)
{
    static telem_pkt_t pkt;
    telem_sample_t s;
    int nsamp = 0;
    int64_t last_log = 0;
    pkt.len = 0;
    // Sensor de temperatura interno del chip (para vigilar el calentamiento)
#if SOC_TEMP_SENSOR_SUPPORTED
    temperature_sensor_handle_t tsens = NULL;
    temperature_sensor_config_t tcfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
    if (temperature_sensor_install(&tcfg, &tsens) == ESP_OK) temperature_sensor_enable(tsens);
    else tsens = NULL;
#endif
    while (1) {
        if (s_hdr_req) {
            s_hdr_req = false;
            net_send(TELEM_HEADER, sizeof(TELEM_HEADER) - 1);
        }
        if (xQueueReceive(s_telem_q, &s, pdMS_TO_TICKS(500)) == pdTRUE) {
            // Una línea por muestra, TELEM_BATCH líneas por paquete
            int room = (int)sizeof(pkt.data) - pkt.len;
            int w = snprintf(pkt.data + pkt.len, room,
                "D,%lld,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%ld,%ld,"
                "%.4f,%.4f,%.4f,%.4f,%.4f,%ld,%d\n",
                s.t_ms, s.imu.ax, s.imu.ay, s.imu.az, s.imu.gx, s.imu.gy, s.imu.gz,
                s.imf.ax, s.imf.ay, s.imf.az, s.imf.gx, s.imf.gy, s.imf.gz,
                s.wl, s.wr, s.wl_f, s.wr_f, s.ref_l, s.ref_r, s.al, s.ar,
                (long)s.ticks_l, (long)s.ticks_r,
                s.od.x, s.od.y, s.od.theta, s.od.theta_enc, s.od.theta_gyro, (long)s.filt_us, s.mode);
            if (w > 0 && w < room) pkt.len += w;
            if (++nsamp >= TELEM_BATCH) {
                net_send(pkt.data, pkt.len);
                pkt.len = 0;
                nsamp = 0;
            }
        }
        int64_t now = esp_timer_get_time();
        if (now - last_log > 2000000) {
            last_log = now;
            static const char *modes[] = {"STOP", "VEL", "OPEN"};
            float tc = -1.0f;
#if SOC_TEMP_SENSOR_SUPPORTED
            if (tsens) temperature_sensor_get_celsius(tsens, &tc);
#endif
            ESP_LOGI(TAG, "IP %s | WiFi %s | PC %s | modo %s | lazo max %ld us, excesos %lu | IMU err %lu | paquetes perdidos %lu | chip %.0f C",
                     net_ip_str(), net_connected() ? "ok" : "no", net_pc_known() ? "ok" : "esperando",
                     modes[s_stats.mode], (long)s_stats.max_loop_us, (unsigned long)s_stats.overruns,
                     (unsigned long)s_stats.imu_errors, (unsigned long)s_stats.telem_drops, tc);
            s_stats.max_loop_us = 0;
        }
    }
}

void app_main(void)
{
    motors_init();      // primero los motores: durante el reinicio sus pines quedan sin control
    motors_stop();
    static const char *rst[] = {"desconocido", "encendido", "pin EN", "software", "excepcion/panic",
                                "watchdog int", "watchdog tarea", "watchdog", "deep sleep",
                                "BROWNOUT (caida de voltaje)", "SDIO", "USB", "JTAG", "eFuse",
                                "pin de energia", "CPU lockup"};
    esp_reset_reason_t rr = esp_reset_reason();
    ESP_LOGW(TAG, "Minisumo arrancando. Causa del ultimo reinicio: %s",
             (rr < sizeof(rst) / sizeof(rst[0])) ? rst[rr] : "otra");
    enc_init();

    s_imu_ok = (mpu_init() == ESP_OK);
    if (s_imu_ok) {
        ESP_LOGI(TAG, "Calibrando giroscopio: no mover el robot");
        vTaskDelay(pdMS_TO_TICKS(1000));   // tiempo para soltar el robot después de encenderlo
        mpu_calibrate_gyro(GYRO_CAL_SAMPLES);
    } else {
        ESP_LOGE(TAG, "Sin IMU: se continúa con encoders solamente");
    }

    s_cmd.mode = MODE_STOP;
    s_cmd.kp = PID_KP;
    s_cmd.ki = PID_KI;
    s_cmd.kd = PID_KD;
    s_cmd.kff = PID_KFF;
    s_cmd.dmin_l = PID_DUTY_MIN_L;
    s_cmd.dmin_r = PID_DUTY_MIN_R;
    s_cmd.trim = PID_TRIM;
    s_cmd.yaw_kp = YAW_KP;
    s_cmd.yaw_ki = YAW_KI;

    s_telem_q = xQueueCreate(64, sizeof(telem_sample_t));   // ~0.3 s de margen si el WiFi se atasca
    net_start(on_cmd);

    // Dos núcleos: el WiFi y la red trabajan en el núcleo 0; el lazo de control tiene el núcleo 1
    // para él solo, así el WiFi no lo atrasa.
    xTaskCreatePinnedToCore(telem_task, "telem", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(control_task, "control", 6144, NULL, 10, &s_ctrl_task, 1);

    const esp_timer_create_args_t ta = {.callback = control_tick, .name = "ctrl"};
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&ta, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000000 / CONTROL_HZ));
}
