# Informe de laboratorio — Práctica 2: Robot minisumo teleoperado

**Asignatura:** Procesamiento de Señales
**Práctica:** 2. Filtrado digital. Parte 3: construcción del robot minisumo teleoperado con ROS2
**Integrantes:** ⟨completar⟩
**Fecha:** ⟨completar⟩

> Los campos marcados con **⟨completar⟩** dependen de mediciones experimentales del equipo y deben llenarse con los datos reales. Todo lo demás describe el diseño, la implementación y las pruebas hechas durante el desarrollo.

---

## Resumen

Se diseñó y construyó un robot minisumo diferencial teleoperado de forma inalámbrica desde un PC con Linux y ROS2. Su controlador es un ESP32-C3 Super Mini que:

- **Muestrea a 200 muestras/s** una IMU MPU-6050 y dos encoders ópticos de herradura.
- **Filtra las señales en el microcontrolador:** un IIR Butterworth de orden 2 en los seis ejes de la IMU y una media móvil en la velocidad de las ruedas.
- **Controla el movimiento** con un PID de velocidad por rueda, con prealimentación y compensación de zona muerta, más un control de rumbo PI con el giroscopio.
- **Estima la posición** fusionando encoders y giroscopio.
- **Envía al PC** todos los datos, crudos y filtrados, por UDP.

En el PC, un paquete de ROS2 hace cinco cosas:
- traduce velocidades del robot a velocidades de rueda con el modelo cinemático diferencial;
- comunica el PC con el robot;
- guarda la telemetría en archivos CSV;
- ejecuta rutinas de movimiento conocidas;
- permite manejar el robot con un guante (ESP32 + MPU-6050) que mide la inclinación de la mano con un filtro complementario.

El informe documenta el diseño, el tratamiento de las señales, la estrategia de control, los problemas encontrados en la integración y cómo se resolvieron, y el experimento de posición final.

---

## 1. Objetivos

### 1.1 Objetivo general

Diseñar, construir y teleoperar con ROS2 un robot minisumo diferencial que adquiera, filtre y transmita las señales de sus sensores, controle la velocidad de sus ruedas y estime su posición.

### 1.2 Objetivos específicos

1. Caracterizar las señales de la IMU y de los encoders (ancho de banda, ruido, resolución) y diseñar filtros digitales para el microcontrolador.
2. Implementar un control de velocidad por rueda en el microcontrolador.
3. Desarrollar en ROS2 la teleoperación inalámbrica y el almacenamiento de los datos crudos y filtrados en CSV.
4. Programar rutinas de movimiento con velocidad y tiempo conocidos, y comparar la posición final deseada (modelo), la estimada (sensores) y la real (medida).

---

## 2. Marco teórico

### 2.1 Modelo cinemático del robot diferencial

Sean $r$ el radio de las ruedas, $L$ la distancia entre ellas, y $\omega_l$, $\omega_r$ las velocidades angulares de las ruedas izquierda y derecha. Las velocidades lineal $v$ y angular $\omega$ del robot son:

$$
v = \frac{r(\omega_r + \omega_l)}{2}, \qquad \omega = \frac{r(\omega_r - \omega_l)}{L}
$$

El modelo inverso, que usa el nodo `diff_kinematics` para convertir `/cmd_vel` en la velocidad de cada rueda, es:

$$
\omega_l = \frac{v - \omega L/2}{r}, \qquad \omega_r = \frac{v + \omega L/2}{r}
$$

Si alguna rueda supera $\omega_{max} = 18$ rad/s, ambas se escalan por el mismo factor. Así se conserva el radio de giro pedido.

La pose $(x, y, \theta)$ evoluciona según:

$$
\dot x = v\cos\theta, \qquad \dot y = v\sin\theta, \qquad \dot\theta = \omega
$$

Para un tramo con $v$ y $\omega$ constantes durante un tiempo $T$, la integración exacta es:

$$
\omega \neq 0:\quad
\begin{aligned}
x_{k+1} &= x_k + \tfrac{v}{\omega}\left[\sin(\theta_k + \omega T) - \sin\theta_k\right] \\
y_{k+1} &= y_k - \tfrac{v}{\omega}\left[\cos(\theta_k + \omega T) - \cos\theta_k\right] \\
\theta_{k+1} &= \theta_k + \omega T
\end{aligned}
\qquad\quad
\omega = 0:\quad x_{k+1} = x_k + vT\cos\theta_k,\; y_{k+1} = y_k + vT\sin\theta_k
$$

El nodo `routines` usa esta integración para calcular la **posición final deseada**.

### 2.2 Odometría con fusión de sensores

En cada período de control ($\Delta t = 5$ ms), los encoders dan los ángulos girados por cada rueda, $\Delta\phi_l$ y $\Delta\phi_r$:

$$
\Delta s = \frac{r(\Delta\phi_r + \Delta\phi_l)}{2}, \qquad \Delta\psi_{enc} = \frac{r(\Delta\phi_r - \Delta\phi_l)}{L}
$$

El giroscopio mide el cambio de rumbo directamente: $\Delta\psi_{gyro} = g_z \Delta t$.

- **Encoders:** su rumbo se degrada cuando las ruedas patinan.
- **Giroscopio:** su rumbo deriva lentamente por el bias.

Por eso se combinan con un filtro complementario:

$$
\Delta\psi = \alpha\,\Delta\psi_{gyro} + (1-\alpha)\,\Delta\psi_{enc}, \qquad \alpha = 0{,}98
$$

La posición se integra con el rumbo del punto medio del paso:

$$
x \mathrel{+}= \Delta s\cos\!\left(\theta + \tfrac{\Delta\psi}{2}\right), \qquad y \mathrel{+}= \Delta s\sin\!\left(\theta + \tfrac{\Delta\psi}{2}\right)
$$

Con el robot detenido, el giroscopio no se integra. Así se evita acumular el bias residual.

### 2.3 Filtros digitales

**IIR Butterworth.** Tiene respuesta máximamente plana en la banda de paso. Se diseñó con la transformada bilineal y se implementó como una sección de segundo orden (biquad) en forma directa II transpuesta:

$$
H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{1 + a_1 z^{-1} + a_2 z^{-2}}
$$

**Media móvil de $N$ muestras.** Es un FIR con todos los coeficientes iguales a $1/N$:

$$
y[n] = \frac{1}{N}\sum_{k=0}^{N-1} x[n-k], \qquad |H(f)| = \left|\frac{\sin(\pi f N / f_s)}{N\sin(\pi f / f_s)}\right|
$$

Tiene retardo de grupo constante, $(N-1)/2$ muestras, y se implementa con un acumulador en $O(1)$ por muestra.

**Filtro complementario (guante).** Combina el giroscopio, bueno a alta frecuencia, con el acelerómetro, bueno a baja frecuencia:

$$
\theta[n] = \alpha\left(\theta[n-1] + \dot\theta_{gyro}\,\Delta t\right) + (1-\alpha)\,\theta_{acc}[n], \qquad \tau = \frac{\alpha\,\Delta t}{1-\alpha}
$$

### 2.4 Control PID con prealimentación

Para cada rueda, con error $e = \omega_{ref} - \omega_{med}$:

$$
u = \underbrace{k_{ff}\,\omega_{ref} + \operatorname{sgn}(\omega_{ref})\,d_{min}}_{\text{prealimentación}} + k_p e + k_i\!\int e\,dt + k_d\,\frac{d(-\omega_{med})}{dt}
$$

- **Prealimentación:** el término $k_{ff}\,\omega_{ref}$ entrega casi todo el ciclo útil necesario.
- **Zona muerta:** $d_{min}$ compensa la zona muerta del motor, es decir, el PWM mínimo con el que empieza a girar.
- **Corrección limitada:** el PID solo corrige el error residual.
- **Anti-windup:** la integral se congela cuando la salida está saturada.

---

## 3. Diseño del robot

### 3.1 Requisitos

| Requisito de la guía | Valor |
|---|---|
| Dimensiones máximas | 10 cm × 10 cm, altura libre |
| Peso máximo | 500 g |
| Dojo | Circular, 70 cm de diámetro |
| Tracción | Diferencial: 2 ruedas motrices y una rueda de apoyo opcional |
| Sensores | IMU en el chasis y encoders en los motores |
| Control | Velocidad de cada motor en el microcontrolador |
| Comunicación | Inalámbrica con un PC Linux con ROS2 |

### 3.2 Componentes

| Componente | Función | Comentario |
|---|---|---|
| ESP32-C3 Super Mini | Microcontrolador RISC-V de 160 MHz con WiFi | Sin FPU: la aritmética en punto flotante es por software |
| MPU-6050 (clon, WHO_AM_I = 0x70) | Acelerómetro ±4 g (8192 LSB/g) y giroscopio ±500 °/s (65,5 LSB/(°/s)) | DLPF interno en modo 2 (≈94 Hz) |
| TB6612FNG | Puente H dual | PWM a 20 kHz, inaudible |
| Motorreductores TT | Tracción | Ruedas de 65 mm de diámetro ($r = 32{,}5$ mm) |
| Sensores de herradura LM393 | Encoders de un canal | Disco de 20 ranuras, ambos flancos: 40 flancos/vuelta (9° por flanco) |
| LM2596 | Regulador de la lógica | Alimenta el ESP a través del diodo 1N5819 |
| LiPo 2S | Batería | 7,4 V nominal, 8,4 V cargada |

**Geometría:** $r = 0{,}0325$ m y $L = 0{,}085$ m.

**Rango de velocidades:** con el ciclo útil limitado a 0,70 (porque los motores son de 6 V y la batería da hasta 8,4 V), la rueda alcanza aproximadamente 25 rad/s, es decir, $v_{max} \approx 0{,}8$ m/s. En un dojo de 70 cm eso es excesivo. Por eso se trabajó entre 0,10 y 0,30 m/s. Por ejemplo, 0,1 m/s equivale a $\omega_{rueda} = 3{,}08$ rad/s.

### 3.3 Alimentación y conexiones

- **Motores:** la batería alimenta directamente el VM del TB6612, con 470 µF y 102 en VM, y un 103 en cada motor contra el ruido de las escobillas.
- **Lógica:** el LM2596 entrega la tensión de la lógica, que llega al pin 5V del ESP a través de un 1N5819. El diodo impide que el USB del PC y el regulador se alimenten entre sí.
- **Sensores:** los encoders y la IMU se alimentan a 3,3 V desde el ESP, así sus salidas nunca superan el nivel lógico del chip.

| Señal | GPIO | Señal | GPIO |
|---|---|---|---|
| PWMA (motor izquierdo) | 5 | PWMB (motor derecho) | 10 |
| AIN1 / AIN2 | 6 / 7 | BIN1 / BIN2 | 20 / 21 |
| Encoder izquierdo | 3 | Encoder derecho | 4 |
| SDA / SCL | 8 / 9 | STBY | 3,3 V |

GPIO20 y 21 son la UART0 del chip, así que la consola se movió al USB nativo (USB-Serial-JTAG). El diagrama completo está en [docs/cableado.html](docs/cableado.html).

---

## 4. Arquitectura del software

### 4.1 Firmware del robot (ESP-IDF 5.3.1, FreeRTOS)

| Tarea | Prioridad | Función |
|---|---|---|
| Temporizador `esp_timer` | — | Despierta el lazo de control cada 5 ms (200 Hz) |
| `control` | 10 | Lee la IMU y los encoders, filtra, controla, calcula la odometría y encola la muestra |
| `net_rx` | 5 | Recibe comandos UDP y envía el anuncio de 1 Hz |
| `telem` | 4 | Formatea las muestras a texto y envía un paquete por cada 4 muestras (50 paquetes/s) |
| Watchdog de tareas | — | Si el lazo se traba más de 3 s, reinicia el ESP y los motores se apagan |

**Por qué el formateo va en otra tarea:** en una primera versión, el lazo armaba el texto de la telemetría (`snprintf` con unos 30 números con decimales). Como el C3 no tiene FPU, eso ocupaba unos **4 ms de los 5 ms disponibles**, y el lazo se pasaba de su tiempo cuando el WiFi estaba ocupado. Al mover el formateo a la tarea de baja prioridad (el lazo solo copia la muestra binaria a una cola de 64 posiciones), el tiempo del lazo **bajó a unos 0,4 ms** sin la IMU y a unos **1,5–2 ms** con la IMU por I2C.

**Seguridad:** si pasan 300 ms sin comandos, el robot se detiene. Además, al arrancar se inicializan primero los motores.

### 4.2 Paquete ROS2 `minisumo_ros`

| Nodo | Entradas | Salidas |
|---|---|---|
| `robot_bridge` | `/wheel_cmd`, `/robot_cmd`; UDP del robot | UDP al robot; `/telemetry`, `/telemetry_header`, `/imu/raw`, `/imu/filtered`, `/wheels/speed`, `/odom` |
| `diff_kinematics` | `/cmd_vel` | `/wheel_cmd` |
| `data_logger` | `/telemetry`, `/telemetry_header` | Archivo CSV |
| `routines` | `/odom` | `/cmd_vel`, `/robot_cmd`, `rutinas.csv` |
| `glove_teleop` | UDP del guante | `/cmd_vel`, `/glove/angles` |

- **Descubrimiento del robot:** el robot anuncia su IP por broadcast (puerto 8889), así que el PC no necesita configurarla.
- **Ruta de la telemetría:** vuelve al puerto desde el que se envió el último comando.
- **Cabecera:** se publica en un tópico de durabilidad *transient local*, para que el `data_logger` la reciba aunque arranque después del bridge.

### 4.3 Guante de teleoperación

Un ESP32 con un MPU-6050 lee la IMU a 100 Hz y calcula dos ángulos de la mano:
- **Cabeceo** $\theta_p = \operatorname{atan2}(-a_x, \sqrt{a_y^2 + a_z^2})$
- **Alabeo** $\theta_r = \operatorname{atan2}(a_y, a_z)$

Los combina con el giroscopio mediante un filtro complementario ($\alpha = 0{,}98$, $\Delta t = 10$ ms, $\tau \approx 0{,}49$ s) y los suaviza con un IIR de primer orden ($\alpha_{IIR} = 0{,}25$). Luego los envía por broadcast UDP a 50 Hz, junto con los ángulos crudos del acelerómetro para comparar.

El nodo `glove_teleop` aplica una zona muerta de 8° y escala linealmente hasta 35°:

$$
v = -v_{max}\, s(\theta_p), \qquad \omega = -\omega_{max}\, s(\theta_r), \qquad
s(\theta) = \operatorname{sgn}(\theta)\,\min\!\left(\frac{|\theta| - 8^\circ}{35^\circ - 8^\circ},\, 1\right)
$$

Si se pierden los datos del guante durante 0,5 s, el nodo publica una orden de detener.

---

## 5. Tratamiento de las señales

### 5.1 Caracterización

**IMU**
- **Ancho de banda útil:** el movimiento de un robot de este tamaño tiene su información útil por debajo de unos 10 Hz (aceleraciones, giros, choques).
- **Ruido:** el ruido de alta frecuencia viene de la vibración de los motores y de los engranajes, y del ruido propio del sensor.
- **Antialiasing:** el DLPF interno del MPU-6050 en el modo 2 (≈94 Hz) atenúa por debajo de la frecuencia de Nyquist ($f_s/2 = 100$ Hz).
- **Bias del giroscopio:** se midió en reposo al arrancar, en el orden de 2–5 °/s por eje (ejemplo: 2,39 / 4,89 / −4,15 °/s). Se resta en cada lectura y se reajusta lentamente cuando el robot está quieto, porque cambia con la temperatura del chip.

**Encoders**
- **Resolución:** 40 flancos por vuelta, $\Delta\phi = 2\pi/40 = 0{,}157$ rad.
- **Problema de contar en ventana fija:** a 3 rad/s llega un flanco cada ≈52 ms, más de diez períodos de control. Contar flancos por ventana daría una señal muy cuantizada, que salta entre 0 y 31 rad/s.
- **Método usado:** se estima la velocidad por el **período entre flancos** (ángulo de los flancos nuevos dividido entre el tiempo transcurrido entre ellos). Cuando no llegan flancos, la velocidad se acota por $\Delta\phi/t_{desde\ el\ último}$, de modo que decae suavemente hasta cero.
- **Rebotes del comparador:** el LM393 del módulo no tiene histéresis. Cuando la rueda se detiene con el borde de una ranura frente al sensor, la salida oscila y genera pulsos falsos.
- **Ruido de los motores:** las escobillas inducen ruido en el cable D0.

### 5.2 Diseño de los filtros

**IMU: IIR Butterworth de orden 2, $f_c = 20$ Hz, $f_s = 200$ Hz**

| Coeficiente | Valor |
|---|---|
| $b_0, b_1, b_2$ | 0,067455 · 0,134911 · 0,067455 |
| $a_1, a_2$ | −1,142981 · 0,412802 |

Respuesta calculada con la transformada bilineal:

| Frecuencia | Ganancia |
|---|---|
| 10 Hz | −0,24 dB |
| 20 Hz | −3 dB |
| 50 Hz | −19,6 dB |
| 100 Hz | cero de transmisión |

El retardo de grupo en baja frecuencia es ≈ $\sqrt2/(2\pi f_c) \approx 11$ ms, aceptable para el lazo de control. El costo es de 5 multiplicaciones y 4 sumas por canal y muestra. El tiempo real que toma el filtrado completo (6 canales de IIR y 2 de media móvil) se mide en cada ciclo y se guarda en la columna `filt_us` de la telemetría.

> Tiempo medido de filtrado: **⟨completar con el promedio de `filt_us` del CSV⟩ µs** por ciclo.

**Velocidad de las ruedas: media móvil de $N = 5$ muestras**

| Característica | Valor |
|---|---|
| Ventana | 25 ms |
| Retardo | 2 muestras = 10 ms |
| Primer cero de la respuesta | $f_s/N = 40$ Hz |
| Costo | Una suma, una resta y una división por muestra |

Suaviza los saltos de la estimación por período sin retrasar el PID de forma apreciable.

**Por qué IIR y media móvil, y no FIR ni adaptativos:** en las partes anteriores de la práctica (filtrado de señales periódicas y de sensores), el equipo comparó filtros FIR, IIR y adaptativos. Los FIR necesitaban muchos más coeficientes para una banda de transición equivalente, con más memoria, más tiempo de cómputo y más retardo. Los adaptativos tuvieron el peor desempeño con estas señales. En un microcontrolador sin FPU y con un lazo de 5 ms, el IIR de orden 2 y la media móvil dan la mejor relación entre atenuación, retardo y costo.

**Filtrado de pulsos falsos en los encoders**, aplicado en la interrupción:
1. **Tiempo mínimo entre flancos:** 4 ms. Rechaza los rebotes y aun así permite medir hasta ≈39 rad/s reales.
2. **Umbral adaptativo:** un flanco que llega antes del 35 % del intervalo anterior se descarta como ruido, con un tope de 10 ms.
3. **Lecturas imposibles:** se descarta cualquier velocidad mayor a 25 rad/s, que el motor no puede alcanzar.

### 5.3 Datos crudos y filtrados

El `data_logger` guarda, para cada muestra, la IMU cruda y filtrada y la velocidad de las ruedas cruda y filtrada (las 30 columnas descritas en el [README](README.md#protocolo-udp)). Con esos CSV se comparan la señal cruda y la filtrada en el tiempo y en frecuencia (FFT), y se verifica la atenuación de la vibración de los motores.

> Figuras sugeridas, a generar con los CSV del equipo: **⟨completar⟩**
> - $g_z$ crudo y filtrado con el robot avanzando, y su espectro (FFT).
> - $\omega_l$ y $\omega_r$ crudas y filtradas frente a la referencia.

---

## 6. Control

### 6.1 Velocidad de cada rueda

En la primera versión, el PID podía bajar el ciclo útil hasta cero. Con los datos de la telemetría se encontró una falla: cuando una rueda se detenía en el borde de una ranura, su encoder marcaba velocidades falsas. Por ejemplo:

| Rueda | Encoder | Ciclo útil | Referencia |
|---|---|---|---|
| Izquierda | 11 a 55 rad/s (falsos) | **0** | 3,08 rad/s |
| Derecha | 1,3 a 2,5 rad/s | 0,26 a 0,40 | 3,08 rad/s |

El PID de la izquierda veía "exceso de velocidad", cortaba el motor, y la rueda quedaba bloqueada en esa misma ranura. Como la rueda que fallaba dependía de dónde había quedado detenida, el síntoma "cambiaba de rueda" entre pruebas.

La estrategia final hace que el control sea robusto a un encoder ruidoso:

1. **Prealimentación con la zona muerta de cada motor:** los motores TT no son iguales. El derecho gira más con el mismo PWM. Se usan $d_{min,izq} = 0{,}15$, $d_{min,der} = 0{,}085$ y $k_{ff} = 0{,}020$/(rad/s). Para 3,08 rad/s esto da 0,21 a la izquierda y 0,15 a la derecha.
2. **Corrección limitada del PID:** el PID ($k_p = 0{,}02$, $k_i = 0{,}25$) solo puede mover la salida **±0,03** alrededor de la prealimentación. Un encoder con ruido ya no puede frenar ni disparar la rueda.
3. **Sin inversión:** la salida nunca cambia de sentido respecto a la referencia, porque el encoder de un canal toma el sentido de la referencia.
4. **Arranque suave:** el ciclo útil sube como máximo 3 por segundo (de 0 a 0,3 en 100 ms). Así se reduce el pico de corriente al arrancar. Frenar sigue siendo inmediato.

### 6.2 Rumbo con el giroscopio

El PID por rueda no garantiza que el robot vaya recto: cada rueda puede creer que cumple su referencia mientras el robot se desvía. Por eso se agregó un lazo PI sobre la velocidad de giro medida por el giroscopio:

$$
e_\psi = \underbrace{\frac{r(\omega_{r,ref} - \omega_{l,ref})}{L}}_{\text{giro pedido}} - \underbrace{s_z\,g_{z,f}}_{\text{giro medido}}, \qquad
c = k_{p\psi}\,e_\psi + k_{i\psi}\!\int e_\psi\,dt, \qquad
u_l \mathrel{-}= c,\; u_r \mathrel{+}= c
$$

La integral de la velocidad de giro es el error de rumbo, así que el término integral se comporta como un control proporcional sobre la orientación. Corrige diferencias constantes, como un motor más débil, sin depender de los encoders.

**Signo del giroscopio:** en la primera prueba en el piso, el robot pivoteó sobre la rueda izquierda. La corrección empujaba hacia el lado equivocado porque la IMU está montada de forma que el giro antihorario da $g_z$ negativo. Se corrigió con $s_z = -1$ (`GYRO_Z_SIGN`).

**Sintonización** (pruebas en el piso a 0,15 m/s):

| $k_{p\psi}$ | $k_{i\psi}$ | Corrección máx. | Resultado |
|---|---|---|---|
| 0,05 | 0,40 | 0,20 | Se desvía despacio a la derecha |
| **0,08** | **1,00** | **0,30** | **Va recto, con un zigzag leve** |
| 0,10 | 0,70 | 0,30 | Zigzaguea más |
| 0,05 | 0,90 | 0,30 | Vuelve a desviarse |
| 0,065 | 1,00 | 0,30 | Se desvía |

Se eligió $k_{p\psi} = 0{,}08$, $k_{i\psi} = 1{,}00$ y corrección máxima de 0,30. Las pruebas muestran que $k_{p\psi}$ controla sobre todo la oscilación y $k_{i\psi}$ la desviación sostenida.

---

## 7. Problemas encontrados y soluciones

| # | Síntoma | Diagnóstico | Solución |
|---|---|---|---|
| 1 | Al arrancar los motores, errores I2C. Después el lazo se congelaba y **los motores quedaban girando** con el último PWM | El ruido de los motores corrompía el I2C. Al intentar recuperar el bus, el driver de ESP-IDF 5.3.1 entra en una espera sin límite de tiempo (`i2c_ll_master_clr_bus`) | I2C por software con todas las esperas limitadas y recuperación del bus. Watchdog de tareas de 3 s que reinicia el ESP (con los motores apagados) si el lazo se traba |
| 2 | El lazo usaba ≈4 ms de 5 ms y se pasaba de su tiempo | `snprintf` con 30 números decimales en un chip sin FPU | Formatear la telemetría en una tarea de baja prioridad. El lazo bajó a ≈1,5–2 ms con la IMU |
| 3 | Una rueda no giraba; a veces la izquierda y a veces la derecha | Rebotes del encoder en el borde de una ranura y PID que cortaba el motor | Piso de ciclo útil, corrección del PID limitada y filtros de pulsos falsos (sección 6.1) |
| 4 | La rueda derecha nunca giraba | Un cable del motor mal soldado | Volver a soldar. Diagnóstico con el modo de lazo abierto (`M,dl,dr`) |
| 5 | Telemetría en 0/s y el robot en `PC esperando` | El hotspot cambió de subred (10.161.6.x → 10.42.110.x) y la VM se quedó con la IP vieja. Los broadcast llegaban, pero los paquetes dirigidos no | Renovar la conexión de red de la VM |
| 6 | Calentamiento del ESP32-C3 | Medido con el sensor interno: se estabiliza en **50–55 °C** con el WiFi siempre activo. Está dentro de lo normal (máximo del chip: 105 °C) | Temperatura del chip incluida en el mensaje de estado |
| 7 | Con batería (no con USB), la telemetría cae a 0 y el LED azul se apaga y vuelve en ciclos | El LED azul está en GPIO8 (SDA). Si se apaga, el ESP dejó de leer la IMU, es decir, se reinició. El LED rojo seguía encendido y el ciclo se repetía: caída de voltaje al arrancar los motores (brownout), con el bridge enviando órdenes otra vez después de cada arranque | Arranque suave en el firmware y causa del reinicio en el monitor. Recomendado en hardware: alimentar el ESP aparte (power bank) o poner diodo y 1000 µF antes del LM2596, y resistencias de 10 kΩ de PWMA y PWMB a GND |
| 8 | `RTPS_TRANSPORT_SHM Error` en ROS2 | Archivos de memoria compartida de FastDDS que quedaron de nodos cerrados con Ctrl+C. FastDDS usa UDP en su lugar | `ros2 daemon stop` y borrar `/dev/shm/fastrtps_*` |

---

## 8. Experimento: posición final

### 8.1 Procedimiento

1. Se marca el origen y el eje $x$ en el dojo. El robot se coloca en el origen, mirando hacia $+x$.
2. Se lanzan `robot_bridge`, `diff_kinematics` y `data_logger` (para guardar los datos crudos y filtrados de cada iteración).
3. Se ejecuta la rutina con el nodo `routines`. En cada iteración, el nodo:
   - reinicia la odometría (`R`);
   - envía los tramos $(v, \omega, T)$ por `/cmd_vel`;
   - detiene el robot;
   - registra la **posición deseada** (integración exacta del modelo, sección 2.1) y la **estimada** (`/odom`, fusión de encoders y giroscopio).
4. Se mide con cinta la **posición real** del centro del eje de las ruedas.
5. Se repite 3 veces por rutina.

```bash
ros2 run minisumo_ros routines --ros-args -p rutina:=recta    -p distancia:=0.5 -p v:=0.15 -p iteraciones:=3
ros2 run minisumo_ros routines --ros-args -p rutina:=arco     -p radio:=0.2     -p v:=0.15 -p iteraciones:=3
ros2 run minisumo_ros routines --ros-args -p rutina:=cuadrado -p distancia:=0.3 -p v:=0.15 -p w:=3.0 -p iteraciones:=3
ros2 run minisumo_ros routines --ros-args -p rutina:=giro     -p angulo:=90     -p w:=3.0  -p iteraciones:=3
```

| Rutina | Tramos | Duración | Posición final deseada $(x, y, \theta)$ |
|---|---|---|---|
| Recta 0,5 m a 0,15 m/s | 1 | 3,33 s | (0,500 m; 0,000 m; 0°) |
| Arco de 90°, $R = 0{,}2$ m, a 0,15 m/s | 1 | 2,09 s | (0,200 m; 0,200 m; 90°) |
| Cuadrado de 0,3 m de lado, giros de 90° a 3 rad/s | 16 (con pausas) | 13,29 s | (0,000 m; 0,000 m; 0°) |
| Giro en el sitio de 90° a 3 rad/s | 1 | 0,52 s | (0,000 m; 0,000 m; 90°) |

### 8.2 Resultados

| Rutina | Iteración | Deseada $x$ (m) | Deseada $y$ (m) | Real $x$ (m) | Real $y$ (m) | Estimada $x$ (m) | Estimada $y$ (m) |
|---|---|---|---|---|---|---|---|
| Recta | 1 | 0,500 | 0,000 | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ |
| Recta | 2 | 0,500 | 0,000 | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ |
| Recta | 3 | 0,500 | 0,000 | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ |
| Arco | 1 | 0,200 | 0,200 | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ |
| Arco | 2 | 0,200 | 0,200 | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ |
| Arco | 3 | 0,200 | 0,200 | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ |
| Cuadrado | 1 | 0,000 | 0,000 | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ |
| Cuadrado | 2 | 0,000 | 0,000 | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ |
| Cuadrado | 3 | 0,000 | 0,000 | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ |

Las columnas deseada y estimada se copian de `~/minisumo_datos/rutinas.csv`. La real se mide.

### 8.3 Cálculo del error

Para cada iteración, el error de la estimación respecto a la posición real:

$$
e_{est} = \sqrt{(x_{est} - x_{real})^2 + (y_{est} - y_{real})^2}
$$

y el error del modelo (la posición pedida frente a la alcanzada):

$$
e_{mod} = \sqrt{(x_{des} - x_{real})^2 + (y_{des} - y_{real})^2}
$$

| Rutina | $\bar e_{est}$ (cm) | $\sigma_{e_{est}}$ (cm) | $\bar e_{mod}$ (cm) | $\sigma_{e_{mod}}$ (cm) |
|---|---|---|---|---|
| Recta | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ |
| Arco | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ |
| Cuadrado | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ | ⟨completar⟩ |

### 8.4 Fuentes de error esperadas

Este análisis se hace a partir del diseño. Con los resultados de la tabla 8.2 se puede confirmar cuál fuente domina.

**Deseada frente a real ($e_{mod}$).** El modelo supone que las ruedas alcanzan la velocidad pedida al instante y no patinan. En el robot real:
- **Arranque suave:** el ciclo útil sube con una rampa de unos 100 ms, y la inercia del motor añade su propio retardo. El robot recorre **menos** que $vT$, sobre todo en tramos cortos y en los giros del cuadrado.
- **Inercia al frenar:** al detenerse, el robot sigue avanzando un poco, lo que compensa parcialmente lo anterior.
- **Patinaje y zona muerta:** las ruedas patinan, sobre todo al girar en el sitio a 3 rad/s, y la zona muerta de los motores se suma.
- **Errores de rumbo acumulados:** en el cuadrado, cada giro con error de rumbo desplaza el punto final. El error crece con el número de tramos.
- **Geometría:** el radio efectivo de la rueda depende de la presión de la llanta, y la distancia efectiva entre las ruedas $L$ depende de la zona de contacto.
- **Tiempo de los tramos:** en ROS2 el tiempo de cada tramo tiene variaciones de unos pocos milisegundos, y los comandos tienen latencia de WiFi.

**Estimada frente a real ($e_{est}$).**
- **Resolución del encoder:** 9° de rueda equivalen a 5,1 mm de avance.
- **Sin sentido de giro:** el encoder de un canal toma el sentido de la referencia.
- **Pulsos falsos:** los que no logran filtrarse aumentan la distancia estimada.
- **Patinaje:** los encoders no lo ven. El rumbo, que pesa 98 % del giroscopio, sí lo corrige en parte.
- **Bias residual del giroscopio:** se integra mientras el robot se mueve.
- **Errores en $r$ y $L$:** afectan por igual a la estimación y al modelo.

---

## 9. Conclusiones

1. **El filtrado debe ir en el microcontrolador, ajustado a sus recursos.** En un chip sin FPU y con un lazo de 5 ms, un IIR Butterworth de orden 2 (≈11 ms de retardo, −19,6 dB a 50 Hz) y una media móvil de 5 muestras (10 ms de retardo) quitan el ruido de vibración sin degradar la información útil, que está por debajo de unos 10 Hz. Un FIR con atenuación equivalente sería bastante más costoso.
2. **El costo de cómputo se nota en un lazo en tiempo real.** Mover el formateo de la telemetría fuera del lazo redujo su tiempo de ≈4 ms a ≈1,5–2 ms con la IMU.
3. **El ruido de un sensor puede bloquear el control si no se limita su efecto.** Los encoders de herradura de un canal tienen poca resolución y son muy sensibles al ruido eléctrico. La estimación por período entre flancos, los filtros de pulsos falsos y limitar cuánto puede corregir el PID alrededor de una prealimentación por motor evitaron que los pulsos falsos bloquearan las ruedas.
4. **Para ir recto hace falta realimentar el rumbo.** El control de rumbo PI con el giroscopio corrigió la diferencia entre motores sin depender de los encoders. La sintonización mostró la relación esperada: $k_p$ amortigua y $k_i$ elimina el error sostenido.
5. **La integración exige diagnóstico sistemático.** Fueron clave el monitor de estado (tiempo del lazo, errores de la IMU, temperatura, causa del reinicio), el modo de lazo abierto `M` y la telemetría completa en ROS2. Con ellos se separaron fallas de software (driver I2C), de hardware (soldadura, alimentación) y de red (subred del hotspot).
6. **Experimento de posición final:** ⟨completar con la comparación numérica entre deseada, real y estimada: qué rutina tuvo mayor error y por qué, y si la estimación por sensores se acercó más a la posición real que el modelo⟩.

---

## Referencias

1. R. Siegwart, I. Nourbakhsh y D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2.ª ed. MIT Press, 2011. Cinemática de robots diferenciales y odometría.
2. A. V. Oppenheim y R. W. Schafer, *Discrete-Time Signal Processing*, 3.ª ed. Pearson, 2010. Filtros FIR e IIR, transformada bilineal.
3. K. J. Åström y T. Hägglund, *Advanced PID Control*. ISA, 2006. Anti-windup y prealimentación.
4. Espressif Systems, *ESP-IDF Programming Guide v5.3*. https://docs.espressif.com/projects/esp-idf/
5. InvenSense, *MPU-6000 and MPU-6050 Product Specification*, rev. 3.4.
6. Toshiba, *TB6612FNG Driver IC for Dual DC Motor*, hoja de datos.
7. Open Robotics, *ROS 2 Documentation*. https://docs.ros.org/
8. Guía de la Práctica 2, *Filtrado digital y robot minisumo teleoperado*, Procesamiento de Señales.

---

## Anexo A. Código

Todo el código está en este repositorio:

- [firmware/robot](firmware/robot): firmware del robot (`main.c`, `encoders.c`, `pid.c`, `filters.c`, `odometry.c`, `mpu6050.c`, `motors.c`, `net.c`, `config.h`).
- [firmware/guante](firmware/guante): firmware del guante.
- [ros2/minisumo_ros](ros2/minisumo_ros): nodos de ROS2.

El protocolo UDP y las columnas de la telemetría están en el [README](README.md#protocolo-udp).
