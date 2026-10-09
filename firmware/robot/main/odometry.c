#include <math.h>
#include <string.h>
#include "odometry.h"
#include "config.h"

static float wrap_pi(float a)
{
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void odom_reset(odom_t *o)
{
    memset(o, 0, sizeof(*o));
}

void odom_update(odom_t *o, float dth_l, float dth_r, float gz, float dt, bool moving)
{
    float dl = WHEEL_RADIUS_M * dth_l;
    float dr = WHEEL_RADIUS_M * dth_r;
    float ds = 0.5f * (dl + dr);
    float dpsi_enc = (dr - dl) / WHEEL_BASE_M;
    // Robot detenido: no se integra el giroscopio para no acumular deriva del bias
    float dpsi_gyro = moving ? gz * dt : 0.0f;
    float dpsi = HEADING_ALPHA * dpsi_gyro + (1.0f - HEADING_ALPHA) * dpsi_enc;

    float th_mid = o->theta + 0.5f * dpsi;
    o->x += ds * cosf(th_mid);
    o->y += ds * sinf(th_mid);
    o->theta = wrap_pi(o->theta + dpsi);
    o->theta_enc = wrap_pi(o->theta_enc + dpsi_enc);
    o->theta_gyro = wrap_pi(o->theta_gyro + dpsi_gyro);
}
