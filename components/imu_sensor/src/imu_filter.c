/**
 * @file imu_filter.c
 * @brief Digital low-pass filter for IMU data (FR-002)
 *
 * Implements simple IIR low-pass filter to reduce noise.
 */

#include "imu_sensor.h"
#include "fc_types.h"
#include <math.h>

/* Filter state */
static float s_alpha = 0.1f;
static sensor_data_t s_filtered = {0};
static bool s_initialized = false;

void imu_filter_init(float cutoff_hz, float sample_rate_hz)
{
    /* Calculate filter coefficient
     * alpha = dt / (RC + dt)
     * where RC = 1 / (2 * PI * cutoff)
     */
    float dt = 1.0f / sample_rate_hz;
    float rc = 1.0f / (2.0f * FC_PI * cutoff_hz);
    s_alpha = dt / (rc + dt);

    /* Clamp alpha to reasonable range */
    if (s_alpha < 0.01f) s_alpha = 0.01f;
    if (s_alpha > 1.0f) s_alpha = 1.0f;

    s_initialized = false; /* Will initialize on first sample */
}

void imu_filter_apply(sensor_data_t *data)
{
    if (data == NULL) {
        return;
    }

    if (!s_initialized) {
        /* Initialize filter state with first sample */
        s_filtered = *data;
        s_initialized = true;
        return;
    }

    /* Apply IIR low-pass filter: y[n] = alpha * x[n] + (1 - alpha) * y[n-1] */
    float beta = 1.0f - s_alpha;

    s_filtered.accel_x = s_alpha * data->accel_x + beta * s_filtered.accel_x;
    s_filtered.accel_y = s_alpha * data->accel_y + beta * s_filtered.accel_y;
    s_filtered.accel_z = s_alpha * data->accel_z + beta * s_filtered.accel_z;

    s_filtered.gyro_x = s_alpha * data->gyro_x + beta * s_filtered.gyro_x;
    s_filtered.gyro_y = s_alpha * data->gyro_y + beta * s_filtered.gyro_y;
    s_filtered.gyro_z = s_alpha * data->gyro_z + beta * s_filtered.gyro_z;

    /* Copy filtered values back */
    data->accel_x = s_filtered.accel_x;
    data->accel_y = s_filtered.accel_y;
    data->accel_z = s_filtered.accel_z;
    data->gyro_x = s_filtered.gyro_x;
    data->gyro_y = s_filtered.gyro_y;
    data->gyro_z = s_filtered.gyro_z;
}

void imu_filter_reset(void)
{
    s_initialized = false;
}
