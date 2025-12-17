# IMU Sensor Component

idf_component_register(
    SRCS
        "src/imu_sensor.c"
        "src/imu_filter.c"
        "src/imu_calibration.c"
        "src/imu_task.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        freertos
        esp_system
        esp_log
        driver
        flight_controller
)
