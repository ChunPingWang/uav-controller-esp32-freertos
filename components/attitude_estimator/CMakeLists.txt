# Attitude Estimator Component (with EKF)

idf_component_register(
    SRCS
        "src/attitude_estimator.c"
        "src/ekf_core.c"
        "src/matrix_math.c"
        "src/attitude_task.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        freertos
        esp_system
        esp_log
        esp_timer
        flight_controller
        imu_sensor
)
