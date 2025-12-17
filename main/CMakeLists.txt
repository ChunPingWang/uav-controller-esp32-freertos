# Main application component

idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        freertos
        esp_system
        esp_log
        nvs_flash
        driver
        flight_controller
        imu_sensor
        attitude_estimator
        motor_controller
        gps_navigator
        xbee_telemetry
)
