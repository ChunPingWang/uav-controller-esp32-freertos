# Flight Controller Core Component

idf_component_register(
    SRCS
        "src/fc_log.c"
        "src/fc_memory.c"
        "src/fc_events.c"
        "src/fc_nvs.c"
        "src/fc_failsafe.c"
        "src/fc_health.c"
        "src/fc_battery.c"
        "src/fc_selftest.c"
        "src/fc_monitor_task.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        freertos
        esp_system
        esp_log
        nvs_flash
        driver
        esp_adc
    PRIV_REQUIRES
        esp_timer
)
