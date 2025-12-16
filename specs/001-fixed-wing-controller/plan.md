# Implementation Plan: 固定翼飛機控制器

**Branch**: `001-fixed-wing-controller` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-fixed-wing-controller/spec.md`

## Summary

開發基於 ESP32 與 FreeRTOS 的固定翼無人機飛行控制器，包含六軸感測器姿態估算、動力控制、GPS 導航、XBee 遙測通訊等核心功能。系統需滿足 100Hz 控制迴路、<15ms 端對端延遲的即時性要求，並實現通訊中斷自動返航（RTL）等失效安全機制。

## Technical Context

**Language/Version**: C11 (ESP-IDF v5.x with FreeRTOS kernel)
**Primary Dependencies**: ESP-IDF, FreeRTOS, driver libraries (I2C/SPI/UART/PWM)
**Storage**: NVS (Non-Volatile Storage) for calibration data and configuration
**Testing**: Unity (ESP-IDF built-in), pytest + ESP-IDF Test Framework for hardware tests
**Target Platform**: ESP32 (Xtensa LX6 dual-core, 240MHz)
**Project Type**: Embedded single project with ESP-IDF component structure
**Performance Goals**: 100Hz control loop (10ms period), <15ms end-to-end latency, <2° attitude accuracy
**Constraints**: Real-time hard deadline, ~320KB SRAM, battery-powered, ISR < 100μs
**Scale/Scope**: Single flight controller, 6 functional components, 7 FreeRTOS tasks

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| 原則 | 要求 | 符合狀態 |
|------|------|----------|
| I. 模組化架構 | 每個功能以獨立 Component 實作 | ✅ 規劃 6 個獨立元件 |
| II. Task 設計 | 明確優先順序與 Stack 分析 | ✅ 將定義 7 個 Task 優先級 |
| III. 記憶體管理 | 靜態分配優先 | ✅ 使用 xTaskCreateStatic |
| IV. 同步通訊 | 使用 FreeRTOS IPC | ✅ Queue/EventGroup/Mutex |
| V. ISR 設計 | < 100μs，延遲至 Task | ✅ 最小化 ISR |
| VI. 錯誤處理 | esp_err_t 統一回傳 | ✅ 所有 API 使用 esp_err_t |
| VII. 測試驅動 | 先寫測試再實作 | ✅ Unity 單元測試 |
| VIII. 電源管理 | 定義 Sleep 條件 | ✅ 飛行中禁用 Sleep |
| IX. 版本控制 | ESP-IDF 建置系統 | ✅ CMake + idf.py |

**Gate Status**: ✅ PASS - 所有原則符合，可進入 Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/001-fixed-wing-controller/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (internal module APIs)
└── tasks.md             # Phase 2 output (/speckit.tasks)
```

### Source Code (repository root)

```text
components/
├── imu_sensor/                 # 六軸感測器模組
│   ├── include/
│   │   └── imu_sensor.h
│   ├── src/
│   │   └── imu_sensor.c
│   ├── test/
│   │   └── test_imu_sensor.c
│   └── CMakeLists.txt
├── attitude_estimator/         # 姿態估算模組
│   ├── include/
│   │   └── attitude_estimator.h
│   ├── src/
│   │   └── attitude_estimator.c
│   ├── test/
│   │   └── test_attitude_estimator.c
│   └── CMakeLists.txt
├── motor_controller/           # 動力控制模組
│   ├── include/
│   │   └── motor_controller.h
│   ├── src/
│   │   └── motor_controller.c
│   ├── test/
│   │   └── test_motor_controller.c
│   └── CMakeLists.txt
├── gps_navigator/              # GPS 導航模組
│   ├── include/
│   │   └── gps_navigator.h
│   ├── src/
│   │   └── gps_navigator.c
│   ├── test/
│   │   └── test_gps_navigator.c
│   └── CMakeLists.txt
├── xbee_telemetry/             # XBee 遙測模組
│   ├── include/
│   │   └── xbee_telemetry.h
│   ├── src/
│   │   └── xbee_telemetry.c
│   ├── test/
│   │   └── test_xbee_telemetry.c
│   └── CMakeLists.txt
└── flight_controller/          # 飛行控制器整合模組
    ├── include/
    │   └── flight_controller.h
    ├── src/
    │   └── flight_controller.c
    ├── test/
    │   └── test_flight_controller.c
    └── CMakeLists.txt

main/
├── main.c                      # 應用程式進入點
├── app_config.h                # 應用程式配置
└── CMakeLists.txt

test_apps/                      # 硬體整合測試
├── test_imu/
├── test_gps/
└── test_xbee/
```

**Structure Decision**: 採用 ESP-IDF Component 架構，每個功能模組為獨立元件，符合憲章第一條模組化原則。所有元件包含獨立的 include/src/test 目錄。

## FreeRTOS Task Architecture

| Task Name | Priority | Stack | 週期 | 說明 |
|-----------|----------|-------|------|------|
| vTask_IMU_Read | configMAX_PRIORITIES-1 | 4096 | 10ms | 感測器讀取（硬即時） |
| vTask_Attitude_Compute | configMAX_PRIORITIES-2 | 4096 | 10ms | 姿態估算（硬即時） |
| vTask_Motor_Control | configMAX_PRIORITIES-2 | 2048 | 10ms | 動力輸出（硬即時） |
| vTask_GPS_Parse | tskIDLE_PRIORITY+3 | 4096 | 200ms | GPS 解析 |
| vTask_Telemetry_TX | tskIDLE_PRIORITY+2 | 4096 | 50ms | 遙測發送 |
| vTask_Telemetry_RX | tskIDLE_PRIORITY+2 | 4096 | Event | 指令接收 |
| vTask_System_Monitor | tskIDLE_PRIORITY+1 | 2048 | 1000ms | 系統監控 |

## IPC Mechanisms

| 來源 | 目標 | 機制 | 說明 |
|------|------|------|------|
| IMU_Read | Attitude_Compute | Queue | 感測器資料傳遞 |
| Attitude_Compute | Motor_Control | Direct (shared) | 姿態狀態共享 |
| GPS_Parse | Flight_Controller | Queue | GPS 位置更新 |
| Telemetry_RX | Flight_Controller | Queue | 控制指令 |
| All Tasks | Telemetry_TX | Mutex + Struct | 遙測資料彙整 |
| Any | System_Monitor | EventGroup | 異常事件通知 |

## Complexity Tracking

> **無違規項目** - 設計符合所有憲章原則
