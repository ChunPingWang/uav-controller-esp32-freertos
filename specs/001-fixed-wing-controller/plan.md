# Implementation Plan: 固定翼飛機控制器

**Branch**: `001-fixed-wing-controller` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-fixed-wing-controller/spec.md`

## Summary

開發基於 ESP32 與 FreeRTOS 的固定翼無人機飛行控制器，包含六軸感測器姿態估算、動力控制、GPS 導航、XBee 遙測通訊等核心功能。系統需滿足 100Hz 控制迴路、<15ms 端對端延遲的即時性要求，並實現通訊中斷自動返航（RTL）等失效安全機制。

**關鍵技術決策**：採用 Extended Kalman Filter (EKF) 進行姿態估算，融合 IMU 與 GPS 資料以提供更精確且抗雜訊的姿態輸出。

## Technical Context

**Language/Version**: C11 (ESP-IDF v5.x with FreeRTOS kernel)
**Primary Dependencies**: ESP-IDF, FreeRTOS, driver libraries (I2C/SPI/UART/PWM)
**Algorithm Library**: Custom EKF implementation (no external math library for embedded optimization)
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
| III. 記憶體管理 | 靜態分配優先 | ✅ 使用 xTaskCreateStatic，EKF 矩陣靜態配置 |
| IV. 同步通訊 | 使用 FreeRTOS IPC | ✅ Queue/EventGroup/Mutex |
| V. ISR 設計 | < 100μs，延遲至 Task | ✅ 最小化 ISR |
| VI. 錯誤處理 | esp_err_t 統一回傳 | ✅ 所有 API 使用 esp_err_t |
| VII. 測試驅動 | 先寫測試再實作 | ✅ Unity 單元測試，含 EKF 數值驗證 |
| VIII. 電源管理 | 定義 Sleep 條件 | ✅ 飛行中禁用 Sleep |
| IX. 版本控制 | ESP-IDF 建置系統 | ✅ CMake + idf.py |

**Gate Status**: ✅ PASS - 所有原則符合，可進入 Phase 0

## Kalman Filter Design

### Why Extended Kalman Filter (EKF)

| 方案 | 優點 | 缺點 | 決策 |
|------|------|------|------|
| Complementary Filter | 簡單、低運算 | 調參困難、精度有限 | ❌ |
| **Extended Kalman Filter** | 最佳估計、可融合多感測器 | 較複雜、需矩陣運算 | ✅ 採用 |
| Unscented Kalman Filter | 高非線性精度 | 運算量大、ESP32 可能不足 | ❌ |
| Madgwick/Mahony | 專為 IMU 設計、高效 | 無 GPS 融合能力 | ❌ |

### EKF State Vector

```
x = [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]^T
```

- **狀態量 (6 維)**：三軸姿態角 + 三軸陀螺儀偏差
- **量測更新**：加速度計（重力方向）、磁力計（航向，若有）、GPS（航向校正）
- **預測更新**：陀螺儀角速度積分

### Implementation Considerations

1. **矩陣運算**：自行實作 6x6 矩陣乘法/求逆，避免外部依賴
2. **數值穩定**：使用 Joseph form 更新協方差矩陣
3. **計算效率**：預計單次 EKF 更新 < 500μs @ 240MHz
4. **靜態配置**：所有矩陣在編譯時靜態分配

## Project Structure

### Documentation (this feature)

```text
specs/001-fixed-wing-controller/
├── plan.md              # This file
├── research.md          # Phase 0 output (Kalman Filter research)
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
├── attitude_estimator/         # 姿態估算模組 (含 EKF)
│   ├── include/
│   │   ├── attitude_estimator.h
│   │   ├── ekf_types.h         # EKF 資料結構
│   │   └── matrix_math.h       # 矩陣運算
│   ├── src/
│   │   ├── attitude_estimator.c
│   │   ├── ekf_core.c          # EKF 核心演算法
│   │   └── matrix_math.c       # 矩陣運算實作
│   ├── test/
│   │   ├── test_attitude_estimator.c
│   │   ├── test_ekf_core.c     # EKF 數值驗證
│   │   └── test_matrix_math.c  # 矩陣運算測試
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

**Structure Decision**: 採用 ESP-IDF Component 架構，每個功能模組為獨立元件，符合憲章第一條模組化原則。EKF 實作置於 attitude_estimator 元件內，包含獨立的矩陣運算模組。

## FreeRTOS Task Architecture

| Task Name | Priority | Stack | 週期 | 說明 |
|-----------|----------|-------|------|------|
| vTask_IMU_Read | configMAX_PRIORITIES-1 | 4096 | 10ms | 感測器讀取（硬即時） |
| vTask_Attitude_Compute | configMAX_PRIORITIES-2 | 8192 | 10ms | EKF 姿態估算（硬即時，需較大 Stack） |
| vTask_Motor_Control | configMAX_PRIORITIES-2 | 2048 | 10ms | 動力輸出（硬即時） |
| vTask_GPS_Parse | tskIDLE_PRIORITY+3 | 4096 | 200ms | GPS 解析 |
| vTask_Telemetry_TX | tskIDLE_PRIORITY+2 | 4096 | 50ms | 遙測發送 |
| vTask_Telemetry_RX | tskIDLE_PRIORITY+2 | 4096 | Event | 指令接收 |
| vTask_System_Monitor | tskIDLE_PRIORITY+1 | 2048 | 1000ms | 系統監控 |

**Note**: vTask_Attitude_Compute Stack 增加至 8192 bytes 以容納 EKF 矩陣運算所需的堆疊空間。

## IPC Mechanisms

| 來源 | 目標 | 機制 | 說明 |
|------|------|------|------|
| IMU_Read | Attitude_Compute | Queue | 感測器資料傳遞 |
| GPS_Parse | Attitude_Compute | Queue | GPS 航向用於 EKF 量測更新 |
| Attitude_Compute | Motor_Control | Direct (shared) | 姿態狀態共享 |
| GPS_Parse | Flight_Controller | Queue | GPS 位置更新 |
| Telemetry_RX | Flight_Controller | Queue | 控制指令 |
| All Tasks | Telemetry_TX | Mutex + Struct | 遙測資料彙整 |
| Any | System_Monitor | EventGroup | 異常事件通知 |

## Complexity Tracking

| 增加項目 | 為何需要 | 拒絕簡單替代方案的理由 |
|----------|----------|------------------------|
| EKF 矩陣運算模組 | 最佳姿態估計需要 | Complementary Filter 精度不足以達成 <2° 要求 |
| 8KB Stack for EKF Task | 6x6 矩陣運算需較大堆疊 | 減少矩陣維度會降低估計品質 |

> 上述複雜度增加經評估為達成 SC-003（<2° 姿態精度）所必要。
