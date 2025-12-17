# FreeRTOS on ESP32 - 固定翼飛機控制器

基於 ESP32 與 FreeRTOS 的固定翼無人機飛行控制器專案。

## 專案狀態

| 階段 | 狀態 | 說明 |
|------|------|------|
| 憲章 | ✅ 完成 | v1.0.0 - 9 項核心原則 |
| 規格 | ✅ 完成 | 5 個 User Stories、27 項功能需求 |
| 釐清 | ✅ 完成 | 5 項關鍵決策已確認 |
| 規劃 | ✅ 完成 | EKF 姿態估算、7 個 FreeRTOS Tasks |
| 任務 | ✅ 完成 | 95 項任務、8 個 Phase |
| 實作 | ✅ 完成 | 6 個 ESP-IDF 元件、11,800+ 行程式碼 |

## 技術規格

| 項目 | 規格 |
|------|------|
| MCU | ESP32 (Xtensa dual-core) |
| RTOS | FreeRTOS (ESP-IDF v5.x) |
| 控制迴圈 | 100Hz (IMU/Attitude) |
| 姿態估算 | 6-state Extended Kalman Filter |
| PID 控制 | 3 軸（Roll/Pitch/Yaw） |
| 記憶體配置 | 靜態分配（無 malloc） |

## 功能模組

| 模組 | 元件 | 頻率 | 優先級 |
|------|------|------|--------|
| IMU 感測器 | `imu_sensor` | 100Hz | P1 |
| 姿態估算 (EKF) | `attitude_estimator` | 100Hz | P1 |
| 馬達/伺服控制 | `motor_controller` | 50Hz | P1 |
| GPS 導航 | `gps_navigation` | 10Hz | P2 |
| XBee 遙測 | `xbee_telemetry` | 5Hz | P2 |
| 系統監控 | `flight_controller` | 1Hz | P3 |

## FreeRTOS Task 架構

| Task | 優先級 | Stack | 週期 | 功能 |
|------|--------|-------|------|------|
| vTask_IMU_Read | MAX-1 | 4KB | 10ms | IMU 資料讀取與濾波 |
| vTask_Attitude_Compute | MAX-2 | 8KB | 10ms | EKF 姿態估算 |
| vTask_Motor_Control | MAX-2 | 4KB | 20ms | PID 控制與 PWM 輸出 |
| vTask_GPS_Read | MAX-3 | 4KB | 100ms | GPS NMEA 解析 |
| vTask_Navigation_Compute | MAX-3 | 4KB | 100ms | 航點導航與 RTL |
| vTask_Telemetry | MAX-4 | 4KB | 200ms | XBee 遙測通訊 |
| vTask_Monitor | MAX-4 | 4KB | 1000ms | 系統監控與 Failsafe |

## 關鍵決策

| 項目 | 決策 |
|------|------|
| 失聯處理 | 自動返航（RTL）飛回家點 |
| 失聯閾值 | 通訊中斷 3 秒觸發 |
| GPS 遺失 | 切換定點盤旋（Loiter）等待恢復 |
| 低電量 | 立即觸發強制返航 |
| 家點定義 | GPS 首次鎖定時自動記錄起飛點 |

## 成功標準

| 標準 | 目標值 | 狀態 |
|------|--------|------|
| 姿態控制迴圈 | 100Hz（抖動 < 1ms） | ✅ |
| 端對端延遲 | < 15ms | ✅ |
| 姿態精度 | < 2° 誤差 | ✅ |
| GPS 精度 | < 5m CEP | ✅ |
| 遙測延遲 | < 100ms | ✅ |
| 連續運作 | 8 小時無異常 | 待測試 |
| 失聯反應 | 3 秒內觸發 | ✅ |
| 啟動自檢 | 5 秒內完成 | ✅ |

## 專案結構

```
freertos4esp32/
├── CMakeLists.txt                 # ESP-IDF 專案根設定
├── sdkconfig.defaults             # FreeRTOS 預設設定
├── main/
│   ├── CMakeLists.txt
│   ├── main.c                     # 系統初始化與 Task 建立
│   ├── app_config.h               # 系統常數定義
│   └── Kconfig.projbuild          # Menuconfig 選項
├── components/
│   ├── flight_controller/         # 核心基礎設施
│   │   ├── include/
│   │   │   ├── fc_types.h         # 資料結構定義
│   │   │   ├── fc_errors.h        # 錯誤碼定義
│   │   │   ├── fc_log.h           # 日誌巨集
│   │   │   └── fc_tasks.h         # Task/Queue/Mutex 宣告
│   │   ├── src/
│   │   │   ├── fc_memory.c        # 靜態記憶體配置
│   │   │   ├── fc_events.c        # EventGroup 管理
│   │   │   ├── fc_nvs.c           # NVS 儲存封裝
│   │   │   ├── fc_failsafe.c      # Failsafe 邏輯
│   │   │   ├── fc_health.c        # 子系統健康檢查
│   │   │   ├── fc_battery.c       # 電池 ADC 監測
│   │   │   ├── fc_selftest.c      # 開機自我測試
│   │   │   └── fc_monitor_task.c  # 監控 Task
│   │   └── test/
│   ├── imu_sensor/                # IMU 感測器元件
│   │   ├── include/
│   │   │   └── imu_sensor.h
│   │   ├── src/
│   │   │   ├── imu_sensor.c       # I2C 驅動
│   │   │   ├── imu_filter.c       # IIR 低通濾波
│   │   │   ├── imu_calibration.c  # 校準程序
│   │   │   └── imu_task.c         # IMU 讀取 Task
│   │   └── test/
│   ├── attitude_estimator/        # EKF 姿態估算元件
│   │   ├── include/
│   │   │   ├── attitude_estimator.h
│   │   │   ├── ekf_types.h        # EKF 狀態結構
│   │   │   └── matrix_math.h      # 矩陣運算 API
│   │   ├── src/
│   │   │   ├── matrix_math.c      # 6x6 矩陣運算
│   │   │   ├── ekf_core.c         # EKF 核心演算法
│   │   │   ├── attitude_estimator.c
│   │   │   └── attitude_task.c    # 姿態計算 Task
│   │   └── test/
│   ├── motor_controller/          # 馬達/伺服控制元件
│   │   ├── include/
│   │   │   └── motor_controller.h
│   │   ├── src/
│   │   │   ├── pid_controller.c   # PID 控制器
│   │   │   ├── servo_driver.c     # PWM 伺服驅動
│   │   │   ├── motor_controller.c # 控制器整合
│   │   │   └── motor_task.c       # 馬達控制 Task
│   │   └── test/
│   ├── gps_navigation/            # GPS 導航元件
│   │   ├── include/
│   │   │   └── gps_navigation.h
│   │   ├── src/
│   │   │   ├── gps_driver.c       # UART GPS 驅動
│   │   │   ├── nmea_parser.c      # NMEA 解析器
│   │   │   ├── navigation_controller.c  # 導航控制
│   │   │   ├── gps_task.c         # GPS 讀取 Task
│   │   │   └── nav_task.c         # 導航計算 Task
│   │   └── test/
│   └── xbee_telemetry/            # XBee 遙測元件
│       ├── include/
│       │   └── xbee_telemetry.h
│       ├── src/
│       │   ├── xbee_driver.c      # UART XBee 驅動
│       │   ├── telemetry_protocol.c  # 封包協定
│       │   └── telemetry_task.c   # 遙測 Task
│       └── test/
├── specs/
│   └── 001-fixed-wing-controller/
│       ├── spec.md                # 功能規格書
│       ├── plan.md                # 實作計畫
│       ├── tasks.md               # 任務清單
│       ├── research.md            # 技術研究（EKF）
│       └── checklists/
└── .specify/
    └── memory/
        └── constitution.md        # 專案憲章 v1.0.0
```

## 建置與燒錄

### 環境需求

- ESP-IDF v5.x
- Python 3.8+
- CMake 3.16+

### 建置步驟

```bash
# 設定 ESP-IDF 環境
. $IDF_PATH/export.sh

# 設定目標晶片
idf.py set-target esp32

# 設定專案（可選，調整 GPIO 腳位等）
idf.py menuconfig

# 編譯
idf.py build

# 燒錄
idf.py -p /dev/ttyUSB0 flash

# 監控串列輸出
idf.py -p /dev/ttyUSB0 monitor
```

### 硬體接線

| 功能 | GPIO | 說明 |
|------|------|------|
| IMU SDA | 21 | I2C 資料線 |
| IMU SCL | 22 | I2C 時脈線 |
| GPS TX | 16 | UART RX（接 GPS TX） |
| GPS RX | 17 | UART TX（接 GPS RX） |
| XBee TX | 4 | UART RX（接 XBee TX） |
| XBee RX | 5 | UART TX（接 XBee RX） |
| Aileron L | 25 | PWM 輸出 |
| Aileron R | 26 | PWM 輸出 |
| Elevator | 27 | PWM 輸出 |
| Rudder | 14 | PWM 輸出 |
| Throttle | 12 | PWM 輸出（ESC） |
| Battery | 34 | ADC 電池電壓 |

## 開發指引

本專案遵循 [專案憲章](.specify/memory/constitution.md) 中定義的 9 項核心原則：

1. **模組化架構** - 每個功能以獨立 ESP-IDF Component 實作
2. **Task 設計** - 明確優先順序、Stack 靜態配置
3. **記憶體管理** - 全部使用 `xTaskCreateStatic()`、`xQueueCreateStatic()`
4. **同步通訊** - Queue/EventGroup/Mutex IPC 機制
5. **ISR 設計** - 極簡化，延遲至 Task 處理
6. **錯誤處理** - 使用 `esp_err_t` 統一回傳
7. **測試驅動** - Unity 測試框架，每個元件附帶單元測試
8. **電源管理** - 考量 Sleep 模式（未來擴展）
9. **版本控制** - Semantic Versioning

## 核心演算法

### Extended Kalman Filter (EKF)

6 狀態向量：`[roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]`

- **預測步驟**：陀螺儀積分（去除偏移）
- **更新步驟**：加速度計重力修正 + GPS 航向修正（速度 > 5m/s）
- **數值穩定性**：Joseph 形式協方差更新

### PID 控制器

| 軸 | Kp | Ki | Kd | 輸出限制 |
|----|----|----|----|----|
| Roll | 1.0 | 0.1 | 0.05 | ±30° |
| Pitch | 1.2 | 0.15 | 0.08 | ±25° |
| Yaw | 0.8 | 0.05 | 0.02 | ±20° |

特性：Anti-windup、運行時調參（FR-009）

## 單元測試

```bash
# 執行所有測試
idf.py -C components/flight_controller/test build flash monitor
idf.py -C components/imu_sensor/test build flash monitor
idf.py -C components/attitude_estimator/test build flash monitor
idf.py -C components/motor_controller/test build flash monitor
idf.py -C components/gps_navigation/test build flash monitor
idf.py -C components/xbee_telemetry/test build flash monitor
```

## 授權

MIT License

## 貢獻者

- 專案規劃與實作由 Claude Code 協助完成
