# Tasks: 固定翼飛機控制器

**Input**: Design documents from `/specs/001-fixed-wing-controller/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: 依據憲章第七條（測試驅動開發），每個元件 MUST 包含單元測試。

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **ESP-IDF Project**: `components/` for modules, `main/` for application entry
- **Tests**: Each component has `test/` subdirectory

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: ESP-IDF 專案初始化與基礎結構建立

- [ ] T001 Create ESP-IDF project structure with CMakeLists.txt in project root
- [ ] T002 Configure sdkconfig.defaults with FreeRTOS settings (tick rate, stack sizes)
- [ ] T003 [P] Create main/CMakeLists.txt with component dependencies
- [ ] T004 [P] Create main/app_config.h with system-wide configuration constants
- [ ] T005 [P] Create main/Kconfig.projbuild for menuconfig options

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: 所有 User Story 共用的核心基礎設施

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Create components/flight_controller/ directory structure per constitution
- [ ] T007 [P] Define common data types in components/flight_controller/include/fc_types.h (SensorData, AttitudeState, GpsPosition, etc.)
- [ ] T008 [P] Define error codes in components/flight_controller/include/fc_errors.h
- [ ] T009 [P] Implement logging wrapper in components/flight_controller/src/fc_log.c
- [ ] T010 Create FreeRTOS task handles and IPC structures in components/flight_controller/include/fc_tasks.h
- [ ] T011 Implement static memory allocation helpers in components/flight_controller/src/fc_memory.c
- [ ] T012 Create system event group for cross-task notifications in components/flight_controller/src/fc_events.c
- [ ] T013 Implement NVS storage wrapper for calibration data in components/flight_controller/src/fc_nvs.c
- [ ] T014 Write unit tests for fc_types in components/flight_controller/test/test_fc_types.c

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - 姿態穩定控制 (Priority: P1) 🎯 MVP

**Goal**: 即時讀取六軸感測器數據並使用 EKF 計算飛機姿態，輸出修正指令維持穩定

**Independent Test**: 在地面測試台傾斜控制器，驗證姿態數據更新與補償輸出正確對應

**Algorithm**: Extended Kalman Filter (EKF) - 詳見 [research.md](./research.md)

### Tests for User Story 1

- [ ] T015 [P] [US1] Write unit test for IMU data structures in components/imu_sensor/test/test_imu_sensor.c
- [ ] T016 [P] [US1] Write unit test for attitude estimator in components/attitude_estimator/test/test_attitude_estimator.c
- [ ] T016a [P] [US1] Write unit test for matrix operations in components/attitude_estimator/test/test_matrix_math.c
- [ ] T016b [P] [US1] Write unit test for EKF core algorithm in components/attitude_estimator/test/test_ekf_core.c

### Implementation for User Story 1

#### IMU Sensor Component

- [ ] T017 [P] [US1] Create components/imu_sensor/ directory structure with CMakeLists.txt
- [ ] T018 [P] [US1] Define IMU public API in components/imu_sensor/include/imu_sensor.h
- [ ] T019 [US1] Implement I2C/SPI driver initialization in components/imu_sensor/src/imu_sensor.c
- [ ] T020 [US1] Implement sensor data read function (100Hz) in components/imu_sensor/src/imu_sensor.c
- [ ] T021 [US1] Implement digital low-pass filter in components/imu_sensor/src/imu_filter.c
- [ ] T022 [US1] Implement sensor calibration (offset, scale) in components/imu_sensor/src/imu_calibration.c
- [ ] T023 [US1] Implement data quality detection in components/imu_sensor/src/imu_sensor.c

#### Attitude Estimator Component (EKF)

- [ ] T024 [P] [US1] Create components/attitude_estimator/ directory structure with CMakeLists.txt
- [ ] T025 [P] [US1] Define attitude estimator API in components/attitude_estimator/include/attitude_estimator.h
- [ ] T025a [P] [US1] Define EKF types and constants in components/attitude_estimator/include/ekf_types.h
- [ ] T025b [P] [US1] Define matrix math API in components/attitude_estimator/include/matrix_math.h
- [ ] T026a [US1] Implement 6x6 matrix operations (multiply, transpose, add) in components/attitude_estimator/src/matrix_math.c
- [ ] T026b [US1] Implement 6x6 matrix inversion (Gauss-Jordan) in components/attitude_estimator/src/matrix_math.c
- [ ] T026c [US1] Implement EKF state prediction (陀螺儀積分) in components/attitude_estimator/src/ekf_core.c
- [ ] T026d [US1] Implement EKF covariance prediction (P = FPF' + Q) in components/attitude_estimator/src/ekf_core.c
- [ ] T026e [US1] Implement EKF measurement update (加速度計) in components/attitude_estimator/src/ekf_core.c
- [ ] T026f [US1] Implement EKF GPS heading update (速度 > 5m/s 時) in components/attitude_estimator/src/ekf_core.c
- [ ] T026g [US1] Implement Joseph form covariance update for numerical stability in components/attitude_estimator/src/ekf_core.c
- [ ] T026 [US1] Implement attitude estimator wrapper integrating EKF in components/attitude_estimator/src/attitude_estimator.c

#### Task Integration

- [ ] T027 [US1] Implement vTask_IMU_Read (priority: configMAX_PRIORITIES-1) in components/imu_sensor/src/imu_task.c
- [ ] T028 [US1] Implement vTask_Attitude_Compute (priority: configMAX_PRIORITIES-2, Stack: 8192) in components/attitude_estimator/src/attitude_task.c
- [ ] T029 [US1] Create Queue for IMU→Attitude data passing in main/main.c
- [ ] T029a [US1] Create Queue for GPS→Attitude heading update in main/main.c
- [ ] T030 [US1] Verify 100Hz loop timing with uxTaskGetStackHighWaterMark() in main/main.c
- [ ] T030a [US1] Profile EKF computation time (<500μs target) in components/attitude_estimator/src/attitude_task.c

**Checkpoint**: EKF 姿態估算功能完成，可獨立測試驗證 100Hz 迴路與 <2° 精度

---

## Phase 4: User Story 2 - 動力輸出控制 (Priority: P1)

**Goal**: 透過 PWM 控制 ESC，回應推力指令，支援失聯自動返航

**Independent Test**: 輸入不同推力指令，驗證 PWM 輸出線性對應

### Tests for User Story 2

- [ ] T031 [P] [US2] Write unit test for motor controller in components/motor_controller/test/test_motor_controller.c

### Implementation for User Story 2

- [ ] T032 [P] [US2] Create components/motor_controller/ directory structure with CMakeLists.txt
- [ ] T033 [P] [US2] Define motor controller API in components/motor_controller/include/motor_controller.h
- [ ] T034 [US2] Implement PWM initialization for ESC in components/motor_controller/src/motor_controller.c
- [ ] T035 [US2] Implement throttle output (0-100% to PWM) in components/motor_controller/src/motor_controller.c
- [ ] T036 [US2] Implement rate limiter for motor protection in components/motor_controller/src/motor_controller.c
- [ ] T037 [US2] Implement emergency stop function in components/motor_controller/src/motor_controller.c
- [ ] T038 [US2] Implement vTask_Motor_Control (priority: configMAX_PRIORITIES-2) in components/motor_controller/src/motor_task.c
- [ ] T039 [US2] Integrate attitude state for stabilization output in components/motor_controller/src/motor_task.c
- [ ] T040 [US2] Implement failsafe RTL trigger (3-second timeout) in components/flight_controller/src/fc_failsafe.c

**Checkpoint**: 動力控制完成，可驗證 PWM 輸出與 20ms 回應時間

---

## Phase 5: User Story 3 - GPS 導航定位 (Priority: P2)

**Goal**: 接收 GPS 訊號，提供位置/速度/航向，支援家點記錄與返航

**Independent Test**: 戶外測試 GPS 模組，驗證定位精度 <5m 與 5Hz 更新

### Tests for User Story 3

- [ ] T041 [P] [US3] Write unit test for GPS parser in components/gps_navigator/test/test_gps_navigator.c

### Implementation for User Story 3

- [ ] T042 [P] [US3] Create components/gps_navigator/ directory structure with CMakeLists.txt
- [ ] T043 [P] [US3] Define GPS navigator API in components/gps_navigator/include/gps_navigator.h
- [ ] T044 [US3] Implement UART initialization for GPS module in components/gps_navigator/src/gps_navigator.c
- [ ] T045 [US3] Implement NMEA sentence parser in components/gps_navigator/src/gps_nmea.c
- [ ] T046 [US3] Implement GPS position struct update in components/gps_navigator/src/gps_navigator.c
- [ ] T047 [US3] Implement GPS fix quality detection (invalid/2D/3D) in components/gps_navigator/src/gps_navigator.c
- [ ] T048 [US3] Implement home point recording on first GPS lock (FR-027) in components/gps_navigator/src/gps_home.c
- [ ] T049 [US3] Implement last known position retention in components/gps_navigator/src/gps_navigator.c
- [ ] T050 [US3] Implement vTask_GPS_Parse (priority: tskIDLE_PRIORITY+3) in components/gps_navigator/src/gps_task.c
- [ ] T051 [US3] Create Queue for GPS→FlightController data passing in main/main.c

**Checkpoint**: GPS 導航完成，可驗證定位精度與家點記錄功能

---

## Phase 6: User Story 4 - XBee 遙測通訊 (Priority: P2)

**Goal**: 透過 XBee 發送遙測資料、接收控制指令，偵測通訊品質

**Independent Test**: 地面測試通訊延遲 <100ms 與封包完整性

### Tests for User Story 4

- [ ] T052 [P] [US4] Write unit test for telemetry packet in components/xbee_telemetry/test/test_xbee_telemetry.c

### Implementation for User Story 4

- [ ] T053 [P] [US4] Create components/xbee_telemetry/ directory structure with CMakeLists.txt
- [ ] T054 [P] [US4] Define telemetry API in components/xbee_telemetry/include/xbee_telemetry.h
- [ ] T055 [US4] Implement UART initialization for XBee in components/xbee_telemetry/src/xbee_telemetry.c
- [ ] T056 [US4] Define telemetry packet structure with sequence number and checksum in components/xbee_telemetry/include/xbee_packet.h
- [ ] T057 [US4] Implement packet encoding/serialization in components/xbee_telemetry/src/xbee_packet.c
- [ ] T058 [US4] Implement packet decoding/deserialization in components/xbee_telemetry/src/xbee_packet.c
- [ ] T059 [US4] Implement vTask_Telemetry_TX (priority: tskIDLE_PRIORITY+2, 50ms cycle) in components/xbee_telemetry/src/xbee_tx_task.c
- [ ] T060 [US4] Implement vTask_Telemetry_RX (priority: tskIDLE_PRIORITY+2, event-driven) in components/xbee_telemetry/src/xbee_rx_task.c
- [ ] T061 [US4] Implement command parser for control commands in components/xbee_telemetry/src/xbee_command.c
- [ ] T062 [US4] Implement communication quality monitoring (packet loss rate) in components/xbee_telemetry/src/xbee_telemetry.c
- [ ] T063 [US4] Implement 3-second timeout detection for failsafe trigger in components/xbee_telemetry/src/xbee_telemetry.c
- [ ] T064 [US4] Create Mutex for telemetry data aggregation in main/main.c

**Checkpoint**: 遙測通訊完成，可驗證雙向資料傳輸與失聯偵測

---

## Phase 7: User Story 5 - 系統狀態監控 (Priority: P3)

**Goal**: 監控各子系統健康狀態，異常時發出警告

**Independent Test**: 模擬各種異常，驗證監控與警告機制正確觸發

### Tests for User Story 5

- [ ] T065 [P] [US5] Write unit test for system monitor in components/flight_controller/test/test_fc_monitor.c

### Implementation for User Story 5

- [ ] T066 [US5] Define system status structure in components/flight_controller/include/fc_status.h
- [ ] T067 [US5] Implement subsystem health check functions in components/flight_controller/src/fc_health.c
- [ ] T068 [US5] Implement battery voltage monitoring (ADC) in components/flight_controller/src/fc_battery.c
- [ ] T069 [US5] Implement low battery forced RTL trigger (FR-026) in components/flight_controller/src/fc_failsafe.c
- [ ] T070 [US5] Implement GPS-invalid Loiter mode (FR-025) in components/flight_controller/src/fc_failsafe.c
- [ ] T071 [US5] Implement vTask_System_Monitor (priority: tskIDLE_PRIORITY+1, 1000ms cycle) in components/flight_controller/src/fc_monitor_task.c
- [ ] T072 [US5] Implement event logging to serial console in components/flight_controller/src/fc_log.c
- [ ] T073 [US5] Implement startup self-test sequence (5-second limit, FR-023) in components/flight_controller/src/fc_selftest.c

**Checkpoint**: 系統監控完成，可驗證異常偵測與警告機制

---

## Phase 8: Integration & Polish

**Purpose**: 系統整合、跨模組測試與最終驗證

- [ ] T074 Implement main() with all task creation in main/main.c
- [ ] T075 Configure all Queue/EventGroup/Mutex IPC handles in main/main.c
- [ ] T076 Implement system initialization sequence in main/main.c
- [ ] T077 [P] Create hardware integration test for IMU in test_apps/test_imu/main/test_imu_main.c
- [ ] T078 [P] Create hardware integration test for GPS in test_apps/test_gps/main/test_gps_main.c
- [ ] T079 [P] Create hardware integration test for XBee in test_apps/test_xbee/main/test_xbee_main.c
- [ ] T080 Perform end-to-end latency measurement (<15ms requirement)
- [ ] T081 Perform 8-hour stability test (SC-006)
- [ ] T082 Validate all success criteria (SC-001 through SC-008)
- [ ] T083 Update README.md with build and flash instructions

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - US1 & US2 (P1) can proceed in parallel after Foundational
  - US3 & US4 (P2) can proceed after Foundational, independent of US1/US2
  - US5 (P3) depends on some US3/US4 components for monitoring
- **Integration (Phase 8)**: Depends on all user stories being complete

### User Story Dependencies

| Story | Depends On | Can Parallel With |
|-------|------------|-------------------|
| US1 (姿態控制) | Foundational | US2, US3, US4 |
| US2 (動力控制) | Foundational, partially US1 | US3, US4 |
| US3 (GPS 導航) | Foundational | US1, US2, US4 |
| US4 (XBee 遙測) | Foundational | US1, US2, US3 |
| US5 (系統監控) | Foundational, US3 (battery), US4 (telemetry) | - |

### Within Each User Story

- Tests MUST be written first (TDD per constitution)
- Directory structure before implementation
- API header before implementation
- Core functions before task implementation
- Task implementation before IPC integration

---

## Parallel Execution Examples

### Phase 2 Parallel Tasks

```bash
# These can run simultaneously:
Task: "Define common data types in components/flight_controller/include/fc_types.h"
Task: "Define error codes in components/flight_controller/include/fc_errors.h"
Task: "Implement logging wrapper in components/flight_controller/src/fc_log.c"
```

### User Story 1 Parallel Tasks

```bash
# Tests can run in parallel:
Task: "Write unit test for IMU in components/imu_sensor/test/test_imu_sensor.c"
Task: "Write unit test for attitude estimator in components/attitude_estimator/test/test_attitude_estimator.c"

# Directory creation can run in parallel:
Task: "Create components/imu_sensor/ directory structure"
Task: "Create components/attitude_estimator/ directory structure"
```

---

## Implementation Strategy

### MVP First (User Story 1 + 2 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T014)
3. Complete Phase 3: User Story 1 - 姿態控制 (T015-T030)
4. Complete Phase 4: User Story 2 - 動力控制 (T031-T040)
5. **STOP and VALIDATE**: Test attitude + motor control on bench
6. Deploy to test aircraft if ready

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add US1 (姿態控制) → Test independently → Bench test
3. Add US2 (動力控制) → Test independently → Ground test
4. Add US3 (GPS 導航) → Test independently → Outdoor GPS test
5. Add US4 (XBee 遙測) → Test independently → Ground station test
6. Add US5 (系統監控) → Full system integration test
7. Phase 8: Final validation and flight test

---

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 95 |
| Phase 1 (Setup) | 5 tasks |
| Phase 2 (Foundational) | 9 tasks |
| Phase 3 (US1 - 姿態控制 + EKF) | 28 tasks |
| Phase 4 (US2 - 動力控制) | 10 tasks |
| Phase 5 (US3 - GPS 導航) | 11 tasks |
| Phase 6 (US4 - XBee 遙測) | 13 tasks |
| Phase 7 (US5 - 系統監控) | 9 tasks |
| Phase 8 (Integration) | 10 tasks |
| Parallel Opportunities | 28 tasks marked [P] |
| MVP Scope | Phase 1-4 (52 tasks) |
| EKF Related | 12 new tasks (T016a-b, T025a-b, T026a-g, T029a, T030a) |

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- TDD: Write tests first, ensure they FAIL before implementation
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- All tasks follow Constitution principles (modular, static allocation, esp_err_t, etc.)
