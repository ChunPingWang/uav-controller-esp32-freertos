<!--
============================================================================
SYNC IMPACT REPORT
============================================================================
Version Change: 1.0.0 (initial) → 1.0.0

Modified Principles: N/A (initial population from user-provided constitution)

Added Sections:
- 9 Core Principles (Articles I-IX)
- AI Collaboration Guidelines (Appendix)
- Governance section with amendment procedures

Removed Sections: N/A (template placeholders replaced)

Templates Requiring Updates:
- .specify/templates/plan-template.md: ✅ Compatible (Constitution Check section exists)
- .specify/templates/spec-template.md: ✅ Compatible (TDD principle aligns with test-first emphasis)
- .specify/templates/tasks-template.md: ✅ Compatible (task phases align with modular approach)

Follow-up TODOs: None
============================================================================
-->

# FreeRTOS on ESP32 專案憲章

## Core Principles

### I. 模組化架構原則 (Modular Architecture)

每個功能 MUST 以獨立元件（Component）形式實作，禁止在主程式中直接實現業務邏輯。

**強制要求：**
- 每個元件 MUST 有獨立的 `.h` 標頭檔定義公開介面
- 元件內部實作細節 MUST 透過 `.c` 檔案隱藏
- 元件之間 MUST 只透過公開 API 或 FreeRTOS IPC 機制通訊
- 禁止跨元件直接存取全域變數

**目錄結構：**
```
components/
├── <module_name>/
│   ├── include/
│   │   └── <module_name>.h
│   ├── src/
│   │   └── <module_name>.c
│   └── CMakeLists.txt
```

### II. FreeRTOS Task 設計原則 (Task Design)

所有任務（Task）MUST 遵循明確的優先順序策略與資源分配規範。

**強制要求：**
- 每個 Task MUST 明確定義其優先順序（Priority）與理由
- Task Stack 大小 MUST 經過分析，禁止使用任意值
- MUST 使用 `uxTaskGetStackHighWaterMark()` 驗證 Stack 使用量
- 硬即時任務優先順序 MUST >= `configMAX_PRIORITIES - 3`
- 背景任務優先順序 MUST <= `tskIDLE_PRIORITY + 2`

**Task 命名規範：**
```c
// 格式：vTask_<模組>_<功能>
void vTask_Sensor_Read(void *pvParameters);
void vTask_WiFi_Monitor(void *pvParameters);
void vTask_Display_Update(void *pvParameters);
```

**禁止事項：**
- 禁止在 Task 中使用 `vTaskDelay(0)` 進行忙碌等待
- 禁止在 ISR 中建立或刪除 Task
- 禁止使用 `vTaskSuspend()` 作為同步機制

### III. 記憶體管理原則 (Memory Management)

ESP32 記憶體資源受限，MUST 採用靜態分配優先策略。

**強制要求：**
- MUST 優先使用 `xTaskCreateStatic()` 而非 `xTaskCreate()`
- MUST 優先使用 `xQueueCreateStatic()` 而非 `xQueueCreate()`
- 禁止在 Task 運行期間動態分配大區塊記憶體
- 所有動態分配 MUST 有對應的釋放機制
- MUST 使用 `heap_caps_get_free_size()` 監控 Heap 使用

**記憶體區域選擇：**
```c
// DMA 緩衝區 MUST 在 DMA capable memory
uint8_t *dma_buffer = heap_caps_malloc(size, MALLOC_CAP_DMA);

// 大型緩衝區 SHOULD 優先放在 SPIRAM（若有）
uint8_t *large_buffer = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
```

### IV. 同步與通訊機制 (Synchronization & IPC)

Task 間通訊 MUST 使用 FreeRTOS 提供的 IPC 原語，禁止自行實作同步機制。

**允許的 IPC 機制：**
| 用途 | 推薦機制 |
|------|----------|
| 資料傳遞 | Queue、Stream Buffer |
| 事件通知 | Event Group、Task Notification |
| 資源保護 | Mutex、Semaphore |
| 生產者-消費者 | Queue |

**強制要求：**
- Mutex MUST 使用 `xSemaphoreCreateMutex()` 而非二元信號量
- 持有 Mutex 時禁止呼叫可能阻塞的 API
- Queue 操作 MUST 設定合理的 Timeout，禁止永久等待
- ISR 中 MUST 只使用 `FromISR` 後綴的 API

### V. 中斷服務程式原則 (ISR Design)

ISR MUST 極度精簡，將工作延遲到 Task 處理。

**強制要求：**
- ISR 執行時間 MUST < 100 微秒
- ISR 中禁止使用任何可能阻塞的操作
- MUST 使用 `xHigherPriorityTaskWoken` 正確處理上下文切換
- 所有 ISR MUST 標記 `IRAM_ATTR`

**ISR 模板：**
```c
void IRAM_ATTR gpio_isr_handler(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 最小化處理：僅發送通知
    xTaskNotifyFromISR(task_handle, GPIO_EVENT,
                       eSetBits, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

### VI. 錯誤處理與診斷 (Error Handling & Diagnostics)

所有元件 MUST 提供可觀察的錯誤狀態與診斷資訊。

**強制要求：**
- MUST 使用 ESP-IDF 的 `esp_err_t` 作為統一錯誤回傳型別
- 錯誤 MUST 向上傳播，禁止靜默忽略
- MUST 使用 `ESP_ERROR_CHECK()` 處理不可恢復錯誤
- MUST 實作串列埠日誌輸出，使用 `ESP_LOGI/W/E` 巨集

**錯誤處理模板：**
```c
esp_err_t component_init(void) {
    esp_err_t ret;

    ret = sub_component_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sub-component init failed: %s", esp_err_to_name(ret));
        return ret;  // 向上傳播錯誤
    }

    return ESP_OK;
}
```

### VII. 測試驅動開發 (Test-Driven Development)

所有元件 MUST 先撰寫測試規格，再進行實作。

**強制要求：**
- MUST 使用 Unity 測試框架（ESP-IDF 內建）
- 每個公開 API MUST 有對應的單元測試
- MUST Mock 外部依賴以實現隔離測試
- 硬體相關測試 MUST 透過 pytest + ESP-IDF Test Framework

**測試目錄結構：**
```
components/
├── <module_name>/
│   ├── test/
│   │   ├── test_<module_name>.c
│   │   └── CMakeLists.txt
```

### VIII. 電源管理原則 (Power Management)

ESP32 應用 MUST 考量電源效率，特別是電池供電場景。

**強制要求：**
- MUST 明確定義 Light Sleep / Deep Sleep 進入條件
- MUST 使用 `esp_pm_configure()` 配置動態頻率調整
- RTC 記憶體 SHOULD 保存跨睡眠週期的狀態
- Wi-Fi MUST 使用 Modem Sleep 或 Light Sleep 模式

### IX. 版本控制與建置 (Version Control & Build)

專案 MUST 使用 ESP-IDF 建置系統並遵循版本控制規範。

**強制要求：**
- MUST 使用 `sdkconfig.defaults` 管理預設配置
- 敏感資訊（Wi-Fi 密碼、API Key）MUST 透過 `Kconfig` 或環境變數注入
- 每個 Release MUST 標記 Git Tag
- MUST 遵循 Semantic Versioning（主版本.次版本.修訂號）

**建置命令：**
```bash
# 配置
idf.py set-target esp32
idf.py menuconfig

# 建置與燒錄
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## AI 協作指引 (AI Collaboration Guidelines)

使用 AI 助手開發時，MUST 遵循以下原則：

1. **逐步驗證**：每次生成的程式碼 MUST 先編譯通過，再進行下一步
2. **規格優先**：MUST 先完成 `/speckit.specify` 產生完整規格，再進入實作
3. **測試先行**：MUST 要求 AI 先產生測試案例，再產生實作程式碼
4. **增量開發**：每次 SHOULD 只開發一個元件，確保可獨立測試
5. **程式碼審查**：AI 產生的程式碼 MUST 人工審查後才能合併

## Governance

本憲章為專案最高指導原則，所有開發決策 MUST 優先考量：即時性、記憶體效率、電源管理與系統穩定性。

**修訂程序：**
- 憲章修訂 MUST 提出變更文件說明理由
- MAJOR 版本變更（原則移除或重新定義）MUST 經專案負責人批准
- MINOR 版本變更（新增原則或擴展指引）SHOULD 記錄於修訂紀錄
- PATCH 版本變更（措辭修正、錯字修復）可直接進行

**合規審查：**
- 所有 PR/Review MUST 驗證是否符合本憲章
- 複雜度增加 MUST 有正當理由並記錄

**Version**: 1.0.0 | **Ratified**: 2025-12-15 | **Last Amended**: 2025-12-16
