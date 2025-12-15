# FreeRTOS on ESP32 專案憲章

> 版本: 1.0.0
> 批准日期: 2025-12-15
> 最後修訂: 2025-12-15

## 專案願景

本專案使用 FreeRTOS 作為即時作業系統，在 ESP32 平台上開發嵌入式應用。
所有開發決策必須優先考量：即時性、記憶體效率、電源管理與系統穩定性。

---

## 第一條：模組化架構原則

每個功能必須以獨立元件（Component）形式實作，禁止在主程式中直接實現業務邏輯。

**強制要求：**
- 每個元件必須有獨立的 `.h` 標頭檔定義公開介面
- 元件內部實作細節透過 `.c` 檔案隱藏
- 元件之間只能透過公開 API 或 FreeRTOS IPC 機制通訊
- 禁止跨元件直接存取全域變數

**目錄結構：**
```
components/
├── wifi_manager/
│   ├── include/
│   │   └── wifi_manager.h
│   ├── src/
│   │   └── wifi_manager.c
│   └── CMakeLists.txt
├── sensor_driver/
│   ├── include/
│   │   └── sensor_driver.h
│   ├── src/
│   │   └── sensor_driver.c
│   └── CMakeLists.txt
```

---

## 第二條：FreeRTOS Task 設計原則

所有任務（Task）必須遵循明確的優先順序策略與資源分配規範。

**強制要求：**
- 每個 Task 必須明確定義其優先順序（Priority）與理由
- Task Stack 大小必須經過分析，禁止使用任意值
- 使用 `uxTaskGetStackHighWaterMark()` 驗證 Stack 使用量
- 硬即時任務優先順序 >= `configMAX_PRIORITIES - 3`
- 背景任務優先順序 <= `tskIDLE_PRIORITY + 2`

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

---

## 第三條：記憶體管理原則

ESP32 記憶體資源受限，必須採用靜態分配優先策略。

**強制要求：**
- 優先使用 `xTaskCreateStatic()` 而非 `xTaskCreate()`
- 優先使用 `xQueueCreateStatic()` 而非 `xQueueCreate()`
- 禁止在 Task 運行期間動態分配大區塊記憶體
- 所有動態分配必須有對應的釋放機制
- 使用 `heap_caps_get_free_size()` 監控 Heap 使用

**記憶體區域選擇：**
```c
// DMA 緩衝區必須在 DMA capable memory
uint8_t *dma_buffer = heap_caps_malloc(size, MALLOC_CAP_DMA);

// 大型緩衝區優先放在 SPIRAM（若有）
uint8_t *large_buffer = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
```

---

## 第四條：同步與通訊機制

Task 間通訊必須使用 FreeRTOS 提供的 IPC 原語，禁止自行實作同步機制。

**允許的 IPC 機制：**
| 用途 | 推薦機制 |
|------|----------|
| 資料傳遞 | Queue、Stream Buffer |
| 事件通知 | Event Group、Task Notification |
| 資源保護 | Mutex、Semaphore |
| 生產者-消費者 | Queue |

**強制要求：**
- Mutex 必須使用 `xSemaphoreCreateMutex()` 而非二元信號量
- 持有 Mutex 時禁止呼叫可能阻塞的 API
- Queue 操作必須設定合理的 Timeout，禁止永久等待
- ISR 中只能使用 `FromISR` 後綴的 API

---

## 第五條：中斷服務程式（ISR）原則

ISR 必須極度精簡，將工作延遲到 Task 處理。

**強制要求：**
- ISR 執行時間必須 < 100 微秒
- ISR 中禁止使用任何可能阻塞的操作
- 使用 `xHigherPriorityTaskWoken` 正確處理上下文切換
- 所有 ISR 必須標記 `IRAM_ATTR`

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

---

## 第六條：錯誤處理與診斷

所有元件必須提供可觀察的錯誤狀態與診斷資訊。

**強制要求：**
- 使用 ESP-IDF 的 `esp_err_t` 作為統一錯誤回傳型別
- 錯誤必須向上傳播，禁止靜默忽略
- 使用 `ESP_ERROR_CHECK()` 處理不可恢復錯誤
- 實作串列埠日誌輸出，使用 `ESP_LOGI/W/E` 巨集

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

---

## 第七條：測試驅動開發（TDD）

所有元件必須先撰寫測試規格，再進行實作。

**強制要求：**
- 使用 Unity 測試框架（ESP-IDF 內建）
- 每個公開 API 必須有對應的單元測試
- Mock 外部依賴以實現隔離測試
- 硬體相關測試透過 pytest + ESP-IDF Test Framework

**測試目錄結構：**
```
components/
├── sensor_driver/
│   ├── test/
│   │   ├── test_sensor_driver.c
│   │   └── CMakeLists.txt
```

---

## 第八條：電源管理原則

ESP32 應用必須考量電源效率，特別是電池供電場景。

**強制要求：**
- 明確定義 Light Sleep / Deep Sleep 進入條件
- 使用 `esp_pm_configure()` 配置動態頻率調整
- RTC 記憶體保存跨睡眠週期的狀態
- Wi-Fi 使用 Modem Sleep 或 Light Sleep 模式

---

## 第九條：版本控制與建置

專案必須使用 ESP-IDF 建置系統並遵循版本控制規範。

**強制要求：**
- 使用 `sdkconfig.defaults` 管理預設配置
- 敏感資訊（Wi-Fi 密碼、API Key）透過 `Kconfig` 或環境變數注入
- 每個 Release 必須標記 Git Tag
- 遵循 Semantic Versioning（主版本.次版本.修訂號）

**建置命令：**
```bash
# 配置
idf.py set-target esp32
idf.py menuconfig

# 建置與燒錄
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## 附錄：AI 協作指引

使用 Claude 開發時，必須遵循以下原則：

1. **逐步驗證**：每次生成的程式碼必須先編譯通過，再進行下一步
2. **規格優先**：先完成 `/speckit.specify` 產生完整規格，再進入實作
3. **測試先行**：要求 Claude 先產生測試案例，再產生實作程式碼
4. **增量開發**：每次只開發一個元件，確保可獨立測試
5. **程式碼審查**：Claude 產生的程式碼必須人工審查後才能合併

---

## 修訂紀錄

| 版本 | 日期 | 變更說明 |
|------|------|----------|
| 1.0.0 | 2025-12-15 | 初始版本 |
