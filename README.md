# FreeRTOS on ESP32 - 固定翼飛機控制器

基於 ESP32 與 FreeRTOS 的固定翼無人機飛行控制器專案。

## 專案狀態

| 階段 | 狀態 | 說明 |
|------|------|------|
| 憲章 | ✅ 完成 | v1.0.0 - 9 項核心原則 |
| 規格 | ✅ 完成 | 5 個 User Stories、27 項功能需求 |
| 釐清 | ✅ 完成 | 5 項關鍵決策已確認 |
| 規劃 | ⏳ 待執行 | `/speckit.plan` |
| 任務 | ⏳ 待執行 | `/speckit.tasks` |
| 實作 | ⏳ 待執行 | `/speckit.implement` |

## 功能模組

| 模組 | 說明 | 優先級 |
|------|------|--------|
| 六軸感測器 | IMU 資料讀取與濾波（100Hz） | P1 |
| 姿態估算 | 俯仰/滾轉/偏航角度計算 | P1 |
| 動力控制 | PWM 輸出控制 ESC | P1 |
| GPS 導航 | 位置/速度/航向解析（5Hz） | P2 |
| XBee 遙測 | 雙向通訊與失聯偵測 | P2 |
| 系統監控 | 健康狀態與異常警告 | P3 |

## 關鍵決策

| 項目 | 決策 |
|------|------|
| 失聯處理 | 自動返航（RTL）飛回家點 |
| 失聯閾值 | 通訊中斷 3 秒觸發 |
| GPS 遺失 | 切換定點盤旋（Loiter）等待恢復 |
| 低電量 | 立即觸發強制返航 |
| 家點定義 | GPS 首次鎖定時自動記錄起飛點 |

## 成功標準

- 姿態控制迴路：100Hz（抖動 < 1ms）
- 端對端延遲：< 15ms
- 姿態精度：< 2° 誤差
- GPS 精度：< 5m CEP
- 遙測延遲：< 100ms
- 連續運作：8 小時無異常
- 失聯反應：3 秒內觸發
- 啟動自檢：5 秒內完成

## 專案結構

```
freertos4esp32/
├── .specify/                    # Speckit 設定與模板
│   ├── memory/
│   │   └── constitution.md      # 專案憲章 v1.0.0
│   └── templates/               # 規格模板
├── specs/
│   └── 001-fixed-wing-controller/
│       ├── spec.md              # 功能規格書
│       └── checklists/
│           └── requirements.md  # 品質檢查清單
├── components/                  # ESP-IDF 元件（待建立）
└── README.md                    # 本文件
```

## 開發指引

本專案遵循 [專案憲章](.specify/memory/constitution.md) 中定義的 9 項核心原則：

1. **模組化架構** - 每個功能以獨立 Component 實作
2. **Task 設計** - 明確優先順序與 Stack 分析
3. **記憶體管理** - 靜態分配優先
4. **同步通訊** - 使用 FreeRTOS IPC 原語
5. **ISR 設計** - 極簡化，延遲至 Task 處理
6. **錯誤處理** - 使用 `esp_err_t` 統一回傳
7. **測試驅動** - 先寫測試再實作
8. **電源管理** - 考量 Sleep 模式
9. **版本控制** - Semantic Versioning

## 下一步

```bash
# 執行實作規劃
/speckit.plan
```

## 授權

MIT License
