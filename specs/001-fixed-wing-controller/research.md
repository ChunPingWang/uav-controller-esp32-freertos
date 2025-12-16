# Research: 固定翼飛機控制器

**Date**: 2025-12-16
**Branch**: `001-fixed-wing-controller`

## Research Topics

1. 姿態估算演算法選擇
2. ESP32 上的 Kalman Filter 實作
3. 感測器融合策略

---

## Topic 1: 姿態估算演算法選擇

### 問題

如何從 IMU（加速度計 + 陀螺儀）資料中精確估算飛機姿態（roll, pitch, yaw），並達成 <2° 誤差的精度要求？

### 研究結果

#### 方案比較

| 演算法 | 運算複雜度 | 精度 | 抗雜訊 | 感測器融合 | ESP32 適用性 |
|--------|------------|------|--------|------------|--------------|
| 純陀螺儀積分 | O(1) | 差（漂移） | 差 | 無 | ✅ |
| 純加速度計 | O(1) | 中（震動敏感） | 差 | 無 | ✅ |
| Complementary Filter | O(1) | 中 | 中 | IMU | ✅ |
| Madgwick AHRS | O(n) | 好 | 好 | IMU+Mag | ✅ |
| Mahony AHRS | O(n) | 好 | 好 | IMU+Mag | ✅ |
| **Extended Kalman Filter** | O(n³) | 最佳 | 最佳 | IMU+GPS | ✅ |
| Unscented Kalman Filter | O(n³) | 最佳 | 最佳 | 全部 | ⚠️ 邊緣 |

#### 決策：Extended Kalman Filter (EKF)

**理由**：
1. **精度要求**：規格要求 <2° 姿態誤差，EKF 提供最佳估計
2. **感測器融合**：可整合 IMU、GPS、磁力計等多種感測器
3. **陀螺儀偏差補償**：EKF 可同時估計並補償陀螺儀偏差
4. **理論基礎完善**：有成熟的數學理論支持
5. **ESP32 可行性**：6 維狀態的 EKF 在 240MHz 下可達 100Hz

**拒絕 Complementary Filter 的理由**：
- 調參困難，需針對不同飛行條件調整
- 無法自動補償陀螺儀偏差
- 無法利用 GPS 資訊校正航向

**拒絕 UKF 的理由**：
- 運算量約為 EKF 的 2-3 倍
- ESP32 可能無法在 10ms 週期內完成
- 對於 6 維狀態，EKF 已足夠精確

### 結論

採用 **Extended Kalman Filter (EKF)** 進行姿態估算。

---

## Topic 2: ESP32 上的 Kalman Filter 實作

### 問題

如何在 ESP32（240MHz, 320KB SRAM）上高效實作 6 維 EKF？

### 研究結果

#### 記憶體需求分析

```
狀態向量 x: 6 × 4 bytes = 24 bytes
協方差矩陣 P: 6 × 6 × 4 bytes = 144 bytes
過程雜訊 Q: 6 × 6 × 4 bytes = 144 bytes
量測雜訊 R: 3 × 3 × 4 bytes = 36 bytes (加速度計量測)
Jacobian F: 6 × 6 × 4 bytes = 144 bytes
Jacobian H: 3 × 6 × 4 bytes = 72 bytes
Kalman Gain K: 6 × 3 × 4 bytes = 72 bytes
暫存矩陣: 約 500 bytes

總計: 約 1.2 KB (靜態配置)
```

#### 計算時間估計

| 運算 | 複雜度 | 估計時間 @ 240MHz |
|------|--------|-------------------|
| 狀態預測 | O(n) | 10 μs |
| P 預測 (F*P*F' + Q) | O(n³) | 150 μs |
| Kalman Gain (P*H'*(H*P*H'+R)⁻¹) | O(n³) | 200 μs |
| 狀態更新 | O(n²) | 30 μs |
| P 更新 (Joseph form) | O(n³) | 100 μs |
| **總計** | | **~500 μs** |

#### 實作策略

1. **靜態記憶體配置**
   ```c
   static float ekf_state[6];
   static float ekf_P[6][6];
   static float ekf_Q[6][6];
   // ... 所有矩陣靜態宣告
   ```

2. **矩陣運算優化**
   - 避免動態記憶體配置
   - 展開小矩陣迴圈 (6x6)
   - 利用對稱性減少運算

3. **數值穩定性**
   - 使用 Joseph form 更新 P：`P = (I-KH)P(I-KH)' + KRK'`
   - 強制 P 對稱：`P = (P + P') / 2`
   - 確保 P 正定

4. **Stack 需求**
   - 暫存變數約需 1KB
   - 加上安全邊際，設定 Task Stack 為 8KB

### 決策

自行實作輕量級 6x6 矩陣運算庫，避免外部依賴，靜態配置所有記憶體。

---

## Topic 3: 感測器融合策略

### 問題

如何有效融合 IMU（100Hz）與 GPS（5Hz）資料？

### 研究結果

#### 融合架構

```
IMU (100Hz)  ──┬──► EKF Predict (100Hz)
               │
               └──► EKF Update (Accel, 100Hz)
                         ↓
GPS (5Hz)    ────────► EKF Update (GPS heading, 5Hz)
                         ↓
                    AttitudeState
```

#### 量測模型

**加速度計量測（重力方向）**：
```
z_accel = [ax, ay, az]^T
h(x) = R(roll, pitch) * [0, 0, g]^T

用於估計 roll 和 pitch
```

**GPS 航向量測（選用）**：
```
z_gps = [heading]
h(x) = yaw

僅在速度 > 5 m/s 時使用（低速時 GPS 航向不可靠）
```

#### 更新頻率策略

| 更新類型 | 頻率 | 條件 |
|----------|------|------|
| EKF Predict | 100 Hz | 每次 IMU 讀取 |
| Accel Update | 100 Hz | 加速度有效時 |
| GPS Heading Update | 5 Hz | 速度 > 5 m/s 且 GPS 3D fix |

### 決策

採用異步感測器融合架構，IMU 驅動 100Hz 預測/更新，GPS 在可用時提供航向校正。

---

## 總結

| 研究主題 | 決策 | 理由 |
|----------|------|------|
| 姿態估算演算法 | Extended Kalman Filter | 最佳精度、可融合多感測器 |
| EKF 實作方式 | 自製輕量級矩陣庫 | 避免外部依賴、靜態配置 |
| 感測器融合 | 異步融合（IMU 100Hz, GPS 5Hz） | 配合感測器特性 |

### 後續行動

1. 實作 `matrix_math.c` - 6x6 矩陣運算
2. 實作 `ekf_core.c` - EKF 核心演算法
3. 實作 `attitude_estimator.c` - 整合 EKF 與感測器
4. 撰寫單元測試驗證數值正確性
