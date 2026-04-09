# 慣性導航系統（INS）
## 中遠程應用技術參考手冊

---

## 目錄

1. [慣性導航基礎原理](#1-慣性導航基礎原理)
2. [IMU 感測器技術與精度等級](#2-imu-感測器技術與精度等級)
3. [INS 機械編排](#3-ins-機械編排)
4. [誤差來源與漂移分析](#4-誤差來源與漂移分析)
5. [感測器融合演算法](#5-感測器融合演算法)
6. [輔助訊號源與多感測器整合](#6-輔助訊號源與多感測器整合)
7. [GPS 拒止環境下的導航策略](#7-gps-拒止環境下的導航策略)
8. [性能等級比較](#8-性能等級比較)
9. [中遠程無人機設計考量](#9-中遠程無人機設計考量)
10. [感測器驅動與演算法實作指南](#10-感測器驅動與演算法實作指南)

---

## 1. 慣性導航基礎原理

**慣性導航系統（INS, Inertial Navigation System）** 是一種完全自主的導航技術，僅依靠機載感測器估計載體的位置、速度與姿態（PVA），完全不依賴外部訊號，如 GPS、無線電信標或視覺地標。

### 1.1 核心物理原理

INS 以**牛頓運動定律**結合剛體動力學為基礎：

| 量測量 | 感測器 | 推導量 |
|---|---|---|
| 角速度 **ω** | 陀螺儀 | 姿態（滾轉 / 俯仰 / 偏航） |
| 線性加速度 **a** | 加速度計 | 速度 → 位置 |

導航方程式對感測器量測值進行連續積分：

```
角速度  →  （積分）  →  姿態（四元數 / 方向餘弦矩陣）
加速度  →  （轉換至導航坐標系）  →  （去除重力）  →
       →  （積分）  →  速度  →  （積分）  →  位置
```

### 1.2 漂移的成因

由於 INS 依賴積分運算，**任何感測器誤差都會隨時間累積放大**：

- 加速度計偏置誤差 → 速度誤差隨時間 *t* 線性增長
- 速度誤差 → 位置誤差以 *t²* 增長
- 陀螺儀偏置誤差 → 姿態誤差線性增長，進而導致位置誤差以 *t³* 增長

> **中遠程任務的關鍵意義：** 若無外部輔助修正，即使是導航級 IMU 也會累積約 1–2 km/hr 的位置誤差。消費級 MEMS IMU 則可能在數分鐘內漂移數公里。

### 1.3 坐標系定義

| 坐標系 | 說明 |
|---|---|
| 載體坐標系（b 系） | 固定於載體；IMU 的量測參考系 |
| 導航坐標系（n 系） | 本地水平面，北-東-地（NED）或東-北-天（ENU） |
| 地心慣性坐標系（i 系） | 固定於星空；用於高空 / 太空應用 |
| 地心地固坐標系（e 系，ECEF） | 隨地球自轉；GNSS 所使用的坐標系 |

INS 機械編排的核心運算，是利用**方向餘弦矩陣（DCM）** 或**四元數**將加速度計讀值從載體坐標系轉換至導航坐標系。

---

## 2. IMU 感測器技術與精度等級

### 2.1 陀螺儀技術

#### 2.1.1 MEMS 陀螺儀（微機電系統）

- **原理：** 振動質量塊受科里奧利力（Coriolis Effect）作用產生偏移
- **製造：** 矽微加工（批量生產，成本極低）
- **主要廠商：** Bosch（BMI088、BMI270）、STMicroelectronics（LSM6DSO）、InvenSense（ICM-42688）

| 參數 | 消費級 MEMS | 工業級 MEMS | 戰術級 MEMS |
|---|---|---|---|
| 偏置不穩定性 | > 10 °/hr | 1–10 °/hr | 0.1–1 °/hr |
| 角度隨機遊走（ARW） | > 0.1 °/√hr | 0.05–0.1 °/√hr | 0.01–0.05 °/√hr |
| 典型應用 | 手機、玩具無人機 | 機器人 | 軍用 UAV、精確制導彈藥 |
| 代表型號 | MPU-6050 | BMI088 | ADIS16505 |

> **BMI088 說明：** BMI088 是具備高振動抗擾性的工業級 MEMS IMU，適用於無人機飛控（如 RP2350 系統），處於**工業級**邊界。搭配 GNSS 輔助時，適合短至中程任務；長程任務則需要更積極的融合策略。

#### 2.1.2 光纖陀螺儀（FOG, Fiber Optic Gyroscope）

- **原理：** Sagnac 效應——光纖線圈中兩道反向傳播光束的干涉
- **無移動部件：** 可靠性高，無需預熱
- **典型性能：**
  - 偏置不穩定性：0.0001–0.01 °/hr（導航級）
  - ARW：0.001–0.005 °/√hr
- **廠商：** Honeywell、iXBlue（現為 Exail）、KVH Industries、Fibersense

| FOG 等級 | 偏置不穩定性 | 應用 |
|---|---|---|
| 戰術級 FOG | 0.1–1 °/hr | 無人機、精確制導彈藥 |
| 導航級 FOG | 0.001–0.01 °/hr | 飛機、潛艇、飛彈 |
| 戰略級 FOG | < 0.0001 °/hr | 洲際彈道飛彈、深海潛艇 |

#### 2.1.3 環形雷射陀螺儀（RLG, Ring Laser Gyroscope）

- **原理：** 在剛性三角或方形玻璃空腔中，利用兩道雷射光束的 Sagnac 效應
- **性能：** 與導航級 FOG 相當；偏置不穩定性 < 0.001 °/hr
- **特性：**
  - 溫度穩定性極佳
  - 低轉速時存在**鎖定效應（Lock-in Effect）**，需要抖動機構（Dithering）補償
  - 比 FOG 更重且昂貴
- **廠商：** Honeywell（GG1308、GG1342）、Northrop Grumman（LN-100G）
- **典型應用：** 商用航空（波音 777 三餘度 INS）、彈道飛彈

#### 2.1.4 機械式 / 旋轉質量陀螺儀

- **原理：** 角動量守恆——高速旋轉的轉子抵抗姿態變化
- **類型：**
  - **速率陀螺儀：** 單自由度，彈簧限位
  - **積分陀螺儀：** 兩自由度，自由轉子
  - **動態調諧陀螺儀（DTG）：** 彈性萬向節調諧轉子
- **性能：** 依設計可達導航至戰略級
- **缺點：** 含移動部件（軸承）→ 磨損與可靠性問題；對振動敏感；需數分鐘預熱
- **現狀：** 在新設計中已大多被 FOG/RLG 取代；仍見於舊型飛機與潛艇平台
- **廠商：** Northrop Grumman、Kearfott、Raytheon

#### 2.1.5 半球諧振陀螺儀（HRG, Hemispherical Resonator Gyroscope）

- **原理：** 石英酒杯形諧振子振動時受科里奧利力作用
- **性能：** 戰略級——偏置不穩定性 < 0.00001 °/hr
- **核心優勢：** 工作壽命超長（> 15 年）、零移動部件、真空密封
- **應用：** 深空探測器、戰略飛彈、潛艇
- **廠商：** Northrop Grumman

#### 2.1.6 原子干涉儀陀螺儀（新興技術）

- **原理：** 雷射冷卻原子在科里奧利加速下的量子力學干涉
- **潛在性能：** 比最佳 RLG/FOG 高出 100 倍——超戰略級
- **現狀：** 實驗室 / 原型階段，尚未野外部署
- **研究機構：** Muquans（法國）、AOSense（美國）、史丹佛大學

---

### 2.2 加速度計技術

| 技術 | 原理 | 等級 | 偏置穩定性 | 代表型號 |
|---|---|---|---|---|
| MEMS 電容式 | 質量塊位移 → 電容變化 | 消費至戰術級 | 10–1000 µg | BMI088、ADXL355 |
| MEMS 壓電式 | 質量塊應力 → 壓電訊號 | 工業至戰術級 | 1–100 µg | Kistler 8315 |
| 石英撓性（QA） | 電磁力平衡石英質量塊 | 導航級 | 1–50 µg | Honeywell QA-2000 |
| 振動樑（VBA） | 石英樑諧振頻率偏移 | 導航至戰略級 | < 1 µg | Draper Lab VBA |
| 冷原子 | 雷射冷卻原子自由落體干涉 | 戰略級（實驗室） | 次微克級 | AOSense |

---

### 2.3 精度等級總覽

```
性能（從最佳漂移 → 最差漂移）
─────────────────────────────────────────────────────────────────────────────
戰略級       │  導航級   │    戰術級      │   工業級    │    消費級
─────────────────────────────────────────────────────────────────────────────
HRG         │ FOG/RLG  │ 戰術 MEMS     │  MEMS IMU  │  手機 IMU
原子干涉     │ QA 加速   │ 戰術 FOG      │  BMI088    │  MPU-6050
< 0.001°/hr │< 0.01°/hr│  0.1–1°/hr   │  1–10°/hr  │  > 10°/hr
─────────────────────────────────────────────────────────────────────────────
ICBM        │ 潛艇     │ 軍用 UAV      │  飛控板     │  玩具 / 手機
深空探測     │ 民航機    │ 精確制導彈藥   │  機器人     │
            │          │              │            │
```

---

## 3. INS 機械編排

### 3.1 捷聯式 vs. 平台式

| 架構 | 說明 | 現狀 |
|---|---|---|
| **平台式（Gimballed）** | IMU 裝於穩定平台上，物理隔離載體旋轉 | 傳統設計；仍用於戰略應用 |
| **捷聯式（Strapdown）** | IMU 固定於載體機體，旋轉以數學方式補償 | 現代標準——所有無人機 / 飛彈 / 飛機飛控 |

現代捷聯式 INS 需要：
- **姿態表示：** 方向餘弦矩陣（DCM）或單位四元數
- **高頻取樣：** 400 Hz – 2 kHz，以最小化圓錐 / 雕刻誤差（Coning/Sculling Error）
- **地球模型修正：** WGS-84 橢球體、重力模型、地球自轉（科里奧利效應）

### 3.2 機械編排流程

```
┌─────────────┐     ┌────────────────────┐     ┌──────────────────────┐
│ 原始 IMU    │────▶│ 姿態更新            │────▶│ 坐標系轉換            │
│ ω（陀螺儀） │     │（四元數 / DCM）     │     │（載體系 → 導航系）     │
│ f（加速度計）│     └────────────────────┘     └──────────┬───────────┘
└─────────────┘                                            │
                                                           ▼
                                               ┌──────────────────────┐
                                               │ 重力補償              │
                                               │ g = f(緯度, 高度)    │
                                               └──────────┬───────────┘
                                                          │
                                                          ▼
                                               ┌──────────────────────┐
                                               │ 速度積分              │
                                               │ v = ∫(f_nav - g) dt  │
                                               └──────────┬───────────┘
                                                          │
                                                          ▼
                                               ┌──────────────────────┐
                                               │ 位置積分              │
                                               │ p = ∫v dt            │
                                               └──────────────────────┘
```

### 3.3 初始化與對準

INS 在開始導航前必須完成**對準（Alignment）階段**：

| 階段 | 方法 | 時間 | 目的 |
|---|---|---|---|
| **粗對準** | 解析法（重力 + 地球自轉） | 秒級 | 初始姿態估計 |
| **精對準** | 卡爾曼濾波（靜止狀態） | 分鐘級 | 偏置與姿態精修 |
| **飛行中對準** | 從主 INS 轉移對準 | 即時 | 次級 INS 初始化 |

> **無人機注意事項：** 大多數無人機飛控在解鎖前於地面完成粗對準。假設 IMU 靜止不動：重力向量決定俯仰 / 滾轉，磁力計決定偏航。此過程約需 2–10 秒。

---

## 4. 誤差來源與漂移分析

### 4.1 慣性感測器誤差模型

```
總感測器誤差 = 偏置 + 比例因子誤差 + 交叉軸耦合
             + 雜訊（ARW/VRW） + 非線性 + 溫度效應
```

| 誤差項 | 符號 | 影響 | 緩解方法 |
|---|---|---|---|
| 陀螺偏置 | *b_g* | 姿態漂移 → 位置誤差 ∝ t³ | 溫度校準、卡爾曼估計 |
| 加速度計偏置 | *b_a* | 位置誤差 ∝ t² | 在線校準、融合 |
| 角度隨機遊走 | ARW（°/√hr） | 姿態雜訊 | 低通濾波 / 卡爾曼 |
| 速度隨機遊走 | VRW（m/s/√hr） | 速度雜訊 | 低通濾波 / 卡爾曼 |
| 比例因子 | *s* | 高角速率下的比例誤差 | 出廠校準 |
| 溫度漂移 | *TC_b* | 偏置隨溫度偏移 | 熱穩定化、查找表 |
| 振動整流誤差 | VRE | 非對稱振動產生的直流偏置 | 減振安裝、濾波 |

### 4.2 位置誤差增長（無外部輔助）

| IMU 等級 | 位置漂移率 | 1 分鐘後 | 10 分鐘後 |
|---|---|---|---|
| 消費級 MEMS | km/min | > 1 km | > 10 km |
| 工業級 MEMS（BMI088） | ~10–100 m/min | ~100–500 m | ~1–5 km |
| 戰術 MEMS / FOG | < 1 m/min | < 50 m | < 500 m |
| 導航級 FOG/RLG | < 30 m/hr（~0.5 m/min） | < 1 m | < 10 m |
| 戰略級 HRG | < 1 海里/hr（~0.03 m/min） | < 0.1 m | < 0.5 m |

---

## 5. 感測器融合演算法

### 5.1 互補濾波器（Complementary Filter）

- **概念：** 陀螺儀高通濾波 + 加速度計 / 磁力計低通濾波
- **實作：** 極為輕量；Mahony、Madgwick 演算法即屬此類
- **Madgwick 濾波器：**
  - 使用梯度下降法最小化姿態誤差
  - 6-DOF（加速度計 + 陀螺儀）或 9-DOF（+ 磁力計）
  - 單一調參參數 β（收斂速率）
  - 適合在 Cortex-M0/M33 上以 1 kHz 運行

```
q̂_dot = q̂ × [0, ω] / 2  -  β × ∇f
q̂ = q̂ + q̂_dot × dt
```

### 5.2 卡爾曼濾波器（KF）——現代 INS 的核心

卡爾曼濾波器為**線性高斯系統**提供最優（最小方差）狀態估計：

```
預測階段：  x̂⁻ = F·x̂ + B·u          （IMU 傳播）
            P⁻  = F·P·Fᵀ + Q          （誤差協方差傳播）

更新階段：  K   = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹   （卡爾曼增益）
            x̂  = x̂⁻ + K·(z - H·x̂⁻)        （狀態修正）
            P   = (I - K·H)·P⁻              （協方差更新）
```

**完整 INS-GNSS 系統的狀態向量**通常包含：

```
x = [δp（位置誤差，3 維）
     δv（速度誤差，3 維）
     δψ（姿態誤差，3 維）
     b_g（陀螺偏置，3 維）
     b_a（加速度計偏置，3 維）
     ...]        → 最少 15 個狀態量
```

### 5.3 擴展卡爾曼濾波器（EKF）

- **處理非線性問題**——透過雅可比矩陣在當前估計附近線性化
- **標準應用於：** PX4、ArduPilot（EKF2/EKF3）、大多數導航級 INS 系統
- **限制：** 若初始估計與真值偏差過大，可能發散（強非線性系統）

### 5.4 無跡卡爾曼濾波器（UKF）

- **Sigma 點傳播**——無需顯式計算雅可比矩陣
- **精度更高**——適用於高度非線性系統（大角度誤差、激烈機動）
- **計算代價**——比同維度 EKF 高約 3 倍
- **應用於：** 高性能飛彈制導、精準農業 UAV

### 5.5 粒子濾波器（Particle Filter）

- **蒙地卡羅估計**——以加權粒子群代表概率分佈
- **適用任意非線性 / 非高斯**分佈
- **代價：** O(N) 粒子——計算量大；實時 INS 中少見
- **利基應用：** 地形導航、SLAM（Rao-Blackwellized 粒子濾波器）

### 5.6 因子圖 / 最佳化融合

- **將導航問題表述為滑動窗口內的非線性最小二乘問題**
- **框架：** GTSAM、g2o、Ceres Solver
- **應用於：** 高精度視覺慣性 SLAM（VINS-Mono、OKVIS、ROVIO）
- **優勢：** 可對過去狀態重新線性化 → 比 EKF 更精準（VIO 場景）
- **限制：** 計算量大；通常需要嵌入式 CPU（Cortex-A 等級以上）

### 5.7 AI / 機器學習輔助 INS（新興方向）

神經網路被越來越廣泛地用於增強或取代傳統濾波器：

| 方法 | 說明 | 代表工作 |
|---|---|---|
| **偏置 / 漂移估計** | LSTM/TCN 從訓練數據學習 IMU 誤差模型 | TLIO、IONet |
| **端對端里程計** | CNN 將原始 IMU 數據映射至位置變化 | IMU2Pose |
| **殘差學習** | 神經網路修正 EKF 的殘差誤差 | AIDE、RINS-W |
| **地形匹配** | CNN 將感測器特徵與地圖匹配 | Deep TERCOM |

> **限制：** 基於機器學習的 INS 在訓練分佈外的動態條件下可能失效。對於安全關鍵系統，傳統 EKF 仍是可靠性標準。

---

## 6. 輔助訊號源與多感測器整合

### 6.1 GNSS（全球衛星導航系統）

GNSS 是 INS 最主要的長期位置錨定來源：

| 系統 | 歸屬 | 頻率 | 獨立精度 |
|---|---|---|---|
| GPS | 美國 | L1/L2/L5 | 2–5 m（民用） |
| GLONASS | 俄羅斯 | L1/L2 | 3–6 m |
| Galileo | 歐盟 | E1/E5 | < 1 m（開放服務） |
| 北斗（BeiDou） | 中國 | B1/B2/B3 | 2–5 m |
| QZSS | 日本 | L1/L2/L5 | < 1 m（區域） |

**RTK GNSS**（實時動態定位）：利用載波相位達到公分級精度；需要基地站或 NTRIP 網路。

#### GNSS-INS 耦合等級

| 耦合等級 | 共享內容 | 優點 | 缺點 |
|---|---|---|---|
| **鬆耦合** | GNSS 接收機輸出的位置與速度 | 簡單；子系統獨立 | 衛星數 < 4 顆時失效 |
| **緊耦合** | GNSS 的原始偽距與都卜勒頻移 | 1–3 顆衛星時仍可工作 | 複雜；需存取原始 GNSS 資料 |
| **深耦合 / 超緊耦合** | 相關器層級的追蹤迴路整合至 INS | 抗干擾強；重新捕獲最快 | 極度複雜；需客製化 GNSS 硬體 |

### 6.2 視覺 / 相機輔助

#### 6.2.1 視覺慣性里程計（VIO, Visual Inertial Odometry）

- **數據來源：** 相機（單目 / 雙目）+ IMU
- **輸出：** 相對起始點的 6-DOF 位姿估計
- **主要演算法：** MSCKF、OKVIS、VINS-Mono、VINS-Fusion、OpenVINS
- **優勢：** 可在室內 / GPS 拒止環境工作；特徵資訊豐富
- **限制：** 在無紋理 / 黑暗 / 動態場景中失效；單目存在尺度模糊性

#### 6.2.2 光流（Optical Flow）

- **數據來源：** 向下的相機追蹤地面紋理的運動
- **輸出：** 載體坐標系中的 2D 速度估計
- **有效範圍：** 高度 < 30 m（取決於相機解析度與地面紋理）
- **常用感測器：** PMW3901、ADNS3080、PX4FLOW

#### 6.2.3 地形參考導航（TRN）/ 等高線匹配（TERCOM）

- **數據來源：** 雷達高度計或 LiDAR 掃描地面剖面 vs. 預載數字高程模型（DEM）
- **輸出：** 透過地形特徵匹配進行絕對位置修正
- **精度：** 10–100 m CEP（取決於 DEM 解析度與地形起伏）
- **應用：** 巡弋飛彈（戰斧）、長程自主 UAV
- **關鍵限制：** 需預載地形數據庫；在平坦 / 海洋地形上失效

### 6.3 LiDAR 輔助 INS

- **3D LiDAR SLAM：** 即時建立並匹配 3D 點雲地圖
- **主要演算法：** Cartographer、LIO-SAM、LOAM、LeGO-LOAM、FAST-LIO2
- **精度：** 3D 定位 5–20 cm（特徵豐富的環境）
- **硬體平台：** Velodyne、Ouster、Livox（固態）
- **限制：** 功耗 / 重量 / 成本較高；戶外條件下有效距離有限；雨霧干擾

### 6.4 磁力計輔助

- **用途：** 利用地球磁場提供絕對航向（偏航角）參考
- **限制：** 對局部磁場干擾高度敏感（馬達、鐵磁結構）
- **無人機關鍵要求：** 必須距主板和電源線 ≥ 5–10 cm

| 磁力計 | 技術原理 | 最大 ODR | 雜訊本底 | 說明 |
|---|---|---|---|---|
| HMC5883L | AMR | 75 Hz | 中 | 已停產 |
| QMC5883L | AMR | 200 Hz | 中 | HMC5883L 替代品 |
| IST8310 | AMR | 200 Hz | 中低 | Pixhawk 生態常用 |
| BMM150 | 霍爾效應（FlipCore） | 30 Hz | 低 | Bosch 10-DOF 傳統配對 |
| **BMM350** | **隧穿磁阻（TMR）** | **400 Hz** | **超低** | **推薦搭配 BMI088** |
| RM3100 | 磁感應 | 600 Hz | 超低 | 專業 / 軍用等級 |

> **BMM350 用於長程 UAV：** BMM350 的 400 Hz ODR 匹配 BMI088 的迴路頻率，TMR 技術提供超低雜訊的航向估計，而**磁場衝擊恢復（Field Shock Recovery）** 功能對於馬達電流反覆造成磁場干擾的無人機至關重要。

### 6.5 氣壓高度計

- **用途：** 高度錨定（防止加速度計積分的垂直漂移）
- **典型精度：** 相對 ±0.5–2 m，絕對 ±10–30 m（視天氣而定）
- **融合策略：** 用於約束 INS 垂直通道（對漂移最敏感的軸向）

| 感測器 | 解析度 | RMS 雜訊 | 說明 |
|---|---|---|---|
| BMP280 | 0.16 Pa | ~0.12 Pa | 消費級 |
| **BMP580** | **0.06 Pa** | **~0.085 Pa** | **高精度，Bosch 旗艦型** |
| MS5611 | ~0.012 mbar | 極低 | Pixhawk/APM 常用 |

### 6.6 空速感測器（皮托管）

- **用途：** 風相對速度——有助於區分風漂移與真實運動
- **固定翼長程 UAV 關鍵：** 約束水平面的加速度計積分誤差
- **融合方式：** 作為 EKF 中的速度量測量；補充 GNSS 速度

### 6.7 無線電輔助

| 技術 | 距離 | 精度 | 應用 |
|---|---|---|---|
| UWB（超寬頻） | < 100 m | 10–30 cm | 室內精確降落、蜂群 |
| LoRa TDOA | 1–5 km | 1–10 m | 戶外 GPS 拒止追蹤 |
| 射頻測向（RDF） | 無限 | 僅方位角 | 終端歸向制導 |
| TACAN / VOR | 200–400 km | 10–100 m | 航空；軍事 |
| 地面信標 | 視配置 | 點參考 | TERPSCOM、SITAN |

---

## 7. GPS 拒止環境下的導航策略

對於**GPS 不可用或被干擾**的中遠程任務，**分層備援架構**至關重要：

### 7.1 備援層次架構

```
┌─────────────────────────────────────────────────────────────────┐
│                     GPS 可用                                    │
│   INS 傳播 + GNSS 位置 / 速度更新（EKF）                        │
│   → 持續估計偏置；INS 誤差受約束                                 │
└─────────────────────────┬───────────────────────────────────────┘
                          │ GPS 中斷
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│              第一層：純慣性航位推測                              │
│   純 INS，使用最後已知偏置；磁力計航向；氣壓高度；空速速度       │
│   → 精度：隨時間退化（秒至分鐘級）                              │
└─────────────────────────┬───────────────────────────────────────┘
                          │ 漂移趨於嚴重
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│              第二層：視覺里程計 / 光流                           │
│   相機速度與位置更新；VIO 或光流                                 │
│   → 精度：行進距離的 1–5%                                       │
└─────────────────────────┬───────────────────────────────────────┘
                          │ 飛越無特徵地形
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│              第三層：地形匹配（TERCOM / TRN）                    │
│   雷達 / LiDAR 高度計剖面 vs. 機載 DEM                          │
│   → 絕對位置修正；精度 10–100 m CEP                             │
└─────────────────────────┬───────────────────────────────────────┘
                          │ 進入末段 / 發現目標
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│              第四層：目標捕獲（ATA）                             │
│   EO/IR 相機 + AI 偵測（YOLO/SSD）；切換至視線（LOS）制導       │
│   不依賴坐標的末段導航                                           │
└─────────────────────────────────────────────────────────────────┘
```

### 7.2 SLAM（同步定位與建圖）

SLAM 在未知環境中即時建立地圖，同時追蹤載體在地圖中的位置：

- **度量 SLAM：** 3D 點雲或佔用柵格（LiDAR SLAM、視覺 SLAM）
- **拓撲 SLAM：** 特徵地點的圖形結構（適用於長廊 / 室內）
- **語義 SLAM：** 物件級理解；對感知混淆具有強韌性

### 7.3 偽 GNSS 技術

| 技術 | 原理 | 精度 |
|---|---|---|
| eLoran | 地基無線電到達時間差 | 10–100 m |
| DME/DME | 測距設備三角定位 | 100–500 m |
| 天文導航 | 星敏感器 + 地平線感測器 | < 1 海里 |
| 脈衝星導航 | X 射線脈衝星計時（太空） | < 1 km |

---

## 8. 性能等級比較

| 等級 | IMU 類型 | 陀螺偏置 | 加速度偏置 | 位置漂移 | 典型平台 |
|---|---|---|---|---|---|
| **消費級** | MEMS | > 10 °/hr | > 1 mg | km/min | 智慧型手機、玩具無人機 |
| **工業級** | MEMS（BMI088、ADIS） | 1–10 °/hr | 0.1–1 mg | 10–100 m/min | 無人機飛控、機器人 |
| **戰術級** | MEMS / FOG | 0.1–1 °/hr | 0.05–0.1 mg | 1–10 m/min | 軍用 UAV、精確制導彈藥 |
| **導航級** | FOG / RLG / QA | 0.001–0.01 °/hr | 1–50 µg | 0.3–30 m/hr（~1 海里/hr） | 飛機、潛艇、飛彈 |
| **戰略級** | HRG / 原子干涉 | < 0.001 °/hr | < 1 µg | < 30 m/hr（< 0.1 海里/hr） | ICBM、深空探測器 |

---

## 9. 中遠程無人機設計考量

### 9.1 IMU 選型建議

**中程（10–100 km）** 且 GNSS 可用：
- 工業級 MEMS（BMI088）+ 緊耦合 EKF + GNSS 已足夠
- 搭配磁力計（BMM350）+ 氣壓計（BMP580）提供 3D 冗餘

**長程（100–1000+ km）** 或 **GPS 拒止環境**：
- 升級至**戰術 MEMS**（ADIS16507、ADIS16545）或**戰術 FOG**
- 搭配 TERCOM 地形數據庫，進行地形參考位置重置
- 考慮 IMU 溫度校準（−40°C 至 +85°C 範圍）

### 9.2 雙核心處理架構（以 RP2350 為例）

```
Core 0（確定性控制迴路）                  Core 1（估計與通訊）
────────────────────────────             ──────────────────────────────────
BMI088 SPI @ 1 kHz 取樣                  GNSS UART 解析
姿態傳播（四元數）                        相機 / VIO 管線
串級 PID（角速率 + 姿態）                 EKF 狀態更新（10–100 Hz）
電機輸出（PWM / DSHOT）                  磁力計 + 氣壓計輪詢
seqlock 共享記憶體寫入                    MAVLink / CRSF 遙測
```

### 9.3 任務剖面對應的耦合策略

| 任務類型 | GNSS 狀態 | 推薦策略 |
|---|---|---|
| 超視距（BVLOS）民用無人機 | 可用 | 鬆耦合 EKF（GNSS + IMU + 氣壓計） |
| 軍用 UAV（對抗環境） | 間歇性 / 被干擾 | 緊耦合 + TERCOM + 光流 |
| 長程固定翼 | 可用 + 降質 | 深耦合 + VIO + 空速 |
| GPS 拒止室內 | 不可用 | LiDAR SLAM + VIO + INS |
| 末段歸向 | 被拒止 | INS 航位推測 + EO/IR 目標捕獲 |

### 9.4 誤差預算設計清單

- [ ] 選擇與任務距離及 GPS 可用性匹配的 IMU 等級
- [ ] 實施溫度補償 / 校準查找表
- [ ] 隔離磁力計的磁場干擾（距馬達 / ESC ≥ 10 cm）
- [ ] 採用減振安裝，降低振動整流誤差（VRE）
- [ ] 實作 seqlock 或 DMA 方式的跨核心 IMU 數據傳遞
- [ ] 定義 GNSS 中斷容忍度：純 INS 可接受多長時間 / 多遠距離？
- [ ] 設計備援模式行為（返航、懸停、TERCOM 重置）
- [ ] 以蒙地卡羅模擬驗證 EKF 調參（Q/R 矩陣）

### 9.5 主要演算法參考

| 演算法 | 應用場景 | 開源函式庫 |
|---|---|---|
| EKF2（PX4） | 無人機狀態估計 | PX4 Autopilot GitHub |
| Madgwick / Mahony | 輕量 AHRS | MadgwickAHRS（Arduino） |
| MSCKF | 視覺慣性里程計 | OpenVINS |
| LIO-SAM | LiDAR 慣性 SLAM | LIO-SAM GitHub |
| GTSAM | 因子圖最佳化 | GTSAM（喬治亞理工大學） |
| TLIO | 基於學習的 IMU 里程計 | TLIO（Meta Research） |

---

## 參考文獻與延伸閱讀

1. Groves, P.D. *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed. Artech House, 2013.
2. Titterton, D.H., Weston, J.L. *Strapdown Inertial Navigation Technology*, 2nd ed. IET, 2004.
3. Woodman, O.J. *An Introduction to Inertial Navigation*. Cambridge University Technical Report, 2007.
4. PX4 EKF2 文件：https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html
5. Bosch Sensortec BMI088 產品頁：https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/
6. Bosch Sensortec BMM350 產品頁：https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm350/
7. Geneva, P. et al. *OpenVINS: A Research Platform for Visual-Inertial Estimation*. ICRA 2020.

---

---

## 10. 感測器驅動與演算法實作指南

本章提供前述所有感測器類型的具體實作範式。程式碼範例以 **C/C++** 撰寫，目標平台為嵌入式系統（RP2350 / Cortex-M），並在適當處提供 Python 版本供主機端原型驗證使用。

> **慣例說明：** `SPI_transfer()`、`I2C_write()`、`I2C_read()` 代表平台 HAL 函式呼叫。在 RP2350/Arduino-pico 環境中，對應至 `SPI.transfer()` / `Wire.write()` / `Wire.read()`。

---

### 10.1 MEMS IMU — BMI088（SPI，RP2350）

BMI088 將陀螺儀與加速度計以兩個獨立 SPI/I2C 裝置對外呈現，各有獨立的晶片選擇腳位（CS）。

#### 10.1.1 暫存器層級初始化

```c
// bmi088.h — 暫存器映射節錄
#define BMI088_GYRO_CHIP_ID     0x0F  // 預期值：0x0F
#define BMI088_GYRO_RANGE       0x0F  // ±2000 °/s = 0x00
#define BMI088_GYRO_BANDWIDTH   0x10  // ODR 2000 Hz, BW 532 Hz = 0x81
#define BMI088_GYRO_INT3_INT4   0x18
#define BMI088_GYRO_INT_CTRL    0x15  // 資料就緒中斷致能

#define BMI088_ACCEL_CHIP_ID    0x00  // 預期值：0x1E
#define BMI088_ACCEL_PWR_CTRL   0x7D  // 0x04 = 加速度計開啟
#define BMI088_ACCEL_CONF       0x40  // ODR 與頻寬設定
#define BMI088_ACCEL_RANGE      0x41  // ±24 g = 0x03

// 陀螺儀初始化序列（SPI）
void bmi088_gyro_init(void) {
    spi_cs_gyro_deassert();
    delay_us(1);                        // CS 取消選中保護時間（≥450 ns）

    // 軟體重置
    spi_write_reg(CS_GYRO, 0x14, 0xB6);
    delay_ms(30);

    // 驗證晶片 ID
    uint8_t id = spi_read_reg(CS_GYRO, BMI088_GYRO_CHIP_ID);
    assert(id == 0x0F);

    spi_write_reg(CS_GYRO, BMI088_GYRO_RANGE,     0x00); // ±2000 °/s
    spi_write_reg(CS_GYRO, BMI088_GYRO_BANDWIDTH, 0x81); // ODR 2000 Hz
    spi_write_reg(CS_GYRO, BMI088_GYRO_INT_CTRL,  0x80); // 資料就緒中斷致能
    spi_write_reg(CS_GYRO, BMI088_GYRO_INT3_INT4, 0x01); // INT3 主動高電位推挽輸出
}

// 加速度計初始化序列
void bmi088_accel_init(void) {
    // RP2350 關鍵注意：Flash 屬性不得快取 SPI 周邊暫存器
    // 對 ISR 與驅動程式碼使用 __attribute__((section(".time_critical")))

    spi_write_reg(CS_ACCEL, 0x7E, 0xB6);   // 軟體重置
    delay_ms(50);                            // 加速度計重置後需 >50 ms

    // 上電後切換至 SPI 模式前，需進行一次虛擬讀取
    (void)spi_read_reg(CS_ACCEL, BMI088_ACCEL_CHIP_ID);
    uint8_t id = spi_read_reg(CS_ACCEL, BMI088_ACCEL_CHIP_ID);
    assert(id == 0x1E);

    spi_write_reg(CS_ACCEL, BMI088_ACCEL_CONF,     0xAC); // ODR 1600 Hz, OSR4
    spi_write_reg(CS_ACCEL, BMI088_ACCEL_RANGE,    0x03); // ±24 g
    spi_write_reg(CS_ACCEL, BMI088_ACCEL_PWR_CTRL, 0x04); // 上電
    delay_ms(5);
}
```

#### 10.1.2 高頻資料讀取（中斷驅動，1 kHz）

```c
// 使用 seqlock 的共享資料結構，確保雙核心安全（RP2350）
typedef struct {
    volatile uint32_t seq;       // seqlock 序列計數器（奇數 = 寫入中）
    float gx, gy, gz;            // rad/s
    float ax, ay, az;            // m/s²
    uint64_t timestamp_us;
} __attribute__((aligned(4))) ImuData;

static ImuData imu_shared;

// 陀螺 LSB 至 rad/s：±2000°/s 範圍 → 16.384 LSB/(°/s)
#define GYRO_SCALE  (1.0f / 16.384f * (M_PI / 180.0f))
// 加速 LSB 至 m/s²：±24g 範圍 → 1365.33 LSB/g
#define ACCEL_SCALE (1.0f / 1365.33f * 9.80665f)

// 在 Core 0 計時器 ISR 中以 1 kHz 呼叫
void __time_critical_func(imu_read_isr)(void) {
    uint8_t buf[6];

    // --- 陀螺儀連續讀取（暫存器 0x02–0x07） ---
    spi_read_burst(CS_GYRO, 0x02, buf, 6);
    int16_t raw_gx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_gy = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_gz = (int16_t)((buf[5] << 8) | buf[4]);

    // --- 加速度計連續讀取（暫存器 0x12–0x17） ---
    spi_read_burst(CS_ACCEL, 0x12, buf, 6);
    int16_t raw_ax = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_ay = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_az = (int16_t)((buf[5] << 8) | buf[4]);

    // Seqlock 寫入（奇數序列 = 寫入中）
    __dmb();                          // 寫入前的資料記憶體屏障
    imu_shared.seq++;
    __dmb();
    imu_shared.gx = raw_gx * GYRO_SCALE;
    imu_shared.gy = raw_gy * GYRO_SCALE;
    imu_shared.gz = raw_gz * GYRO_SCALE;
    imu_shared.ax = raw_ax * ACCEL_SCALE;
    imu_shared.ay = raw_ay * ACCEL_SCALE;
    imu_shared.az = raw_az * ACCEL_SCALE;
    imu_shared.timestamp_us = time_us_64();
    __dmb();
    imu_shared.seq++;                 // 回到偶數 = 資料一致
    __dmb();
}

// Core 1 讀取端（EKF 更新執行緒）
bool imu_read_safe(ImuData *out) {
    uint32_t seq1, seq2;
    do {
        seq1 = imu_shared.seq;
        __dmb();
        *out = imu_shared;           // 結構體複製
        __dmb();
        seq2 = imu_shared.seq;
    } while ((seq1 & 1) || (seq1 != seq2)); // 寫入中則重試
    return true;
}
```

#### 10.1.3 溫度補償

```c
// BMI088 內建溫度感測器（加速度計側暫存器 0x22–0x23）
float bmi088_read_temp_celsius(void) {
    uint8_t buf[2];
    spi_read_burst(CS_ACCEL, 0x22, buf, 2);
    int16_t raw = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (raw > 1023) raw -= 2048;     // 11 位元有號數
    return raw * 0.125f + 23.0f;     // 資料表公式
}

// 陀螺偏置溫度模型（離線校準後擬合）
// b_g(T) = b0 + b1*(T - T_ref)
float gyro_bias_compensated(float raw_bias, float temp) {
    const float T_ref = 25.0f;
    const float b0    = 0.0012f;   // rad/s — 來自校準
    const float b1    = 0.00003f;  // rad/s/°C
    return raw_bias - (b0 + b1 * (temp - T_ref));
}
```

---

### 10.2 導航級陀螺儀 — FOG / RLG / HRG

高端陀螺儀（FOG、RLG、HRG）不直接暴露原始暫存器，而是透過**數位串列通訊協定**輸出已處理的角速率或角增量資料。

#### 10.2.1 常見介面標準

| 介面 | 典型鮑率 | 協定 | 使用裝置 |
|---|---|---|---|
| RS-422（非同步） | 115200–921600 | 二進制封包 | KVH 1775 IMU、iXblue Gyropak |
| RS-422（同步） | SPI 類時脈 | 二進制突發 | Honeywell GG1308 RLG |
| ARINC 429 | 12.5 / 100 kbps | BCD / BNR 字組 | 航空 FOG/RLG INS |
| UART TTL | 115200–1000000 | NMEA / 二進制 | 低端戰術 IMU |
| SPI（客製化） | 最高 10 MHz | 二進制幀 | ADIS16507（戰術 MEMS） |

#### 10.2.2 KVH 1775 IMU — RS-422 二進制協定解析器（C）

KVH 1775 是導航級 FOG IMU，透過 RS-422 以最高 1000 Hz 輸出 36 位元組的二進制幀。

```c
// KVH 1775 幀格式（36 位元組，大端序）
// [0x01][0xFE] 標頭 | [狀態 1B] | [角增量 12B] | [速度增量 12B] |
// [溫度 4B] | [序列號 1B] | [校驗碼 2B] | [0x00][0x03] 幀尾

#define KVH_FRAME_LEN  36
#define KVH_HDR0       0x01
#define KVH_HDR1       0xFE

typedef struct {
    float delta_angle[3];   // rad（自上次取樣以來的角增量）
    float delta_vel[3];     // m/s（自上次取樣以來的速度增量）
    float temperature;      // °C
    uint8_t sequence;
    uint8_t status;
} Kvh1775Frame;

// 大端序浮點數解碼
static float be_float(const uint8_t *p) {
    union { uint8_t b[4]; float f; } u;
    u.b[3] = p[0]; u.b[2] = p[1]; u.b[1] = p[2]; u.b[0] = p[3];
    return u.f;
}

// CRC-16/CCITT-FALSE，計算位元組 [0..33]
static uint16_t kvh_crc16(const uint8_t *buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

bool kvh1775_parse(const uint8_t *buf, Kvh1775Frame *out) {
    if (buf[0] != KVH_HDR0 || buf[1] != KVH_HDR1) return false;

    uint16_t rx_crc = (uint16_t)(buf[34] << 8) | buf[35];
    if (kvh_crc16(buf, 34) != rx_crc)             return false;

    out->status           = buf[2];
    out->delta_angle[0]   = be_float(buf + 3);
    out->delta_angle[1]   = be_float(buf + 7);
    out->delta_angle[2]   = be_float(buf + 11);
    out->delta_vel[0]     = be_float(buf + 15);
    out->delta_vel[1]     = be_float(buf + 19);
    out->delta_vel[2]     = be_float(buf + 23);
    out->temperature      = be_float(buf + 27);
    out->sequence         = buf[31];
    return true;
}

// 將角增量累積為姿態（簡化版，不含圓錐誤差修正）
void kvh_integrate_attitude(const Kvh1775Frame *f, float q[4]) {
    float dax = f->delta_angle[0];
    float day = f->delta_angle[1];
    float daz = f->delta_angle[2];
    float norm = sqrtf(dax*dax + day*day + daz*daz);
    if (norm < 1e-10f) return;
    float s = sinf(norm / 2.0f) / norm;
    float dq[4] = { cosf(norm / 2.0f), dax*s, day*s, daz*s };
    // q = q ⊗ dq（四元數乘法）
    float q0 = q[0]*dq[0] - q[1]*dq[1] - q[2]*dq[2] - q[3]*dq[3];
    float q1 = q[0]*dq[1] + q[1]*dq[0] + q[2]*dq[3] - q[3]*dq[2];
    float q2 = q[0]*dq[2] - q[1]*dq[3] + q[2]*dq[0] + q[3]*dq[1];
    float q3 = q[0]*dq[3] + q[1]*dq[2] - q[2]*dq[1] + q[3]*dq[0];
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q[0]=q0/norm; q[1]=q1/norm; q[2]=q2/norm; q[3]=q3/norm;
}
```

#### 10.2.3 ADIS16507 戰術 MEMS — SPI 突發讀取（RP2350）

```c
// ADIS16507 關鍵暫存器
#define ADIS_DIAG_STAT    0x02
#define ADIS_X_GYRO_LOW   0x04   // 32 位元：LOW @ 0x04，OUT @ 0x06
#define ADIS_BURST_CMD    0x68   // 觸發突發讀取

// 突發讀取返回 20 位元組：DIAG|GX|GY|GZ|AX|AY|AZ|TEMP|CNT|CRC
typedef struct {
    int32_t gyro[3];    // 原始 32 位元，LSB = 0.1°/s / 2^16
    int32_t accel[3];   // 原始 32 位元，LSB = 1.25 mg / 2^16
    int16_t temp;       // 0.1°C / LSB
    uint16_t cnt;
} Adis16507Burst;

#define ADIS_GYRO_SCALE   (0.1f / 65536.0f * M_PI / 180.0f)   // rad/s per LSB
#define ADIS_ACCEL_SCALE  (0.00125f / 65536.0f * 9.80665f)     // m/s² per LSB

bool adis16507_burst_read(Adis16507Burst *out) {
    uint8_t tx[22] = {0x68, 0x00};  // 突發命令 + 20 個虛擬位元組
    uint8_t rx[22];

    gpio_put(CS_ADIS, 0);
    spi_write_read_blocking(SPI_PORT, tx, rx, 22);
    gpio_put(CS_ADIS, 1);

    // CRC-16 檢查，範圍位元組 2..19
    uint16_t crc_rx = (uint16_t)(rx[20] << 8) | rx[21];
    if (crc16_burst(rx + 2, 18) != crc_rx) return false;

    // 解析有號 32 位元值（大端序）
    for (int i = 0; i < 3; i++) {
        out->gyro[i]  = (int32_t)((rx[4+i*4]<<24)|(rx[5+i*4]<<16)|
                                   (rx[6+i*4]<<8) | rx[7+i*4]);
        out->accel[i] = (int32_t)((rx[16+i*4]<<24)|(rx[17+i*4]<<16)|
                                   (rx[18+i*4]<<8) | rx[19+i*4]);
    }
    out->temp = (int16_t)((rx[28] << 8) | rx[29]);
    out->cnt  = (uint16_t)((rx[30] << 8) | rx[31]);
    return true;
}
```

---

### 10.3 磁力計 — BMM350（I2C）

```c
#define BMM350_I2C_ADDR   0x14   // SDO 低電位；SDO 高電位則為 0x15
#define BMM350_CHIP_ID    0x00   // 預期值 0x33
#define BMM350_PMU_CMD    0x06
#define BMM350_DATA_X_LSB 0x31

// 電源模式
#define BMM350_PMU_NORMAL 0x01
#define BMM350_PMU_FORCED 0x03

void bmm350_init(void) {
    // 透過 PMU_CMD 進行軟體重置
    i2c_write_reg(BMM350_I2C_ADDR, BMM350_PMU_CMD, 0x01); // 待機 → 正常
    delay_ms(3);    // 模式切換後的啟動時間

    uint8_t id = i2c_read_reg(BMM350_I2C_ADDR, BMM350_CHIP_ID);
    assert(id == 0x33);

    // 設定 ODR 400 Hz，致能所有軸
    i2c_write_reg(BMM350_I2C_ADDR, 0x04, 0x04); // ODR_400Hz
    i2c_write_reg(BMM350_I2C_ADDR, 0x05, 0x07); // 致能 XYZ 軸
}

typedef struct { float x, y, z; } MagData; // 單位：µT

MagData bmm350_read(void) {
    uint8_t buf[6];
    i2c_read_burst(BMM350_I2C_ADDR, BMM350_DATA_X_LSB, buf, 6);

    // 20 位元有號數，以 24 位元（每軸 3 位元組）儲存，右移 4 位
    int32_t raw_x = ((int32_t)(buf[2]<<16 | buf[1]<<8 | buf[0])) >> 4;
    int32_t raw_y = ((int32_t)(buf[5]<<16 | buf[4]<<8 | buf[3])) >> 4;

    uint8_t buf2[3];
    i2c_read_burst(BMM350_I2C_ADDR, BMM350_DATA_X_LSB + 6, buf2, 3);
    int32_t raw_z = ((int32_t)(buf2[2]<<16 | buf2[1]<<8 | buf2[0])) >> 4;

    const float LSB_TO_UT = 0.0003f; // 300 nT/LSB（資料表）
    return (MagData){ raw_x * LSB_TO_UT,
                      raw_y * LSB_TO_UT,
                      raw_z * LSB_TO_UT };
}

// 硬鐵 / 軟鐵校準（離線校準後套用）
MagData bmm350_calibrate(MagData raw, float hard_iron[3], float soft_iron[3][3]) {
    float v[3] = { raw.x - hard_iron[0],
                   raw.y - hard_iron[1],
                   raw.z - hard_iron[2] };
    MagData out;
    out.x = soft_iron[0][0]*v[0] + soft_iron[0][1]*v[1] + soft_iron[0][2]*v[2];
    out.y = soft_iron[1][0]*v[0] + soft_iron[1][1]*v[1] + soft_iron[1][2]*v[2];
    out.z = soft_iron[2][0]*v[0] + soft_iron[2][1]*v[1] + soft_iron[2][2]*v[2];
    return out;
}
```

---

### 10.4 氣壓高度計 — BMP580（SPI）

```c
#define BMP580_CHIP_ID     0x50   // 暫存器 0x01，預期值 0x51
#define BMP580_ODR_CONFIG  0x37
#define BMP580_OSR_CONFIG  0x36
#define BMP580_DATA_P_XLSB 0x20   // 3 位元組氣壓
#define BMP580_DATA_T_XLSB 0x23   // 3 位元組溫度

void bmp580_init(void) {
    spi_write_reg(CS_BARO, 0x7E, 0xB6);    // 軟體重置
    delay_ms(5);
    uint8_t id = spi_read_reg(CS_BARO, BMP580_CHIP_ID);
    assert(id == 0x51);

    // OSR：氣壓 x8，溫度 x2，IIR 濾波係數 4
    spi_write_reg(CS_BARO, BMP580_OSR_CONFIG, 0x35);
    // ODR：50 Hz 正常模式
    spi_write_reg(CS_BARO, BMP580_ODR_CONFIG, 0xD0);
}

typedef struct { float pressure_pa; float temp_c; } BaroData;

BaroData bmp580_read(void) {
    uint8_t buf[6];
    spi_read_burst(CS_BARO, BMP580_DATA_P_XLSB, buf, 6);

    uint32_t raw_p = (uint32_t)buf[0] | ((uint32_t)buf[1]<<8) | ((uint32_t)buf[2]<<16);
    uint32_t raw_t = (uint32_t)buf[3] | ((uint32_t)buf[4]<<8) | ((uint32_t)buf[5]<<16);

    // BMP580 輸出已完成補償的定點數
    // 氣壓 LSB = 1/64 Pa，溫度 LSB = 1/65536 °C
    return (BaroData){
        .pressure_pa = (float)raw_p / 64.0f,
        .temp_c      = (float)((int32_t)raw_t) / 65536.0f
    };
}

// 由氣壓換算高度（國際標準大氣模型）
float baro_pressure_to_altitude(float p_pa, float p0_pa) {
    // 氣壓高度公式
    return 44330.0f * (1.0f - powf(p_pa / p0_pa, 0.190294f));
}
```

---

### 10.5 光流感測器 — PMW3901（SPI）

PMW3901 是近地光流感測器，提供 2D 像素位移量測。

```c
#define PMW3901_PRODUCT_ID  0x00   // 預期值 0x49
#define PMW3901_MOTION      0x02
#define PMW3901_DELTA_X_L   0x03
#define PMW3901_DELTA_Y_L   0x05

void pmw3901_init(void) {
    // 依資料表的上電初始化序列（完整 42 暫存器序列，節錄）
    spi_write_reg(CS_FLOW, 0x7F, 0x00);
    spi_write_reg(CS_FLOW, 0x55, 0x01);
    spi_write_reg(CS_FLOW, 0x50, 0x07);
    spi_write_reg(CS_FLOW, 0x7F, 0x0E);
    delay_ms(100);
    uint8_t id = spi_read_reg(CS_FLOW, PMW3901_PRODUCT_ID);
    assert(id == 0x49);
}

typedef struct { int16_t dx, dy; bool motion; } FlowData;

FlowData pmw3901_read(void) {
    uint8_t motion = spi_read_reg(CS_FLOW, PMW3901_MOTION);
    int16_t dx = (int16_t)((spi_read_reg(CS_FLOW, PMW3901_DELTA_X_L)) |
                            (spi_read_reg(CS_FLOW, 0x04) << 8));
    int16_t dy = (int16_t)((spi_read_reg(CS_FLOW, PMW3901_DELTA_Y_L)) |
                            (spi_read_reg(CS_FLOW, 0x06) << 8));
    return (FlowData){ dx, dy, (motion & 0x80) != 0 };
}

// 將像素位移轉換為速度（m/s），需提供當前高度
void flow_to_velocity(FlowData f, float altitude_m, float dt_s,
                      float *vx, float *vy) {
    // focal_length_px：感測器焦距（像素單位，依鏡頭校準）
    const float focal_length_px = 30.0f; // PMW3901 + 2mm 鏡頭的典型值
    *vx = -(float)f.dx / focal_length_px * altitude_m / dt_s;
    *vy = -(float)f.dy / focal_length_px * altitude_m / dt_s;
}
```

---

### 10.6 視覺 / 相機 — 視覺慣性里程計（VIO）

VIO 為主機 CPU / 嵌入式 Linux 任務。以下展示關鍵管線階段，使用 **OpenCV**（C++），適用於 RK3588、RPi CM4 或 Hailo-8 等平台。

#### 10.6.1 特徵追蹤（Lucas-Kanade 光流）

```cpp
#include <opencv2/opencv.hpp>
#include <deque>

class FeatureTracker {
public:
    struct TrackedPoint {
        cv::Point2f pt;
        uint64_t    id;
    };

    void process(const cv::Mat &frame_gray, uint64_t ts_us) {
        if (prev_pts_.empty()) {
            // 偵測新特徵點（FAST + goodFeaturesToTrack）
            cv::goodFeaturesToTrack(frame_gray, corners_, 200,
                                    0.01, 15, cv::noArray(), 3, true);
            for (auto &c : corners_)
                prev_pts_.push_back({c, next_id_++});
            prev_frame_ = frame_gray.clone();
            return;
        }

        // 使用 LK 光流追蹤
        std::vector<cv::Point2f> in_pts, out_pts;
        for (auto &tp : prev_pts_) in_pts.push_back(tp.pt);

        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_frame_, frame_gray,
                                  in_pts, out_pts, status, err,
                                  cv::Size(21, 21), 3);

        // RANSAC 基本矩陣剔除外點
        if (in_pts.size() > 8) {
            std::vector<uchar> mask;
            cv::findFundamentalMat(in_pts, out_pts, cv::FM_RANSAC, 1.0, 0.99, mask);
            for (size_t i = 0; i < mask.size(); i++)
                if (!mask[i]) status[i] = 0;
        }

        // 保留成功追蹤的點
        std::vector<TrackedPoint> next_pts;
        for (size_t i = 0; i < status.size(); i++)
            if (status[i]) next_pts.push_back({out_pts[i], prev_pts_[i].id});

        // 特徵點不足時補充新點
        if (next_pts.size() < 80) {
            cv::goodFeaturesToTrack(frame_gray, corners_, 150, 0.01, 15);
            for (auto &c : corners_)
                next_pts.push_back({c, next_id_++});
        }

        prev_pts_ = next_pts;
        prev_frame_ = frame_gray.clone();
    }

    const std::vector<TrackedPoint>& tracked() const { return prev_pts_; }

private:
    cv::Mat prev_frame_;
    std::vector<TrackedPoint> prev_pts_;
    std::vector<cv::Point2f> corners_;
    uint64_t next_id_ = 0;
};
```

#### 10.6.2 IMU 預積分（相機幀間）

```cpp
// 積分兩個相機時間戳之間的 IMU 量測
// 作為 EKF / 因子圖的緊湊相對運動約束
struct ImuPreintegration {
    float dt_total;
    float delta_p[3];   // 位置變化量
    float delta_v[3];   // 速度變化量
    float delta_R[9];   // 旋轉矩陣（3x3，行主序）

    void reset() {
        dt_total = 0;
        memset(delta_p, 0, sizeof(delta_p));
        memset(delta_v, 0, sizeof(delta_v));
        // delta_R = 單位矩陣
        delta_R[0]=delta_R[4]=delta_R[8]=1.0f;
        delta_R[1]=delta_R[2]=delta_R[3]=
        delta_R[5]=delta_R[6]=delta_R[7]=0.0f;
    }

    void integrate(float gx, float gy, float gz,
                   float ax, float ay, float az, float dt) {
        // 呼叫前應先套用陀螺偏置修正
        // 一階積分（高精度需求請改用 RK4）
        float wx = gx*dt, wy = gy*dt, wz = gz*dt;
        float angle = sqrtf(wx*wx + wy*wy + wz*wz);

        if (angle > 1e-8f) {
            // 使用 Rodrigues 旋轉公式更新 delta_R
            // （完整實作請參考 VINS-Mono 或 OpenVINS 原始碼）
        }

        // delta_v += delta_R * a_body * dt
        delta_v[0] += (delta_R[0]*ax + delta_R[1]*ay + delta_R[2]*az) * dt;
        delta_v[1] += (delta_R[3]*ax + delta_R[4]*ay + delta_R[5]*az) * dt;
        delta_v[2] += (delta_R[6]*ax + delta_R[7]*ay + delta_R[8]*az) * dt;

        // delta_p += delta_v * dt
        delta_p[0] += delta_v[0]*dt;
        delta_p[1] += delta_v[1]*dt;
        delta_p[2] += delta_v[2]*dt;

        dt_total += dt;
    }
};
```

---

### 10.7 LiDAR — UART 單點測距（TFmini-S）與 FAST-LIO2

#### 10.7.1 TFmini-S 單束 LiDAR（UART，C）

單束 LiDAR 適用於地形跟隨或精確降落的測距。

```c
// TFmini-S 幀格式：[0x59][0x59][Dist_L][Dist_H][Str_L][Str_H][Temp_L][Temp_H][Cksum]
#define TFMINI_FRAME_LEN 9
#define TFMINI_HDR       0x59

typedef struct { uint16_t dist_cm; uint16_t strength; float temp_c; } TfminiData;

bool tfmini_parse(const uint8_t *buf, TfminiData *out) {
    if (buf[0] != TFMINI_HDR || buf[1] != TFMINI_HDR) return false;
    uint8_t cksum = 0;
    for (int i = 0; i < 8; i++) cksum += buf[i];
    if (cksum != buf[8]) return false;

    out->dist_cm  = (uint16_t)(buf[2] | (buf[3] << 8));
    out->strength = (uint16_t)(buf[4] | (buf[5] << 8));
    out->temp_c   = ((float)(buf[6] | (buf[7] << 8)) / 8.0f) - 256.0f;
    return true;
}

// UART 串流資料的狀態機
typedef enum { WAIT_H1, WAIT_H2, RECV_DATA } TfState;

void tfmini_feed(uint8_t byte, TfminiData *out, bool *valid) {
    static TfState state = WAIT_H1;
    static uint8_t frame[TFMINI_FRAME_LEN];
    static uint8_t idx = 0;
    *valid = false;
    switch (state) {
        case WAIT_H1: if (byte==TFMINI_HDR){frame[0]=byte; state=WAIT_H2;} break;
        case WAIT_H2: if (byte==TFMINI_HDR){frame[1]=byte; idx=2; state=RECV_DATA;}
                      else                 {state=WAIT_H1;} break;
        case RECV_DATA:
            frame[idx++] = byte;
            if (idx == TFMINI_FRAME_LEN) {
                *valid = tfmini_parse(frame, out);
                state = WAIT_H1;
            }
            break;
    }
}
```

#### 10.7.2 FAST-LIO2 整合（ROS 2 / Linux，C++）

```cpp
// ROS 2 節點骨架：FAST-LIO2 + 外部 INS 輔助
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

class FastLioAidedNode : public rclcpp::Node {
public:
    FastLioAidedNode() : Node("fastlio_aided") {
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw", 10,
            [this](sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(msg); });

        lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_lidar(msg); });

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/lidar_odom", 10);
    }

private:
    void on_imu(sensor_msgs::msg::Imu::SharedPtr msg) {
        // 轉送至 FAST-LIO2 內部緩衝區（帶時間戳）
        imu_buffer_.push_back(*msg);
        if (imu_buffer_.size() > 500) imu_buffer_.pop_front();
    }

    void on_lidar(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // FAST-LIO2：使用 iKd-Tree 進行掃描至地圖匹配
        // 輸出修正後的位姿；回饋至 INS EKF 作為位置量測值
        auto odom = run_fastlio2(msg, imu_buffer_);
        odom_pub_->publish(odom);

        // 透過 MAVLink 或共享記憶體，將位置更新傳送至 MCU 端的 INS EKF
        send_position_update(odom.pose.pose.position.x,
                             odom.pose.pose.position.y,
                             odom.pose.pose.position.z);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::deque<sensor_msgs::msg::Imu> imu_buffer_;
};
```

---

### 10.8 GNSS — UBX 二進制協定解析器（u-blox M9/M10）

```c
// u-blox UBX 幀格式：[0xB5][0x62][class][id][len_L][len_H][payload][ck_a][ck_b]
#define UBX_HDR0   0xB5
#define UBX_HDR1   0x62
#define UBX_NAV_PVT_CLASS  0x01
#define UBX_NAV_PVT_ID     0x07
#define UBX_NAV_PVT_LEN    92

typedef struct {
    uint32_t iTOW;        // GPS 週內時間（ms）
    int32_t  lat;         // 緯度（1e-7 度）
    int32_t  lon;         // 經度（1e-7 度）
    int32_t  hMSL;        // 平均海平面高度（mm）
    int32_t  velN;        // NED 北向速度（mm/s）
    int32_t  velE;        // NED 東向速度（mm/s）
    int32_t  velD;        // NED 下向速度（mm/s）
    uint32_t hAcc;        // 水平精度（mm）
    uint32_t vAcc;        // 垂直精度（mm）
    uint8_t  fixType;     // 0=無定位, 3=3D 定位, 5=RTK 固定解
    uint8_t  numSV;       // 使用的衛星數量
} UbxNavPvt;

static void ubx_checksum(const uint8_t *buf, size_t len,
                          uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = *ck_b = 0;
    for (size_t i = 0; i < len; i++) {
        *ck_a += buf[i];
        *ck_b += *ck_a;
    }
}

bool ubx_parse_nav_pvt(const uint8_t *frame, UbxNavPvt *out) {
    if (frame[0] != UBX_HDR0 || frame[1] != UBX_HDR1)              return false;
    if (frame[2] != UBX_NAV_PVT_CLASS || frame[3] != UBX_NAV_PVT_ID) return false;
    uint16_t len = (uint16_t)(frame[4] | (frame[5] << 8));
    if (len != UBX_NAV_PVT_LEN) return false;

    uint8_t ck_a, ck_b;
    ubx_checksum(frame + 2, len + 4, &ck_a, &ck_b);
    if (ck_a != frame[6+len] || ck_b != frame[7+len]) return false;

    const uint8_t *p = frame + 6;  // Payload 起始位置
    out->iTOW    = (uint32_t)(p[0] | p[1]<<8 | p[2]<<16 | p[3]<<24);
    out->fixType = p[20];
    out->numSV   = p[23];
    out->lon     = (int32_t)(p[24] | p[25]<<8 | p[26]<<16 | p[27]<<24);
    out->lat     = (int32_t)(p[28] | p[29]<<8 | p[30]<<16 | p[31]<<24);
    out->hMSL    = (int32_t)(p[36] | p[37]<<8 | p[38]<<16 | p[39]<<24);
    out->velN    = (int32_t)(p[48] | p[49]<<8 | p[50]<<16 | p[51]<<24);
    out->velE    = (int32_t)(p[52] | p[53]<<8 | p[54]<<16 | p[55]<<24);
    out->velD    = (int32_t)(p[56] | p[57]<<8 | p[58]<<16 | p[59]<<24);
    out->hAcc    = (uint32_t)(p[40] | p[41]<<8 | p[42]<<16 | p[43]<<24);
    out->vAcc    = (uint32_t)(p[44] | p[45]<<8 | p[46]<<16 | p[47]<<24);
    return true;
}
```

---

### 10.9 EKF 感測器融合骨架（15 狀態 INS-GNSS）

此為整合所有感測器來源的最小化 EKF 結構。

```c
// 15 狀態誤差狀態 EKF，用於 INS-GNSS 鬆耦合
// 狀態：[δp(3) δv(3) δψ(3) bg(3) ba(3)]

#define N_STATES 15

typedef struct {
    float x[N_STATES];           // 誤差狀態向量
    float P[N_STATES][N_STATES]; // 誤差協方差矩陣
    float Q[N_STATES];           // 過程雜訊（對角線）
    float R_gnss_pos[3];         // GNSS 位置量測雜訊
    float R_gnss_vel[3];         // GNSS 速度量測雜訊
    float R_baro;                // 氣壓計高度雜訊
} EkfState;

// --- 預測步驟（每次 IMU 取樣時呼叫，~1 kHz） ---
void ekf_predict(EkfState *ekf, float gx, float gy, float gz,
                 float ax, float ay, float az, float dt) {
    // 1. 套用估計偏置
    float g[3] = { gx - ekf->x[9],  gy - ekf->x[10], gz - ekf->x[11] };
    float a[3] = { ax - ekf->x[12], ay - ekf->x[13], az - ekf->x[14] };

    // 2. 傳播名義狀態（四元數 + 速度 + 位置）
    //    （與誤差狀態 x 分開維護）
    nominal_state_propagate(g, a, dt);

    // 3. 建立狀態轉移矩陣 F（15×15，稀疏矩陣）
    //    F ≈ I + Fc*dt，Fc 為連續時間系統矩陣
    float F[N_STATES][N_STATES];
    build_F_matrix(F, g, a, dt);

    // 4. 協方差傳播：P = F*P*Fᵀ + Q*dt
    float FP[N_STATES][N_STATES], FPFt[N_STATES][N_STATES];
    mat_mul_15(F, ekf->P, FP);
    mat_mul_15_transposed(FP, F, FPFt);
    for (int i = 0; i < N_STATES; i++)
        FPFt[i][i] += ekf->Q[i] * dt;
    memcpy(ekf->P, FPFt, sizeof(ekf->P));
}

// --- GNSS 位置 / 速度更新（以 GNSS 頻率呼叫，~10 Hz） ---
void ekf_update_gnss(EkfState *ekf, float dp[3], float dv[3]) {
    // 量測殘差：z = [量測值 - 預測值]
    // H 矩陣：位置為 [I(3) 0 0 0 0]，速度為 [0 I(3) 0 0 0]
    float z[6];
    for (int i = 0; i < 3; i++) {
        z[i]   = dp[i]; // 位置殘差
        z[i+3] = dv[i]; // 速度殘差
    }

    // 卡爾曼增益（簡化 6 量測更新）
    // 實務上：K = P*Hᵀ*(H*P*Hᵀ + R)⁻¹
    float K[N_STATES][6];
    compute_kalman_gain_pos_vel(ekf->P, ekf->R_gnss_pos, ekf->R_gnss_vel, K);

    // 狀態修正：x = x + K*z
    for (int i = 0; i < N_STATES; i++)
        for (int j = 0; j < 6; j++)
            ekf->x[i] += K[i][j] * z[j];

    // 協方差更新：P = (I - K*H)*P
    update_covariance_pos_vel(ekf->P, K);

    // 將誤差狀態修正量注入名義狀態，並重置誤差狀態 x 為零
    inject_and_reset(ekf);
}

// --- 氣壓計高度更新（以氣壓計頻率呼叫，~50 Hz） ---
void ekf_update_baro(EkfState *ekf, float dh) {
    // H = [0,0,1, 0,0,0, 0,0,0, 0,0,0, 0,0,0]（僅高度）
    float S = ekf->P[2][2] + ekf->R_baro;    // 新息方差
    float K[N_STATES];
    for (int i = 0; i < N_STATES; i++)
        K[i] = ekf->P[i][2] / S;             // 高度的卡爾曼增益

    for (int i = 0; i < N_STATES; i++)
        ekf->x[i] += K[i] * dh;

    for (int i = 0; i < N_STATES; i++)
        for (int j = 0; j < N_STATES; j++)
            ekf->P[i][j] -= K[i] * ekf->P[2][j];

    inject_and_reset(ekf);
}
```

---

### 10.10 介面與時序總覽

| 感測器 | 介面 | 典型頻率 | 驅動位置 |
|---|---|---|---|
| BMI088 陀螺儀 | SPI（10 MHz） | 2000 Hz | Core 0 ISR |
| BMI088 加速度計 | SPI（10 MHz） | 1600 Hz | Core 0 ISR |
| BMM350 磁力計 | I2C（400 kHz） | 400 Hz | Core 1 任務 |
| BMP580 氣壓計 | SPI（10 MHz） | 50 Hz | Core 1 任務 |
| PMW3901 光流 | SPI（2 MHz） | 100 Hz | Core 1 任務 |
| TFmini-S LiDAR | UART（115200） | 100 Hz | Core 1 UART ISR |
| u-blox GNSS | UART（921600） | 10 Hz | Core 1 UART ISR |
| KVH FOG（如使用） | RS-422 UART | 1000 Hz | Core 0 ISR |
| EKF 預測 | — | 1000 Hz | Core 0（IMU 後） |
| EKF 更新 | — | 10–400 Hz | Core 1 任務 |
| VIO 管線 | USB/CSI 相機 | 30–60 Hz | 伴飛電腦 |
| FAST-LIO2 | USB LiDAR | 10–20 Hz | 伴飛電腦（ROS 2） |

---

*本文件為：RB-RP2354A / Curio 飛控專案*
*用途：中遠程無人機 INS 設計技術參考*
*最後更新：2026 年*
