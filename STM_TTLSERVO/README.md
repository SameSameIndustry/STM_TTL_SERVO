# STM32G431 BLDC Driver

STM32G431CB + EG2104 ハーフブリッジドライバによる 3 相 BLDC モータ制御実装です。
AS5600 磁気エンコーダによるクローズドループ PD 位置制御と、強制転流（センサレスオープンループ）の両方の API を備えます。現在の `main.c` はクローズドループ位置制御（電源投入時の位置から 90° の目標角へ）を行います。

---

## ハードウェア構成

| 信号     | MCU ピン | 接続先              | 役割                        |
|----------|----------|---------------------|-----------------------------|
| IN1      | PA6      | EG2104 U4 IN        | Phase A PWM 入力            |
| IN2      | PA7      | EG2104 U5 IN        | Phase B PWM 入力            |
| IN3      | PB0      | EG2104 U6 IN        | Phase C PWM 入力            |
| SD1      | PA0      | EG2104 U4 SD        | Phase A 有効化              |
| SD2      | PA1      | EG2104 U5 SD        | Phase B 有効化              |
| SD3      | PA2      | EG2104 U6 SD        | Phase C 有効化              |
| LED (黄) | PB3      | D6                  | 動作表示（ハートビート）     |

### EG2104 動作

| SD  | IN  | HO (上アーム) | LO (下アーム) |
|-----|-----|---------------|---------------|
| H   | PWM | PWM           | 相補 PWM      |
| H   | L   | OFF           | ON（シンク）  |
| L   | any | OFF           | OFF（フロート）|

---

## VSCode でのビルド・書き込み・デバッグ

リポジトリルート（`STM_BLDC_Driver_cheaper`）を VSCode のワークスペースとして開くと、`.vscode/` 以下の設定でビルド・書き込み・デバッグができます。

### 前提

- CMake Tools 拡張機能、cortex-debug 拡張機能をインストール済みであること
- ARM GNU Toolchain（`arm-none-eabi-gcc`）がインストールされていること
- OpenOCD（xPack 版）がインストールされ、パスが `.vscode/settings.json` の `cortex-debug.openocdPath` と一致していること（別環境の場合はここを調整）
- ST-Link がボードに USB 接続されていること

### ビルド

- `Ctrl+Shift+B` でデフォルトタスク「**Build Debug**」を実行
  （中身は `cmake --build STM_core/build/Debug --config Debug -j 4`）
- または `Ctrl+Shift+P` → `Tasks: Run Task` → **Build Debug**

### 書き込み（フラッシュ）

- `Ctrl+Shift+P` → `Tasks: Run Task` → **Flash**
  - 依存タスクとして自動的に「Build Debug」が先に実行される
  - OpenOCD + ST-Link 経由で `STM_core/build/Debug/STM_core.elf` を書き込む
    （`openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program ... verify reset exit"`）

### デバッグ実行

- デバッグパネル（虫アイコン）から「**Debug (OpenOCD)**」を選択して `F5`
  - ビルド → 書き込み → `main` にブレークして停止
  - ブレークポイントを置いて `error_deg` / `duty` などの変数をウォッチしながら PD 制御の挙動を確認できる

---

## ソフトウェア構成

```
Core/
├── Inc/
│   └── bldc.h      ← モータパラメータ・API 宣言
└── Src/
    ├── bldc.c      ← 転流ロジック本体
    └── main.c      ← 初期化・メインループ
```

---

## モータパラメータの設定方法

`Core/Inc/bldc.h` の上部にあるマクロを変更します。

```c
#define MOTOR_KV                    100    // KV 値 [RPM/V]
#define MOTOR_POLE_PAIRS            7      // 極対数 (極数 / 2)
#define MOTOR_RESISTANCE_MOHM       11200  // 相抵抗 [mΩ]
#define MOTOR_MAX_VOLTAGE_MV        14800  // 最大電圧 [mV]
#define MOTOR_MAX_CURRENT_MA        5000   // 最大電流 [mA]
#define MOTOR_MAX_RPM               2180   // 最大回転数 [RPM]
```

### 各パラメータの調べ方

| パラメータ          | 調べ方                                                  |
|---------------------|---------------------------------------------------------|
| `MOTOR_KV`          | データシートまたは `no-load RPM / 電圧`                  |
| `MOTOR_POLE_PAIRS`  | モータ内部の磁石数を数えて 2 で割る                       |
| `MOTOR_RESISTANCE_MOHM` | テスターで相間抵抗を測定し mΩ 単位に変換             |
| `MOTOR_MAX_VOLTAGE_MV`  | データシートの最大電圧を mV 単位に変換                |
| `MOTOR_MAX_CURRENT_MA`  | データシートの最大電流を mA 単位に変換                |
| `MOTOR_MAX_RPM`     | データシートまたは `KV × 最大電圧`                       |

---

## 起動パラメータの設定方法

```c
#define BLDC_STARTUP_DUTY           800U   // 起動デューティ (0 ～ BLDC_PWM_PERIOD)
#define BLDC_STARTUP_STEP_PERIOD_US 20000U // 起動時ステップ周期 [µs]
#define BLDC_TARGET_RPM             300U   // ランプ後の目標回転数 [RPM]
#define BLDC_RAMP_CYCLES            50U    // ランプの電気角サイクル数
```

### デューティ比の計算

起動時の印加電圧が低すぎるとモータが動かず、高すぎると突入電流が流れます。

```
推奨起動デューティ = BLDC_PWM_PERIOD × (目標電圧 / 電源電圧)
目標電圧 [V]      = 目標RPM / KV + 推定電流[A] × 相抵抗[Ω]
```

例（本モータ、300 RPM、電流 0.3A 想定）：
```
目標電圧 = 300/100 + 0.3×11.2 = 3.0 + 3.36 = 6.36 V
デューティ = 3999 × (6.36 / 11.1) ≈ 2291
```

`BLDC_StartupRamp()` はこの起動デューティを**最初から**印加し、電気周波数（回転速度）のみを徐々に上げます。振幅を速度と一緒にゆっくり上げると、ランプの終盤までトルクが不足し実質的に動かない期間が長くなる（減速機付きなど負荷が大きい場合に顕著）ため、あえて振幅は最初から固定にしています。

### ステップ周期と回転数の関係

```
step_period_us = 60,000,000 / (RPM × 極対数 × 6)
```

| 目標 RPM | ステップ周期 (7 極対数) |
|----------|------------------------|
| 70       | 20,408 µs              |
| 100      | 14,286 µs              |
| 300      |  4,762 µs              |
| 500      |  2,857 µs              |
| 1000     |  1,429 µs              |
| 2180     |    654 µs              |

### ランプ時間の計算

```
ランプ時間[ms] ≈ BLDC_RAMP_CYCLES × 6 × 起動ステップ周期[µs] / 2 / 1000
```

例（デフォルト設定）：
```
50 × 6 × 20000 / 2 / 1000 ≈ 3,000 ms（約 3 秒）
```

---

## 6 ステップ転流テーブル

| ステップ | Phase A | Phase B | Phase C | 電流経路 |
|----------|---------|---------|---------|----------|
| 0        | PWM     | SINK    | FLOAT   | A → B   |
| 1        | PWM     | FLOAT   | SINK    | A → C   |
| 2        | FLOAT   | PWM     | SINK    | B → C   |
| 3        | SINK    | PWM     | FLOAT   | B → A   |
| 4        | SINK    | FLOAT   | PWM     | C → A   |
| 5        | FLOAT   | SINK    | PWM     | C → B   |

逆転させる場合はステップ順を逆にします（5→4→3→2→1→0）。

---

## API リファレンス

```c
// 初期化（MX_TIM3_Init() の後に呼ぶ）
void BLDC_Init(void);

// 即時停止（全ゲートドライバ無効化）
void BLDC_Stop(void);

// デューティ比設定（0 ～ BLDC_PWM_PERIOD）
void BLDC_SetDuty(uint32_t duty);

// 単一ステップを適用（0 ～ 5）
void BLDC_CommutateStep(uint8_t step);

// 固定周期で連続転流（num_cycles=0 で無限）
void BLDC_Run(uint32_t step_period_us, uint32_t num_cycles);

// 起動ランプ実行（静止→BLDC_TARGET_RPM）
void BLDC_StartupRamp(void);

// RPM → ステップ周期[µs] 変換
uint32_t BLDC_RPMToStepPeriodUs(uint32_t rpm);

// 絶対電気角で即座に転流（クローズドループ制御用）
void BLDC_Commutate(uint8_t elec_angle, uint32_t duty);
```

---

## クローズドループ位置制御（速度カスケード）

`main.c` は AS5600 のフィードバックを使い、電源投入時の位置から 90° ずれた位置へシャフトを追従させます。電流センサは無いため、直接トルク（デューティ）を位置誤差に比例させる方式ではなく、**位置誤差を「転流フィールドの回転速度」の指令値としてマッピングし、デューティは常に実績のある固定値を使う「速度カスケード」方式**を採用しています。

### なぜこの方式か

デューティを誤差に比例させる方式（PID で直接トルクを出す方式）を試したところ、誤差が小さくなるとデューティも小さくなり、この減速機付きの負荷では静止摩擦に負けて動けなくなる領域がありました。一方、`BLDC_StartupRamp()` のような**一定レートで電気角を進め続けるだけのオープンループ**は確実に回転することが確認できていました。そこで、この「確実に回るオープンループ転流」の仕組みをそのまま流用し、**転流角を進める速度だけを位置誤差に応じて可変にする**ことで、トルク不足の問題を構造的に回避しています。ロータの実電気角をエンコーダから逆算する必要が無くなるため、電気角アライメント（`mech_offset`）も不要になりました。

速度の決め方は当初「誤差に比例（P制御）」でしたが、目標近くでの減速カーブが物理的な最適減速になっておらず、必要以上に遅くなっていたため、現在は**台形（残り距離から逆算した物理的な減速曲線）速度プロファイル**に変更しています。

### 動作の流れ

1. **初期位置の記録** — モータを一切駆動する前に `AS5600_ReadAngle()` で読んだ角度を基準とし、目標角 = 初期角 + 90°（`TARGET_OFFSET_COUNTS` = 1024 カウント）とする。
2. **開始位置への整定** — `BLDC_Commutate(0, BLDC_ALIGN_DUTY)` で電気角 0° を保持し、`BLDC_ALIGN_TIME_MS` 待ってロータを既知の開始位置に整定させる。
3. **方向キャリブレーション** — 電気角を `BLDC_DIR_CAL_ELEC_STEP`（45°）だけ進めて再度整定を待ち、エンコーダの値が増えたか減ったかを見て `elec_dir_sign`（+1/-1）を決定する。モータの巻線順やエンコーダの取り付け方向によっては「電気角が増える向き」と「エンコーダの値が増える向き」が逆になり得るため、以降のフィールド前進方向はすべてこの符号で補正する。
4. **速度カスケード制御ループ（台形プロファイル）**（`BLDC_POS_LOOP_PERIOD_MS` 周期）
   - 現在角度を読み、目標角との誤差を最短経路（0°/360° 境界をまたぐ場合も考慮）で計算
   - **望ましい速度** = `sqrt(2 × BLDC_POS_MAX_ACCEL_DEG_S2 × 残り距離)` を `±BLDC_POS_MAX_SPEED_DEG_S` でクランプ — これは「今から最大加速度で減速したらちょうど目標でゼロ速度になる」速度なので、目標から遠ければ最大速度（巡航）、近づけば運動方程式 `v² = 2ad` に従って自然に減速し、オーバーシュート無く収束する
   - **加速度リミッタ**: 実際に使う `commanded_speed_deg_s` は、望ましい速度に向かって `BLDC_POS_MAX_ACCEL_DEG_S2` を上限になめらかに近づける（スルーレート制限）。これは主に加速側（静止状態からいきなり高速指令が飛ぶのを防ぐ）で効き、減速側はもともと `sqrt` の式自体が最大加速度ちょうどの減速カーブになっている
   - 自由回転する仮想フィールド角 `field_elec_deg` を `elec_dir_sign × commanded_speed_deg_s × dt × 極対数` だけ毎周期進める（0°〜360° でラップ）
   - `BLDC_Commutate(field_elec_deg を 0-255 に変換した値, BLDC_POS_RUN_DUTY)` を呼ぶ — **デューティは常に固定**、可変なのは速度のみ
   - 目標角の ±5°（`POS_TOLERANCE_COUNTS`）以内で LED (PB3) を点灯（残り距離 0 では望ましい速度も 0 になるので、そのままフィールドが止まり、固定デューティのままステッピングモータ的な保持トルクがかかる）

### チューニング（`bldc.h`）

```c
#define BLDC_ALIGN_DUTY             BLDC_STARTUP_DUTY  // 整定・方向キャリブレーション時の保持デューティ
#define BLDC_ALIGN_TIME_MS          500U               // 整定待ち時間 [ms]
#define BLDC_POS_LOOP_PERIOD_MS     1U                 // 制御周期 [ms]（速度ループの dt）
#define BLDC_POS_RUN_DUTY           BLDC_STARTUP_DUTY  // 走行中の固定デューティ
#define BLDC_POS_MAX_SPEED_DEG_S    (BLDC_TARGET_RPM × 6.0f)  // 最大指令速度 [°/s]（＝巡航速度の上限。BLDC_StartupRamp で実績のある回転数）
#define BLDC_POS_MAX_ACCEL_DEG_S2   320.0f              // 加速度上限 [°/s²]（独立した値。MAX_SPEED からは算出しない）
```

`BLDC_POS_MAX_SPEED_DEG_S` と `BLDC_POS_MAX_ACCEL_DEG_S2` は意図的に**別々の値**にしています。以前は「最大速度 ÷ 到達時間」で加速度を算出していましたが、この結び付けだと最大速度を上げるたびに加速度も比例して跳ね上がり、脱調が再発しました。そのため、最大速度は「実際に確認できている巡航速度（`BLDC_TARGET_RPM`）」を素直に設定し、加速度は「脱調せずに加速できると確認済みの値」に固定しています。

- **`BLDC_POS_MAX_SPEED_DEG_S`（＝`BLDC_TARGET_RPM`）を上げる** → 巡航速度の上限が上がる。加速度は変わらないので脱調のリスクは増えない。ただし移動距離が短いと、この上限に達する前に台形の減速区間に入ってしまい（三角形プロファイル）、90° のような短い移動では効果が出ないことがある点に注意（`BLDC_TARGET_RPM` を上げる場合は `BLDC_StartupRamp()` 側の起動ランプでその回転数まで確実に追従できることも別途確認すること）。
- **`BLDC_POS_MAX_ACCEL_DEG_S2` を上げる** → 加速・減速とも機敏になり、90° のような短い移動でも最高速度まで到達しやすくなる。**「遅い」と感じたら、こちらを上げる方が効果的**。上げすぎるとロータが追従できず脱調するので、脱調するようなら下げる。
- **`BLDC_POS_RUN_DUTY` を上げる** → トルクに余裕ができるが、`BLDC_PWM_DUTY_MAX` を超えないよう注意。
- アライメント・方向キャリブレーション時にロータが期待通り動かない場合は `BLDC_ALIGN_DUTY` を上げる、または `BLDC_ALIGN_TIME_MS` を延ばす。
- 方向キャリブレーション（`BLDC_DIR_CAL_ELEC_STEP`）のステップが小さすぎてエンコーダの変化がノイズに埋もれる場合は値を大きくする（90° 電気角 = 64 まで上げても問題ない）。

---

## 注意事項

- **強制転流（`BLDC_Run` / `BLDC_StartupRamp` / `BLDC_CommutateStep`、および位置制御ループ内の速度カスケード転流）はいずれもオープンループ転流**です。指令速度が速すぎる、または急激すぎる場合は脱調します（負荷過大・速度急変）。
- `BLDC_STARTUP_STEP_PERIOD_US` が短すぎると起動時に脱調します。まず長めの値から試してください。`BLDC_POS_MAX_SPEED_DEG_S` はこの値から自動算出される安全速度なので、通常は変更不要です。
- SD ピンのプルアップは +5V（R3/R4/R5）ですが MCU 出力は 3.3V です。EG2104 の VIH (≈ 2.4V) を満たしているため動作しますが、ノイズマージンは低めです。
- `BLDC_Run()` および位置制御ループは割り込みを使わないビジーウェイトで動作します。他の処理と並行させる場合はタイマ割り込みへの移行を検討してください。
- 位置制御ループの速度→電気角変換はモータの極対数（`MOTOR_POLE_PAIRS`）が正しく設定されていることが前提です。誤っていると指令速度と実際の回転速度がずれます。
