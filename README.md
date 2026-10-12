# stimulate_drone_with_feed

Gazebo Harmonic + ArduPilot SITL 模擬：Anji UA-1400H1-C-S 級大型四旋翼（約 27 kg），掛投料機（5 kg 飼料，可在飛行中放料），場域為**澎湖西嶼鄉外海**。風來自三個來源：NCDR 西嶼測站觀測、ERA5 統計、OpenFOAM 算出的空間風場。用 Python 控制飛行。

## 內容

| 路徑 | 用途 |
|---|---|
| `generator/gen_bigquad.py` | 產生機體 / 世界檔：質量、慣量、旋翼、風阻、投料機（`Drone_Feeder_V6_4_Gazebo` 的 CAD 網格掛在機腹，質量用方塊近似；飼料拆成 20 塊可分離方塊）、障礙物 |
| `generator/windfeed.py` | 即時風場送進 Gazebo：對數風剖面＋OU 紊流；可接 ERA5、NCDR 觀測、OpenFOAM 風場 |
| `generator/bigquad.parm` | ArduPilot 參數（可載入 Mission Planner） |
| `scripts/control.py` | Python 控制 `Drone` 類別：`arm / takeoff / goto / velocity / land / dispense` |
| `scripts/feed_demo.py` | 範例：起飛、邊飛邊放料、返航降落 |
| `scripts/xiyu.sh` | 一鍵啟動：投料機＋障礙物＋西嶼風（NCDR＋CFD） |
| `generator/gen_twin.py` `scripts/twin.sh` `scripts/mirror.py` | **數位孿生**：把真實無人機的 MAVLink 遙測鏡射進 Gazebo（見下節） |
| `Drone_Feeder_V6_4_Gazebo/` | 正式尺寸投料裝置 V6.4 的 CAD 網格與 STEP（靜態剛體，無質量／關節資料） |
| `scripts/serve.sh` `run_sim.sh` `run_gui.sh` `pen.sh` | 較底層的啟動腳本（原有） |
| `wind_cfd/` | OpenFOAM 11 風場流程（`site.json` → `run_cfd.py` → `postprocess.py`） |
| `wind_data/` | `penghu_era5.json`（ERA5）、`xiyu_ncdr.json`（NCDR 快照）、`xiyu_d022.npz`（CFD 風場，已算好） |
| `generated/` | 產生出來的 model/world 備份（執行時會重新產生） |

## 環境

- Windows 10/11 + WSL2 Ubuntu-22.04，或原生 Ubuntu 22.04。
- 在 Ubuntu 內安裝一次（約 30–60 分鐘）：
  ```
  bash scripts/install_all.sh      # Gazebo Harmonic、ArduPilot 依賴
  bash scripts/setup_ap.sh         # 編譯 ArduCopter SITL 與 ardupilot_gazebo 外掛
  ```
  需要 `~/simulators/ardupilot`、`~/simulators/ardupilot_gazebo`（上面腳本會建立）。
- Python（Ubuntu 內）：`pymavlink`、`numpy`、gz 的 python 綁定（`gz-harmonic` 內含）。
- 跑 CFD 才需要：Docker（映像檔 `openfoam/openfoam11-paraview510`，約 4 GB）。

## 執行

```bash
# 1. 啟動模擬（投料機 + 障礙物 + 西嶼風：NCDR 近 48 小時最大風 11.4 m/s、陣風 17.8 m/s）
GUI=1 bash scripts/xiyu.sh max_48h        # 不加 GUI=1 則為無頭模式；窗口可選 max_1h … max_48h

# 2. 另開終端機，用 Python 飛
python3 scripts/control.py                # 起飛 20 m、方形航線、降落
python3 scripts/feed_demo.py              # 邊飛邊放料（預設 0.25 kg/s，5 kg 約 20 秒放完）
```

在你自己的程式裡：

```python
from control import Drone
d = Drone()                    # tcp:127.0.0.1:5760
d.wait_ready(); d.arm(); d.takeoff(20)
d.goto(60, 0, 20)              # north, east, up（相對起飛點，m）
d.dispense(0.25)               # 放完所有料；dispense(0.25, kg=2, wait=False) 在背景放 2 kg
d.release_feed(1.0)            # 立刻丟 1 kg
d.land()
```

SITL 的三個 TCP 埠各只能一個連線：5760 給 Python、5762 給 `windfeed.py`（讀位置取樣風場）、5763 留給 Mission Planner（`tcp:127.0.0.1:5763`）。

## 連接真實無人機（數位孿生）

真機動、Gazebo 內的機體跟著動。真機由自己的飛控飛，模擬端**只讀遙測，不送任何指令**（除非加 `--request-rates`）。

```bash
GUI=1 bash scripts/twin.sh                                 # 起孿生世界：沒有 SITL、沒有機體物理，機體是靜態模型
python3 scripts/mirror.py --conn udpin:0.0.0.0:14550       # 讀遙測，30 Hz 設定機體姿態
```

`mirror.py` 讀 `LOCAL_POSITION_NED`（位置、速度）與 `ATTITUDE`，換成 Gazebo 的 ENU/FLU 座標後呼叫 `/world/bigquad_twin/set_pose`。遙測封包間用速度外推位置（最多 0.5 s）。

遙測來源（擇一）：

| 方式 | `--conn` |
|---|---|
| Mission Planner（Windows）→ `Ctrl+F` → MAVLink → 轉發 UDP 到 WSL 的 IP:14550 | `udpin:0.0.0.0:14550` |
| MAVProxy：`mavproxy.py --master=COMx,57600 --out=udp:<WSL IP>:14550` | `udpin:0.0.0.0:14550` |
| 數傳／USB 直接接到 Linux（WSL2 需 `usbipd` 把 USB 掛進來） | `/dev/ttyUSB0 --baud 57600` |
| 飛控 / 地面站的 TCP | `tcp:<ip>:5760` |

WSL2 找 Windows 的 IP：`ip route show default | awk '{print $3}'`（用 mirrored networking 則可直接用 127.0.0.1）。

注意：
- 原點是真機 EKF 原點（解鎖／GPS 鎖定位置）；要放到地圖別處用 `--offset-n / --offset-e`。
- 飛控要有位置估計（GPS、光流或動捕）才有 `LOCAL_POSITION_NED`；遙測速率太低會顯得一格一格的，可在飛控設 `SRn_POSITION` / `SRn_EXTRA1`，或加 `--request-rates`。
- 孿生只呈現姿態與位置，不模擬風、馬達與投料；投料動作不會鏡射。
- 沒有真機時可以拿 SITL 當替身測試：`sim_vehicle.py -v ArduCopter -f quad --no-mavproxy`，再 `--conn tcp:127.0.0.1:5762`（先讓別的程式連 5760 後 5762 才會開）。

## 風場

| 來源 | 內容 |
|---|---|
| NCDR | `wind_data/ncdr_fetch.py` 抓 https://watch.ncdr.nat.gov.tw/watch_page_wind 背後資料（西嶼站 `C0W120`），存成 `xiyu_ncdr.json`，含近 1/3/6/12/24/48 小時最大平均風與陣風。平均風當 10 m 風速，陣風校正紊流強度（陣風 = 平均 + 3σ）。該資料**沒有風向**。 |
| ERA5 | `penghu_era5.json`，冬季（10–3 月）p50 / p90 / p99 風速與風向（22° / 24° / 26°，即東北季風） |
| OpenFOAM | 穩態 RANS（k-ε）、中性海上大氣邊界層（海面 z0 = 2e-4 m）、`wind_cfd/site.json` 裡的障礙物。風向 22°（NNE）；風向差 10° 內 `windfeed.py` 會沿用，所以涵蓋 p50/p90/p99。 |

`windfeed.py --field` 依無人機位置（MAVLink）在 CFD 風場內插平均風與紊流強度。風速與 k 對風速成線性/平方比例，所以一個風向只需算一次。

更新 NCDR 快照：`python wind_data/ncdr_fetch.py`（需要網路，`--station` 可換站）。

### 重新算 CFD

```bash
python wind_cfd/run_cfd.py --dir 22                       # 網格 + 求解（本機 8 核約 3 分鐘，約 176k 格）
python3 wind_cfd/postprocess.py wind_cfd/cases/d022       # 轉成 wind_data/xiyu_d022.npz（需 numpy）
```

`wind_cfd/site.json` 的障礙物是**範例**（15×15×12 m 平台，在起飛點東 40 m、北 70 m），請改成實際位置；Gazebo 世界檔和 CFD 都讀同一份。改了障礙物要重算 CFD。

## 驗證過的結果（開發機：WSL2，CPU 軟體渲染）

- SITL 由 Python 解鎖、起飛、定點、降落正常。
- 20 塊飼料依序分離落地；滿倉 → 放空後懸停馬達 PWM 1704 → 1656。
- CFD：自由流 10 m 高 9.99 m/s（設定 10）、22 m 高 10.7 m/s（對數律 1.073 倍）、未受擾動 k = 0.48（理論 0.479）；障礙物下游 10 m 高風速降到約 4 m/s、k 升到約 4。
- 全套（NCDR 風 + CFD 風場 + 投料機 + 障礙物）可從起飛飛到降落；風速隨位置與高度變化。

## 適合在大算力電腦（RTX 5060 / 5090）上做的事

GPU 對 OpenFOAM 沒有幫助（求解器是 CPU），OpenFOAM 要看核心數；GPU 的用處是 Gazebo 畫面與相機感測器。

- **Gazebo 畫面 / 相機**：開發機是軟體渲染（`LIBGL_ALWAYS_SOFTWARE=1`），畫面慢。GPU 電腦上用 `GUI=1 LIBGL_ALWAYS_SOFTWARE=0 bash scripts/xiyu.sh`（需新版 NVIDIA 驅動；WSL2 用 WSLg）。
- **更細的 CFD 網格**：`python wind_cfd/run_cfd.py --dir 22 --cell 5 --level 4 --np <核心數>`（網格約 8 倍，記憶體與時間大增）。
- **其他風向 / 季節**：`--dir <風從哪來的度數>`，再 `postprocess.py`；`windfeed.py --field` 指到新檔。夏季西南風沒有算。
- **真實地形或海岸**：目前是平坦海面加方塊障礙物，沒有西嶼島 DEM。
- 長時間批次飛行（多風況、多放料速率）。

## 已知限制

- NCDR 資料沒有風向，模擬用 CFD 的 22°（和 ERA5 冬季強風同向，也和該站風場圖大致相符，但不是逐筆觀測）。
- RANS 的 k 只代表解析尺度的紊流，所以用觀測陣風做了校正（`windfeed.py` 啟動時會印出倍率）；這是工程近似。
- 風用 Gazebo 的 WindEffects（每個 link 受力近似），不是真正的氣動力計算。
- 放料是以 0.25 kg 為單位的離散釋放；料倉內料是剛體方塊，不會流動。
