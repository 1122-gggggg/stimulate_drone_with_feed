# 正式尺寸投料裝置 V6.4 — Gazebo 移轉套件

來源：`Drone_Feeder_V6_3_LandingLegs/cad/Drone_Feeder_V6_4_MasterCAD.step`。
這是正式尺寸的工程開發設計，不代表量產、飛行或材料實測放行。

## 在另一台電腦開啟（建議 Gazebo Harmonic／Ubuntu 22.04 或 24.04）

1. 複製整個 ZIP，解壓縮到任意位置。不要只複製 SDF 或 STL。
2. 在解壓縮後的 `Drone_Feeder_V6_4_Gazebo` 資料夾開啟終端機：

```bash
bash launch.sh
```

按 Gazebo 的播放鍵即可執行場景；也可用 `bash launch.sh -r` 自動播放。
已安裝 Gazebo 的電腦不需要 Fusion、Python、Gmsh、ROS 或網路下載模型。
Gazebo 安裝文件：https://gazebosim.org/docs/harmonic/install_ubuntu/

Windows 建議使用有桌面支援的 WSL2 Ubuntu + Gazebo Harmonic。
在 WSL 終端機切到 ZIP 解壓縮的位置再執行上述命令；`launch.sh` 必須以 Bash 執行。

## 匯入既有 Gazebo 世界

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim /path/to/your_world.sdf
```

在世界 SDF 的 `<world>` 內加入：

```xml
<include>
  <uri>model://drone_feeder_v64</uri>
  <pose>0 0 0.359000000 0 0 0</pose>
</include>
```

或在 Gazebo 資源插入介面選擇模型。原點沿用 CAD；腳底原始 z 約
`-0.358000 m`，上述 pose 使腳底高於地面 1 mm。
坐標為公尺、Z 向上；網格已由 STEP 的毫米轉換，**不要再乘 0.001**。

Gazebo Classic 11 只能使用本套件的模型，不使用含新版系統插件的示範世界：
將 `GAZEBO_MODEL_PATH` 加入解壓縮後的 `models` 絕對路徑，從 Insert 插入模型。
Classic 路徑未列為本套件的實機驗證範圍。

## 內容與模擬範圍

- `models/drone_feeder_v64/`：模型設定、SDF、實際 STEP 表面三角網格。
- `worlds/feeder_demo.sdf`：不依賴 Fuel 或外部模型的地面、燈光、展示場景。
- `source_cad/`：V6.4 STEP、Fusion F3D、現行零件清冊，供後續編輯。
- `manifest.json`：單位、尺寸、網格數、來源與各檔案 SHA-256。

包含料斗、閘門與連桿的當前姿態、扣合／無人機側介面、螺絲與四隻著陸腳；
沒有完整無人機、機器狗或拖車。CAD 清冊列出 275 個 body，
STEP 匯入為 276 個 solid（body 與 solid 計數口徑不同），全部保留。

模型是 **static=true 的剛性幾何**，可作場景配置、尺寸與靜態障礙物碰撞用途。
**不是**可飛行機體，也沒有閘門驅動關節、彈簧腳壓縮、飼料 DEM、出料控制、
韌體／MAVLink 串接或經校正的質量／慣量。現行 STEP 有材料佔位，不能直接
把 CAD 自動質量當實測物性，因此此封裝不憑空建立動態參數。
複雜凹形碰撞網格的接觸結果依物理引擎而異，不能代替機構或落摔分析。

封裝重建：原工作資料夾執行 `python tools/package_gazebo.py`（需 gmsh、numpy）。
