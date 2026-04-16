# ORCA-True-3D

基于 `ref/` 中论文的三维速度障碍（3D VO）避障复现版本。  
当前仓库可完成：

- 编译 OCaml 仿真程序
- 运行三维避障仿真并导出轨迹 CSV
- 生成三维场景可视化图片（`sim_3d.png`）
- 生成算法运行动图（`sim_3d.gif`）

## 1. 依赖

- OCaml 工具链（含 `ocamlfind`、`ocamlopt`）
- OCaml `unix` 包（Makefile 已使用 `-package unix`）
- Python 3
- Python 包：`matplotlib`、`numpy`、`pandas`

## 2. 快速运行

### 2.1 仅运行仿真

```bash
make opt
./orca.opt
```

或指定自定义场景配置：

```bash
./orca.opt path/to/your_scene.conf
```

运行后会在终端打印每 2 秒的状态，并输出：

- `sim_trace.csv`：所有实体的长表轨迹数据
- `sim_metrics.csv`：每个仿真步的总体指标

### 2.2 一键仿真 + 可视化（推荐）

```bash
make viz
```

该命令会自动：

1. 编译 `orca.opt`
2. 运行仿真并生成 `sim_trace.csv`
3. 调用 Python 绘图脚本生成 `sim_3d.png`

### 2.3 一键仿真 + 动图导出

```bash
make anim
```

该命令会自动：

1. 编译 `orca.opt`
2. 运行仿真并生成 `sim_trace.csv`
3. 调用 Python 绘图脚本生成 `sim_3d.gif`

## 3. 手动可视化

如果你已经有 `sim_trace.csv`，可单独画图：

```bash
MPLCONFIGDIR=/tmp/matplotlib python3 viz/plot_3d_avoidance.py \
  --csv sim_trace.csv \
  --out sim_3d.png
```

如果你想导出算法运行 GIF：

```bash
MPLCONFIGDIR=/tmp/matplotlib python3 viz/plot_3d_avoidance.py \
  --csv sim_trace.csv \
  --out sim_3d.gif \
  --fps 15 \
  --stride 2
```

可选参数：

- `--dpi 200`：图分辨率
- `--title "自定义标题"`：图标题
- `--fps 15`：GIF 帧率
- `--stride 2`：抽帧步长，用于减小 GIF 体积
- `--tail 80`：动图中保留的轨迹尾迹长度

## 4. 输出文件说明

- `orca.opt`：仿真可执行文件
- `sim_trace.csv`：仿真轨迹数据
- `sim_3d.png`：三维可视化结果图
- `sim_3d.gif`：算法运行动图

`sim_trace.csv` 采用长表格式，主要字段包括：

- 通用字段：`step,t,kind,id`
- 位姿字段：`x,y,z,vx,vy,vz`
- 几何字段：`radius`
- 目标字段：`goal_x,goal_y,goal_z`
- 指标字段：`d_goal,min_clearance,reached`

其中 `kind` 取值为 `static`、`dynamic`、`agent`。

## 5. 参数修改位置

### 5.1 场景参数

默认配置文件是 `scene.conf`，也可以传入自定义配置路径。

配置文件支持以下条目：

- `sim`：仿真设置，例如 `dt`、`max_steps`、输出路径
- `params`：3D VO / ORCA 参数，例如 `max_speed`、`tp_samples`
- `static`：静态障碍物，支持 `id`、`pos`、`radius`
- `dynamic`：动态障碍物，支持 `id`、`start`、`goal`、`speed`、`radius`
- `agent`：执行 3D ORCA 的个体，支持 `id`、`start`、`goal`、`radius`

增加或删除 `static` / `dynamic` / `agent` 行即可改变数量。

一个最小示例：

```txt
sim dt=0.1 max_steps=600 trace=sim_trace.csv metrics=sim_metrics.csv
params max_speed=0.25 tp_samples=72 phi_steps=11 phi_window_deg=30 vo_margin=0.02
static id=stat_0 pos=0.7,1.1,2.4 radius=0.32
dynamic id=dyn_0 start=-0.8,1.9,1.4 goal=3.0,0.6,1.4 speed=0.12 radius=0.18
agent id=agent_0 start=-2.0,-2.0,1.0 goal=3.4,1.8,2.8 radius=0.22
```

### 5.2 算法参数（3D VO）

`params` 行可配置：

- `tp_samples`：边界离散采样数
- `phi_steps` / `phi_window_deg`：避让平面搜索范围与密度
- `max_speed`：agent 最大速度
- `vo_margin`：VO 边界速度外扩系数

修改后重新运行：

```bash
make viz
```

## 6. 关键文件

- `avoid.ml` / `avoid.mli`：核心避障算法实现
- `main.ml`：仿真入口与多实体调度
- `scene_config.ml`：场景配置解析
- `scene.conf`：默认场景配置
- `viz/plot_3d_avoidance.py`：3D 绘图脚本
- `Makefile`：编译与一键可视化命令
- `ref/`：论文 PDF
