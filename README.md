# ORCA-True-3D

基于 `ref/` 中论文的三维速度障碍（3D VO）+ 球形势场融合避障复现版本。  
当前仓库可完成：

- 编译 OCaml 仿真程序
- 运行三维避障仿真并导出轨迹 CSV
- 生成三维场景可视化图片（`sim_3d.png`）

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

运行后会在终端打印每 2 秒的状态，并输出：

- `sim_trace.csv`：完整轨迹与状态数据

### 2.2 一键仿真 + 可视化（推荐）

```bash
make viz
```

该命令会自动：

1. 编译 `orca.opt`
2. 运行仿真并生成 `sim_trace.csv`
3. 调用 Python 绘图脚本生成 `sim_3d.png`

## 3. 手动可视化

如果你已经有 `sim_trace.csv`，可单独画图：

```bash
MPLCONFIGDIR=/tmp/matplotlib python3 viz/plot_3d_avoidance.py \
  --csv sim_trace.csv \
  --out sim_3d.png
```

可选参数：

- `--dpi 200`：图分辨率
- `--title "自定义标题"`：图标题

## 4. 输出文件说明

- `orca.opt`：仿真可执行文件
- `sim_trace.csv`：仿真轨迹数据
- `sim_3d.png`：三维可视化结果图

`sim_trace.csv` 主要字段包括：

- UAV 位置/速度：`uav_x,uav_y,uav_z,uav_vx,uav_vy,uav_vz`
- 动态障碍位置：`dyn_x,dyn_y,dyn_z`
- 静态障碍中心：`stat_x,stat_y,stat_z`
- 最小距离：`d_min`
- 参数：`rpz,rpf`
- 目标点：`target_x,target_y,target_z`

## 5. 参数修改位置

### 5.1 场景参数（起点、目标、障碍、步长）

修改 `main.ml`：

- `dt`、`max_steps`
- `target`
- `static_obs`
- `dyn0`（动态障碍初始位置和速度）

### 5.2 算法参数（3D VO + 势场）

修改 `avoid.ml` 中 `default_params`：

- `rpz`：保护半径
- `rpf`：势场半径（应大于 `rpz`）
- `kpf`：势场增益
- `tp_samples`：边界离散采样数
- `phi_steps` / `phi_window`：避让平面搜索范围与密度
- `max_speed`：速度上限

修改后重新运行：

```bash
make viz
```

## 6. 关键文件

- `avoid.ml` / `avoid.mli`：核心避障算法实现
- `main.ml`：仿真入口与 CSV 导出
- `viz/plot_3d_avoidance.py`：3D 绘图脚本
- `Makefile`：编译与一键可视化命令
- `ref/`：论文 PDF

