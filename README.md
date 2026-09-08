# myhs2 — HS2 角色姿势模仿与渲染工具链

基于 Blender（headless）的管线：把 BVH 动捕骨架的姿势**重定向（模仿）**到 HS2
角色骨架上，并渲染出「源骨架 / 模仿骨架 / 完整角色模型」三栏对比图，用于
逐帧校验姿势还原质量。

核心方法为**位置域（Key-Point）驱动**：不复制旋转，而是让 HS2 的关节逐帧
去追源骨架关节的世界位置，从根到叶全解析求解（2 段 IK + 方向同构），
天然免疫异构骨架的 rest 朝向差异。

## 工程结构

```
myhs2/
├── README.md                  本文档
├── pose/
│   ├── kp_retarget.py         姿势模仿核心：位置域重定向求解器（可独立运行）
│   ├── kp_trio_render.py      三栏对比渲染：源BVH骨架 | HS2骨架 | HS2模型
│   ├── kp_model_render.py     完整角色（7 部件）单模型渲染
│   └── samples/
│       └── pirouette.bvh      示例动捕数据（CMU 命名骨名）
├── _imgcat.py                 渲染后处理：竖幅栏 → 横幅对比图（Pillow）
├── _gridcat.py                渲染后处理：多帧对比图 → 网格总览图（Pillow）
└── docs/
    └── skeleton.md            HS2 骨架结构与 BVH→HS2 骨骼映射文档
```

## 核心组件

### pose/kp_retarget.py — 位置域姿势模仿

见模块 docstring 与 `docs/skeleton.md`。要点：

- **与旋转域重定向的本质区别**：旋转域把源骨「相对 rest 的旋转」同构复制到
  目标骨，异构骨架 rest 差异在大摆角/高速帧下放大为关节错位（膝内扣、肩扭、
  四肢翻转）；位置域无视 rest 差异，只让关节追位置，段长全部取目标骨架自身
  （自洽），方向取自源骨架（同构），末端解析 2 段 IK 锚定，弯曲侧向
  （膝/肘）用源中间关节相对「根-末端连线」的侧向投影作先验 → 不翻转、不 X 腿。
- **帧内求解顺序**（全解析，无中间 depsgraph update）：
  1. 骨盆 `cf_J_Hips` ← 源骨盆姿态增量（同构，rest 已对齐）
  2. 脊柱 `cf_J_Spine01..03` ← 源脊柱折线弧长映射的切线方向
  3. 头 `cf_J_Head` ← 源「颈顶→头顶」方向
  4. 腿 2 段 IK（髋锚 = 骨盆旋转后髋点；踝目标 = 源踝方向 × 腿长比；膝侧向 = 源膝投影）
  5. 脚 `cf_J_Foot01` ← 源脚骨完整世界旋转增量
  6. 锁骨 `cf_J_Shoulder` ← 源锁骨（Collar）完整世界旋转增量
  7. 臂 2 段 IK（肩锚、肘侧向同腿方案）
  8. 手 ← 源手骨完整世界旋转增量（双向量解）
  9. 手指 ← 源指骨完整世界旋转增量（绝对方向解）
  - 其余骨（dam/roll/_s 辅助骨）保持 rest，由父旋转自然带动。
- **自动对齐**：加载 BVH 后先做源骨架整体旋转对齐（up 轴 + 左右轴 180°
  歧义校验），再做尺度/平移初始对齐（源体高缩放到 HS2 体高、首帧髋中对齐），
  随后逐帧求解并烘焙 rotation keyframes 到 Action。

**API**：`kp_drive(bvh_path, body_arm, action_name=None, fps=30)`
导入 BVH → 对齐 → 逐帧驱动 → 返回 `(action, n_frames, bvh_arm)`。
`kp_trio_render.py` / `kp_model_render.py` 均通过该入口驱动模型。

### pose/kp_trio_render.py — 三栏对比渲染

```
左栏：源 BVH 骨架（绿管）  中栏：HS2 模仿骨架（红管）  右栏：HS2 完整角色
```

- 骨架栏走「骨盆居中系」消除 BVH root 轨迹位移；源侧按 rest 世界垂直跨度
  比缩放（BVH 数据空间为 100m 级未缩放 cm）。
- 三栏各自独立渲染（同场景「每帧改骨架」与「模型渲染」互斥），最后用
  `_imgcat.py` 拼接。
- `--rest` 仅渲染 HS2 rest 骨架+模型两栏（装配自检）；`--headshot` 额外
  渲染头部特写三视图（检查前发装配）。

### pose/kp_model_render.py — 完整角色渲染

装配 HS2 全部 7 部件（body/head/hair_f/hair_b/top/bot/shoe）并渲染指定帧。
装配要点（蒙皮贴合 / 刚体跟随 / 显示系）详见 `docs/skeleton.md`。

### _imgcat.py / _gridcat.py — 渲染后处理（Pillow）

```bash
python3 _imgcat.py <tmp_dir> <out_dir> "1,150,300"       # 竖幅栏→横幅对比图
python3 _gridcat.py <out_dir> <overview.png> "1,150,300"  # 多帧→网格总览
```

## 快速开始

### 依赖

- **Blender ≥ 2.9**（`bpy`、`mathutils`，headless 运行）
- **Python 3 + Pillow**（仅 `_imgcat.py` / `_gridcat.py` 拼接工具需要）

### 资产准备（重要）

本工程**不含** HS2 游戏资产。需要自备以下 FBX（游戏解包件）放入任一目录：

```
<fbx-dir>/body.fbx  head.fbx  hair_f.fbx  hair_b.fbx  top.fbx  bot.fbx  shoe.fbx
```

其中 `body.fbx` 是 184 骨蒙皮骨架（姿势模仿的驱动目标），其余部件按
`docs/skeleton.md` 的装配规则挂接。

### 运行姿势模仿 + 三栏渲染

```bash
cd myhs2

# 1) 三栏渲染（自动导入 fbx + BVH，位置域模仿，输出三栏竖幅图到 /tmp）
blender -b --factory-startup --python pose/kp_trio_render.py -- \
    --fbx-dir /path/to/HoneySelect/Assets/Cosmetic \
    --bvh pose/samples/pirouette.bvh \
    --frames 1,150,300,450,592

# 2) 拼接为横幅对比图（kp_trio_render 结束时会打印该命令）
python3 _imgcat.py /tmp/trio_bars pose/renders "1,150,300,450,592"

# 3) 可选：多帧网格总览
python3 _gridcat.py pose/renders /tmp/overview.png "1,150,300,450,592"
```

### 仅求解（不渲染）

```bash
blender -b --factory-startup --python pose/kp_retarget.py -- \
    --fbx-dir /path/to/fbx --bvh pose/samples/pirouette.bvh
# 输出: RESULT action=<name> frames=<n>（Action 烘焙在 body 骨架上）
```

### 仅渲染完整角色（已含模仿）

```bash
blender -b --factory-startup --python pose/kp_model_render.py -- \
    --fbx-dir /path/to/fbx --bvh pose/samples/pirouette.bvh \
    --frames 1,150,300 --out pose/renders
```

## BVH 兼容性

源骨架骨名采用 **CMU 命名**（`hip / lThigh / lShin / lFoot / abdomen / chest /
neck / head / lCollar / lShldr / lForeArm / lHand / lIndex1..` 等），
完整映射表见 `docs/skeleton.md`。Mixamo 等其他命名需先改 `kp_retarget.py`
中的 `SRC_*` 常量表。

## 已知设计细节

- **趾骨平放补偿**：HS2 靴鞋底前 1/4 段 rocker 上翘（rest 标定 11.7mm），
  源 BVH 无趾骨 → `cf_J_Toes01` 恒 rest 会致站姿鞋尖翘起，常量下压
  pitch 8.4°（`TOES_FLAT_PITCH`）。
- **蒙皮贴合公式**：`mesh.matrix_world = arm.matrix_world @ Rx(-90°) @ S(0.01)`
  （body/top/bot/shoe 骨架数据空间与网格相差 Rx(-90°)，mesh 顶点为 cm）。
- **头/发部件**：顶点组是面骨/发骨（与 body 184 骨架 0 交集），不能 rebind，
  走对象级刚体跟随 `cf_J_Head_s`（delta = B_t @ B_ref⁻¹）。
- 模型右栏平移必须并入 armature 对象矩阵而非 mesh 矩阵（否则 armature
  modifier 空间变夹心相似变换 → 蒙皮畸变）。

详细骨架结构与全部骨名映射：**[docs/skeleton.md](docs/skeleton.md)**
