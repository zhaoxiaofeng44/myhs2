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
│       ├── pirouette.bvh      示例动捕数据（CMU/Daz 命名骨名）
│       ├── 05_02__dance-…bvh   舞蹈（AMASS 命名）
│       ├── 09_12__navigate-walk-…bvh  前进/后退/侧向走
│       ├── 10_01__soccer-kick-ball__120fps.bvh  踢球
│       ├── 90_02__cartwheel__120fps.bvh         侧手翻
│       └── 104_10__jogstop__120fps.bvh          慢跑急停
├── character/                 捏人（骨骼增量）模块
│   ├── shape_data.py          HoneySelect 捏人数据表解析与评估
│   ├── shape_apply.py         将骨骼增量应用到 Blender Armature pose
│   ├── category_names.py      分类中文名参考
│   └── sliders.example.json   滑杆配置示例
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
- 朝向基准三栏统一：骨盆居中系末步为「逆旋首帧(f1)骨盆 → `R_SHOW`」，即把
  f1 朝向当正面；模型栏同样左乘这个复合旋转 `R_SHOW @ q_f1⁻¹`，否则右栏会带上
  源 f1 的世界转身（样本实测 0°~156° 不等）而与骨架栏"站立角度"对不上。
  只乘 `q_f1⁻¹` 会把角色上轴转到 +y（模型躺倒）；`--rest` 下该乘子≈单位阵。
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

### 捏人（角色定制）

本工程支持基于 HoneySelect 原作的捏人数据表对角色形象进行骨骼级定制。
需要额外提供 `HoneySelect/Assets/Data/` 下的数据表（`cf_customhead.txt`、
`cf_custombody.txt`、`cf_anmShapeHead.txt`、`cf_anmShapeBody.txt`、
`face_bone_mappings.csv`、`bone_mappings.csv`）。

```bash
# 1) 复制示例并编辑滑杆值（0..1，0.5=默认中性）
cp character/sliders.example.json my_shape.json
# ... 修改 face / body 下的分类值

# 2) 渲染时指定 --shape-json 与 --data-dir
blender -b --factory-startup --python pose/kp_model_render.py -- \
    --fbx-dir /path/to/HoneySelect/Assets/Cosmetic \
    --data-dir /path/to/HoneySelect/Assets/Data \
    --bvh pose/samples/pirouette.bvh \
    --shape-json my_shape.json \
    --frames 1,150,300 --out pose/renders
```

`--shape-json` 同样适用于 `kp_trio_render.py`。分类 ID 的中文语义参考
`character/category_names.py`。

## BVH 兼容性

源骨架骨名自动识别三种命名变体（`_detect_src_variant`），且 `--bvh`
参数同时接受 **BVH 与 Mixamo FBX**（按扩展名分流导入，FBX 动画在
Hips 骨上、直接复用同款管线）：

- **CMU/Daz 命名**（`hip / lThigh / lShin / lFoot / abdomen / chest / neck /
  head / lCollar / lShldr / lForeArm / lHand / lIndex1..` 等），完整映射表见
  `docs/skeleton.md`。
- **AMASS 命名**（`root / lowerback / upperback / thorax / lowerneck /
  upperneck / head / lfemur / ltibia / lfoot / ltoes / lclavicle /
  lhumerus / lradius / lwrist`），如
  [metrixel-cmu-mocap-clean](https://huggingface.co/datasets/EntVista/metrixel-cmu-mocap-clean)
  清洗版 CMU 动作库（120fps、去抖、foot-locked）。该变体无手/指骨，
  手由腕骨世界旋转兜底驱动、手指保持 rest。
- **Mixamo 命名**（`mixamorig:` 前缀，BVH 导出与 FBX 导入同名）：
  `Hips / Spine / Spine1 / Spine2 / Neck / Head / LeftShoulder / LeftArm /
  LeftForeArm / LeftHand / LeftUpLeg / LeftLeg / LeftFoot / LeftToeBase` +
  手指链（Thumb/Index/Middle/Ring/Pinky 各 4 节，取前两节）。FBX 导入自带
  轴系换算（matrix_world 含 Rx90° + 0.01 缩放），朝向对齐用左乘合成保留该
  换算（整体替换会翻转源骨架 180°）；单位缩放由后续整体 S 对齐吸收。

其余命名需在 `kp_retarget.py` 的 `SRC_VARIANTS` 中新增条目。

### 样本（pose/samples/）

- **metrixel CMU 清洗版**（AMASS 命名，120fps）：`jogstop`、`cartwheel`、
  `soccer-kick-ball`、`walk`、`dance` 等 5 段，见
  [metrixel-cmu-mocap-clean](https://huggingface.co/datasets/EntVista/metrixel-cmu-mocap-clean)。
- **Mixamo FBX**（30fps）：`Walking.fbx`（43f）、`Running.fbx`（77f，实为
  奔跑中摔倒起身动作）、`Macarena_Dance.fbx`（248f）、`Cartwheel.fbx`（107f）、
  `Kick_Soccerball.fbx`（18f）。
- **Mixamo BVH**：`ZombieKicking_mixamo.bvh`（250f，全关节 6 通道）。

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
