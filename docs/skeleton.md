# HS2 骨架结构与骨骼映射

本文档描述 HS2 角色骨架的结构、姿势模仿所用的**驱动骨集合**、源 BVH 骨名
映射，以及多部件角色的装配规则。对应实现：`pose/kp_retarget.py`（常量表）、
`pose/kp_trio_render.py`（语义点表 PTS）、`pose/kp_model_render.py`（装配）。

## 1. 角色组成

HS2 角色由 7 个独立 FBX 部件组成：

| 部件 | 内容 | 骨架 | 装配方式 |
|---|---|---|---|
| `body` | 身体网格 | **184 骨蒙皮骨架**（驱动目标） | 蒙皮 |
| `top` / `bot` / `shoe` | 上衣/下装/鞋 | 各自小骨架 | **rebind 到 body 骨架** |
| `head` / `hair_f` / `hair_b` | 头/前发/后发 | 面骨、发骨（与 body 184 骨 **0 交集**） | **对象级刚体跟随** `cf_J_Head_s` |

- 蒙皮组（`body/top/bot/shoe`）：top/bot/shoe 的 armature modifier 目标
  统一改指 body 骨架，蒙皮权重按顶点组名匹配。
- 头/发组：顶点组是面骨/发骨，body 骨架上无同名骨，**不能 rebind**；
  逐帧做对象级刚体跟随 `cf_J_Head_s`：`delta = B_t @ B_ref⁻¹`，
  部件总变换 `H_t = delta @ H_rest`。

## 2. 姿势驱动骨与源骨架映射

源骨架为 BVH 动捕（**CMU 命名**）。位置域求解器每帧驱动下表 HS2 骨。

### 2.1 中轴线（骨盆 → 头）

| 源骨（CMU） | HS2 驱动骨 | 求解方式 |
|---|---|---|
| `hip` | `cf_J_Hips` | 源骨盆**相对自身 rest 的姿态增量**同构复制（两侧 rest 已整体对齐，无 180° 翻转歧义） |
| `abdomen`→`neck` 折线 | `cf_J_Spine01` `cf_J_Spine02` `cf_J_Spine03` `cf_J_Neck` | 源脊柱折线（`abdomen/chest/neck/head` 采样点）按**弧长映射**取切线方向（含低头/弯腰） |
| `head` | `cf_J_Head` | 源「颈顶→头顶」方向 |

- 源脊柱采样点：`SRC_SPINE_PTS = [abdomen, chest, neck, head]`（髋中起算）。
- 骨盆髋锚偏移：`cf_J_LegUp00_*` head 相对 `cf_J_Hips` head 的 rest 偏移
  存在骨盆系 local（`hip_off`），随骨盆旋转。

### 2.2 腿链（每侧 2 段 IK + 脚/趾）

| 源骨 | HS2 驱动骨 | 求解方式 |
|---|---|---|
| `lThigh` / `rThigh` | `cf_J_LegUp00_L/R` | 解析 2 段 IK |
| `lShin` / `rShin` | `cf_J_LegLow01_L/R` | 解析 2 段 IK |
| `lFoot` / `rFoot` | `cf_J_Foot01_L/R` | 源脚骨**完整世界旋转增量**（f1 基准） |
| （源无趾骨） | `cf_J_Toes01_L/R` | 恒 rest + 常量平放补偿（见 §5） |

- 踝目标 = 源踝方向 × HS2 腿长比；膝弯曲侧向 = 源膝相对「髋-踝连线」的侧向投影（先验）→ 不翻转、不 X 腿。

### 2.3 肩臂链

| 源骨 | HS2 驱动骨 | 求解方式 |
|---|---|---|
| `lCollar` / `rCollar` | `cf_J_Shoulder_L/R` | 源锁骨完整世界旋转增量 |
| `lShldr` / `rShldr` | `cf_J_ArmUp00_L/R` | 解析 2 段 IK |
| `lForeArm` / `rForeArm` | `cf_J_ArmLow01_L/R` | 解析 2 段 IK |
| `lHand` / `rHand` | `cf_J_Hand_L/R` | 源手骨完整世界旋转增量（f1 **双向量解**：掌纵轴 `lIndex1` + 掌横轴 `lPinky1`） |

- HS2 层级：`Spine03 → ShoulderIK → Shoulder → ArmUp00`；肩部蒙皮
  （512/520 号顶点）挂在 `cf_J_Shoulder` 上，因此锁骨必须驱动。

### 2.4 手指（每指 2 节）

源指骨完整世界旋转增量（f1 **绝对方向解**作基准）：

| 指名 | HS2（左） | 源（左） |
|---|---|---|
| 食指 | `cf_J_Hand_Index01/02_L` | `lIndex1 / lIndex2` |
| 中指 | `cf_J_Hand_Middle01/02_L` | `lMid1 / lMid2` |
| 无名指 | `cf_J_Hand_Ring01/02_L` | `lRing1 / lRing2` |
| 小指 | `cf_J_Hand_Little01/02_L` | `lPinky1 / lPinky2` |
| 拇指 | `cf_J_Hand_Thumb01/02_L` | `lThumb1 / lThumb2` |

（右侧同名 `_R` / `r*`。）

### 2.5 非驱动骨

184 骨中其余骨（`dam*` 分布骨、`*roll*` 扭转骨、`*_s` 端点辅助骨等）
保持 rest，由父旋转自然带动——蒙皮变形仍正常发生。

## 3. 骨架语义点表（对比渲染用）

`kp_trio_render.py` 的 `PTS` 表（源/HS2 各 21 点）用于三栏对比图中的
骨架管绘制与逐点误差统计；`CHAINS` 定义连线（躯干含骨盆闭合、臂含肩
连接、腿含髋连接）。映射与 §2 一致，另含 `lToeBase/rToeBase ↔
cf_J_Toes01_L/R`。

## 4. 空间与矩阵约定

| 项 | 值 | 说明 |
|---|---|---|
| FBX 导入修正 | `M_fix = RotX(180°)` | 导入矩阵把模型倒置，须左乘翻正（body 骨架及部件骨架） |
| 蒙皮贴合公式 | `mesh.mw = arm.mw @ Rx(-90°) @ S(0.01)` | body/top/bot/shoe 的骨架数据空间（Y-up）与网格空间（Z-up、cm 顶点）相差 Rx(-90°)；骨架对象矩阵任意变换后按此重同步即贴合 |
| 显示系 | `R_up = RotX(90°)` | body 骨架乘 R_up → 世界 Z-up（骨骼/相机语义） |
| 头/发 rest 对位 | `T(头骨位 + lift) @ Rx(-90°)` | 发数据空间沿 +Y 长 0.57m，Rx(-90°) 后沿世界 -Z 自然下垂；`lift = HAIR_LIFT = 0.05`（仅 hair_f/hair_b） |
| BVH 数据空间 | 100m 级（cm 未缩放） | 骨架栏显示须按 rest 世界垂直跨度比缩放（×K） |
| 模型平移 | 并入 **armature 对象矩阵** | 若乘在 mesh.matrix_world 上，armature modifier 的 mesh→arm 空间变夹心相似变换 → 蒙皮畸变巨网 |

## 5. 已标定常量

| 常量 | 值 | 含义 |
|---|---|---|
| `TOES_FLAT_PITCH` | 8.4° | 趾骨平放补偿：HS2 靴鞋底前 1/4 段 rocker 上翘 11.7mm（鞋头顶点挂未驱动 `Toes01` 蒙皮，权重 116 vs `Foot02` 仅 4），源 BVH 无趾骨 → 绕世界 x 下压；标定后鞋头 11.7→3.5mm、尖-跟 4.0→1.2mm。世界 x 与 `R_up` 共轴 → 显示系俯仰角不变 |
| `HAIR_LIFT` | 0.05 m | hair_f/hair_b rest 对位时的整体上抬量 |
| `H_OFF` | (0, 0, 0) | 头组 rest 对位微调偏移（保留为标定接口，当前 0 即认可版） |

## 6. 姿势模仿的对齐流程（kp_drive）

1. **旋转对齐**：源骨架整体旋转，源 up（`hip` 骨 Y）→ HS2 骨盆 up
   （`cf_J_Hips` Y）；再以 `lThigh-rThigh` 方向做左右轴 180° 歧义校验。
2. **尺度 + 平移对齐**：rest 态把源骨架 uniform 缩放到 HS2 体高
   （髋中→头骨 head 距离比）；平移基准取**动画首帧** pose 髋中
   （BVH root 轨迹常离原点数米）。
3. 逐帧求解（§2）并烘焙 rotation keyframes（四元数）到 body 骨架 Action。
