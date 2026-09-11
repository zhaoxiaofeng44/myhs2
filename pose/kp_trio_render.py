#!/usr/bin/env python3
# kp_trio_render.py — 三栏并列展示: 模型(HS2 网格) | 源BVH骨骼(绿) | HS2 模仿骨架(红)
# ---------------------------------------------------------------------------
# 布局: 左栏 = 源 BVH 骨架(绿管, 骨盆居中系 ×K); 中栏 = HS2 模仿骨架(红管);
#       右栏 = HS2 完整角色模型(7 部件)。
# 关键经验:
#   - BVH 数据空间为 100m 级(cm 未缩放)且骨架乘 R_up 前站向不一致 → 骨架栏必须
#     走 skel_compare 的 collect 骨盆居中系(减两髋中点→骨盆逆旋→R_SHOW)且源侧 ×K
#     (K=rest 世界垂直跨度比, 两骨架同乘 R_up 使 z 都是垂直轴; 否则量到 BVH 深度轴
#     K 大 10 倍 → 绿骨架 13.7m 飞柱)。
#   - T_MODEL 必须并入 arm 对象矩阵(模型右栏平移): 若乘在 mesh.matrix_world 上,
#     armature modifier 的 mesh→arm 空间变夹心相似变换 → 蒙皮畸变巨网。骨盆居中
#     collect 免疫平移 → 红骨架栏自动仍居中。
#   - 同场景"每帧改骨架管 mesh 顶点/几何"与模型渲染互斥(实测模型消失) →
#     三栏各自独立渲染(另两栏 hide_render), 最后 PIL 横向拼接, 完全规避。
# 用法: blender -b --factory-startup --python pose/kp_trio_render.py
#       [--fbx-dir DIR] [--bvh FILE] [--frames 1,150,...] [--out DIR] [--tmp DIR]
# 输出: --tmp 下三栏竖幅 bar 图(fNNNN_L/M/R.png), 之后用 PIL 拼接为横幅对比图:
#       python3 _imgcat.py <tmp> <out> "1,150,..."  →  <out>/cmp3_fNNNN.png
import bpy, os, sys, math, argparse, json
import numpy as np
from mathutils import Matrix, Vector

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
ap = argparse.ArgumentParser()
ap.add_argument("--fbx-dir", default=f"{_ROOT}/HoneySelect/Assets/Cosmetic")
ap.add_argument("--bvh", default=f"{_HERE}/samples/pirouette.bvh")
ap.add_argument("--frames", default="1,150,300,450,592")
ap.add_argument("--out", default=f"{_HERE}/renders")
ap.add_argument("--tmp", default="/tmp/trio_bars")
ap.add_argument("--bar-w", type=int, default=960,
               help="单栏像素宽(高=宽*1.6875 保持 640:1080 比例)")
ap.add_argument("--rest", action="store_true",
                help="仅渲染 HS2 rest 骨架(红)+模型两栏: 不导入 BVH、不模仿")
ap.add_argument("--headshot", action="store_true",
                help="额外渲染头部特写三视图(正脸/左/右): 检查前发装配")
ap.add_argument("--shape-json", default=None,
                help="捏人滑杆 JSON 路径(如 character/sliders.example.json); 默认中性")
ap.add_argument("--data-dir", default=f"{_ROOT}/HoneySelect/Assets/Data",
                help="捏人数据表目录(HoneySelect/Assets/Data)")
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
args = ap.parse_args(argv)
FRAMES = [int(x) for x in args.frames.split(",")]
if args.rest:
    FRAMES = [1]   # 无动画, rest 静态一帧
BAR_W, BAR_H = args.bar_w, round(args.bar_w * 1080 / 640)

PART_COLORS = {
    "body":   (0.93, 0.78, 0.70, 1.0), "head": (0.95, 0.82, 0.74, 1.0),
    "hair_f": (0.28, 0.17, 0.10, 1.0), "hair_b": (0.22, 0.13, 0.08, 1.0),
    "top":    (0.92, 0.93, 0.96, 1.0), "bot": (0.23, 0.30, 0.47, 1.0),
    "shoe":   (0.16, 0.16, 0.19, 1.0),   # 认可版深灰(曾误改紫灰 0.45,0.38,0.44)
}
# head 的眉/睫/眼白等五官是贴图层(cf_m_eyebrow 等 DXT5, 94% 透明细眉),
# 纯色渲染画色块会得粗矩形假眉 → 与认可版(OBJECT 全色无五官)一致: 不做五官配色。
def _paint_mats(p):
    for m in meshes[p]:
        for mat in m.data.materials:
            if mat is not None:
                mat.diffuse_color = (*PART_COLORS[p][:3], 1.0)
PARTS = list(PART_COLORS.keys())
SKIN_GROUP = ["body", "top", "bot", "shoe"]   # rebind + FIX(含 body 同步)
HEAD_GROUP = ["head", "hair_f", "hair_b"]     # 对象级刚体跟随头骨
M_fix = Matrix.Rotation(math.pi, 4, 'X')
R_up = Matrix.Rotation(math.pi / 2, 4, 'X')
# 骨架管栏最终取景系(collect 末步): 骨盆居中系(前轴朝 -y) → 相机系
R_SHOW = Matrix.Rotation(math.pi / 2, 4, 'X')
S01 = Matrix.Scale(0.01, 4)
FIX = Matrix.Rotation(-math.pi / 2, 4, 'X') @ S01
# head/hair 部件装配(修正): 部件原装网格因自带负 scale 呈"脸 -Z/顶 -Y"倒姿,
# 且 mesh 是骨架 child + 负镜像绑定 —— 只改骨架对象矩阵会破坏 mesh_to_arm,
# 正确做法: mesh 解除 parent 并与骨架同步左乘同一 H(delta@H_rest)。
# R180 把原装轴(顶-Y/脸-Z)转到 body 头骨局部轴(骨 y=顶/骨 z=脸)。
R180 = Matrix.Rotation(math.pi, 4, 'X')
# 与 anim_render 认可版逐位对齐验证(_probe_gap_check/_probe_gap_anim 三方数值对比):
#   H_OFF=(0,0,0) 时 head/hair_b/body/top/bot/shoe 与认可版 max|Δ|=0.0000m;
#   旧值 (0,0.034,0) 会使头部组件整体上浮 3.4cm(下颌钉在颅底, 偏高)。
#   anim 版(认可): 下颌位于头骨位下方 3.4cm, 头部刚体跟随基准即头骨位本身。
H_OFF = Vector((0, 0, 0))
# head 组内部件的 rest 级个体修正 X_p(乘在 H 右侧 = H_rest 局部系; 局部 y = 世界 -z)。
#   hair_f = hair_b = Tr((0,-0.05,0))@Rx(-90°): 与认可版(morph/pose/anim_render)逐顶点
#   0.000001m 对齐(_probe_hairf_align)。认可版对 hair_f/hair_b 用同一纯平移挂点
#   Tr(head_pos+(0,0,-0.05)) + M_fix 翻转(mesh 保持 child 跟随), 数学解
#   HAIR_CORR = H_rest⁻¹ @ M_fix @ Tr 与 hair_b 完全同一矩阵。
#   历史: anim 版竖帘盖脸是“装配前孤立几何”误诊(FBX 导入态未含 M_fix 翻转);
#   rx-90 放倒搜索的 bun 也错——正确形态本就是认可版渲染的齐刘海盖额头
#   (z∈[1.53,1.78], 中位 1.70 眉线, 眼带仅两侧鬓发 22/973 顶点)。
HAIR_CORR = {
    "head": Matrix.Identity(4),
    "hair_f": Matrix.Translation((0, -0.05, 0)) @ Matrix.Rotation(-math.pi / 2, 4, 'X'),
    "hair_b": Matrix.Translation((0, -0.05, 0)) @ Matrix.Rotation(-math.pi / 2, 4, 'X'),
}
MODEL_X = 1.9
T_MODEL = Matrix.Translation((MODEL_X, 0, 0))

# ---- 语义点/链(同 skel_compare): BVH名 ↔ HS2骨名 ----
PTS = [
    ("hip", "cf_J_Hips"), ("abdomen", "cf_J_Spine01"), ("chest", "cf_J_Spine02"),
    ("neck", "cf_J_Neck"), ("head", "cf_J_Head"),
    ("lCollar", "cf_J_Shoulder_L"), ("lShldr", "cf_J_ArmUp00_L"),
    ("lForeArm", "cf_J_ArmLow01_L"), ("lHand", "cf_J_Hand_L"),
    ("rCollar", "cf_J_Shoulder_R"), ("rShldr", "cf_J_ArmUp00_R"),
    ("rForeArm", "cf_J_ArmLow01_R"), ("rHand", "cf_J_Hand_R"),
    ("lThigh", "cf_J_LegUp00_L"), ("lShin", "cf_J_LegLow01_L"),
    ("lFoot", "cf_J_Foot01_L"), ("lToeBase", "cf_J_Toes01_L"),
    ("rThigh", "cf_J_LegUp00_R"), ("rShin", "cf_J_LegLow01_R"),
    ("rFoot", "cf_J_Foot01_R"), ("rToeBase", "cf_J_Toes01_R"),
]
CHAINS = [
    # 躯干含骨盆: hip↔abdomen 补齐(否则骨架显示为上下分离两块)
    ["hip", "abdomen", "chest", "neck", "head"],
    # 臂含肩连接(chest↔collar), 腿含髋连接(hip↔thigh)
    ["chest", "lCollar", "lShldr", "lForeArm", "lHand"],
    ["chest", "rCollar", "rShldr", "rForeArm", "rHand"],
    ["hip", "lThigh", "lShin", "lFoot"], ["hip", "rThigh", "rShin", "rFoot"],
]

for o in list(bpy.data.objects):
    bpy.data.objects.remove(o, do_unlink=True)
scene = bpy.context.scene

# ============ 1. 部件导入与装配(kp_model_render 同构) ============
meshes, arms = {}, {}
for p in PARTS:
    before = set(bpy.data.objects)
    bpy.ops.import_scene.fbx(filepath=f"{args.fbx_dir}/{p}.fbx")
    new = [o for o in bpy.data.objects if o not in before]
    meshes[p] = [o for o in new if o.type == "MESH"]
    as_ = [o for o in new if o.type == "ARMATURE"]
    arms[p] = as_[0] if as_ else None
    for m in meshes[p]:
        m.color = PART_COLORS[p]
        # FBX 自带全白 active 顶点色 → WORKBENCH 材质色被抹成银白, 一律删除
        for ca in list(m.data.color_attributes):
            m.data.color_attributes.remove(ca)
    _paint_mats(p)

arm = arms["body"]
arm.matrix_world = M_fix @ arm.matrix_world
bpy.context.view_layer.update()
sys.path.insert(0, _ROOT)
sys.path.insert(0, _HERE)
from kp_retarget import kp_drive, _detect_src_variant, SRC_VARIANTS
from character import ShapeData, apply_morph, reset_pose, CAT_ZH
bvh_arm = None
if args.rest:
    print("[MODE] rest: 仅 HS2 rest 骨架+模型, 不导入 BVH 不模仿")
else:
    act, n, bvh_arm = kp_drive(args.bvh, arm)
# AMASS 等命名变体: 骨架管 schema(PTS/CHAINS)与骨盆参考骨按变体别名翻译
BVH_HIP = "hip"
_alias = {}
if bvh_arm is not None:
    _alias = SRC_VARIANTS[_detect_src_variant(bvh_arm)].get("viz", {})
    if _alias:
        PTS = [(_alias.get(b, b), h) for b, h in PTS]
        CHAINS = [[_alias.get(n, n) for n in ch] for ch in CHAINS]
    BVH_HIP = _alias.get("hip", "hip")
for pb in arm.pose.bones:
    pb.matrix_basis.identity()
bpy.context.view_layer.update()

# ---- 捏人数据加载 ----
# 中性渲染(无 --shape-json)不加载捏人数据表: 表是外部游戏资产, 隔离包内
# 无默认路径(曾因默认 data-dir 指向 myhs2 内不存在路径 → ShapeData 静默
# 崩在 kp_drive 之后、无 [MORPH] 输出且 exit=0, 排障极难)
morph_deltas = {"face": {}, "body": {}}
if args.shape_json:
    if not os.path.isdir(args.data_dir or ""):
        raise SystemExit(f"[trio] --shape-json 需要 --data-dir 指向游戏 Data 目录, 现: {args.data_dir}")
    with open(args.shape_json, encoding="utf-8") as f:
        raw = json.load(f)
    shape_sliders = {sec: {int(k): v for k, v in raw.get(sec, {}).items()} for sec in ("face", "body")}
    for sec in ("face", "body"):
        sd = ShapeData(args.data_dir, sec)
        deltas, missing = sd.evaluate(shape_sliders[sec])
        morph_deltas[sec] = deltas
        print(f"[MORPH] {sec}: sliders={shape_sliders[sec]} -> {len(deltas)} bones"
              + (f", missing={missing}" if missing else ""))
        for cid in sorted(shape_sliders[sec]):
            zh = CAT_ZH[sec].get(cid, "?")
            print(f"    cat {cid} {zh}: slider={shape_sliders[sec][cid]}")

# rebind 组(top/bot/shoe) → arm; 部件骨架隐藏
for p in SKIN_GROUP:
    if p == "body":
        continue
    for m in meshes[p]:
        for mod in m.modifiers:
            if mod.type == 'ARMATURE':
                mod.object = arm
    arms[p].hide_viewport = True
    arms[p].hide_render = True

# 显示系: arm(HS2)与 bvh_arm(源)都转 Z-up; 模型右栏 = T_MODEL 并入 arm 对象矩阵
arm.matrix_world = T_MODEL @ R_up @ arm.matrix_world
if bvh_arm is not None:
    bvh_arm.matrix_world = R_up @ bvh_arm.matrix_world
# 模型栏朝向归一：管栏在 collect() 里减髋中点 → 逆旋 f1 骨盆 → 末步 R_SHOW，
# 即骨架栏最终取景 = R_SHOW @ q_f1⁻¹ @ (p-髋)。模型留在世界系就带上源 f1 转身
# (实测 jogstop/cartwheel f1 骨盆前向 yaw ≈ ±90°) → 右栏侧身、骨架栏正面。
# 给模型左乘同一个 M = R_SHOW @ q_f1⁻¹ 后：
#   · 模型 f1 体轴朝向 = R_SHOW @ v，与骨架栏逐轴一致（只乘 q_f1⁻¹ 会把角色
#     上轴转到 +y，模型在取景系里躺倒——实测已踩）；
#   · collect 预热到的 HS2 骨盆逆旋变成 R_SHOW⁻¹，与末步 R_SHOW 相消
#     → 骨架管栏像素级不变；
#   · 蒙皮相对矩阵 arm⁻¹@mesh=FIX 不变 → 姿势形不变，只是整体刚性转一记。
_ARM_W_PRE = arm.matrix_world.copy()      # 归一前矩阵, 供 K 量跨栏垂直跨度
scene.frame_set(1)                        # 量 f1 已解算骨盆姿态需要动作求值
bpy.context.view_layer.update()
q_f1_hips = (arm.matrix_world @ arm.pose.bones["cf_J_Hips"].matrix).to_quaternion().normalized()
arm.matrix_world = (R_SHOW @ q_f1_hips.inverted().to_matrix().to_4x4()) @ arm.matrix_world
bpy.context.view_layer.update()
_front = q_f1_hips @ Vector((0, 0, 1))    # HS2 骨盆局部 +Z = 角色前向
print(f"[FACE] 模型归一 M=R_SHOW@q_f1⁻¹, 归一前 f1 前向 yaw="
      f"{math.degrees(math.atan2(_front.x, -_front.y)):.1f}°")
# A/B 调试: HS2_YAW=180 → 模型侧(arm/蒙皮)整体绕世界 Z 再转 180°, 用于
# 与默认版对照 "模型前后朝向" 哪个对。必须排在上面的朝向归一之后, 否则该 180°
# 会被 q_f1 一并旋掉而失效(红骨架骨盆居中系对此免疫, 不受影响)。
if os.environ.get("HS2_YAW"):
    yaw = math.radians(float(os.environ["HS2_YAW"]))
    arm.matrix_world = Matrix.Rotation(yaw, 4, 'Z') @ arm.matrix_world
    bpy.context.view_layer.update()
HEAD_BONE = "cf_J_Head_s"
B_ref = arm.matrix_world @ arm.pose.bones[HEAD_BONE].bone.matrix_local
# head/hair 装配基准(rest): 快照原装矩阵, mesh 解除 parent(否则对象矩阵赋值与
# parent 链互相干扰, 旋转公共根实测会使蒙皮顶点飞出 ~100x)
W0_arm, W0_mesh = {}, {}
for p in HEAD_GROUP:
    W0_arm[p] = arms[p].matrix_world.copy()
    arms[p].hide_render = True
    W0_mesh[p] = []
    for m in meshes[p]:
        W0_mesh[p].append(m.matrix_world.copy())   # 解绑前快照(不依赖惰性旧值)
        m.parent = None
bpy.context.view_layer.update()
# 部件总变换 H_t = delta @ H_rest: H_rest 把原装部件(底在 +y)送到 rest 头骨位
# H_rest = Tr(A.t + A.R@H_OFF) @ A.R @ R180, A = rest 头骨世界矩阵(底点钉在 A.t)
RA = B_ref.to_3x3()
H_rest = Matrix.Translation(B_ref.translation + RA @ H_OFF) @ RA.to_4x4() @ R180
ALLMESH = [m for mm in meshes.values() for m in mm]

# ============ 2. 骨架语义点 schema ============
# rest 模式无源骨架: exist_bvh 取全集让 PAIRS 按 HS2 侧完整匹配(绿栏不渲染)
exist_bvh = set(bvh_arm.pose.bones.keys()) if bvh_arm is not None else {b for b, _ in PTS}
exist_hs2 = set(arm.pose.bones.keys())
PAIRS = [(b, h) for b, h in PTS if b in exist_bvh and h in exist_hs2]
IDX = {b: i for i, (b, h) in enumerate(PAIRS)}
EDGES = []
for ch in CHAINS:
    c = [n for n in ch if n in IDX]
    for a, b in zip(c, c[1:]):
        EDGES.append((IDX[a], IDX[b]))
print(f"[PTS] {len(PAIRS)} pairs, {len(EDGES)} edges")

def _hipm(arm_obj, bn):
    pb = arm_obj.pose.bones.get(bn)
    if pb is None:
        return Matrix.Identity(4)
    return arm_obj.matrix_world @ pb.matrix

def _origin_mid(arm_obj, pair):
    ps = []
    for b in pair:
        pb = arm_obj.pose.bones.get(b)
        if pb:
            ps.append((arm_obj.matrix_world @ pb.matrix).to_translation())
    return sum(ps, Vector()) / len(ps) if ps else Vector()

ORIGIN_BVH = [_alias.get(n, n) for n in ("lThigh", "rThigh")]
ORIGIN_HS2 = ["cf_J_LegUp00_L", "cf_J_LegUp00_R"]
BVH_X = -1.9

# 骨盆逆旋固定基准(f1)：旧版 collect 每帧用当前骨盆旋转逆旋——红骨架被
# 归一到“永远面朝镜头”，而模型栏是世界系(跟随骨盆转身，如 f450 转 146°)
# → 两栏姿势朝向不一致。改为首帧(预热时)缓存骨盆逆旋、之后固定不变：
# 保留每帧世界转身朝向。HS2 侧的这份 f1 逆旋现另存一步左乘进了 arm 对象矩阵
# (见上方 [FACE] 模型栏朝向归一) → 这里预热缓存到的 HS2 基准即单位阵，
# 红骨架不变、模型随之转正，三栏共享同一“f1 朝向 = 正面”的基准。
_F1_HIPROT = {}

def collect(arm_obj, rot_bone, origin_pair, scale=1.0, side=""):
    """骨盆居中系点集: 减两髋中点 → f1 骨盆基准逆旋(固定) → R_SHOW 转 Z-up"""
    m = _hipm(arm_obj, rot_bone)
    hip_pos = _origin_mid(arm_obj, origin_pair)
    key = (id(arm_obj), rot_bone)
    if key not in _F1_HIPROT:
        _F1_HIPROT[key] = m.to_quaternion().normalized().inverted()
    hip_rot_inv = _F1_HIPROT[key]
    dx = Vector((BVH_X, 0, 0)) if side == "bvh" else Vector((0, 0, 0))
    out = []
    for b, h in PAIRS:
        bn = b if side == "bvh" else h
        pb = arm_obj.pose.bones.get(bn)
        if pb is None:
            out.append(None)
            continue
        p = (arm_obj.matrix_world @ pb.matrix).to_translation()
        p = hip_rot_inv @ ((p - hip_pos) * scale)
        p = R_SHOW @ p + dx
        out.append(p)
    return out

# K: rest 世界垂直跨度比(两骨架都已 R_up → z 即垂直)
scene.frame_set(1)
bpy.context.view_layer.update()
if bvh_arm is not None:
    zb = [bvh_arm.matrix_world @ pb.matrix for pb in bvh_arm.pose.bones]
    zh = [_ARM_W_PRE @ pb.matrix for pb in arm.pose.bones]
    K = (max(v.to_translation().z for v in zh) - min(v.to_translation().z for v in zh)) / \
        (max(v.to_translation().z for v in zb) - min(v.to_translation().z for v in zb))
    print(f"[K] {K:.5f}")
else:
    K = 1.0
# f1 骨盆基准逆旋预热(当前已 frame_set(1))：首帧调用各 collect 一次缓存
# 基准，之后帧循环不再重算（rest 模式同样适用——无动画 f1==rest）
if bvh_arm is not None:
    collect(bvh_arm, BVH_HIP, ORIGIN_BVH, scale=K, side="bvh")
collect(arm, "cf_J_Hips", ORIGIN_HS2, side="hs2")

# ============ 3. 骨架管 mesh(绿=源BVH / 红=HS2), 固定拓扑+逐帧改顶点 ============
def build_mesh_obj(name, color):
    me = bpy.data.meshes.new(name)
    obj = bpy.data.objects.new(name, me)
    # WORKBENCH color_type='MATERIAL' 读材质色, obj.color(OBJECT 模式)不生效;
    # 无材质时骨架管渲染为灰白(实测 89,93,101) → 补材质槽承色
    mat = bpy.data.materials.new(name + "_mat")
    mat.diffuse_color = (*color, 1.0)
    me.materials.append(mat)
    obj.color = (*color, 1.0)
    scene.collection.objects.link(obj)
    return obj

def tube_update(obj, pts, radius=0.017, seg=6):
    """拓扑只建一次, 每帧 foreach_set 顶点坐标(跨帧重建几何会出渲染怪象)"""
    n_ed = len(EDGES)
    if "_tube_topology" not in obj:
        me = obj.data
        vs0 = [(0.0, 0.0, 0.0)] * (n_ed * (2 + 2 * seg))
        fs0 = []
        for idx in range(n_ed):
            base = idx * (2 + 2 * seg)
            ca, cb = base, base + 1
            ra, rb = base + 2, base + 2 + seg
            for k in range(seg):
                k2 = (k + 1) % seg
                fs0.append((ra + k, rb + k, rb + k2, ra + k2))
                fs0.append((ca, ra + k2, ra + k))
                fs0.append((cb, rb + k, rb + k2))
        me.clear_geometry()
        me.from_pydata(vs0, [], fs0)
        me.update()
        obj["_tube_topology"] = True
    co = []
    fv = next((p for p in pts if p is not None), Vector((0, 0, 0)))
    for a, b in EDGES:
        pa, pb = pts[a], pts[b]
        if pa is None or pb is None:
            pa = pb = fv
        d = pb - pa
        L = d.length
        if L < 1e-6:
            e = t1 = t2 = Vector((1, 0, 0))
        else:
            e = d / L
            ref = Vector((0, 0, 1)) if abs(e.z) < 0.9 else Vector((1, 0, 0))
            t1 = e.cross(ref).normalized()
            t2 = e.cross(t1).normalized()
        ring = [radius * (math.cos(2 * math.pi * k / seg) * t1
                         + math.sin(2 * math.pi * k / seg) * t2) for k in range(seg)]
        co.append(pa.to_tuple())
        co.append(pb.to_tuple())
        co.extend((pa + v).to_tuple() for v in ring)
        co.extend((pb + v).to_tuple() for v in ring)
    obj.data.vertices.foreach_set("co", [c for p in co for c in p])
    obj.data.update()

mesh_g = build_mesh_obj("SKEL_BVH", (0.15, 0.75, 0.30))
mesh_r = build_mesh_obj("SKEL_HS2", (0.90, 0.25, 0.20))

def tube_bbox(ob):
    if not len(ob.data.vertices):
        return None
    arr = np.empty(len(ob.data.vertices) * 3)
    ob.data.vertices.foreach_get('co', arr)
    p = arr.reshape(-1, 3)
    return p.min(axis=0), p.max(axis=0)

# ============ 4. 渲染设置(每栏独立 640×1080, 最后拼接) ============
cam_data = bpy.data.cameras.new("Cam"); cam_data.type = 'ORTHO'
cam = bpy.data.objects.new("Cam", cam_data)
scene.collection.objects.link(cam)
scene.camera = cam
world = bpy.data.worlds.new("W"); scene.world = world
# WORKBENCH 背景取 world.color(线性); 节点输入不生效(实测)
world.color = (0.10, 0.11, 0.13)
scene.render.engine = 'BLENDER_WORKBENCH'
scene.view_settings.view_transform = 'Standard'   # AgX 会把材质色压灰(实测"银球脸")
sh = scene.display.shading
sh.light = 'FLAT'; sh.color_type = 'MATERIAL'
sh.background_type = 'WORLD'   # 默认 THEME 背景会盖掉 world.color
# 曲率着色(立体感): WORKBENCH cavity 面板的 curvature 行 —— cavity_ridge/valley
# 归零、只用 curvature_ridge/valley。实测量化(_probe_renderq f150): 立体感指标
# (模型区域亮度梯度) 1.13→2.81(+148%)而色彩几乎无损(dRGB 仅 5.4,
# 饱和度 0.118→0.110); 旧注译"STUDIO+cavity 压灰"实为 STUDIO 灯光问题
# (dRGB 97), cavity/curvature 本身不压色。TB_FLAT=1 恢复纯平色对照。
if not os.environ.get("TB_FLAT"):
    sh.show_cavity = True
    sh.cavity_type = 'BOTH'
    sh.cavity_ridge_factor = 0.0
    sh.cavity_valley_factor = 0.0
    sh.curvature_ridge_factor = 1.0
    sh.curvature_valley_factor = 1.0
# TB_STUDIO=1 恢复旧光照作对照(实测会把材质色压灰, 仅调试用)
if os.environ.get("TB_STUDIO"):
    sh.light = 'STUDIO'; sh.show_cavity = True; sh.cavity_type = 'BOTH'
az, el, dist = math.radians(15), math.radians(8), 9.0

def render_bar(fp, lo, hi, pad=1.18):
    """按世界 bbox 取景渲染单栏竖幅"""
    fc = (lo + hi) / 2
    span = hi - lo
    need_h = max(span[2], span[0] / (BAR_W / BAR_H) * 0.9)
    cam_data.ortho_scale = max(need_h * pad, 1.1)
    cam.location = fc + Vector((math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el),
                                math.sin(el))) * dist
    cam.rotation_euler = (fc - cam.location).normalized().to_track_quat('-Z', 'Y').to_euler()
    scene.render.resolution_x = BAR_W
    scene.render.resolution_y = BAR_H
    scene.render.filepath = fp
    bpy.ops.render.render(write_still=True)

def mesh_world_bbox():
    """模型组 evaluated 世界 bbox(右栏取景)"""
    lo = Vector((1e9,) * 3); hi = Vector((-1e9,) * 3)
    deps = bpy.context.evaluated_depsgraph_get()
    for m in ALLMESH:
        ev = m.evaluated_get(deps); me = ev.to_mesh()
        arr = np.empty(len(me.vertices) * 3)
        me.vertices.foreach_get('co', arr)
        w = arr.reshape(-1, 3)
        M = np.array(ev.matrix_world)
        w = w @ M[:3, :3].T + M[:3, 3]
        ev.to_mesh_clear()
        lo = Vector((min(lo.x, w[:, 0].min()), min(lo.y, w[:, 1].min()), min(lo.z, w[:, 2].min())))
        hi = Vector((max(hi.x, w[:, 0].max()), max(hi.y, w[:, 1].max()), max(hi.z, w[:, 2].max())))
    return lo, hi

def set_vis(show_model, show_g, show_r):
    for m in ALLMESH:
        m.hide_render = not show_model
    mesh_g.hide_render = not show_g
    mesh_r.hide_render = not show_r
    bpy.context.view_layer.update()

# ============ 5. 帧循环: 三栏独立渲染 + 拼接 ============
os.makedirs(args.tmp, exist_ok=True)
os.makedirs(args.out, exist_ok=True)
for f in FRAMES:
    # 每帧先清 pose 再让动画求值，然后叠加捏人增量，避免跨帧残留
    reset_pose(arm)
    scene.frame_set(f)
    apply_morph(arm, morph_deltas["body"], verbose=False)
    reset_pose(arms["head"])
    apply_morph(arms["head"], morph_deltas["face"], verbose=False)
    bpy.context.view_layer.update()
    # 模型组/骨架姿态(平移已在 arm 对象矩阵)
    for p in SKIN_GROUP:
        for m in meshes[p]:
            m.matrix_world = arm.matrix_world @ FIX
    B_t = arm.matrix_world @ arm.pose.bones[HEAD_BONE].matrix
    delta = B_t @ B_ref.inverted()
    H_t = delta @ H_rest
    for p in HEAD_GROUP:
        Hc = H_t @ HAIR_CORR[p]
        arms[p].matrix_world = Hc @ W0_arm[p]
        for m, m0 in zip(meshes[p], W0_mesh[p]):
            m.matrix_world = Hc @ m0
    if bvh_arm is not None:
        pts_b = collect(bvh_arm, BVH_HIP, ORIGIN_BVH, scale=K, side="bvh")
        tube_update(mesh_g, pts_b)
    pts_h = collect(arm, "cf_J_Hips", ORIGIN_HS2, side="hs2")
    tube_update(mesh_r, pts_h)
    bpy.context.view_layer.update()
    # 各栏 bbox
    mb = mesh_world_bbox()
    gb = tube_bbox(mesh_g) if bvh_arm is not None else None
    rb = tube_bbox(mesh_r)
    # 左栏: 绿源骨架(rest 模式跳过)
    if bvh_arm is not None:
        set_vis(False, True, False)
        render_bar(f"{args.tmp}/f{f:04d}_L.png",
                   Vector(gb[0]), Vector(gb[1]))
    # 中栏: 红 HS2 骨架
    set_vis(False, False, True)
    render_bar(f"{args.tmp}/f{f:04d}_M.png",
               Vector(rb[0]), Vector(rb[1]))
    # 右栏: 模型
    set_vis(True, False, False)
    render_bar(f"{args.tmp}/f{f:04d}_R.png", mb[0], mb[1])
    if args.headshot:
        # 头部特写三视图(正脸/左/右): 检查前发 hair_f 装配。
        # az: 0=正脸(相机在 -y), ±90°=左右侧; pad 放大容忍侧面 y 跨度大于 x
        deps = bpy.context.evaluated_depsgraph_get()
        hs_lo = Vector((1e9,) * 3); hs_hi = Vector((-1e9,) * 3)
        for p in ("head", "hair_f", "hair_b"):
            for m in meshes[p]:
                ev = m.evaluated_get(deps); me = ev.to_mesh()
                arr = np.empty(len(me.vertices) * 3)
                me.vertices.foreach_get('co', arr)
                w = arr.reshape(-1, 3)
                M = np.array(ev.matrix_world)
                w = w @ M[:3, :3].T + M[:3, 3]
                ev.to_mesh_clear()
                for i in range(3):
                    hs_lo[i] = min(hs_lo[i], w[:, i].min())
                    hs_hi[i] = max(hs_hi[i], w[:, i].max())
        _az = az
        for nm, a in (("front", 0.0), ("left", -math.pi / 2), ("right", math.pi / 2)):
            az = a
            render_bar(f"{args.tmp}/f{f:04d}_hs_{nm}.png", hs_lo, hs_hi, pad=1.7)
        az = _az
        print(f"[HS f{f}] head组 bbox {tuple(round(v, 3) for v in hs_lo)}.."
              f"{tuple(round(v, 3) for v in hs_hi)}")
    set_vis(True, True, True)
    gtag = f"g=({gb[0][0]:.1f}..{gb[1][0]:.1f}) " if gb else ""
    print(f"[BAR f{f}] {gtag}r=({rb[0][0]:.1f}..{rb[1][0]:.1f}) "
          f"m=({mb[0][0]:.1f}..{mb[1][0]:.1f})")
print("[DONE] bars in", args.tmp)
print(f"[JOIN] python3 _imgcat.py {args.tmp} {args.out} {','.join(map(str, FRAMES))}")
