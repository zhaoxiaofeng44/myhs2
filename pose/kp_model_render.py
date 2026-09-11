#!/usr/bin/env python3
# kp_model_render.py — kp_retarget 驱动 + 完整 HS2 角色(7 部件)渲染
# 装配要点(probe4/probe5 定案):
#   1) body/top/bot/shoe 骨架数据空间与网格相差 Rx(-90°)(网格顶点 +Z 站、骨骼 +Y 站),
#      mesh 对象矩阵自带 S(0.01)(cm 顶点)且导入矩阵 RotX180(须 M_fix 翻正)。
#      → 蒙皮贴合统一公式: mesh.mw = arm.mw @ Rx(-90°) @ S01;骨架对象矩阵任意
#        变换后按此重同步即贴合(probe4 四候选实验唯一正确项)。
#   2) head/hair_f/hair_b 顶点组为面骨/发骨(与 184 骨 body 骨架 0 交集),不能 rebind,
#      按 anim_render 同款: 骨架对象级刚体跟随 cf_J_Head_s(delta = B_t @ B_ref⁻¹);
#      rest 对位时骨架 = 纯平移 T(头骨位)(anim_render 推导: 部件数据空间即世界 Z-up 方向)。
#   3) 显示系: body_arm 乘 R_up=RotX(90°) → 世界 Z-up(骨骼/相机语义)。
# 用法: blender -b --factory-startup --python pose/kp_model_render.py
#       [--fbx-dir DIR] [--bvh FILE] [--frames 1,150,...] [--out DIR]
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
ap.add_argument("--shape-json", default=None,
                help="捏人滑杆 JSON 路径(如 character/sliders.example.json); 默认中性")
ap.add_argument("--data-dir", default=f"{_ROOT}/HoneySelect/Assets/Data",
                help="捏人数据表目录(HoneySelect/Assets/Data)")
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
args = ap.parse_args(argv)
FRAMES = [int(x) for x in args.frames.split(",")]

PART_COLORS = {
    "body":   (0.93, 0.78, 0.70, 1.0),
    "head":   (0.95, 0.82, 0.74, 1.0),
    "hair_f": (0.28, 0.17, 0.10, 1.0),
    "hair_b": (0.22, 0.13, 0.08, 1.0),
    "top":    (0.92, 0.93, 0.96, 1.0),
    "bot":    (0.23, 0.30, 0.47, 1.0),
    "shoe":   (0.16, 0.16, 0.19, 1.0),
}
PARTS = list(PART_COLORS.keys())
SKIN_GROUP = ["body", "top", "bot", "shoe"]   # mesh 顶点组含 body 骨名 → rebind + FIX
HEAD_GROUP = ["head", "hair_f", "hair_b"]   # 顶点组为面骨/发骨, 对象级刚体跟随头骨

M_fix = Matrix.Rotation(math.pi, 4, 'X')
R_up = Matrix.Rotation(math.pi / 2, 4, 'X')
S01 = Matrix.Scale(0.01, 4)
FIX = Matrix.Rotation(-math.pi / 2, 4, 'X') @ S01
HAIR_LIFT = 0.05

for o in list(bpy.data.objects):
    bpy.data.objects.remove(o, do_unlink=True)

meshes = {}
arms = {}
for p in PARTS:
    before = set(bpy.data.objects)
    bpy.ops.import_scene.fbx(filepath=f"{args.fbx_dir}/{p}.fbx")
    new = [o for o in bpy.data.objects if o not in before]
    ms = [o for o in new if o.type == "MESH"]
    as_ = [o for o in new if o.type == "ARMATURE"]
    meshes[p] = ms
    arms[p] = as_[0] if as_ else None
    for m in ms:
        m.color = PART_COLORS[p]

arm = arms["body"]
arm.matrix_world = M_fix @ arm.matrix_world
bpy.context.view_layer.update()

sys.path.insert(0, _ROOT)
sys.path.insert(0, _HERE)
from kp_retarget import kp_drive
from character import ShapeData, apply_morph, reset_pose, CAT_ZH
act, n, bvh_arm = kp_drive(args.bvh, arm)
# 回到 rest(装配 head/hair 对位基准)
for pb in arm.pose.bones:
    pb.matrix_basis.identity()
bpy.context.view_layer.update()

# ---- 捏人数据加载 ----
# 中性渲染(无 --shape-json)不加载捏人数据表(同 kp_trio_render: 隔离包无默认路径)
morph_deltas = {"face": {}, "body": {}}
if args.shape_json:
    if not os.path.isdir(args.data_dir or ""):
        raise SystemExit(f"[model_render] --shape-json 需要 --data-dir 指向游戏 Data 目录, 现: {args.data_dir}")
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

# ---- rebind 组(top/bot/shoe): 换绑到 body_arm; body mesh 同套 FIX ----

# 头/发跟随修正(probe6 四候选实验定案): 骨架 = T(头骨位+lift) @ Rx(-90°)
# — 发数据空间沿 +Y 长 0.57m, Rx(-90) 后沿世界 -Z 自然下垂(发根贴头顶/发尖垂到腰);
#   纯平移/±90°反号/180°均会把长发横甩或倒竖(渲染对比判据)
R_HEAD = Matrix.Rotation(-math.pi / 2, 4, 'X')

for p in SKIN_GROUP:
    if p == "body":
        continue
    for m in meshes[p]:
        for mod in m.modifiers:
            if mod.type == 'ARMATURE':
                mod.object = arm
    arms[p].hide_viewport = True
    arms[p].hide_render = True
bpy.context.view_layer.update()

# ---- Z-up 显示系(先装头再转也行,公式随 body_arm 当前矩阵重同步) ----
arm.matrix_world = R_up @ arm.matrix_world
HEAD_BONE = "cf_J_Head_s"
hp = (arm.matrix_world @ arm.pose.bones[HEAD_BONE].matrix).to_translation()

# ---- head/hair 骨架 rest 对位: 纯平移头骨位(+hair 上抬) @ Rx(-90) 修正 ----
def place_follow():
    for p in HEAD_GROUP:
        lift = Vector((0, 0, HAIR_LIFT)) if p != "head" else Vector((0, 0, 0))
        arms[p].matrix_world = Matrix.Translation(hp + lift) @ R_HEAD

place_follow()
for p in HEAD_GROUP:
    arms[p].hide_render = True
bpy.context.view_layer.update()

# ---- 对象跟随基准: B_ref = 对象矩阵 @ rest 骨矩阵(装配位姿); delta 逐帧叠加 ----
B_ref = arm.matrix_world @ arm.pose.bones[HEAD_BONE].bone.matrix_local
rest_mws = {p: arms[p].matrix_world.copy() for p in HEAD_GROUP}
print(f"[HEAD] rest head_pos=({hp.x:.3f},{hp.y:.3f},{hp.z:.3f})")

# ---- 相机 / 场景 ----
scene = bpy.context.scene
cam_data = bpy.data.cameras.new("Cam"); cam_data.type = 'ORTHO'
cam = bpy.data.objects.new("Cam", cam_data)
scene.collection.objects.link(cam)
scene.camera = cam
world = bpy.data.worlds.new("W"); scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs[0].default_value = (0.90, 0.92, 0.95, 1)
scene.render.resolution_x = 768
scene.render.resolution_y = 1152
scene.render.engine = 'BLENDER_WORKBENCH'
sh = scene.display.shading
sh.light = 'STUDIO'; sh.color_type = 'OBJECT'
sh.show_cavity = True; sh.cavity_type = 'BOTH'
az, el, dist = math.radians(28), math.radians(5), 8.0
ALLM = sum(meshes.values(), [])

def bbox_eval():
    deps = bpy.context.evaluated_depsgraph_get()
    lo = Vector((1e9,) * 3); hi = Vector((-1e9,) * 3)
    for m in ALLM:
        ev = m.evaluated_get(deps); me = ev.to_mesh()
        arr = np.empty(len(me.vertices) * 3); me.vertices.foreach_get('co', arr)
        pts = arr.reshape(-1, 3)
        M = np.array(ev.matrix_world)
        w = pts @ M[:3, :3].T + M[:3, 3]
        ev.to_mesh_clear()
        lo = Vector((min(lo.x, w[:, 0].min()), min(lo.y, w[:, 1].min()), min(lo.z, w[:, 2].min())))
        hi = Vector((max(hi.x, w[:, 0].max()), max(hi.y, w[:, 1].max()), max(hi.z, w[:, 2].max())))
    return lo, hi

os.makedirs(args.out, exist_ok=True)
for f in FRAMES:
    # 每帧先清 pose 再让动画求值，然后叠加捏人增量，避免跨帧残留
    reset_pose(arm)
    scene.frame_set(f)
    apply_morph(arm, morph_deltas["body"], verbose=False)
    reset_pose(arms["head"])
    apply_morph(arms["head"], morph_deltas["face"], verbose=False)
    bpy.context.view_layer.update()
    # 姿态帧: skin 组 mesh 与骨架同步; head/hair 刚性跟随头骨
    for p in SKIN_GROUP:
        for m in meshes[p]:
            m.matrix_world = arm.matrix_world @ FIX
    B_t = arm.matrix_world @ arm.pose.bones[HEAD_BONE].matrix
    delta = B_t @ B_ref.inverted()
    for p in HEAD_GROUP:
        arms[p].matrix_world = delta @ rest_mws[p]
    bpy.context.view_layer.update()
    lo, hi = bbox_eval()
    fc = (lo + hi) / 2
    fh = (hi - lo).z
    hp_t = B_t.to_translation()
    cam_data.ortho_scale = max(fh * 1.5, 1.6)
    cam.location = fc + Vector((math.sin(az) * math.cos(el), -math.cos(az) * math.cos(el), math.sin(el))) * dist
    cam.rotation_euler = (fc - cam.location).normalized().to_track_quat('-Z', 'Y').to_euler()
    fp = f"{args.out}/kpm_f{f:04d}.png"
    scene.render.filepath = fp
    bpy.ops.render.render(write_still=True)
    print(f"[RENDER f{f}] head=({hp_t.x:.2f},{hp_t.y:.2f},{hp_t.z:.2f}) bbox_h={fh:.2f} -> {fp}")
print("[DONE]")
