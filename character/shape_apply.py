# -*- coding: utf-8 -*-
"""
Blender 应用层：把 shape_data.evaluate 产出的骨骼增量作用到 Armature pose。
用法（Blender 内）：
    from shape_apply import apply_morph
    report = apply_morph(arm, deltas)   # deltas: {bone: {pos/rot(度)/scale}}
语义对齐 Unity BoneShapeSliderRuntimeDriver 增量模式：
  pos    += delta            （数据为 Y-up 原值，Blender 骨架数据空间同为 Y-up 形式）
  rot     = rot * EulerZXY(delta)     （自身局部系增量，右乘）
  scale  *= ratio
  rot 单位为度，内部转弧度。
"""
import math

from mathutils import Euler

AXIS = ["x", "y", "z"]


def _euler_zxy_quat(rx_deg, ry_deg, rz_deg):
    return Euler((math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)), "ZXY").to_quaternion()


def _read_rot(pb):
    """当前旋转（按骨自身 mode 读取为 quat）"""
    if pb.rotation_mode == "QUATERNION":
        return pb.rotation_quaternion.copy()
    return pb.rotation_euler.to_quaternion()


def _write_rot(pb, q):
    """写回旋转且不改变骨 rotation_mode（避免破坏 action 欧拉曲线驱动）"""
    if pb.rotation_mode == "QUATERNION":
        pb.rotation_quaternion = q
    else:
        pb.rotation_euler = q.to_euler(pb.rotation_mode)


def apply_morph(arm, deltas, verbose=True):
    """deltas 由 shape_data.ShapeData.evaluate 输出。
    返回 (applied, missing_names)；missing 多为骨架中不存在骨（碰撞/虚拟骨），正常。"""
    pose = arm.pose
    applied, missing = [], []
    for name, d in deltas.items():
        pb = pose.bones.get(name)
        if pb is None:
            missing.append(name)
            continue
        pos = d.get("pos") or [0.0, 0.0, 0.0]
        rot = d.get("rot") or [0.0, 0.0, 0.0]
        scale = d.get("scale") or [1.0, 1.0, 1.0]
        if any(v != 0.0 for v in pos):
            pb.location.x += pos[0]
            pb.location.y += pos[1]
            pb.location.z += pos[2]
        if any(v != 0.0 for v in rot):
            _write_rot(pb, _read_rot(pb) @ _euler_zxy_quat(*rot))
        if any(v != 1.0 for v in scale):
            pb.scale.x *= scale[0]
            pb.scale.y *= scale[1]
            pb.scale.z *= scale[2]
        applied.append(name)
    if verbose:
        print(f"[MORPH] {arm.name}: applied {len(applied)} bones, missing {len(missing)}"
              + (f": {missing[:8]}{'...' if len(missing) > 8 else ''}" if missing else ""))
    return applied, missing


def reset_pose(arm):
    """恢复 rest 姿势（head 骨架跨动画循环防残留）"""
    for pb in arm.pose.bones:
        pb.location = (0.0, 0.0, 0.0)
        pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
        pb.rotation_euler = (0.0, 0.0, 0.0)
        pb.scale = (1.0, 1.0, 1.0)
