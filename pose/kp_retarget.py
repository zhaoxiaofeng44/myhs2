# -*- coding: utf-8 -*-
"""
kp_retarget.py — 位置域（Key-Point）驱动：HS2 骨骼模仿源骨架姿势（重写版）。

核心思想（与旋转域 bvh_retarget 的根本区别）：
  旋转域：把源骨"相对 rest 的旋转"同构复制到 HS2 骨 → 异构骨架 rest 差异
          在大摆角/高速帧下放大为关节错位（膝内扣、肩扭、四肢翻转）。
  位置域：无视两侧骨架差异，只让 HS2 的"关节"去追"源关节的位置"。
          每帧自根向叶解析求解，长度全部采用 HS2 自身段长（自洽），
          方向全部取自源骨架（同构），末端用解析 2 段 IK 锚定，
          弯曲侧向（膝/肘）用源中间关节投影作先验 → 不翻转、不 X 腿。

求解顺序（帧内全解析，无中间 depsgraph update）：
  1. 骨盆      cf_J_Hips   ← 源骨盆正交基（髋线 + 髋中→腹点）
  2. 脊柱      节方向      ← 源脊柱折线按弧长映射的切线方向（含低头/弯腰）
  3. 头        cf_J_Head   ← 源"颈顶→头顶"方向
  4. 腿 2 段 IK（髋锚=骨盆旋转后髋关节点；踝目标=源踝方向×腿长比；
                 膝弯曲侧=源膝相对髋-踝线的侧向投影）
  5. 脚        cf_J_Foot01 ← 源脚骨完整世界旋转增量（f1 基准，同头/四肢）
  6. 锁骨      cf_J_Shoulder ← 源锁骨(Collar)完整世界旋转增量（同脚方案）
  7. 臂 2 段 IK（肩锚=脊柱旋转后肩关节点；肘侧向=源肘投影）
  8. 手        源手骨完整世界旋转增量（f1 双向量解作基准）
  9. 手指      源指骨完整世界旋转增量（f1 绝对方向解作基准）
  其余骨（dam/roll/_s 辅助骨）保持 rest（由父旋转自然带动）。
"""

import os
import sys
import math
import argparse

import bpy
from mathutils import Vector, Matrix, Quaternion

V = Vector
_Y = V((0.0, 1.0, 0.0))
_KP_VER = "1.0"

# ---------------------------------------------------------------- 常量表 ---
# 每侧腿/臂链（HS2）：[根骨, 中骨(膝/肘), 末端关节骨(踝/腕)]
LEG = {"L": ["cf_J_LegUp00_L", "cf_J_LegLow01_L", "cf_J_Foot01_L"],
       "R": ["cf_J_LegUp00_R", "cf_J_LegLow01_R", "cf_J_Foot01_R"]}
ARM = {"L": ["cf_J_ArmUp00_L", "cf_J_ArmLow01_L", "cf_J_Hand_L"],
       "R": ["cf_J_ArmUp00_R", "cf_J_ArmLow01_R", "cf_J_Hand_R"]}
# 锁骨：HS2 cf_J_Shoulder_L/R（ArmUp00 父链，携 512/520 顶点肩部蒙皮）
# ←→ 源 rCollar/lCollar。层级：Spine03 → ShoulderIK → Shoulder → ArmUp00。
SHOULDER = {"L": ("cf_J_Shoulder_L", "lCollar"),
            "R": ("cf_J_Shoulder_R", "rCollar")}
# 趾骨平放补偿：HS2 靴鞋底前 1/4 段 rocker 上翘曲线 11.7mm（鞋头顶点挂
# 未驱动 Toes01 蒙皮，鞋头段权重 116 vs Foot02 仅 4），源 BVH 无趾骨
# （脚为末端骨）→ Toes01 恒 rest → 站姿(f1)鞋尖明显翘起。常量下压补偿
# 绕世界 x pitch 8.4°（rest 标定 _probe_foot_calib：鞋头 11.7→3.5mm、
# 尖-跟 4.0→1.2mm；Foot02/Foot01 实测无需补偿——前掌段底本就 0-1mm、
# 微翘被 Toes01 补偿顺带吸收）。世界 x 与 R_up(Rx90°)共轴 → 渲染系
# （R_up 后）俯仰角不变。跟区 2-3mm 为鞋跟底造型，非脚尖问题。
TOES = {"L": "cf_J_Toes01_L", "R": "cf_J_Toes01_R"}
TOES_FLAT_PITCH = math.radians(8.4)
# HS2 脊柱驱动骨（自下而上）
SPINE_BONES = ["cf_J_Spine01", "cf_J_Spine02", "cf_J_Spine03", "cf_J_Neck"]
HEAD_BONE = "cf_J_Head"
# 手指映射：HS2 骨 → 源骨（每指两节）
FINGERS = {
    "L": [("cf_J_Hand_Index01_L", "cf_J_Hand_Index02_L", "lIndex1", "lIndex2"),
          ("cf_J_Hand_Middle01_L", "cf_J_Hand_Middle02_L", "lMid1", "lMid2"),
          ("cf_J_Hand_Ring01_L", "cf_J_Hand_Ring02_L", "lRing1", "lRing2"),
          ("cf_J_Hand_Little01_L", "cf_J_Hand_Little02_L", "lPinky1", "lPinky2"),
          ("cf_J_Hand_Thumb01_L", "cf_J_Hand_Thumb02_L", "lThumb1", "lThumb2")],
    "R": [("cf_J_Hand_Index01_R", "cf_J_Hand_Index02_R", "rIndex1", "rIndex2"),
          ("cf_J_Hand_Middle01_R", "cf_J_Hand_Middle02_R", "rMid1", "rMid2"),
          ("cf_J_Hand_Ring01_R", "cf_J_Hand_Ring02_R", "rRing1", "rRing2"),
          ("cf_J_Hand_Little01_R", "cf_J_Hand_Little02_R", "rPinky1", "rPinky2"),
          ("cf_J_Hand_Thumb01_R", "cf_J_Hand_Thumb02_R", "rThumb1", "rThumb2")],
}
# 源侧对应名（无 L/R 后缀的骨盆/脊柱骨）
SRC_HIPMID = ["lThigh", "rThigh"]
SRC_ABD = "abdomen"
SRC_SPINE_PTS = ["abdomen", "chest", "neck", "head"]   # 源脊柱采样点（髋中起）
SRC_LEG = {"L": ("lThigh", "lShin", "lFoot"), "R": ("rThigh", "rShin", "rFoot")}
SRC_ARM = {"L": ("lShldr", "lForeArm", "lHand"), "R": ("rShldr", "rForeArm", "rHand")}

# HS2 手部双向量参考（手掌纵轴/横轴采样骨）
HAND_LONG_HS2 = {"L": "cf_J_Hand_Index01_L", "R": "cf_J_Hand_Index01_R"}
HAND_LAT_HS2 = {"L": "cf_J_Hand_Little01_L", "R": "cf_J_Hand_Little01_R"}
HAND_LONG_SRC = {"L": "lIndex1", "R": "rIndex1"}
HAND_LAT_SRC = {"L": "lPinky1", "R": "rPinky1"}


# ---------------------------------------------------------------- 工具 -----
def _head_w(arm, bn):
    """骨 head 世界位置（依赖骨架已 update / 外部调用时机）"""
    pb = arm.pose.bones.get(bn)
    if pb is None:
        return None
    return (arm.matrix_world @ pb.matrix).to_translation()


def _y_w(arm, bn):
    """骨 Y 轴（骨方向）世界方向"""
    pb = arm.pose.bones.get(bn)
    if pb is None:
        return None
    m3 = (arm.matrix_world @ pb.matrix).to_3x3()
    return (m3 @ _Y).normalized()


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _rot_between(a, b):
    """把单位向量 a 转到 b 的最短旋转（世界增量）"""
    a = a.normalized()
    b = b.normalized()
    d = a.dot(b)
    if d > 0.999999:
        return Quaternion()
    if d < -0.999999:
        # 反向：绕任意垂直轴转 180°
        ax = a.cross(V((1, 0, 0)))
        if ax.length < 1e-6:
            ax = a.cross(V((0, 0, 1)))
        ax.normalize()
        return Quaternion(ax, math.pi)
    return a.rotation_difference(b)


class KPDriver:
    """位置域驱动：把一个源骨架（BVH，逐帧 FK 世界点）驱动到 HS2 骨架。"""

    def __init__(self, body_arm, bvh_arm, verbose=True):
        self.body = body_arm
        self.bvh = bvh_arm
        self.verbose = verbose

        # ---- rest 静态数据（世界系，M_fix 已应用后）----
        self.base_q = {}     # 骨 rest 世界旋转
        self.rest_p = {}     # 骨 head rest 世界位置
        for pb in body_arm.pose.bones:
            self.rest_p[pb.name] = (body_arm.matrix_world @ pb.bone.matrix_local).to_translation()
            self.base_q[pb.name] = (body_arm.matrix_world @ pb.bone.matrix_local).to_quaternion().normalized()

        # ---- 链段长（HS2，米）与段向量 u0（骨 rest 世界方向）----
        def _seg(root, mid):
            """root 骨 → 其子链中 mid 骨 head 的段长/段方向"""
            p0 = self.rest_p[root]
            p1 = self.rest_p[mid]
            v = p1 - p0
            return v.length, v.normalized()

        self.leg_len = {}    # (L1, L2)
        self.leg_u0 = {}     # (u01, u02)
        self.arm_len = {}
        self.arm_u0 = {}
        self.spine_len = {}  # 每节“上端-下端”段长（骨 head 间距）
        for s in ("L", "R"):
            h, k, a = LEG[s]
            self.leg_len[s] = (_seg(h, k)[0], _seg(k, a)[0])
            self.leg_u0[s] = (_seg(h, k)[1], _seg(k, a)[1])
            h2, e, w = ARM[s]
            self.arm_len[s] = (_seg(h2, e)[0], _seg(e, w)[0])
            self.arm_u0[s] = (_seg(h2, e)[1], _seg(e, w)[1])

        # 脊柱节：HS2 骨 → (骨 head 距髋关节中点的弧长, u0 朝上段向量)
        self.spine_arc = {}
        hipmid_rest = (self.rest_p["cf_J_LegUp00_L"] + self.rest_p["cf_J_LegUp00_R"]) * 0.5
        prev_b = None
        for bn in SPINE_BONES + [HEAD_BONE]:
            # 弧长：髋中 → 骨 head（沿 rest 竖直近似，取 |Δy| 之和）
            p = self.rest_p[bn]
            if prev_b is None:
                arc = (p - hipmid_rest).length
            else:
                arc = self.spine_arc[prev_b]["arc"] + (p - self.rest_p[prev_b]).length
            self.spine_arc[bn] = {"arc": arc, "u0": _Y}
            prev_b = bn
        # 头段驱动向量不是骨 Y：用 “头底(Neck head 上端点) → Head head” 方向，
        # 简化：Head 骨 Y 即上指（CF 头骨轴向），直接用骨 Y 与弧长。
        # 髋关节（LegUp00 head）rel Hips head 的 rest 偏移（骨盆系 local）
        hips_p = self.rest_p["cf_J_Hips"]
        hips_q = self.base_q["cf_J_Hips"]
        hips_q_inv = hips_q.inverted()
        self.hip_off = {}
        for s in ("L", "R"):
            hp = self.rest_p[LEG[s][0]]
            self.hip_off[s] = hips_q_inv @ (hp - hips_p)

        # 源骨盆骨 rest 世界旋转（对齐已完成）——同构增量的基准。
        # 骨盆驱动采用“增量同构”而非“绝对正交基”：两侧 rest 已整体
        # 对齐（up 同向 + 左右校验），把源骨盆相对自身 rest 的姿态增量
        # 复制到 HS2 骨盆，与骨局部轴语义无关 → 无 180° 翻转歧义。
        self.hip_src = next((n for n in ("hip", "Hips") if n in self.bvh.data.bones), None)
        if self.hip_src is not None:
            m = self.bvh.matrix_world @ self.bvh.data.bones[self.hip_src].matrix_local
            self.src_rest_q_hip = m.to_quaternion().normalized()
        else:
            self.src_rest_q_hip = None
            print("[kp_retarget] 警告：源骨架无 hip/Hips 骨，骨盆退回正交基法")

        # 源腿/臂静态全长（rest head 距离，对齐后）→ 当前帧髋→踝/肩→腕
        # 距离按该比例放大到 HS2 链长：保持弯曲形态（若按“当前帧距离”作
        # 比例会与 total_hs 抵消，踝/腕被钉死在链伸直全长 → IK 恒解直腿）。
        self.src_leg_rest = {}
        self.src_arm_rest = {}
        for s in ("L", "R"):
            sh, sk, sa = SRC_LEG[s]
            ah = SRC_ARM[s]
            def _rhead(bn):
                b = self.bvh.data.bones.get(bn)
                if b is None:
                    return None
                return (self.bvh.matrix_world @ b.matrix_local).to_translation()
            pl, pa = _rhead(sh), _rhead(sa)
            self.src_leg_rest[s] = (pa - pl).length if (pl is not None and pa is not None) else 0.0
            ps, pw = _rhead(ah[0]), _rhead(ah[2])
            self.src_arm_rest[s] = (pw - ps).length if (ps is not None and pw is not None) else 0.0

        # 腿总长（供源向量缩放）
        for s in ("L", "R"):
            pass

        # 肩关节点（ArmUp00 head）rel 各脊柱父的 rest 偏移 —— 由 calc_pos 链式算，无需预存

        self.pos_cache = {}   # 帧内位置缓存
        self.q_cache = {}     # 帧内 desired_world 缓存
        self.prev_q = {}      # 每骨上一帧 roll(时域稳定用, 防 180° 翻转)
        self.src_limb_f1 = {}   # 源四肢长骨 f1 世界旋转（roll 增量传递用）
        self.limb_hs2_f1 = {}   # HS2 四肢长骨 f1 实际世界旋转（传递基准）
        self.src_head_f1 = None  # 源 head 骨 f1 世界旋转（头增量传递基准）
        self.head_hs2_f1 = None  # HS2 头骨 f1 实际世界旋转（传递基准）
        # 末端骨（踝/锁骨/手/指）f1 世界旋转基准 —— 完整增量传递用。
        # 旧"方向最短弧"传递只有 1-DOF（绕骨向的 roll 由最短弧任意选取），
        # 转体帧末端骨世界方向大变（130-180°）→ 其轴/角与父链已传的完整
        # 源增量不匹配 → 末端骨局部被叠加出 100-180° 过转、接缝蒙皮撕裂
        # （_probe_joint 踝R 173-180°/肩R 127-172°/腕L 117-120° 实证）。
        # 完整增量传递（同头/_limb_q）后：末端世界增量=源骨世界增量，
        # 末端局部=源骨相对源父的真实局部角（_probe_footdrv 实测 ≤72°）。
        self.src_footq_f1 = {}    # 源脚骨 f1 世界旋转（按侧）
        self.foot_hs2_f1 = {}     # HS2 踝 f1 世界旋转
        self.toes_hs2_f1 = {}     # HS2 趾 f1 世界旋转（平放补偿后，随脚增量）
        self.src_collarq_f1 = {}  # 源锁骨骨 f1 世界旋转（按侧）
        self.collar_hs2_f1 = {}   # HS2 锁骨 f1 世界旋转
        self.src_handq_f1 = {}    # 源手骨 f1 世界旋转（按侧）
        self.hand_hs2_f1 = {}     # HS2 手 f1 世界旋转
        self.src_fingq_f1 = {}    # 源指骨 f1 世界旋转（按 HS2 骨名记）
        self.fing_hs2_f1 = {}     # HS2 指骨 f1 世界旋转

    # ---------------- 源侧采样（世界系，u 单位） ----------------
    def s_head(self, bn):
        pb = self.bvh.pose.bones.get(bn)
        if pb is None:
            return None
        return (self.bvh.matrix_world @ pb.matrix).to_translation()

    def s_dir(self, bn):
        pb = self.bvh.pose.bones.get(bn)
        if pb is None:
            return None
        return ((self.bvh.matrix_world @ pb.matrix).to_3x3() @ _Y).normalized()

    def s_quat(self, bn):
        pb = self.bvh.pose.bones.get(bn)
        if pb is None:
            return None
        return (self.bvh.matrix_world @ pb.matrix).to_quaternion().normalized()

    def _limb_q(self, src_bn, base_q, u0, d_target):
        """四肢长骨求解：源世界旋转增量 R 以 f1 实际姿态为基准传递，段方向由 IK 微调。
        旧法 _rot_between(u0, d)@base 是最短旋转——绕骨轴的 roll（大腿/上臂
        扭转）完全丢失且随方向漂移。新法：① R = 源骨世界旋转相对 f1 增量；
        ② q_roll = R @ q_f1（q_f1 = HS2 f1 实际世界旋转，首帧 IK 解缓存；
        若以 rest(base_q) 为基准，HS2 相对 f1 的增量 = R⊗M₁⁻¹——f1 的 IK
        微调 M₁（源 f1 段方向 vs HS2 rest 段方向的比例差，臂上大）泄漏成
        沿段方向 13~40° twist 误差；以 q_f1 为基准后增量 = M_t⊗R，仅剩
        当帧微调）；③ 绕垂直轴微调 u_roll→d_target（u_roll = R 变换后的
        f1 段方向）。建模几何（实测）：源骨 Y 沿段 0°，HS2 腿骨 Y 反向
        180°、臂骨 Y 垂直 90°——R 是世界系完整旋转，作用与骨轴语义
        无关，视觉段 twist 正确传递（腿实测误差 ≤4°）。源无该骨时返回
        None 由调用方回退旧法。"""
        q_s = self.s_quat(src_bn)
        if q_s is None:
            return None
        ref = self.src_limb_f1.get(src_bn)
        if ref is None:
            # f1: IK 对齐解即传递基准（HS2 f1 实际世界旋转）
            q_f1 = (_rot_between(u0, d_target) @ base_q).normalized()
            self.src_limb_f1[src_bn] = q_s.copy()
            self.limb_hs2_f1[src_bn] = q_f1
            return q_f1
        q_f1 = self.limb_hs2_f1[src_bn]
        R = (q_s @ ref.inverted()).normalized()
        q_roll = (R @ q_f1).normalized()
        u_f1 = (q_f1 @ base_q.inverted()) @ u0   # f1 实际段方向
        u_roll = (R @ u_f1).normalized()
        return (_rot_between(u_roll, d_target) @ q_roll).normalized()

    # ---------------- 解析位置 / 目标旋转 ----------------
    def calc_pos(self, bn):
        """骨 head 世界位置（帧内，父 desired 已算）—— 解析递推"""
        if bn in self.pos_cache:
            return self.pos_cache[bn]
        pb = self.body.pose.bones.get(bn)
        if pb is None:
            return None
        p = pb.parent
        if p is None:
            pos = self.rest_p[bn].copy()
            self.pos_cache[bn] = pos
            return pos
        pp = self.calc_pos(p.name)
        if pp is None:
            pos = self.rest_p[bn].copy()
        else:
            # offset = 父 rest 世界系中 子 head 相对父 head
            off = self.base_q[p.name].inverted() @ (self.rest_p[bn] - self.rest_p[p.name])
            qp = self.q_cache.get(p.name, self.base_q[p.name])
            pos = pp + qp @ off
        self.pos_cache[bn] = pos
        return pos

    def set_desired(self, bn, desired_world_q):
        """登记骨目标世界旋转(含父链继承的最终世界旋转)"""
        self.q_cache[bn] = desired_world_q.normalized()
    
    def _stabilize(self, bn, q_new, axis_d):
        """绕段轴把 roll 拉到与上一帧连续: 最短旋转在段方向接近 180° 翻转时
        旋转轴会任意跳变, 造成骨绕长轴"多拧半圈/一圈"(蒙皮大腿/肘部扭转)。
        在满足段方向的所有解中挑与上帧最接近的 roll(纯时域约束,
        不依赖两侧骨架骨轴解剖语义, 无 180° 歧义)。"""
        _XU = V((1.0, 0.0, 0.0))
        p = self.prev_q.get(bn)
        if p is None or axis_d is None or axis_d.length < 1e-6:
            self.prev_q[bn] = q_new
            return q_new
        xn = q_new @ _XU
        xp = p @ _XU
        xn -= axis_d * xn.dot(axis_d)
        xp -= axis_d * xp.dot(axis_d)
        if xn.length < 1e-4 or xp.length < 1e-4:
            self.prev_q[bn] = q_new
            return q_new
        xn.normalize()
        xp.normalize()
        phi = math.atan2(xn.cross(xp).dot(axis_d), xn.dot(xp))
        if abs(phi) < 1e-3:
            q = q_new
        else:
            q = (Quaternion(axis_d, phi) @ q_new).normalized()
        self.prev_q[bn] = q
        return q
    
    def write_frame(self, frame):
        """把帧内缓存的 desired 世界旋转写入 pose + keyframe。
        basis 公式（实验标定 cand1，误差 1.8e-7）：
          M_child = M_parent @ rest_par⁻¹ @ rest_child @ basis
        四元数反解（arm 对象旋转在链中消去）：
          basis = rest_child⁻¹ ⊗ rest_par ⊗ par_des⁻¹ ⊗ desired_child
        par_des = 父的实际世界旋转（desired 语义）：父被驱动 → q_cache；
        父未驱动 → 沿父链向上找最近被驱动祖先 pk，归纳得
        par_des = desired_pk ⊗ rest_pk⁻¹ ⊗ rest_p（中间未驱动项消去）；
        全链 rest → arm_w_q ⊗ rest_p。旧版对未驱动父一律取世界 rest，
        在“父未驱动但祖父被驱动”时错（如 ShoulderL：父 ShoulderIK
        rest、祖父 Spine03 驱动 → 肩链子树整体偏差，f1 手指/肩 19.7°、
        源断层帧达 120°）；旧式 rest⁻¹@pw⁻¹@desired 共轭错位另致手指
        常帧偏 ~20°、断层帧臂链前后反。纯函数式不读 pose 状态。"""
        body = self.body
        arm_w_q = body.matrix_world.to_quaternion().normalized()
        for bn, desired_q in self.q_cache.items():
            pb = body.pose.bones.get(bn)
            if pb is None:
                continue
            desired_q = desired_q.normalized()
            rest_q = pb.bone.matrix_local.to_quaternion().normalized()
            p = pb.parent
            if p is None:
                basis = (arm_w_q.inverted() @ desired_q @ rest_q.inverted()).normalized()
            else:
                rest_p = p.bone.matrix_local.to_quaternion().normalized()
                par_des = self.q_cache.get(p.name)
                if par_des is None:
                    # 父未驱动：找最近被驱动祖先 pk（含 root 之上的全 rest 链）
                    cur = p.parent
                    anc = None
                    while cur is not None:
                        if cur.name in self.q_cache:
                            anc = cur
                            break
                        cur = cur.parent
                    if anc is None:
                        par_des = arm_w_q @ rest_p
                    else:
                        rest_anc = anc.bone.matrix_local.to_quaternion().normalized()
                        par_des = (self.q_cache[anc.name].normalized()
                                   @ rest_anc.inverted() @ rest_p)
                basis = (rest_q.inverted() @ rest_p @ par_des.normalized().inverted()
                         @ desired_q).normalized()
            pb.rotation_quaternion = basis
            pb.keyframe_insert('rotation_quaternion', frame=frame)

    # ---------------- 帧求解 ----------------
    def solve_frame(self, frame, dbg=False):
        """返回本帧缓存就绪（q_cache），随后调 write_frame"""
        body = self.body
        self.pos_cache.clear()
        self.q_cache.clear()
        _DBG = frame in (1, 150) and dbg

        # 0. 源骨架姿态采样（须已 frame_set + update）
        hipmid = None
        lh = self.s_head(SRC_HIPMID[0])
        rh = self.s_head(SRC_HIPMID[1])
        if lh is not None and rh is not None:
            hipmid = (lh + rh) * 0.5
        if hipmid is None:
            raise RuntimeError("源骨架缺少 lThigh/rThigh")

        # ---- 1. 骨盆（Hips）：增量同构 —— 源骨盆相对自身 rest 的旋转增量，
        # 原样复制到 HS2 骨盆（两侧 rest 已对齐 → 增量域无翻转歧义）。
        # 注：此前用“髋线正交基 Matrix((x0,y0,z0))”行构造，mathutils 行矩阵
        # 旋转语义与预期相反（需转置），导致骨盆 180° 翻转/髋锚飞出，已废弃。
        x0 = (rh - lh).normalized()          # “右减左”方向（腿/臂 IK 退化分支用）
        pelvis_q = None
        if self.src_rest_q_hip is not None:
            hip_pb = self.bvh.pose.bones.get(self.hip_src)
            if hip_pb is not None:
                src_now_q = (self.bvh.matrix_world @ hip_pb.matrix).to_quaternion().normalized()
                r_rel = self.src_rest_q_hip.inverted() @ src_now_q
                pelvis_q = (self.base_q["cf_J_Hips"] @ r_rel).normalized()
        if pelvis_q is None:
            # 兜底：髋线 + 髋中→腹点正交基（注意行基需转置才是正确旋转）
            abd = self.s_head(SRC_ABD)
            y0 = abd - hipmid
            y0 -= x0 * x0.dot(y0)
            y0.normalize()
            z0 = x0.cross(y0).normalized()
            pelvis_q = Matrix((x0, y0, z0)).transposed().to_quaternion()
        self.set_desired("cf_J_Hips", pelvis_q)
        # 登记 Hips head 位置（旋转不移动自身 head）
        self.pos_cache["cf_J_Hips"] = self.rest_p["cf_J_Hips"].copy()
        # 解剖参考轴（供腿/臂 IK 弯曲侧向判据）：源骨盆 X（髋轴）
        _X_ = V((1.0, 0.0, 0.0))
        hip_x_src = None
        if self.src_rest_q_hip is not None and hip_pb is not None:
            hip_x_src = (src_now_q @ _X_).normalized()
        if hip_x_src is None:
            hip_x_src = x0
        # 源肩胛线（臂链弯曲平面参考轴）
        shoulder_ax_src = None
        _s_ls = self.s_head("lShldr")
        _s_rs = self.s_head("rShldr")
        if _s_ls is not None and _s_rs is not None:
            _v = _s_ls - _s_rs
            if _v.length > 1e-4:
                shoulder_ax_src = _v.normalized()

        # ---- 2. 脊柱：弧长场重采样（段方向 + roll twist）----
        # 通用机制：源骨架躯干 = 空间中的“带标架折线”。目标骨数/段长与源
        # 不同 → 把目标每根骨按弧长比例映射到源折线子区间：
        #   段方向 d_j = 源折线上 [p_j,p_{j+1}] 端到端向量（弧长重采样，非
        #                 单点切线 —— 保证目标折线与源空间形态一致）；
        #   roll      = 段中点的源标架 X 轴（源骨当前世界旋转）绕 d_j 对齐，
        #               恢复转体/侧弯的躯干扭转 → 肩点随之自动正确。
        # 源脊柱采样点序列（自髋中向上）与折线弧长表
        src_pts = [hipmid]
        for bn in SRC_SPINE_PTS:
            p = self.s_head(bn)
            if p is None:
                break
            src_pts.append(p)
        if len(src_pts) < 2:
            raise RuntimeError("源骨架脊柱采样点不足")
        seg_a = [0.0]
        acc = 0.0
        for i in range(len(src_pts) - 1):
            acc += (src_pts[i + 1] - src_pts[i]).length
            seg_a.append(acc)
        Ls = acc

        def _q(s):
            """源折线弧长 s → 线性插值点"""
            if s <= 0.0:
                return src_pts[0]
            if s >= Ls:
                return src_pts[-1]
            for i in range(len(src_pts) - 1):
                if s <= seg_a[i + 1]:
                    t = (s - seg_a[i]) / (seg_a[i + 1] - seg_a[i] + 1e-12)
                    return src_pts[i].lerp(src_pts[i + 1], t)
            return src_pts[-1]

        def _frame_q(s):
            """弧长 s 处的源躯干标架旋转（所在段起点骨的当前世界旋转）"""
            if s <= 0.0:
                k = 0
            elif s >= Ls:
                k = len(src_pts) - 2
            else:
                k = 0
                for i in range(len(src_pts) - 1):
                    if s <= seg_a[i + 1]:
                        k = i
                        break
            sb = self.hip_src if k == 0 else SRC_SPINE_PTS[k - 1]
            pb = self.bvh.pose.bones.get(sb)
            if pb is None:
                return None
            return (self.bvh.matrix_world @ pb.matrix).to_quaternion().normalized()

        Lt = self.spine_arc[HEAD_BONE]["arc"]
        sc = Ls / Lt if Lt > 1e-9 else 1.0
        _X = V((1.0, 0.0, 0.0))
        nxts = SPINE_BONES[1:] + [HEAD_BONE]
        for bn, nb in zip(SPINE_BONES, nxts):
            p0 = self.spine_arc[bn]["arc"] * sc
            p1 = self.spine_arc[nb]["arc"] * sc
            d_seg = _q(p1) - _q(p0)
            if d_seg.length < 1e-6:
                continue
            d_seg.normalize()
            u0 = self.rest_p[nb] - self.rest_p[bn]
            if u0.length < 1e-6:
                continue
            u0.normalize()
            base = self.base_q[bn]
            q_tmp = (_rot_between(u0, d_seg) @ base).normalized()
            Rf = _frame_q((p0 + p1) * 0.5)
            if Rf is not None:
                px = Rf @ _X
                rx = q_tmp @ _X
                px -= d_seg * px.dot(d_seg)
                rx -= d_seg * rx.dot(d_seg)
                if px.length > 1e-4 and rx.length > 1e-4:
                    px.normalize()
                    rx.normalize()
                    # 绕 d_seg 把 rx 转到 px 的右手角：atan2((rx×px)·d, rx·px)
                    # （旧式 atan2(rx·(d×px),…) 恒取反号 —— twist 被镜像施加，
                    #   误差 ≈ 2×身体全局朝向 × 离轴半径：脊柱中线点近轴看不
                    #   出，肩/臂远轴点被放大成 0.2m+ 级错位；f300 铁证）
                    phi = math.atan2(rx.cross(px).dot(d_seg), rx.dot(px))
                    if abs(phi) > 1e-3:
                        q_tmp = (Quaternion(d_seg, phi) @ q_tmp).normalized()
            self.set_desired(bn, q_tmp)
        # 头：增量传递（同四肢思想）。旧式逐帧“颈→头方向最短旋转 +
        # 源X投影roll”从 rest 起算，方向对但 roll 分量逐帧漂移（实测
        # 增量差角最大 40.7°，f550/f592 头部朝向明显偏）。新法：f1 用
        # 旧式 IK 解作基准（f1 姿态不变），之后帧 R=源头骨世界旋转
        # 相对 f1 增量完整传递 q = R ⊗ q_f1 —— 歪头/转头全自由度保真。
        nk = self.s_head("neck")
        hd = self.s_head("head")
        q_s_h = self.s_quat("head")
        if q_s_h is not None and HEAD_BONE in self.base_q:
            if self.src_head_f1 is None:
                # f1: 旧式 IK 解（方向 + 源X投影roll）作传递基准
                q_f1 = None
                if nk is not None and hd is not None:
                    hdir = hd - nk
                    if hdir.length > 1e-5:
                        hdir.normalize()
                        base_h = self.base_q[HEAD_BONE]
                        q_f1 = (_rot_between(base_h @ _Y, hdir) @ base_h).normalized()
                        pb_h = self.bvh.pose.bones.get("head")
                        if pb_h is not None:
                            px = ((self.bvh.matrix_world @ pb_h.matrix).to_3x3() @ _X).normalized()
                            rx = q_f1 @ _X
                            px -= hdir * px.dot(hdir)
                            rx -= hdir * rx.dot(hdir)
                            if px.length > 1e-4 and rx.length > 1e-4:
                                px.normalize()
                                rx.normalize()
                                phi = math.atan2(rx.cross(px).dot(hdir), rx.dot(px))
                                if abs(phi) > 1e-3:
                                    q_f1 = (Quaternion(hdir, phi) @ q_f1).normalized()
                if q_f1 is None:
                    q_f1 = self.base_q[HEAD_BONE].copy()
                self.src_head_f1 = q_s_h.copy()
                self.head_hs2_f1 = q_f1
                self.set_desired(HEAD_BONE, q_f1)
            else:
                R = (q_s_h @ self.src_head_f1.inverted()).normalized()
                self.set_desired(HEAD_BONE, (R @ self.head_hs2_f1).normalized())
        elif nk is not None and hd is not None:
            hdir = hd - nk
            if hdir.length > 1e-5:
                hdir.normalize()
                base_h = self.base_q[HEAD_BONE]
                q_h = (_rot_between(base_h @ _Y, hdir) @ base_h).normalized()
                pb_h = self.bvh.pose.bones.get("head")
                if pb_h is not None:
                    px = ((self.bvh.matrix_world @ pb_h.matrix).to_3x3() @ _X).normalized()
                    rx = q_h @ _X
                    px -= hdir * px.dot(hdir)
                    rx -= hdir * rx.dot(hdir)
                    if px.length > 1e-4 and rx.length > 1e-4:
                        px.normalize()
                        rx.normalize()
                        phi = math.atan2(rx.cross(px).dot(hdir), rx.dot(px))
                        if abs(phi) > 1e-3:
                            q_h = (Quaternion(hdir, phi) @ q_h).normalized()
                self.set_desired(HEAD_BONE, q_h)

        # ---- 3. 腿（2 段解析 IK，踝目标=源踝方向 × 腿长比）----
        for s in ("L", "R"):
            h, k, a = LEG[s]
            # 髋锚：骨盆旋转后的髋关节点（解析）
            hips_q_now = self.q_cache.get("cf_J_Hips", self.base_q["cf_J_Hips"])
            hips_pos = self.rest_p["cf_J_Hips"]
            H = hips_pos + hips_q_now @ self.hip_off[s]
            self.pos_cache[h] = H
            # 源关节点（髋/膝/踝）
            sh, sk, sa = SRC_LEG[s]
            S_h, S_k, S_a = self.s_head(sh), self.s_head(sk), self.s_head(sa)
            L1, L2 = self.leg_len[s]
            total_hs = L1 + L2
            if S_h is None or S_k is None or S_a is None:
                continue
            src_total = (S_a - S_h).length
            # 踝目标距离 = 源当前髋→踝距离 × (HS2链长/源rest链长)
            # （保留屈膝形态；旧代码用当前帧比作 scale 与 total_hs 抵消，
            #   踝恒被钉在伸直全长 → 膝弯曲丢失）
            if self.src_leg_rest[s] > 1e-6:
                scale = total_hs / self.src_leg_rest[s]
            else:
                scale = total_hs / src_total if src_total > 1e-6 else 1.0
            A = H + (S_a - S_h).normalized() * (src_total * scale)
            # 2 段 IK
            d_vec = A - H
            d = _clamp(d_vec.length, abs(L1 - L2) + 1e-6, L1 + L2 - 1e-6)
            if d_vec.length < 1e-6:
                # 髋踝重合（源腿卷缩到髋处）：给默认下垂方向防退化
                d_vec = V((0.0, 0.0, -1.0))
            u = d_vec / (d_vec.length + 1e-12)
            # 膝 3D 引导（统一机制）：源膝相对弦点的⊥偏差方向 → 幅值用 HS2
            # 链长 h_c。无符号/无平面假设：源膝的膝内翻、腿外旋等面外弯曲被
            # 忠实复制（旧式 ±side·b 平面 IK 只允许在 (u×髋轴) 平面内弯，
            # f450/f592 左膝 0.14-0.16m 面外残留元凶）。b 仅作退化兜底。
            b = u.cross(hip_x_src)
            if b.length < 1e-4:
                b = u.cross(pelvis_q @ _Y)      # 兜底：骨盆局部上向
            if b.length < 1e-4:
                b = u.cross(V((0.0, 0.0, 1.0)))  # 再兜底：世界 Z
            b.normalize()
            a_c = (d * d + L1 * L1 - L2 * L2) / (2.0 * d)
            h_c = math.sqrt(max(L1 * L1 - a_c * a_c, 0.0))
            C = H + u * a_c
            # 引导向量必须在源域内取（S_k−S_h，勿用 S_k−C：源骨髅有自己
            # 的世界原点/漂移，与 HS2 锚点混算会把原点差投影成伪弯曲方向）
            w = S_k - S_h
            w -= u * w.dot(u)
            if w.length > 1e-5:
                K = C + w.normalized() * h_c
            else:
                # 退化（膝近伸直且源膝在弦上）：解剖平面方向兜底
                K = C + b * h_c * (1.0 if (S_k - S_h).dot(b) >= 0 else -1.0)
            # 写目标：大腿段向量 u0_1 → (K-H) 方向；小腿 u0_2 → (A-K) 方向
            d1 = (K - H).normalized()
            d2 = (A - K).normalized()
            base_h = self.base_q[h]
            q_h = self._limb_q(sh, base_h, self.leg_u0[s][0], d1)
            if q_h is None:
                q_h = (_rot_between(self.leg_u0[s][0], d1) @ base_h).normalized()
                self.set_desired(h, self._stabilize(h, q_h, d1))
            else:
                # roll 已由源世界旋转增量传递(源动画自身连续)，_stabilize
                # 的“拉回上帧”会把源 roll 冻结在 f1 → 大腿/小腿扭转丢失
                self.set_desired(h, q_h)
            base_k = self.base_q[k]
            q_k = self._limb_q(sk, base_k, self.leg_u0[s][1], d2)
            if q_k is None:
                q_k = (_rot_between(self.leg_u0[s][1], d2) @ base_k).normalized()
                self.set_desired(k, self._stabilize(k, q_k, d2))
            else:
                self.set_desired(k, q_k)
            # 脚：完整世界旋转增量传递（同头/四肢思想）。旧法“源脚骨 Y 轴
            # f1→当前的最短弧增量”只有 1-DOF：绕脚向的 roll 分量由最短弧
            # 任意选取，转体帧（pirouette f100+ 脚骨 Y 世界方向转 130-180°）
            # 其轴/角与父链（小腿已由 _limb_q 传完整源增量）不匹配 → 踝
            # 局部被叠出 173-180° 过转（人体踝 ~70°），踝部蒙皮（鞋跟/
            # LegLow02_s↔Foot01 接缝）毁灭性撕裂。实测源踝局部增量
            # （R_shin⁻¹⊗R_foot）全程 ≤72°（_probe_footdrv）——完整增量
            # 传递后踝世界增量=源脚世界增量（含 roll），踝局部=源踝局部，
            # 转体由小腿承担，踝只做真实跖屈/背屈/内外翻。f1（源/HS2 均
            # 平放站姿）增量=0 → 脚保持平放，基态对应关系与旧法一致。
            q_s_ft = self.s_quat(sa)
            if q_s_ft is not None and a in self.base_q:
                ref = self.src_footq_f1.get(s)
                if ref is None:
                    self.src_footq_f1[s] = q_s_ft.copy()
                    self.foot_hs2_f1[s] = self.base_q[a].copy()
                    self.set_desired(a, self.base_q[a])
                else:
                    R = (q_s_ft @ ref.inverted()).normalized()
                    self.set_desired(a, (R @ self.foot_hs2_f1[s]).normalized())
                # 趾骨：平放补偿（常量，见 TOES_FLAT_PITCH 注释）+ 跟随脚的
                # 世界增量。源无趾骨 → 趾相对脚无增量可传：趾世界增量=
                # 脚世界增量（R 同款），f1 基准含 8.4° 下压补偿（鞋头 rocker
                # 压平，站姿平放）。旧法 desired 恒=8.4°⊗rest（世界系常量）
                # ——转体帧脚转 130-180° 而趾钉在世界 rest 姿态 → 趾相对脚
                # 扭 105-162°（probe_joint 趾L 实测），前掌/鞋头蒙皮撕裂。
                tb = TOES[s]
                if tb in self.base_q:
                    if ref is None:
                        self.toes_hs2_f1[s] = (Quaternion((1.0, 0.0, 0.0),
                                                          TOES_FLAT_PITCH)
                                               @ self.base_q[tb]).normalized()
                        self.set_desired(tb, self.toes_hs2_f1[s])
                    else:
                        R = (q_s_ft @ ref.inverted()).normalized()
                        self.set_desired(tb, (R @ self.toes_hs2_f1[s]).normalized())
            if _DBG:
                print(f"    [leg{s}] H={H} A={A} d={d:.3f} d1={d1} d2={d2} foot_q={q_s_ft}")

        # ---- 3.5 锁骨（cf_J_Shoulder）：完整世界旋转增量传递（同脚方案）。
        # 源有 rCollar/lCollar 锁骨骨；HS2 肩部蒙皮（Shoulder02_s，512/520
        # 顶点）挂在 Shoulder 链下，且 ArmUp00 的父链是 Shoulder——驱动后：
        # 耸肩/扩胸的肩部皮肉跟随 + 臂 IK 肩锚（calc_pos 递推用 Shoulder
        # 的 desired）随之源锁骨运动，一并正确。旧法“collar 骨 Y 方向最短
        # 弧”同脚的 1-DOF 缺陷：转体帧锁骨局部 127-172° 过转（人体 ~30°）
        # → 肩部蒙皮（Shoulder02_s↔ArmUp01_s 接缝）被甩 17cm 撕裂 + 臂 IK
        # 肩锚连带错位。完整增量后锁骨世界增量=源 collar 世界增量，锁骨
        # 局部=源 collar 相对源 chest 的真实局部角，转体由脊柱链承担。
        for s in ("L", "R"):
            sb, src_bn = SHOULDER[s]
            q_s_c = self.s_quat(src_bn)
            if q_s_c is None or sb not in self.base_q:
                continue
            ref = self.src_collarq_f1.get(s)
            if ref is None:
                self.src_collarq_f1[s] = q_s_c.copy()
                self.collar_hs2_f1[s] = self.base_q[sb].copy()
                self.set_desired(sb, self.base_q[sb])
            else:
                R = (q_s_c @ ref.inverted()).normalized()
                self.set_desired(sb, (R @ self.collar_hs2_f1[s]).normalized())

        # ---- 4. 臂（2 段 IK + 肘侧向先验）----
        for s in ("L", "R"):
            h2, e, w_ = ARM[s]
            S_sh, S_el, S_wr = SRC_ARM[s]
            H = self.calc_pos(h2)   # 肩关节（脊柱/锁骨链解析后）
            if H is None:
                continue
            S_h2, S_e, S_w = self.s_head(S_sh), self.s_head(S_el), self.s_head(S_wr)
            if None in (S_h2, S_e, S_w):
                continue
            L1, L2 = self.arm_len[s]
            total_hs = L1 + L2
            src_total = (S_w - S_h2).length
            if self.src_arm_rest[s] > 1e-6:
                scale = total_hs / self.src_arm_rest[s]
            else:
                scale = total_hs / src_total if src_total > 1e-6 else 1.0
            A = H + (S_w - S_h2).normalized() * (src_total * scale)
            d_vec = A - H
            d = _clamp(d_vec.length, abs(L1 - L2) + 1e-6, L1 + L2 - 1e-6)
            if d_vec.length < 1e-6:
                d_vec = V((0.0, 0.0, -1.0))
            u = d_vec / (d_vec.length + 1e-12)
            # 肘 3D 引导（同膝）：源肘相对弦点的⊥偏差方向 × HS2 链长 h_c
            ref_ax = shoulder_ax_src if shoulder_ax_src is not None else hip_x_src
            b = u.cross(ref_ax)
            if b.length < 1e-4:
                b = u.cross(pelvis_q @ _Y)
            if b.length < 1e-4:
                b = u.cross(V((0.0, 0.0, 1.0)))
            b.normalize()
            a_c = (d * d + L1 * L1 - L2 * L2) / (2.0 * d)
            h_c = math.sqrt(max(L1 * L1 - a_c * a_c, 0.0))
            C = H + u * a_c
            w = S_e - S_h2   # 源域内引导向量（同膝，避免跨原点混算）
            w -= u * w.dot(u)
            if w.length > 1e-5:
                K = C + w.normalized() * h_c
            else:
                K = C + b * h_c * (1.0 if (S_e - S_h2).dot(b) >= 0 else -1.0)
            d1 = (K - H).normalized()
            d2 = (A - K).normalized()
            q_up = self._limb_q(SRC_ARM[s][0], self.base_q[h2],
                                self.arm_u0[s][0], d1)
            if q_up is None:
                q_up = (_rot_between(self.arm_u0[s][0], d1)
                        @ self.base_q[h2]).normalized()
                self.set_desired(h2, self._stabilize(h2, q_up, d1))
            else:
                # 同腿：源 roll 连续，无需（且不可）冻结
                self.set_desired(h2, q_up)
            q_lo = self._limb_q(SRC_ARM[s][1], self.base_q[e],
                                self.arm_u0[s][1], d2)
            if q_lo is None:
                q_lo = (_rot_between(self.arm_u0[s][1], d2)
                        @ self.base_q[e]).normalized()
                self.set_desired(e, self._stabilize(e, q_lo, d2))
            else:
                self.set_desired(e, q_lo)
            # 手：双向量（纵=指根向，横=食指根-小指根）恢复完整 3DOF
            self._solve_hand(s, S_w)
            if _DBG:
                print(f"    [arm{s}] H={H} A={A} d={d:.3f} d1={d1} src_sh={S_h2} src_w={S_w}")

        # ---- 5. 手指：逐节绝对方向 ----
        for s in ("L", "R"):
            for hs_a, hs_b, src_a, src_b in FINGERS[s]:
                self._solve_finger(hs_a, hs_b, src_a, src_b)

    # ---------------- 手/脚/指细节 ----------------

    def _solve_hand(self, side, wrist_pos):
        """手：完整世界旋转增量传递（f1 用双向量解定基准，同脚/头思想）。
        旧法逐帧“双向量绝对对齐”把源手世界朝向（含转体大旋转）绝对地
        施加在 rest 手上，而前臂已由 _limb_q 传源前臂完整增量 → 转体被
        算两遍，腕局部过转（实测 HS2 腕L 117-120° vs 源腕L 局部仅
        34-43°）。f1 双向量解只作姿态基准（两侧手建模基态差被吸收），
        之后帧 R=源手骨世界旋转增量 → 手世界增量=源手世界增量（含
        roll/翻转/转掌全保真），腕局部=源腕局部（_probe_footdrv）。"""
        h2, e, hand_bn = ARM[side]
        q_s_h = self.s_quat(SRC_ARM[side][2])
        if q_s_h is None or hand_bn not in self.base_q:
            return
        ref = self.src_handq_f1.get(side)
        if ref is not None:
            R = (q_s_h @ ref.inverted()).normalized()
            self.set_desired(hand_bn, (R @ self.hand_hs2_f1[side]).normalized())
            return
        # ---- f1: 双向量解（纵=指根向，横=食指根-小指根）作传递基准 ----
        # 目标手系：源 指根向 + 食指根-小指根横轴（腕点= wrist_pos）
        long_src = self.s_head(HAND_LONG_SRC[side])
        lat_src = self.s_head(HAND_LAT_SRC[side])
        if long_src is None or lat_src is None or wrist_pos is None:
            return
        v_long_src = long_src - wrist_pos
        v_lat_src = lat_src - wrist_pos
        if v_long_src.length < 1e-5 or v_lat_src.length < 1e-5:
            return
        v_long_src.normalize()
        v_lat_src -= v_long_src * v_lat_src.dot(v_long_src)
        v_lat_src.normalize()
        # HS2 侧同一结构（rest 世界向量）
        rp = self.rest_p[hand_bn]
        l2 = self.rest_p.get(HAND_LONG_HS2[side])
        t2 = self.rest_p.get(HAND_LAT_HS2[side])
        if l2 is None or t2 is None:
            return
        u_long = (l2 - rp).normalized()
        u_lat = t2 - rp
        u_lat -= u_long * u_lat.dot(u_long)
        u_lat.normalize()
        # 两段旋转精确对齐双向量（避免 Matrix 行基构造的转置歧义）：
        #   R1: u_long → v_long 的最短旋转
        #   R2: 绕 v_long（源长轴）旋转，把 R1 后的横轴 lat1 转到 v_lat
        base = self.base_q[hand_bn]
        R1 = u_long.rotation_difference(v_long_src)
        lat1 = R1 @ u_lat
        v_ax = v_long_src
        # 绕 v_ax 把 lat1 转到 v_lat 的右手角：atan2((lat1×v_lat)·ax, lat1·v_lat)
        # （旧式 lat1·(v_ax×v_lat) 恒取反号 —— 手 twist 镜像，同脊柱 roll bug）
        s = lat1.cross(v_lat_src).dot(v_ax)
        c = lat1.dot(v_lat_src)
        R2 = Quaternion(v_ax, math.atan2(s, c))
        q_hand = ((R2 @ R1) @ base).normalized()
        self.set_desired(hand_bn, q_hand)
        self.src_handq_f1[side] = q_s_h.copy()
        self.hand_hs2_f1[side] = q_hand.copy()

    def _solve_finger(self, hs_a, hs_b, src_a, src_b):
        """指节：完整世界旋转增量传递（f1 用绝对方向解定基准，同手方案）。
        旧法逐帧“rest 段向量→源骨 Y 方向”绝对对齐——转体帧源指方向大变
        + 手已传转体 → 指骨局部过转（同腕机理）。f1 基准吸收两侧指骨建
        模差（源指骨 Y 沿指节、HS2 指骨 Y⊥指节），之后帧纯增量跟随，
        源手指弯曲/张开全自由度保真。"""
        for hs_bn, src_bn, nxt_hs in ((hs_a, src_a, hs_b), (hs_b, src_b, None)):
            if hs_bn not in self.base_q or hs_bn not in self.rest_p:
                continue
            q_s_f = self.s_quat(src_bn)
            if q_s_f is None:
                continue
            ref = self.src_fingq_f1.get(hs_bn)
            if ref is not None:
                R = (q_s_f @ ref.inverted()).normalized()
                self.set_desired(hs_bn, (R @ self.fing_hs2_f1[hs_bn]).normalized())
                continue
            # ---- f1: 绝对方向解作传递基准 ----
            src_d = self.s_dir(src_bn)
            if src_d is None:
                continue
            # 段向量：本骨 head → 下骨 head。HS2 指骨骨 Y ⊥ 指节（审计实测
            # 90.0°/拇指 100.1°），而源指骨 Y 沿指节（0.0°）——末节(02)
            # 若用骨 Y 当段向量会被多转 ~90°（指尖弯折）；必须用其子骨
            # (03) head 取段向量（五指 02 节都有 03 子骨，见蒙皮审计）。
            u0 = None
            if nxt_hs is not None and nxt_hs in self.rest_p:
                u0 = self.rest_p[nxt_hs] - self.rest_p[hs_bn]
            else:
                db = self.body.data.bones.get(hs_bn)
                for c in (db.children if db else ()):
                    if c.name in self.rest_p:
                        u0 = self.rest_p[c.name] - self.rest_p[hs_bn]
                        break
            if u0 is None or u0.length < 1e-6:
                u0 = self.base_q[hs_bn] @ _Y
            u0.normalize()
            base = self.base_q[hs_bn]
            q_f1 = (_rot_between(u0, src_d) @ base).normalized()
            self.set_desired(hs_bn, q_f1)
            self.src_fingq_f1[hs_bn] = q_s_f.copy()
            self.fing_hs2_f1[hs_bn] = q_f1.copy()


# ---------------------------------------------------------------- 顶层 ----
def kp_drive(bvh_path, body_arm, action_name=None, fps=30):
    """加载 BVH 源骨架，位置域逐帧驱动 HS2 骨架并烘焙 rotation keyframes。

    返回 (body_act, n_frames)
    """
    before = set(bpy.data.objects)
    bpy.ops.import_anim.bvh(filepath=bvh_path)
    bvh_arm = next(o for o in bpy.data.objects if o not in before and o.type == "ARMATURE")
    bvh_act = bvh_arm.animation_data.action if bvh_arm.animation_data else None
    if bvh_act is None:
        cands = sorted(bpy.data.actions, key=lambda a: a.users, reverse=True)
        bvh_act = cands[0]
    bvh_arm.animation_data_create()
    bvh_arm.animation_data.action = bvh_act
    fr = bvh_act.frame_range
    n = int(fr[1] - fr[0] + 1)

    # ---- 源骨架整体旋转对齐（关键）：位置域用绝对方向，两侧骨架 rest
    # 世界朝向必须一致（例：BVH 源沿 +Z 站立，tachi_pose06 M_fix 后沿 -Z）。
    # 把源骨架对象整体旋转：源 up → HS2 骨盆 up；若左右轴反号再绕 up 转 180°。
    def _y_rest(arm, bn):
        b = arm.data.bones.get(bn)
        if b is None:
            return None
        m = arm.matrix_world @ b.matrix_local
        return m.to_3x3() @ _Y
    src_up = _y_rest(bvh_arm, "hip")
    hs_arm_bone = body_arm.data.bones.get("cf_J_Hips")
    if hs_arm_bone is not None:
        hs_up = _y_rest(body_arm, "cf_J_Hips")
    else:
        hs_up = None
    if src_up is not None and hs_up is not None and src_up.length > 0.5 and hs_up.length > 0.5:
        Q_a = src_up.normalized().rotation_difference(hs_up.normalized())
        # 左右轴符号校验（源 lThigh-rThigh 方向 vs HS2 LegUp00 同侧差，rest）
        sl = bvh_arm.data.bones.get("lThigh")
        sr = bvh_arm.data.bones.get("rThigh")
        hl = body_arm.data.bones.get("cf_J_LegUp00_L")
        hr = body_arm.data.bones.get("cf_J_LegUp00_R")
        if sl and sr and hl and hr:
            sv = ((bvh_arm.matrix_world @ sl.matrix_local).to_translation() -
                  (bvh_arm.matrix_world @ sr.matrix_local).to_translation())
            hv = ((body_arm.matrix_world @ hl.matrix_local).to_translation() -
                  (body_arm.matrix_world @ hr.matrix_local).to_translation())
            if (Q_a @ sv.normalized()).dot(hv.normalized()) < 0:
                Q_a = Quaternion(hs_up.normalized(), math.pi) @ Q_a
        M_a = Q_a.to_matrix().to_4x4()
        bvh_arm.matrix_world = M_a
        bpy.context.view_layer.update()
        print(f"[kp_retarget] 源骨架对齐 src_up={src_up.normalized()} hs_up={hs_up.normalized()} Q={Q_a}")
    else:
        print("[kp_retarget] 警告：无法对齐源骨架朝向（缺 up 参考骨）")

    # ---- 初始对齐（尺度 + 位置）：rest 态把源骨架 uniform 缩放到 HS2 体高、
    # 髋中平移到 HS2 髋中。KPDriver 全程消费相对量（段长比例/方向/姿态增量），
    # 故缩放平移不改变动作映射本身 → 仅统一两侧世界参考系：模仿开始前
    # 源骨架与 HS2 初始骨骼位置/模型处于同一位置同一尺度，可逐点直接对比。
    def _bhead(a, bn):
        b = a.data.bones.get(bn)
        return (a.matrix_world @ b.matrix_local).to_translation() if b is not None else None
    _bv = [_bhead(bvh_arm, x) for x in ("lThigh", "rThigh", "head")]
    _hv = [_bhead(body_arm, x) for x in ("cf_J_LegUp00_L", "cf_J_LegUp00_R", "cf_J_Head_s")]
    if all(v is not None for v in _bv + _hv):
        bvm = (_bv[0] + _bv[1]) * 0.5   # 源髋中（旋转对齐后, rest）
        hsm = (_hv[0] + _hv[1]) * 0.5   # HS2 髋中（rest）
        tb = (_bv[2] - bvm).length      # 源髋中→头骨 head = 体高基准
        th = (_hv[2] - hsm).length
        S = th / tb if tb > 1e-6 else 1.0
        # 平移基准取动画首帧 pose 髋中（非 rest）：BVH MOTION 段 root 位置是
        # 全局轨迹, 常离骨架定义原点数米 → 对齐到首帧才能让“模仿动作开始
        # 时”源骨架与 HS2 骨盆同点起步; 后续帧仅余动作本身的移动。
        _scn = bpy.context.scene
        _scn.frame_set(int(fr[0]))
        bpy.context.view_layer.update()
        def _fhead(a, bn):
            pb = a.pose.bones.get(bn)
            return (a.matrix_world @ pb.matrix).to_translation() if pb is not None else None
        _fb = [_fhead(bvh_arm, x) for x in ("lThigh", "rThigh")]
        if all(v is not None for v in _fb):
            fbm = (_fb[0] + _fb[1]) * 0.5   # 动画首帧源髋中
            T = hsm - fbm * S
        else:
            fbm = bvm
            T = hsm - bvm * S
        bvh_arm.matrix_world = Matrix.Translation(T) @ Matrix.Scale(S, 4) @ bvh_arm.matrix_world
        bpy.context.view_layer.update()
        print(f"[kp_retarget] 初始对齐: S={S:.5f} (体高 {tb:.2f}->{th:.3f}), "
              f"首帧髋中 ({fbm.x:.2f},{fbm.y:.2f},{fbm.z:.2f}) -> HS2髋中 ({hsm.x:.2f},{hsm.y:.2f},{hsm.z:.2f})")
    else:
        print("[kp_retarget] 警告：初始对齐跳过（缺髋/头参考骨）")

    # 重置 HS2 到 rest
    if body_arm.animation_data is None:
        body_arm.animation_data_create()
    act = bpy.data.actions.new(name=action_name or f"KP_{os.path.basename(bvh_path)}")
    body_arm.animation_data.action = act
    for pb in body_arm.pose.bones:
        pb.matrix_basis.identity()
    # rotation_mode 统一四元数
    for pb in body_arm.pose.bones:
        pb.rotation_mode = 'QUATERNION'
    bpy.context.view_layer.update()

    drv = KPDriver(body_arm, bvh_arm)
    print(f"[kp_retarget] 腿长 L/R: {drv.leg_len}  臂长 L/R: {drv.arm_len}")

    scene = bpy.context.scene
    for i in range(n):
        bvh_frame = fr[0] + i
        scene.frame_set(int(bvh_frame))
        bpy.context.view_layer.update()
        drv.solve_frame(i + 1, dbg=True)
        drv.write_frame(i + 1)
        if i % 120 == 0:
            print(f"[kp_retarget] frame {i + 1}/{n}")
    print(f"[kp_retarget] done {n} frames → action '{act.name}'")
    return act, n, bvh_arm


if __name__ == "__main__":
    # 独立运行（加载 fbx + bvh 全流程）
    ap = argparse.ArgumentParser()
    ap.add_argument("--fbx-dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "HoneySelect/Assets/Cosmetic"))
    ap.add_argument("--bvh", required=True)
    ap.add_argument("--frames", default=None)
    ap.add_argument("--out-action", default=None)
    args = ap.parse_args()

    M_fix = Matrix.Rotation(math.pi, 4, 'X')
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.ops.import_scene.fbx(filepath=os.path.join(args.fbx_dir, "body.fbx"))
    arm = next(o for o in bpy.data.objects if o.type == "ARMATURE")
    arm.matrix_world = M_fix @ arm.matrix_world
    bpy.context.view_layer.update()
    act, nfr, _bvh_arm = kp_drive(args.bvh, arm, action_name=args.out_action)
    print(f"RESULT action={act.name} frames={nfr}")
