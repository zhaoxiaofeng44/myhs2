#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
捏人数据驱动 · 纯数据层
========================================
把 HoneySelect 的 10 张捏人数据表解析为结构化模型，并按 Unity 运行时
(BoneShapeSliderRuntimeDriver.cs) 的公式把"滑杆值"评估为骨骼增量。

数据表（Assets/Data/）：
  cf_customhead.txt / cf_custombody.txt   categoryId \t oldBone \t 9轴掩码(px py pz rx ry rz sx sy sz)
  cf_anmShapeHead.txt / Body.txt          oldBone \t (keyIdx + px py pz rx ry rz sx sy sz) × 25帧
  face_bone_mappings.csv / bone_mappings.csv  oldBone,newBone[,mirror...]
  cf_headBoneOld.txt / cf_headBone.txt    仅诊断用（映射以 csv 为准）

评估公式（与 C# 逐行一致，关键帧组索引 0..24，中性帧 = 12）：
  keyFloat = clamp01(slider) × 24 → 相邻两帧插值（旋转通道最短路径绕回）
  positionDelta = (interp - neutral) / 100        —— 加性（米）
  rotationDelta = NormalizeAngle(interp - neutral) —— 加性（欧拉度）
  scaleRatio    = interp / neutral                —— 乘性
  多分类共享同一目标骨：pos/rot 累加，scale 累乘；mirror 骨同步追加。

用法：
  python3 shape/shape_data.py dump     --data-dir HoneySelect/Assets/Data
  python3 shape/shape_data.py eval     --data-dir ... --face '{"0":0.2,"5":0.9}' --body '{}'
  python3 shape/shape_data.py cats     --data-dir ...   # 每分类涉及的骨+掩码（供命名参考）
"""
import argparse
import json
import os
import sys

POS_DIVISOR = 100.0
NEUTRAL_KEY = 12          # KeyCount(25) // 2
KEY_COUNT = 25
EPS = 1e-4

CAT_PREFIXES = ("cf_s_", "cf_J_", "cf_N_", "cf_hit_")
SUFFIX_EXACT = {"h", "s", "l", "r", "a", "d", "t"}
SUFFIX_AXIS = {"sx", "sy", "sz", "rx", "ry", "rz", "tx", "ty", "tz"}
SUFFIX_SHAPE = {"ss", "ss_02", "ss_03", "ss_02sz", "ss_03sz", "ss_ty", "ty"}


def _strip_suffix_variants(name):
    while True:
        last = name.rfind("_")
        if last <= 0:
            return name
        suffix = name[last + 1:].lower()
        if suffix in SUFFIX_EXACT or suffix in SUFFIX_AXIS or suffix in SUFFIX_SHAPE or suffix in {"l", "r"}:
            name = name[:last]
        else:
            return name


def old_to_new_dynamic(old_name):
    """C# ConvertOldBoneNameToNewDynamic 的移植（仅当 csv 映射未覆盖时兜底）"""
    if "_dam" in old_name:
        return old_name
    if not old_name.startswith("cf_s_"):
        return old_name
    wp = old_name[5:]
    lr = ""
    for suf in ("_L", "_R", "_a", "_b"):
        if wp.endswith(suf):
            lr = suf
            wp = wp[: -len(suf)]
            break
    base = _strip_suffix_variants(wp)
    if base == "height":
        return "cf_N_height"
    if base.startswith("sk_"):
        return "cf_J_" + base + lr
    if base.startswith("hit_"):
        return "cf_" + base + lr
    return "cf_J_" + base + "_s" + lr


def parse_keyframes(path):
    """{old_bone: [25帧 × [px py pz rx ry rz sx sy sz]]}"""
    out = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.rstrip("\r\n").split("\t")
            if len(parts) < 11:
                continue
            bone = parts[0].strip()
            if not bone:
                continue
            n = (len(parts) - 1) // 10
            frames = []
            for k in range(n):
                base = 1 + k * 10
                if base + 9 >= len(parts):
                    break
                try:
                    frames.append([float(parts[base + 1 + f]) for f in range(9)])
                except ValueError:
                    pass
            if frames:
                out[bone] = frames
    return out


def parse_category_map(path):
    """{cat_id: [(old_bone, mask9bool)]}"""
    out = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.rstrip("\r\n").split("\t")
            if len(parts) < 11:
                continue
            try:
                cid = int(parts[0].strip())
            except ValueError:
                continue
            bone = parts[1].strip()
            if not bone:
                continue
            mask = [parts[2 + m].strip() not in ("", "0") for m in range(9)]
            out.setdefault(cid, []).append((bone, mask))
    return out


def parse_mapping_csv(path):
    """{old_bone: (new_bone, [mirror...])}；mirror 仅当 csv 提供第 3+ 列"""
    out = {}
    with open(path, encoding="utf-8-sig") as f:
        for line in f:
            parts = [p.strip() for p in line.rstrip("\r\n").split(",")]
            if len(parts) < 2 or parts[0] == "OldBoneName":
                continue
            out[parts[0]] = (parts[1], parts[2:])
    return out


def _normalize_angle_deg(d):
    return ((d + 180.0) % 360.0) - 180.0


def interpolate_keyframe(frames, slider_t):
    key_float = max(0.0, min(1.0, slider_t)) * (KEY_COUNT - 1)
    lo_i = min(int(key_float), len(frames) - 2)
    lerp_t = key_float - lo_i
    lo, hi = frames[lo_i], frames[lo_i + 1]
    res = [0.0] * 9
    for i in range(9):
        if 3 <= i <= 5:
            d = hi[i] - lo[i]
            res[i] = lo[i] + ((d + 180.0) % 360.0 - 180.0) * lerp_t
        else:
            res[i] = lo[i] + (hi[i] - lo[i]) * lerp_t
    return res


class ShapeData:
    """一个 section（face 或 body）的全部表 + 评估器"""

    def __init__(self, data_dir, section):
        pre = "cf_" if section == "body" else "cf_"
        if section == "face":
            self.cat_map = parse_category_map(os.path.join(data_dir, "cf_customhead.txt"))
            self.frames = parse_keyframes(os.path.join(data_dir, "cf_anmShapeHead.txt"))
            self.mapping = parse_mapping_csv(os.path.join(data_dir, "face_bone_mappings.csv"))
        else:
            self.cat_map = parse_category_map(os.path.join(data_dir, "cf_custombody.txt"))
            self.frames = parse_keyframes(os.path.join(data_dir, "cf_anmShapeBody.txt"))
            self.mapping = parse_mapping_csv(os.path.join(data_dir, "bone_mappings.csv"))

    def resolve(self, old_bone):
        """old → (new, mirrors)，csv 缺失时动态兜底"""
        if old_bone in self.mapping:
            new, mirrors = self.mapping[old_bone]
        else:
            new, mirrors = old_to_new_dynamic(old_bone), []
        return new, list(mirrors)

    def evaluate(self, sliders):
        """{cat_id: 0..1} → {target_bone: {'pos':[3], 'rot':[3]度, 'scale':[3]}}
        与 C# 一致：先按目标骨名累积（pos 加 / rot 加 / scale 乘），mirror 同步追加"""
        pos = {}
        rot = {}
        scale = {}
        missing_frames = set()
        for cat_id, val in sliders.items():
            for old_bone, mask in self.cat_map.get(int(cat_id), []):
                frames = self.frames.get(old_bone)
                if frames is None or len(frames) == 0:
                    missing_frames.add(old_bone)
                    continue
                interp = interpolate_keyframe(frames, val)
                neutral = frames[min(NEUTRAL_KEY, len(frames) - 1)]
                targets = [self.resolve(old_bone)]
                new_bone, mirrors = targets[0]
                if mirrors:
                    targets.append((None, mirrors))
                for _new, names in targets:
                    name_list = [new_bone] + mirrors if _new is None else [new_bone]
                    for tgt in name_list:
                        # 位置（加性，掩码轴才生效）
                        if mask[0] or mask[1] or mask[2]:
                            d = pos.get(tgt, [0.0, 0.0, 0.0])
                            for i in range(3):
                                if mask[i]:
                                    d[i] += (interp[i] - neutral[i]) / POS_DIVISOR
                            pos[tgt] = d
                        # 旋转（加性欧拉度）
                        if mask[3] or mask[4] or mask[5]:
                            d = rot.get(tgt, [0.0, 0.0, 0.0])
                            for i in range(3):
                                if mask[3 + i]:
                                    d[i] += _normalize_angle_deg(interp[3 + i] - neutral[3 + i])
                            rot[tgt] = d
                        # 缩放（乘性，相对中性帧）
                        if mask[6] or mask[7] or mask[8]:
                            s = scale.get(tgt, [1.0, 1.0, 1.0])
                            for i in range(3):
                                if mask[6 + i]:
                                    n = neutral[6 + i]
                                    ns = n if abs(n) > EPS else 1.0
                                    s[i] *= interp[6 + i] / ns
                            scale[tgt] = s
        bones = set(pos) | set(rot) | set(scale)
        out = {}
        for b in bones:
            out[b] = {
                "pos": pos.get(b, [0.0, 0.0, 0.0]),
                "rot": rot.get(b, [0.0, 0.0, 0.0]),
                "scale": scale.get(b, [1.0, 1.0, 1.0]),
            }
        return out, sorted(missing_frames)

    def categories_brief(self):
        """每分类：骨名 + 掩码轴名（供命名/核对）"""
        AX = ["px", "py", "pz", "rx", "ry", "rz", "sx", "sy", "sz"]
        out = {}
        for cid in sorted(self.cat_map):
            rows = []
            for bone, mask in self.cat_map[cid]:
                rows.append("%s[%s]" % (bone, "+".join(a for a, m in zip(AX, mask) if m) or "-"))
            out[cid] = rows
        return out


def main():
    ap = argparse.ArgumentParser(description="HS 捏人数据层")
    ap.add_argument("cmd", choices=["eval", "dump", "cats"])
    ap.add_argument("--data-dir", default="HoneySelect/Assets/Data")
    ap.add_argument("--face", default="{}", help='JSON: {"catId": 0..1}')
    ap.add_argument("--body", default="{}")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    dd = a.data_dir
    if a.cmd == "cats":
        for section in ("face", "body"):
            sd = ShapeData(dd, section)
            print("==== %s 分类 (%d) ====" % (section, len(sd.cat_map)))
            for cid, rows in sd.categories_brief().items():
                print(f"  {cid:3d}: " + " | ".join(rows))
        return
    result = {}
    for section in ("face", "body"):
        sd = ShapeData(dd, section)
        sl = json.loads(getattr(a, section))
        if not sl:
            # 显式空对象 = 不改；纯 dump 模式输出结构
            result[section] = {"_note": "no sliders"}
            continue
        deltas, missing = sd.evaluate(sl)
        result[section] = {"bones": deltas, "missing_frames": missing,
                           "n_bones": len(deltas)}
    text = json.dumps(result, indent=1, ensure_ascii=False)
    if a.out:
        with open(a.out, "w") as f:
            f.write(text)
        print("written:", a.out)
    else:
        print(text)


if __name__ == "__main__":
    main()
