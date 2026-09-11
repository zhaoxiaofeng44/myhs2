# -*- coding: utf-8 -*-
"""HS2 角色捏人（骨骼增量）模块。

基于 HoneySelect 的捏人数据表，把 0..1 的滑杆值评估为骨骼位置/旋转/缩放增量，
并作用到 Blender Armature 的 pose 上，实现用户自定义形象。
"""
from .shape_data import ShapeData, POS_DIVISOR, NEUTRAL_KEY, KEY_COUNT
from .shape_apply import apply_morph, reset_pose
from .category_names import CAT_ZH

__all__ = [
    "ShapeData",
    "apply_morph",
    "reset_pose",
    "CAT_ZH",
    "POS_DIVISOR",
    "NEUTRAL_KEY",
    "KEY_COUNT",
]
