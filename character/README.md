# character/ — 捏人（骨骼增量）模块

基于 HoneySelect 原作捏人数据表，把 0..1 的滑杆值评估为骨骼级位置/旋转/缩放增量，
并叠加到 Blender Armature 的 pose 上，实现角色形象定制。

## 文件

- `shape_data.py` — 数据层：解析 `cf_customhead.txt` / `cf_custombody.txt` /
  `cf_anmShapeHead.txt` / `cf_anmShapeBody.txt` / `*.csv` 映射，按 Unity
  `BoneShapeSliderRuntimeDriver.cs` 同款公式评估滑杆。
- `shape_apply.py` — 应用层：把 `shape_data.py` 输出的增量作用到
  `bpy.types.Armature` 的 pose bones。
- `category_names.py` — 分类 ID 的中文语义参考（face 66 项 / body 33 项）。
- `sliders.example.json` — 滑杆配置示例，可复制后修改。

## 滑杆配置格式

```json
{
  "face": {
    "0": 0.35,
    "5": 0.35,
    "6": 0.70
  },
  "body": {
    "0": 0.42,
    "1": 0.78
  }
}
```

- `0..1` 线性映射到数据表的关键帧范围；`0.5` 为默认中性。
- 未列出的分类保持中性值。
- 分类中文名见 `category_names.py`。

## 在渲染管线中使用

`pose/kp_model_render.py` 与 `pose/kp_trio_render.py` 均已集成 `--shape-json`
与 `--data-dir` 参数：

```bash
blender -b --factory-startup --python pose/kp_model_render.py -- \
    --fbx-dir /path/to/HoneySelect/Assets/Cosmetic \
    --data-dir /path/to/HoneySelect/Assets/Data \
    --shape-json character/sliders.example.json \
    --frames 1 --out pose/renders
```

数据表目录也可通过环境变量或软链接放在工程默认路径
`myhs2/HoneySelect/Assets/Data`，即可省略 `--data-dir`。

## 数据依赖

需要原作 `HoneySelect/Assets/Data/` 下的：

- `cf_customhead.txt`
- `cf_custombody.txt`
- `cf_anmShapeHead.txt`
- `cf_anmShapeBody.txt`
- `face_bone_mappings.csv`
- `bone_mappings.csv`

这些数据表**不**包含在本仓库中，需自备。

## 独立使用

```bash
# 查看所有分类及涉及的骨
python3 character/shape_data.py cats --data-dir /path/to/Data

# 评估一组滑杆
python3 character/shape_data.py eval --data-dir /path/to/Data \
    --face '{"0":0.35,"5":0.35}' --body '{"1":0.78}'
```
