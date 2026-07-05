#!/usr/bin/env python3
"""诊断 Sony BoneData JSON 文件格式"""

import json
import sys
from pathlib import Path

def diagnose(json_file: Path) -> None:
    print(f"检查文件: {json_file}")
    print(f"文件存在: {json_file.exists()}")
    if not json_file.exists():
        print("❌ 文件不存在")
        return

    print(f"文件大小: {json_file.stat().st_size / 1024:.1f} KB")

    try:
        with open(json_file, 'r', encoding='utf-8-sig') as f:
            data = json.load(f)
    except Exception as e:
        print(f"❌ JSON 解析失败: {e}")
        return

    print(f"✓ JSON 解析成功")
    print(f"\n顶层字段: {list(data.keys())}")

    # 检查必需字段
    for key in ("name", "position", "rotation"):
        if key not in data:
            print(f"❌ 缺少字段: {key}")
            return

    name_len = len(data['name'])
    pos_len = len(data['position'])
    rot_len = len(data['rotation'])

    print(f"\n数组长度:")
    print(f"  name: {name_len}")
    print(f"  position: {pos_len}")
    print(f"  rotation: {rot_len}")

    # 检查元素类型
    print(f"\n元素类型:")
    print(f"  name[0]: {type(data['name'][0]).__name__} = {data['name'][0]}")
    print(f"  position[0]: {type(data['position'][0]).__name__} = {data['position'][0]}")
    print(f"  rotation[0]: {type(data['rotation'][0]).__name__} = {data['rotation'][0]}")

    # 判断格式
    pos_is_dict = isinstance(data['position'][0], dict)
    pos_is_list = isinstance(data['position'][0], list)
    pos_is_str = isinstance(data['position'][0], str)
    pos_is_num = isinstance(data['position'][0], (int, float))

    print(f"\nposition 格式判断:")
    if pos_is_dict:
        print(f"  ✓ 对象数组格式 (正确): {{{', '.join(data['position'][0].keys())}}}")
        if 'x' in data['position'][0]:
            print(f"    第一个 position = ({data['position'][0]['x']}, {data['position'][0]['y']}, {data['position'][0]['z']})")
    elif pos_is_list:
        print(f"  ✓ 嵌套列表格式 (正确): 长度 {len(data['position'][0])}")
        print(f"    第一个 position = {data['position'][0]}")
    elif pos_is_str or pos_is_num:
        print(f"  ❌ 平铺数值格式 (错误): 需要转换为嵌套格式")
        print(f"    前 3 个元素应该是第一个关节的 [x, y, z]")
        print(f"    position[0:3] = {data['position'][:3]}")

    rot_is_dict = isinstance(data['rotation'][0], dict)
    rot_is_list = isinstance(data['rotation'][0], list)
    rot_is_str = isinstance(data['rotation'][0], str)
    rot_is_num = isinstance(data['rotation'][0], (int, float))

    print(f"\nrotation 格式判断:")
    if rot_is_dict:
        print(f"  ✓ 对象数组格式 (正确): {{{', '.join(data['rotation'][0].keys())}}}")
        if 'x' in data['rotation'][0]:
            print(f"    第一个 rotation = ({data['rotation'][0]['x']}, {data['rotation'][0]['y']}, {data['rotation'][0]['z']}, {data['rotation'][0]['w']})")
    elif rot_is_list:
        print(f"  ✓ 嵌套列表格式 (正确): 长度 {len(data['rotation'][0])}")
        print(f"    第一个 rotation = {data['rotation'][0]}")
    elif rot_is_str or rot_is_num:
        print(f"  ❌ 平铺数值格式 (错误): 需要转换为嵌套格式")
        print(f"    前 4 个元素应该是第一个关节的 [x, y, z, w]")
        print(f"    rotation[0:4] = {data['rotation'][:4]}")

    # 计算帧数
    joints_per_frame = 27
    print(f"\n帧数计算 (假设 joints_per_frame={joints_per_frame}):")

    if pos_is_dict or pos_is_list:
        if pos_len % joints_per_frame == 0:
            frame_count = pos_len // joints_per_frame
            print(f"  ✓ position 可以整除: {pos_len} / {joints_per_frame} = {frame_count} 帧")
        else:
            print(f"  ❌ position 无法整除: {pos_len} % {joints_per_frame} = {pos_len % joints_per_frame}")
    else:
        # 平铺格式
        pos_elements_per_joint = 3
        total_pos_values = pos_len
        if total_pos_values % (joints_per_frame * pos_elements_per_joint) == 0:
            frame_count = total_pos_values // (joints_per_frame * pos_elements_per_joint)
            print(f"  平铺格式: {total_pos_values} / ({joints_per_frame} × 3) = {frame_count} 帧")
        else:
            print(f"  ❌ 无法计算帧数")

    if rot_is_dict or rot_is_list:
        if rot_len % joints_per_frame == 0:
            frame_count = rot_len // joints_per_frame
            print(f"  ✓ rotation 可以整除: {rot_len} / {joints_per_frame} = {frame_count} 帧")
        else:
            print(f"  ❌ rotation 无法整除: {rot_len} % {joints_per_frame} = {rot_len % joints_per_frame}")
    else:
        # 平铺格式
        rot_elements_per_joint = 4
        total_rot_values = rot_len
        if total_rot_values % (joints_per_frame * rot_elements_per_joint) == 0:
            frame_count = total_rot_values // (joints_per_frame * rot_elements_per_joint)
            print(f"  平铺格式: {total_rot_values} / ({joints_per_frame} × 4) = {frame_count} 帧")
        else:
            print(f"  ❌ 无法计算帧数")

    # name 模式
    print(f"\nname 数组模式:")
    if name_len == joints_per_frame:
        print(f"  ✓ 单帧模式: 27 个关节名")
    elif name_len == pos_len:
        print(f"  ✓ 全帧模式: 每帧重复关节名")
    else:
        print(f"  ❌ 长度不匹配")

    # 最终判断
    print(f"\n最终判断:")
    if (pos_is_dict or pos_is_list) and (rot_is_dict or rot_is_list) and pos_len == rot_len:
        print(f"  ✅ 格式正确，可以直接使用")
    elif (pos_is_str or pos_is_num) and (rot_is_str or rot_is_num):
        print(f"  ❌ 格式错误，需要转换")
        print(f"  需要运行转换脚本将平铺格式转为嵌套格式")
    else:
        print(f"  ⚠️  格式混合或不一致")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: diagnose_json_format.py <json_file>")
        sys.exit(1)

    diagnose(Path(sys.argv[1]).expanduser().resolve())
