#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试 msgpack_numpy 序列化"""

import numpy as np
import msgpack
import msgpack_numpy

# 创建测试数据
state = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0], dtype=np.float32)
wrist_image = np.random.randint(0, 256, (3, 480, 640), dtype=np.uint8)
top_image = np.random.randint(0, 256, (3, 480, 640), dtype=np.uint8)

print("=" * 60)
print("原始数据:")
print(f"state shape: {state.shape}, dtype: {state.dtype}")
print(f"wrist_image shape: {wrist_image.shape}, dtype: {wrist_image.dtype}")
print(f"top_image shape: {top_image.shape}, dtype: {top_image.dtype}")

# 创建观测字典
obs = {
    "state": state,
    "wrist_image_left": wrist_image,
    "top": top_image,
    "prompt": "Pick up the green bowl"
}

print("\n" + "=" * 60)
print("序列化...")
obs_packed = msgpack_numpy.packb(obs)
print(f"序列化后大小: {len(obs_packed)} 字节")

print("\n" + "=" * 60)
print("反序列化...")
obs_unpacked = msgpack_numpy.unpackb(obs_packed)

print(f"\n解包后的键: {list(obs_unpacked.keys())}")
for key, value in obs_unpacked.items():
    if isinstance(value, np.ndarray):
        print(f"{key}: shape={value.shape}, dtype={value.dtype}")
    else:
        print(f"{key}: type={type(value)}, value={value}")

print("\n" + "=" * 60)
print("验证数据完整性...")
assert np.array_equal(obs["state"], obs_unpacked["state"]), "state 不匹配!"
assert np.array_equal(obs["wrist_image_left"], obs_unpacked["wrist_image_left"]), "wrist_image_left 不匹配!"
assert np.array_equal(obs["top"], obs_unpacked["top"]), "top 不匹配!"
assert obs["prompt"] == obs_unpacked["prompt"], "prompt 不匹配!"

print("✅ 所有数据验证通过!")
print("\n" + "=" * 60)


