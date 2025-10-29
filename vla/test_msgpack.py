#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试 msgpack_numpy 序列化和反序列化
"""

import numpy as np
import msgpack
import msgpack_numpy

# 初始化 msgpack_numpy
msgpack_numpy.patch()

# 创建测试数据
test_data = {
    "state": np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0], dtype=np.float32),
    "wrist_image_left": np.random.randint(0, 255, (3, 480, 640), dtype=np.uint8),
    "top": np.random.randint(0, 255, (3, 480, 640), dtype=np.uint8),
    "prompt": "test prompt"
}

print("=" * 60)
print("测试 msgpack_numpy 序列化/反序列化")
print("=" * 60)

print("\n1️⃣ 原始数据:")
for key, value in test_data.items():
    if isinstance(value, np.ndarray):
        print(f"   {key}: shape={value.shape}, dtype={value.dtype}, type={type(value)}")
    else:
        print(f"   {key}: {type(value).__name__}, value={value}")

# 序列化
print("\n2️⃣ 序列化...")
packed = msgpack_numpy.packb(test_data)
print(f"   序列化后大小: {len(packed)} 字节")

# 反序列化
print("\n3️⃣ 反序列化...")
unpacked = msgpack_numpy.unpackb(packed)
print("   反序列化后的数据:")
for key, value in unpacked.items():
    if isinstance(value, np.ndarray):
        print(f"   {key}: shape={value.shape}, dtype={value.dtype}, type={type(value)}")
    else:
        print(f"   {key}: {type(value).__name__}, value={value}")

# 验证数据完整性
print("\n4️⃣ 验证数据完整性:")
for key in test_data.keys():
    if isinstance(test_data[key], np.ndarray):
        if np.array_equal(test_data[key], unpacked[key]):
            print(f"   ✅ {key}: 数据一致")
        else:
            print(f"   ❌ {key}: 数据不一致!")
    else:
        if test_data[key] == unpacked[key]:
            print(f"   ✅ {key}: 数据一致")
        else:
            print(f"   ❌ {key}: 数据不一致!")

print("\n" + "=" * 60)
print("测试完成!")
print("=" * 60)


