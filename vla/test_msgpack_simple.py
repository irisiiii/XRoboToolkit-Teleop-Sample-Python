#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""简单测试 msgpack_numpy"""

import numpy as np
import msgpack
import msgpack_numpy

# 初始化
msgpack_numpy.patch()

print("=" * 60)
print("测试 1: 简单的 numpy 数组")
print("=" * 60)

# 创建测试数据
img = np.random.randint(0, 255, (3, 480, 640), dtype=np.uint8)
print(f"原始图像: shape={img.shape}, dtype={img.dtype}, type={type(img)}")

# 序列化
packed = msgpack_numpy.packb(img)
print(f"序列化后大小: {len(packed)} 字节")

# 反序列化
unpacked = msgpack_numpy.unpackb(packed)
print(f"反序列化后: shape={unpacked.shape}, dtype={unpacked.dtype}, type={type(unpacked)}")
print(f"数据一致: {np.array_equal(img, unpacked)}")

print("\n" + "=" * 60)
print("测试 2: 包含多个数组的字典")
print("=" * 60)

data = {
    "state": np.array([1.0, 2.0, 3.0], dtype=np.float32),
    "image": np.random.randint(0, 255, (3, 480, 640), dtype=np.uint8),
    "prompt": "test"
}

print("原始数据:")
for key, value in data.items():
    if isinstance(value, np.ndarray):
        print(f"  {key}: shape={value.shape}, dtype={value.dtype}")
    else:
        print(f"  {key}: {type(value).__name__}")

# 序列化
packed = msgpack_numpy.packb(data)
print(f"\n序列化后大小: {len(packed)} 字节")

# 反序列化
unpacked = msgpack_numpy.unpackb(packed)
print("\n反序列化后:")
for key, value in unpacked.items():
    if isinstance(value, np.ndarray):
        print(f"  {key}: shape={value.shape}, dtype={value.dtype}")
    else:
        print(f"  {key}: {type(value).__name__}")

print("\n" + "=" * 60)
print("测试 3: 使用 msgpack.packb (不使用 msgpack_numpy)")
print("=" * 60)

try:
    # 尝试不使用 msgpack_numpy
    packed_plain = msgpack.packb(data, default=msgpack_numpy.encode)
    unpacked_plain = msgpack.unpackb(packed_plain, object_hook=msgpack_numpy.decode)
    print("使用 default/object_hook 方式:")
    for key, value in unpacked_plain.items():
        if isinstance(value, np.ndarray):
            print(f"  {key}: shape={value.shape}, dtype={value.dtype}")
        else:
            print(f"  {key}: {type(value).__name__}")
except Exception as e:
    print(f"错误: {e}")

print("\n" + "=" * 60)
print("所有测试完成!")
print("=" * 60)


