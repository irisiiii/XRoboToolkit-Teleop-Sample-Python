# Frame Sync 配置说明

## 概述
Frame Sync 节点支持两种数据采集启动方式，并提供了丰富的配置选项来定制采集行为。

## 配置文件位置
```
src/frame_sync/config/frame_sync.yaml
```

## 主要配置参数

### 基础配置
```yaml
frame_sync_node:
  ros__parameters:
    publisher_hz: 200                    # 发布频率
    enable_save_dataset: true            # 是否开启dataset保存模式
    enable_limit_numbers_of_frames_to_save: true  # 是否开启限制帧数
    limit_numbers_of_frames_to_save: 900          # 最多采集多少帧数
```

### 夹爪触发配置
```yaml
    gripper_trigger_timeout_sec: 4.0     # 夹爪序列检测超时时间（秒）
    gripper_trigger_countdown_sec: 3.0   # 夹爪触发后倒计时秒数
    enable_continuous_collection: true   # 是否开启连续采集模式
```

### Topic 配置
```yaml
    camera_topic_names:
      - "/left_camera/color/image_raw"
      - "/right_camera/color/image_raw"
      - "/top_camera/color/image_raw"
    
    arm_joints_topic_names:
      - "/left_arm/joint_states"
      - "/right_arm/joint_states"
    
    tcp_topic_names:
      - "left_arm/tcp_pose"
      - "/right_arm/tcp_pose"
    
    gripper_topic_names:
      - "/left_arm/gripper_state"
      - "/right_arm/gripper_state"
```

## 功能详解

### 1. 数据采集启动方式

#### 方式1: 手动服务调用
```bash
ros2 service call /start_capture std_srvs/srv/Trigger "{}"
```
- **特点**: 立即开始采集，无倒计时
- **适用场景**: 精确控制启动时机

#### 方式2: 夹爪序列触发
- **序列**: 闭合(<5) → 开启(>20) → 闭合(<5) → 开启(>20)
- **特点**: 触发后显示倒计时，然后开始采集
- **适用场景**: 现场操作，无需额外终端

### 2. 倒计时功能
- **参数**: `gripper_trigger_countdown_sec`
- **功能**: 夹爪触发后显示倒计时，给操作者准备时间
- **显示**: 实时更新倒计时数字，彩色终端输出

### 3. 采集模式

#### 连续采集模式 (`enable_continuous_collection: true`)
- 完成一轮采集后自动重新开始监听
- 适合需要多轮数据采集的场景
- 支持无限次循环采集

#### 单次采集模式 (`enable_continuous_collection: false`)
- 完成一轮采集后自动关闭节点
- 适合单次数据收集任务

### 4. 状态管理
系统具有完整的状态管理机制：

- **WAITING_FOR_TRIGGER**: 等待触发（监听夹爪/服务调用）
- **COUNTDOWN_ACTIVE**: 倒计时进行中（不响应新触发）
- **COLLECTING_DATA**: 数据采集进行中（不响应新触发）
- **COLLECTION_COMPLETE**: 采集完成（准备重置或关闭）

## 使用建议

### 推荐配置
```yaml
# 实验室环境推荐配置
gripper_trigger_countdown_sec: 3.0      # 3秒倒计时
enable_continuous_collection: true      # 连续采集
limit_numbers_of_frames_to_save: 900    # 每轮900帧

# 生产环境推荐配置
gripper_trigger_countdown_sec: 5.0      # 5秒倒计时
enable_continuous_collection: false     # 单次采集
limit_numbers_of_frames_to_save: 1500   # 单次1500帧
```

### 参数调整指南

1. **倒计时时间**: 根据操作者需要的准备时间调整
2. **超时时间**: 根据夹爪操作的熟练度调整
3. **帧数限制**: 根据存储空间和数据需求调整
4. **连续采集**: 根据实验流程需要选择

## 故障排除

### 常见问题

1. **夹爪触发不响应**
   - 检查夹爪数值范围是否符合要求（<5 或 >20）
   - 检查是否在超时时间内完成序列

2. **倒计时不显示**
   - 检查终端是否支持彩色输出
   - 确认配置文件中的倒计时时间大于0

3. **连续采集不工作**
   - 确认 `enable_continuous_collection` 设置为 true
   - 检查帧数限制是否已达到

### 调试命令
```bash
# 查看节点状态
ros2 node info /frame_sync_node

# 监听夹爪状态
ros2 topic echo /left_arm/gripper_state

# 查看服务是否可用
ros2 service list | grep start_capture

# 手动测试服务
ros2 service call /start_capture std_srvs/srv/Trigger "{}"
```

## 日志输出

系统提供丰富的彩色日志输出：
- 🟢 绿色: 系统提示和成功信息
- 🔵 蓝色: 操作指引
- 🟡 黄色: 状态变化和倒计时
- 🟣 紫色: 模式说明
- 🔴 红色: 错误信息（如有）

## 版本更新

### v2.0 新功能
- ✅ 倒计时功能
- ✅ 连续采集支持
- ✅ 状态管理优化
- ✅ 配置参数扩展
- ✅ 彩色终端输出增强