# lerobot 0.33 版本创建数据集 av1 编码
# 灵活配置版本：自动检测相机文件夹，支持排除指定相机和机械臂数据
# 支持两种目录结构：
#   1. 三层结构：total_data_20251011/{jwq,ljy,zsq}/0xxx
#   2. 两层结构：20251015_side_cam_piper/0xxx (直接在根目录下)
# 支持新的JSON格式：包含data和metadata字段
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageOps
from tqdm import tqdm
import logging
from typing import List, Dict, Optional

# 配置日志记录
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

try:
    # 导入lerobot的核心类
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except ImportError:
    logging.error("无法导入 LeRobotDataset。请确保您已经正确安装了 lerobot 库。")
    logging.error("安装指南: https://github.com/huggingface/lerobot?tab=readme-ov-file#installation")
    exit(1)


def discover_camera_folders(episode_path: Path) -> List[str]:
    """
    自动发现episode目录中的所有相机文件夹。
    假设相机文件夹包含PNG图像文件。
    
    Returns:
        相机文件夹名称列表，按字母顺序排序
    """
    camera_folders = []
    for item in episode_path.iterdir():
        if item.is_dir():
            # 检查文件夹中是否有PNG图像
            png_files = list(item.glob("*.png"))
            if png_files:
                camera_folders.append(item.name)
    
    camera_folders.sort()
    logging.info(f"发现相机文件夹: {camera_folders}")
    return camera_folders


def get_image_properties(episode_path: Path, camera_folders: List[str]) -> tuple[int, int, int]:
    """
    从第一个相机文件夹的第一个图像中获取图像的高度、宽度和通道数。
    """
    for camera_name in camera_folders:
        camera_path = episode_path / camera_name
        first_image_path = next(camera_path.glob("*.png"), None)
        if first_image_path:
            with Image.open(first_image_path) as img:
                width, height = img.size
                channels = len(img.getbands())
                logging.info(f"从 {camera_name} 检测到图像属性: 高={height}, 宽={width}, 通道={channels}")
                return height, width, channels
    
    raise FileNotFoundError("在相机目录中找不到任何PNG图像来确定图像属性。")


def parse_arm_keys(arm_keys_str: str) -> List[str]:
    """
    解析逗号分隔的机械臂键名字符串。
    
    Args:
        arm_keys_str: 逗号分隔的键名，例如 "left_arm_gripper_state,left_arm_joint"
    
    Returns:
        键名列表
    """
    if not arm_keys_str:
        return []
    return [key.strip() for key in arm_keys_str.split(',') if key.strip()]


def calculate_state_dim(arm_keys: List[str], sample_data: dict) -> int:
    """
    根据指定的机械臂键名和样本数据计算状态维度。
    
    Args:
        arm_keys: 机械臂数据键名列表
        sample_data: 来自obs.json的样本数据帧
    
    Returns:
        状态总维度
    """
    total_dim = 0
    
    for key in arm_keys:
        if key.endswith('_gripper_state'):
            # 夹爪状态取第一个值
            total_dim += 1
        elif key.endswith('_joint'):
            # 关节数据，取所有关节值（7轴节卡机器人）
            joint_data = sample_data.get(key, [])
            joint_count = len(joint_data)
            total_dim += joint_count
        elif key.endswith('_tcp'):
            # TCP数据通常不用于状态，但如果需要可以添加
            pass
    
    logging.info(f"计算得到状态维度: {total_dim}, 使用的键: {arm_keys}")
    return total_dim


def define_features(height: int, width: int, channels: int, 
                    camera_folders: List[str], 
                    arm_keys: List[str],
                    sample_data: dict) -> dict:
    """
    根据相机列表和机械臂键名定义LeRobot数据集的features。
    键名必须以 'observation.' 或 'action' 开头。
    
    Args:
        height: 图像高度
        width: 图像宽度
        channels: 图像通道数
        camera_folders: 相机文件夹名称列表
        arm_keys: 机械臂数据键名列表
        sample_data: 样本数据，用于计算维度
    """
    features = {}
    
    # 添加相机特征
    for camera_name in camera_folders:
        # 构建observation键名，例如 "observation.wrist_image_left" 或 "observation.low"
        if camera_name in ['left', 'right']:
            obs_key = f"observation.wrist_image_{camera_name}"
        else:
            obs_key = f"observation.{camera_name}"
        
        features[obs_key] = {
            "dtype": "video",
            "shape": (height, width, channels),
            "names": ["height", "width", "channels"],
        }
    
    logging.info(f"已定义 {len(camera_folders)} 个相机特征: {list(features.keys())}")
    
    # 计算状态维度
    state_dim = calculate_state_dim(arm_keys, sample_data)
    
    if state_dim == 0:
        logging.warning("警告：未找到任何有效的机械臂状态数据！")
        return features
    
    # 构建状态和动作的维度名称
    dim_names = []
    for key in arm_keys:
        if key.endswith('_gripper_state'):
            # 获取臂名称，例如 "left_arm" 或 "right_arm"
            arm_name = key.replace('_gripper_state', '')
            dim_names.append(f"{arm_name}_gripper_pos")
        elif key.endswith('_joint'):
            arm_name = key.replace('_joint', '')
            joint_data = sample_data.get(key, [])
            joint_count = len(joint_data)  # 7轴节卡机器人
            for i in range(1, joint_count + 1):
                dim_names.append(f"{arm_name}_joint_{i}")
    
    # 添加状态特征
    features["observation.state"] = {
        "dtype": "float32",
        "shape": (state_dim,),
        "names": dim_names
    }
    
    # 添加动作特征
    features["action"] = {
        "dtype": "float32",
        "shape": (state_dim,),
        "names": dim_names
    }
    
    logging.info(f"已定义状态/动作维度: {state_dim}, 维度名称: {dim_names}")
    return features


def extract_arm_state(frame_info: dict, arm_keys: List[str]) -> np.ndarray:
    """
    从帧信息中提取机械臂状态数据。
    
    Args:
        frame_info: 单帧的JSON数据
        arm_keys: 要提取的机械臂键名列表
    
    Returns:
        状态向量（numpy数组）
    """
    state_components = []
    
    for key in arm_keys:
        try:
            if key.endswith('_gripper_state'):
                # 夹爪状态取第一个值
                gripper_data = frame_info.get(key, [0.0, 0.0, 0.0, 0.0])
                state_components.append(gripper_data[0])
            elif key.endswith('_joint'):
                # 关节数据，取所有7个关节值（节卡机器人）
                joint_data = frame_info.get(key, [0.0]*7)
                state_components.extend(joint_data)
            # TCP数据暂不处理
        except (KeyError, IndexError, TypeError) as e:
            logging.warning(f"提取键 '{key}' 时出错: {e}，使用零填充。")
            if key.endswith('_gripper_state'):
                state_components.append(0.0)
            elif key.endswith('_joint'):
                state_components.extend([0.0]*7)  # 7轴节卡机器人
    
    return np.array(state_components, dtype=np.float32)


def process_episode(episode_dir: Path, dataset: LeRobotDataset, task_prompt: str, 
                   max_frames: int | None, camera_folders: List[str], 
                   arm_keys: List[str], apply_flip: bool):
    """
    处理单个episode的数据并将其添加到LeRobot数据集中。
    
    Args:
        episode_dir: episode目录路径
        dataset: LeRobot数据集对象
        task_prompt: 任务提示文本
        max_frames: 最大帧数限制
        camera_folders: 要处理的相机文件夹列表
        arm_keys: 要提取的机械臂数据键名列表
        apply_flip: 是否对图像进行左右翻转
    """
    logging.info(f"正在处理 episode: {episode_dir}")
  
    json_path = episode_dir / "obs.json"
    if not json_path.exists():
        logging.warning(f"在 {episode_dir} 中未找到 obs.json，跳过此目录。")
        return

    with open(json_path, 'r') as f:
        try:
            obs_data = json.load(f)
        except json.JSONDecodeError:
            logging.error(f"无法解析 {json_path}。文件可能已损坏。")
            return
    
    # 提取frames数据，忽略metadata
    if "data" in obs_data:
        joint_data = obs_data["data"]
        logging.info(f"新格式检测到，data字段包含 {len(joint_data)} 帧")
    else:
        # 兼容旧格式，直接使用整个JSON作为数据
        joint_data = obs_data
        logging.info(f"旧格式检测到，直接使用整个JSON作为数据")
          
    # 按 id 排序以确保顺序正确
    if isinstance(joint_data, list):
        joint_data.sort(key=lambda x: x.get('id', 0))
    else:
        logging.error(f"obs.json格式错误：期望列表格式，但得到 {type(joint_data)}")
        return

    # 动态确定并截断数据（检查所有相机图像）
    num_json_frames = len(joint_data)
    
    # 检查所有相机的图像数量
    camera_frame_counts = {}
    for camera_name in camera_folders:
        camera_path = episode_dir / camera_name
        if camera_path.exists():
            num_frames = len(list(camera_path.glob("*.png")))
            camera_frame_counts[camera_name] = num_frames
        else:
            logging.warning(f"相机文件夹不存在: {camera_path}")
            camera_frame_counts[camera_name] = 0
    
    num_valid_frames = min(num_json_frames, *camera_frame_counts.values()) if camera_frame_counts else 0

    logging.info(
        f"帧数一致性检查 ({episode_dir.name}): "
        f"JSON={num_json_frames}, 相机={camera_frame_counts}"
    )

    if max_frames is not None:
        if num_valid_frames > max_frames:
            logging.info(f"应用用户定义的最大帧数限制: {max_frames} 帧。")
            num_valid_frames = max_frames

    if num_valid_frames < num_json_frames:
        logging.warning(
            f"数据将被截断：从 {num_json_frames} 帧 (JSON) 截断为 {num_valid_frames} 帧以匹配可用的图像数据。"
        )
  
    joint_data = joint_data[:num_valid_frames]

    # 首先提取所有帧的状态数据
    all_states = []
    for frame_info in joint_data:
        state = extract_arm_state(frame_info, arm_keys)
        all_states.append(state)
    
    all_states = np.array(all_states)
    
    # 计算actions：下一帧的状态作为当前帧的动作（标准模仿学习设置）
    all_actions = np.roll(all_states, -1, axis=0)  # 向前移动一位
    all_actions[-1] = all_states[-1]  # 最后一帧的action设为当前状态

    # 构建相机到特征的映射
    cam_to_feature_map = {}
    for camera_name in camera_folders:
        if camera_name in ['left', 'right']:
            feature_key = f"observation.wrist_image_{camera_name}"
        else:
            feature_key = f"observation.{camera_name}"
        cam_to_feature_map[camera_name] = feature_key

    for idx, frame_info in enumerate(tqdm(joint_data, desc=f"处理 {episode_dir.name} 的帧")):
        frame_id = frame_info["id"]
      
        frame_to_add = {}

        try:
            for cam_source_key, feature_target_key in cam_to_feature_map.items():
                # 图像文件名从0开始编号，直接使用frame_id
                img_path = episode_dir / cam_source_key / f"{frame_id}.png"
                with Image.open(img_path) as img:
                    if apply_flip:
                        flipped_img = img.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
                        frame_to_add[feature_target_key] = flipped_img
                    else:
                        frame_to_add[feature_target_key] = img.copy()
        except FileNotFoundError as e:
            logging.error(f"出现意外错误：找不到图像文件 {e.filename}，即使在截断后。请检查数据。跳过此帧。")
            continue

        # 使用预计算的状态和动作
        frame_to_add["observation.state"] = all_states[idx]
        frame_to_add["action"] = all_actions[idx]  # action是下一帧的状态

        # 将 timestamp 设置为 None，让 LeRobotDataset 自动生成标准化的时间戳
        dataset.add_frame(frame=frame_to_add, task=task_prompt, timestamp=None)

    dataset.save_episode()
    logging.info(f"Episode {episode_dir.name} 已成功保存。")


def main():
    parser = argparse.ArgumentParser(
        description="将自定义采集的机器人数据转换为LeRobot格式。支持灵活配置相机和机械臂数据。"
    )
    parser.add_argument(
        "--source-dir",
        type=str,
        default="/root/gr00t_n15/data/4cam",
        help="包含所有episode子目录的源数据根目录。",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="/root/gr00t_n15/data/4cam_lerobot",
        help="用于保存生成的LeRobot数据集的输出目录。",
    )
    parser.add_argument(
        "--repo-id",
        type=str,
        default="4cam_piper_20251020",
        help="为数据集指定的Hugging Face Hub仓库ID (例如 'my-username/my-robot-dataset')。",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=20,
        help="数据采集的帧率 (FPS)。",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="（可选）为每个episode设置一个处理的最大帧数。例如，设置为350可强制截断。",
    )
    parser.add_argument(
        "--exclude-cameras",
        type=str,
        default="right",
        help="要排除的相机文件夹名称，逗号分隔。例如：'right' 或 'right,top'。默认排除 'right'。",
    )
    parser.add_argument(
        "--arm-keys",
        type=str,
        default="left_arm_gripper_state,left_arm_joint",
        help="要从obs.json中提取的机械臂数据键名，逗号分隔。"
             "默认：'left_arm_gripper_state,left_arm_joint'。"
             "可用键示例：left_arm_gripper_state, left_arm_joint, left_arm_tcp, "
             "right_arm_gripper_state, right_arm_joint, right_arm_tcp",
    )
    parser.add_argument(
        "--robot-type",
        type=str,
        default="piper_dual_arm",
        help="机器人类型标识。",
    )
    parser.add_argument(
        "--task",
        type=str,
        default="Pick up the green bowl and put it into the brown woven basket.",
        help="任务描述文本。",
    )
    parser.add_argument(
        "--no-flip",
        action="store_true",
        help="禁用图像左右翻转。默认情况下会翻转图像。",
    )
    parser.add_argument(
        "--test-episodes",
        type=int,
        default=1,
        help="用作测试集的episode数量（从最后开始取）。默认为1。设置为0则不创建测试集。",
    )
    args = parser.parse_args()

    source_path = Path(args.source_dir)
    output_path = Path(args.output_dir)
    
    if not source_path.is_dir():
        logging.error(f"源目录不存在: {source_path}")
        return

    # 智能查找所有episode目录 - 支持两种目录结构
    episode_dirs = []
    
    # 首先检查根目录下是否直接包含数字命名的episode目录（两层结构）
    direct_episodes = [d for d in source_path.iterdir() if d.is_dir() and d.name.isdigit()]
    
    if direct_episodes:
        # 两层结构：source_dir/0xxx
        logging.info(f"检测到两层目录结构（根目录直接包含episode）")
        episode_dirs = direct_episodes
    else:
        # 三层结构：source_dir/subdir/0xxx
        logging.info(f"检测到三层目录结构（需要扫描子目录）")
        for subdir in source_path.iterdir():
            if subdir.is_dir():
                logging.info(f"扫描子目录: {subdir.name}")
                # 在子目录中查找数字命名的episode目录
                for episode_dir in subdir.iterdir():
                    if episode_dir.is_dir() and episode_dir.name.isdigit():
                        episode_dirs.append(episode_dir)
    
    episode_dirs = sorted(episode_dirs)
    
    if not episode_dirs:
        logging.error(f"在 {source_path} 中未找到任何episode子目录。")
        return
        
    logging.info(f"找到 {len(episode_dirs)} 个 episodes 进行转换。")

    # 从第一个episode发现相机文件夹
    all_camera_folders = discover_camera_folders(episode_dirs[0])
    
    # 解析要排除的相机
    exclude_cameras = parse_arm_keys(args.exclude_cameras)
    
    # 过滤掉要排除的相机
    camera_folders = [cam for cam in all_camera_folders if cam not in exclude_cameras]
    
    if not camera_folders:
        logging.error("没有可用的相机文件夹进行处理！")
        return
    
    logging.info(f"将处理以下相机: {camera_folders}")
    if exclude_cameras:
        logging.info(f"已排除相机: {exclude_cameras}")
    
    # 解析机械臂键名
    arm_keys = parse_arm_keys(args.arm_keys)
    
    if not arm_keys:
        logging.error("未指定任何机械臂数据键！")
        return
    
    logging.info(f"将处理以下机械臂数据键: {arm_keys}")

    # 获取图像属性
    try:
        height, width, channels = get_image_properties(episode_dirs[0], camera_folders)
    except FileNotFoundError as e:
        logging.error(e)
        return

    # 读取第一个episode的第一帧作为样本，用于计算维度
    json_path = episode_dirs[0] / "obs.json"
    with open(json_path, 'r') as f:
        obs_data = json.load(f)
    
    if "data" in obs_data:
        sample_data = obs_data["data"][0]
    else:
        sample_data = obs_data[0]

    # 定义features
    features = define_features(height, width, channels, camera_folders, arm_keys, sample_data)
    
    # 分离训练集和测试集
    test_episodes_count = args.test_episodes
    if test_episodes_count < 0:
        logging.error("测试集episode数量不能为负数！")
        return
    
    if test_episodes_count >= len(episode_dirs):
        logging.error(f"测试集episode数量({test_episodes_count})不能大于或等于总episode数量({len(episode_dirs)})！")
        return
    
    if test_episodes_count > 0:
        train_episodes = episode_dirs[:-test_episodes_count]
        test_episodes = episode_dirs[-test_episodes_count:]
        logging.info(f"数据集划分: 训练集 {len(train_episodes)} episodes, 测试集 {len(test_episodes)} episodes")
        logging.info(f"训练集: {train_episodes[0].name} 到 {train_episodes[-1].name}")
        logging.info(f"测试集: {test_episodes[0].name} 到 {test_episodes[-1].name}")
    else:
        train_episodes = episode_dirs
        test_episodes = []
        logging.info(f"未创建测试集，所有 {len(train_episodes)} episodes 用于训练")
    
    apply_flip = not args.no_flip
    logging.info(f"图像翻转: {'启用' if apply_flip else '禁用'}")

    # 创建训练集
    logging.info(f"正在创建训练集，保存在: {output_path / 'train' / args.repo_id}")
    train_dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        root=output_path / "train",
        features=features,
        fps=args.fps,
        robot_type=args.robot_type,
    )

    logging.info("=" * 70)
    logging.info("开始处理训练集")
    logging.info("=" * 70)
    for episode_dir in train_episodes:
        process_episode(episode_dir, train_dataset, args.task, args.max_frames, 
                       camera_folders, arm_keys, apply_flip)

    # 创建测试集（如果有）
    if test_episodes:
        logging.info("=" * 70)
        logging.info("开始处理测试集")
        logging.info("=" * 70)
        logging.info(f"正在创建测试集，保存在: {output_path / 'test' / args.repo_id}")
        test_dataset = LeRobotDataset.create(
            repo_id=args.repo_id,
            root=output_path / "test",
            features=features,
            fps=args.fps,
            robot_type=args.robot_type,
        )
        
        for episode_dir in test_episodes:
            process_episode(episode_dir, test_dataset, args.task, args.max_frames, 
                           camera_folders, arm_keys, apply_flip)

    logging.info("=" * 70)
    logging.info("所有episodes处理完毕！")
    logging.info("=" * 70)
    logging.info(f"训练集已生成在: {output_path / 'train' / args.repo_id}")
    logging.info(f"训练集包含 {len(train_episodes)} 个episodes")
    if test_episodes:
        logging.info(f"测试集已生成在: {output_path / 'test' / args.repo_id}")
        logging.info(f"测试集包含 {len(test_episodes)} 个episodes")
    logging.info(f"相机: {camera_folders}")
    logging.info(f"机械臂数据键: {arm_keys}")


if __name__ == "__main__":
    main()

