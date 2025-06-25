from typing import Any, Dict

import meshcat.transformations as tf
import mujoco
import numpy as np
import placo
from mujoco import viewer as mj_viewer
from placo_utils.visualization import (
    frame_viz,
    robot_frame_viz,
    robot_viz,
)

from xrobotoolkit_teleop.common.xr_client import XrClient
from xrobotoolkit_teleop.utils.geometry import (
    R_HEADSET_TO_WORLD,
    apply_delta_pose,
    quat_diff_as_angle_axis,
)
from xrobotoolkit_teleop.utils.mujoco_utils import (
    calc_mujoco_ctrl_from_qpos,
    calc_mujoco_qpos_from_placo_q,
    calc_placo_q_from_mujoco_qpos,
    set_mujoco_joint_pos_by_name,
)
from xrobotoolkit_teleop.utils.parallel_gripper_utils import (
    calc_parallel_gripper_position,
)


class MujocoTeleopController:
    def __init__(
        self,
        xml_path: str,
        robot_urdf_path: str,
        end_effector_config: Dict[str, Dict[str, Any]],
        floating_base=False,
        R_headset_world=R_HEADSET_TO_WORLD,
        visualize_placo=False,
        scale_factor=1.0,
        mj_qpos_init=None,
    ):
        """
        Initialize the teleoperation controller.

        Args:
            xml_path: Path to the MuJoCo XML scene file.
            robot_urdf_path: Path to the robot URDF file for IK solving.
            end_effector_config: Dictionary containing end effector configurations.
            floating_base: Whether the robot has a floating base.
            R_headset_world: Rotation matrix to transform headset coordinates to world coordinates.
            visualize_placo: Whether to visualize the placo IK solver.
            scale_factor: Scale factor for teleoperation.
            mj_qpos_init: Initial joint configuration. If None, uses 'home' keyframe from MuJoCo model.
        """
        self.xml_path = xml_path
        self.robot_urdf_path = robot_urdf_path
        self.end_effector_config = end_effector_config
        self.floating_base = floating_base
        self.R_headset_world = R_headset_world
        self.visualize_placo = visualize_placo
        self.scale_factor = scale_factor
        self.mj_qpos_init = mj_qpos_init
        self.mj_qpos_curr = mj_qpos_init

        # To be initialized later
        self.mj_model = None
        self.mj_data = None
        self.placo_robot = None
        self.solver = None
        self.dt = None
        self.t = 0
        self.effector_task = {name: None for name in end_effector_config.keys()}
        self.target_mocap_idx = {name: -1 for name in end_effector_config.keys()}

        # Initial poses
        self.init_ee_xyz = {name: None for name in end_effector_config.keys()}
        self.init_ee_quat = {name: None for name in end_effector_config.keys()}
        self.init_controller_xyz = {name: None for name in end_effector_config.keys()}
        self.init_controller_quat = {name: None for name in end_effector_config.keys()}
        self.gripper_pos_target = {}
        for name, config in end_effector_config.items():
            if "gripper_config" in config:
                gripper_config = config["gripper_config"]
                self.gripper_pos_target[gripper_config["joint_name"]] = gripper_config["open_pos"]

        self.xr_client = XrClient()

        self._setup_mujoco()
        self._setup_placo()

    def _get_end_effector_info(self, ee_name):
        """Get the end effector position and orientation."""
        ee_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, ee_name)
        if ee_id == -1:
            raise ValueError(f"End effector body '{ee_name}' not found in the model.")

        ee_xyz = self.mj_data.xpos[ee_id].copy()
        ee_quat = self.mj_data.xquat[ee_id].copy()

        return ee_xyz, ee_quat

    def _setup_mujoco(self):
        # Load MuJoCo model
        self.mj_model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.mj_data = mujoco.MjData(self.mj_model)

        # Print all joint names
        print("Joint names in the Mujoco model:")
        for i in range(self.mj_model.njnt):
            joint_name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, i)
            print(f"  {joint_name}")

        # Configure scene lighting
        self.mj_model.vis.headlight.ambient = [0.4, 0.4, 0.4]
        self.mj_model.vis.headlight.diffuse = [0.8, 0.8, 0.8]
        self.mj_model.vis.headlight.specular = [0.6, 0.6, 0.6]

        # Set initial position from keyframe if available
        mujoco.mj_resetData(self.mj_model, self.mj_data)
        if self.mj_qpos_init is None:
            mujoco.mj_resetDataKeyframe(self.mj_model, self.mj_data, self.mj_model.key("home").id)
        else:
            self.mj_data.qpos[:] = self.mj_qpos_init
            self.mj_data.ctrl[:] = self.mj_qpos_init
            mujoco.mj_forward(self.mj_model, self.mj_data)

        # Calculate forward kinematics to get initial end effector pose
        mujoco.mj_forward(self.mj_model, self.mj_data)
        self._setup_mocap_target()

    def _setup_placo(self):
        """Set up the placo inverse kinematics solver."""
        self.dt = self.mj_model.opt.timestep
        self.placo_robot = placo.RobotWrapper(self.robot_urdf_path)
        self.solver = placo.KinematicsSolver(self.placo_robot)
        self.solver.dt = self.dt
        self.solver.add_kinetic_energy_regularization_task(1e-6)

        # Print all joint names from placo model
        print("Joint names in the Placo model:")
        for joint_name in self.placo_robot.model.names:
            print(f"  {joint_name}")

        for name, config in self.end_effector_config.items():
            ee_xyz, ee_quat = self._get_end_effector_info(config["link_name"])
            ee_target = tf.quaternion_matrix(ee_quat)
            ee_target[:3, 3] = ee_xyz
            self.effector_task[name] = self.solver.add_frame_task(config["link_name"], ee_target)
            self.effector_task[name].configure(name, "soft", 1.0)
            manipulability = self.solver.add_manipulability_task(config["link_name"], "both", 1.0)
            manipulability.configure("manipulability", "soft", 5e-2)

        # if self.floating_base:
        #     self.placo_robot.state.q = self.mj_data.qpos.copy()
        # else:
        #     self.solver.mask_fbase(True)
        #     self.placo_robot.state.q[7:] = self.mj_data.qpos.copy()
        if not self.floating_base:
            self.solver.mask_fbase(True)

        self.placo_robot.state.q = calc_placo_q_from_mujoco_qpos(
            self.mj_model,
            self.placo_robot,
            self.mj_data.qpos.copy(),
            floating_base=self.floating_base,
        )

        if self.visualize_placo:
            self.placo_robot.update_kinematics()
            self.placo_vis = robot_viz(self.placo_robot)
            self.placo_vis.display(self.placo_robot.state.q)
            for name, config in self.end_effector_config.items():
                robot_frame_viz(self.placo_robot, config["link_name"])
                frame_viz(f"vis_target_{name}", self.effector_task[name].T_world_frame)

    def _setup_mocap_target(self):
        """Find and set up the mocap target body."""
        for name, config in self.end_effector_config.items():
            if "vis_target" not in config:
                print(f"Warning: 'vis_target' not found in config for {name}. Skipping mocap setup.")
                continue
            vis_target = config["vis_target"]
            mocap_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, vis_target)
            if mocap_id == -1:
                raise ValueError(f"Mocap body '{vis_target}' not found in the model.")

            if self.mj_model.body_mocapid[mocap_id] == -1:
                raise ValueError(f"Body '{self.vis_target}' is not configured for mocap.")
            else:
                self.target_mocap_idx[name] = self.mj_model.body_mocapid[mocap_id]

            print(f"Mocap ID for '{vis_target}' body: {self.target_mocap_idx[name]}")

    def run(self):
        """Run the main teleoperation loop."""

        with mj_viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            # Set up viewer camera
            viewer.cam.azimuth = 0
            viewer.cam.elevation = -50
            viewer.cam.distance = 2.0
            viewer.cam.lookat = [0.2, 0, 0]

            while True:

                self.t += self.dt

                for name, config in self.end_effector_config.items():
                    xr_grip = self.xr_client.get_key_value_by_name(config["control_trigger"])
                    active = xr_grip > 0.5

                    # Process current pose
                    if active:
                        if self.init_ee_xyz[name] is None:
                            self.init_ee_xyz[name], self.init_ee_quat[name] = self._get_end_effector_info(
                                config["link_name"]
                            )
                            print(f"{name} is activated.")
                        xr_pose = self.xr_client.get_pose_by_name(config["pose_source"])
                        delta_xyz, delta_rot = self._process_xr_pose(xr_pose, name)

                        target_xyz, target_quat = apply_delta_pose(
                            self.init_ee_xyz[name],
                            self.init_ee_quat[name],
                            delta_xyz,
                            delta_rot,
                        )

                        target = tf.quaternion_matrix(target_quat)
                        target[:3, 3] = target_xyz

                        self.effector_task[name].T_world_frame = target

                        if name in self.target_mocap_idx:
                            target_mocap_id = self.target_mocap_idx[name]
                            if target_mocap_id != -1:
                                self.mj_data.mocap_pos[target_mocap_id] = target_xyz
                                self.mj_data.mocap_quat[target_mocap_id] = target_quat
                    else:
                        if self.init_ee_xyz[name] is not None:
                            # Reset the initial pose when the controller is released
                            self.init_ee_xyz[name] = None
                            self.init_ee_quat[name] = None
                            self.init_controller_xyz[name] = None
                            self.init_controller_quat[name] = None
                            print(f"{name} is deactivated.")

                    if "gripper_config" in config:
                        gripper_config = config["gripper_config"]
                        trigger_value = self.xr_client.get_key_value_by_name(gripper_config["gripper_trigger"])
                        gripper_pos = calc_parallel_gripper_position(
                            gripper_config["open_pos"],
                            gripper_config["close_pos"],
                            trigger_value,
                        )
                        joint_name = gripper_config["joint_name"]
                        self.gripper_pos_target[joint_name] = gripper_pos

                self._update_kinematics()

                # Step simulation and update viewer
                mujoco.mj_step(self.mj_model, self.mj_data)
                viewer.sync()

    def _process_xr_pose(self, xr_pose, src_name):
        """Process the current XR controller pose."""
        # Get position and orientation
        controller_xyz = np.array([xr_pose[0], xr_pose[1], xr_pose[2]])
        controller_quat = [
            xr_pose[6],  # w
            xr_pose[3],  # x
            xr_pose[4],  # y
            xr_pose[5],  # z
        ]

        controller_xyz = self.R_headset_world @ controller_xyz

        R_transform = np.eye(4)
        R_transform[:3, :3] = self.R_headset_world
        R_quat = tf.quaternion_from_matrix(R_transform)
        controller_quat = tf.quaternion_multiply(
            tf.quaternion_multiply(R_quat, controller_quat),
            tf.quaternion_conjugate(R_quat),
        )

        if self.init_controller_xyz[src_name] is None:
            # First time processing the pose
            self.init_controller_xyz[src_name] = controller_xyz
            self.init_controller_quat[src_name] = controller_quat

            delta_xyz = np.zeros(3)
            delta_rot = np.array([0.0, 0.0, 0.0])
        else:
            # Calculate relative transformation from init pose
            delta_xyz = (controller_xyz - self.init_controller_xyz[src_name]) * self.scale_factor
            delta_rot = quat_diff_as_angle_axis(self.init_controller_quat[src_name], controller_quat)

        return delta_xyz, delta_rot

    def _update_kinematics(self):
        try:
            self.solver.solve(True)
            self.placo_robot.update_kinematics()
        except RuntimeError as e:  # Catch RuntimeError
            if "QPError" in str(e) and "NaN in the QP solution" in str(e):
                print(f"IK solver failed with QPError (NaN): {e}")
                self.placo_robot.state.q = calc_placo_q_from_mujoco_qpos(
                    self.mj_model,
                    self.mj_data.qpos,
                    floating_base=self.floating_base,
                )

                self.placo_robot.update_kinematics()
            else:
                print(f"An unexpected RuntimeError occurred in IK solver: {e}")
                return
        except Exception as e:
            print(f"An unexpected error occurred in IK solver: {e}")
            return

        self.mj_qpos_curr = calc_mujoco_qpos_from_placo_q(
            self.mj_model,
            self.placo_robot,
            self.placo_robot.state.q,
            floating_base=self.floating_base,
        )

        for joint_name, gripper_pos in self.gripper_pos_target.items():
            success = set_mujoco_joint_pos_by_name(
                self.mj_model,
                self.mj_qpos_curr,
                joint_name,
                gripper_pos,
            )
            if not success:
                raise ValueError(f"Joint '{joint_name}' not found in MuJoCo model.")

        self.mj_data.ctrl = calc_mujoco_ctrl_from_qpos(self.mj_model, self.mj_qpos_curr)

        if self.visualize_placo:
            self.placo_vis.display(self.placo_robot.state.q)

            for name, config in self.end_effector_config.items():
                robot_frame_viz(self.placo_robot, config["link_name"])
                frame_viz(f"vis_target_{name}", self.effector_task[name].T_world_frame)

    def _set_gripper_position(self, joint_name, position):
        """Set the gripper position in the MuJoCo model."""
        joint_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id == -1:
            raise ValueError(f"Joint '{joint_name}' not found in the MuJoCo model.")

        qpos_addr = self.mj_model.jnt_qposadr[joint_id]
        if qpos_addr < len(self.mj_data.qpos):
            self.mj_data.qpos[qpos_addr] = position
        else:
            raise IndexError(f"Qpos address {qpos_addr} is out of bounds for qpos array.")
