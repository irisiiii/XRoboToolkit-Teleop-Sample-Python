#!/usr/bin/env python3
"""
Quick test script for dual_arm_fixed.urdf

This script loads the fixed URDF in PyBullet and allows interactive testing.
"""

import pybullet as p
import pybullet_data
import numpy as np
import time

def test_dual_arm_fixed():
    """Test the fixed dual arm URDF in PyBullet."""
    
    # Connect to PyBullet
    print("Connecting to PyBullet GUI...")
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set up environment
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    
    # Load the fixed URDF
    print("\nLoading dual_arm_fixed.urdf...")
    robot = p.loadURDF(
        "assets/jaka/dual_arm_fixed.urdf",
        [0, 0, 0],
        useFixedBase=True
    )
    
    # Get joint information
    num_joints = p.getNumJoints(robot)
    print(f"\nRobot loaded with {num_joints} joints")
    print("\n" + "="*70)
    print("JOINT INFORMATION")
    print("="*70)
    
    joint_info = []
    for i in range(num_joints):
        info = p.getJointInfo(robot, i)
        joint_name = info[1].decode('utf-8')
        joint_type = info[2]
        axis = info[13]
        
        if joint_type == p.JOINT_REVOLUTE:
            joint_info.append({
                'index': i,
                'name': joint_name,
                'type': 'revolute',
                'axis': axis
            })
            
            print(f"\nJoint {i}: {joint_name}")
            print(f"  Type: Revolute")
            print(f"  Axis: {axis}")
            
            # Determine axis name
            axis_arr = np.array(axis)
            abs_axis = np.abs(axis_arr)
            max_idx = np.argmax(abs_axis)
            axis_names = ['X', 'Y', 'Z']
            direction = '+' if axis_arr[max_idx] > 0 else '-'
            print(f"  → Rotates around {direction}{axis_names[max_idx]} axis")
    
    # Print summary
    print("\n" + "="*70)
    print("AXIS SUMMARY")
    print("="*70)
    
    right_arm = [j for j in joint_info if j['name'].startswith('r-j')]
    left_arm = [j for j in joint_info if j['name'].startswith('l-j')]
    
    print("\nRIGHT ARM:")
    for j in right_arm:
        axis_arr = np.array(j['axis'])
        print(f"  {j['name']}: axis={j['axis']}")
    
    print("\nLEFT ARM:")
    for j in left_arm:
        axis_arr = np.array(j['axis'])
        print(f"  {j['name']}: axis={j['axis']}")
    
    # Test joint movements
    print("\n" + "="*70)
    print("INTERACTIVE TESTING")
    print("="*70)
    print("\nStarting interactive joint movement test...")
    print("Watch the robot move through test poses.")
    print("Close the PyBullet window to exit.\n")
    
    # Test poses
    test_poses = [
        {
            'name': 'Home Position',
            'angles': [0] * len(right_arm)
        },
        {
            'name': 'Right Arm Forward',
            'angles': [0.5, 0.3, -0.2, 0.4, -0.1, 0.3, 0]
        },
        {
            'name': 'Right Arm Up',
            'angles': [1.0, 0.5, 0, 0.8, 0, 0.5, 0]
        },
        {
            'name': 'Right Arm Side',
            'angles': [-0.5, 0.3, 0.2, -0.4, 0.1, 0.3, 0]
        },
    ]
    
    try:
        pose_idx = 0
        step_count = 0
        while True:
            # Cycle through test poses every 3 seconds
            if step_count % 240 == 0:  # 240 steps * 1/240 sec = 1 sec
                pose = test_poses[pose_idx % len(test_poses)]
                print(f"\nMoving to: {pose['name']}")
                
                # Apply to right arm
                for i, angle in enumerate(pose['angles']):
                    if i < len(right_arm):
                        p.setJointMotorControl2(
                            robot,
                            right_arm[i]['index'],
                            p.POSITION_CONTROL,
                            targetPosition=angle,
                            force=100
                        )
                
                # Apply mirrored to left arm (for visualization)
                for i, angle in enumerate(pose['angles']):
                    if i < len(left_arm):
                        # Mirror some angles for symmetry
                        mirrored_angle = -angle if i in [0, 1, 3, 5] else angle
                        p.setJointMotorControl2(
                            robot,
                            left_arm[i]['index'],
                            p.POSITION_CONTROL,
                            targetPosition=mirrored_angle,
                            force=100
                        )
                
                pose_idx += 1
            
            p.stepSimulation()
            time.sleep(1./240.)
            step_count += 1
            
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    
    print("\nDisconnecting from PyBullet...")
    p.disconnect()
    print("Done!")


def compare_with_original():
    """Compare the fixed URDF with the original."""
    print("\n" + "="*70)
    print("COMPARING ORIGINAL vs FIXED URDF")
    print("="*70)
    
    try:
        # Connect to PyBullet in DIRECT mode for comparison
        p.connect(p.DIRECT)
        
        # Load both URDFs
        print("\nLoading original dual_arm.urdf...")
        robot_orig = p.loadURDF("assets/jaka/dual_arm.urdf", [0, 0, 0], useFixedBase=True)
        
        print("Loading fixed dual_arm_fixed.urdf...")
        robot_fixed = p.loadURDF("assets/jaka/dual_arm_fixed.urdf", [2, 0, 0], useFixedBase=True)
        
        # Test same joint configuration
        test_angles = [0.5, -0.3, 0.2, -0.4, 0.1, 0.6, -0.2]
        
        # Find right arm joints
        orig_joints = []
        fixed_joints = []
        
        for i in range(p.getNumJoints(robot_orig)):
            name = p.getJointInfo(robot_orig, i)[1].decode('utf-8')
            if name.startswith('r-j') and p.getJointInfo(robot_orig, i)[2] == p.JOINT_REVOLUTE:
                orig_joints.append(i)
        
        for i in range(p.getNumJoints(robot_fixed)):
            name = p.getJointInfo(robot_fixed, i)[1].decode('utf-8')
            if name.startswith('r-j') and p.getJointInfo(robot_fixed, i)[2] == p.JOINT_REVOLUTE:
                fixed_joints.append(i)
        
        print(f"\nFound {len(orig_joints)} joints in original, {len(fixed_joints)} in fixed")
        
        # Apply test angles
        print("\nApplying test joint angles...")
        for i, angle in enumerate(test_angles[:min(len(orig_joints), len(fixed_joints))]):
            p.resetJointState(robot_orig, orig_joints[i], angle)
            p.resetJointState(robot_fixed, fixed_joints[i], angle)
        
        # Get end effector positions (last link)
        orig_num_links = p.getNumJoints(robot_orig)
        fixed_num_links = p.getNumJoints(robot_fixed)
        
        # Find end effector (rt for right arm)
        orig_ee_idx = None
        fixed_ee_idx = None
        
        for i in range(orig_num_links):
            name = p.getJointInfo(robot_orig, i)[1].decode('utf-8')
            if name == 'r-t':  # Right tool
                orig_ee_idx = i
                break
        
        for i in range(fixed_num_links):
            name = p.getJointInfo(robot_fixed, i)[1].decode('utf-8')
            if name == 'r-t':
                fixed_ee_idx = i
                break
        
        if orig_ee_idx and fixed_ee_idx:
            pos_orig = p.getLinkState(robot_orig, orig_ee_idx)[0]
            pos_fixed = p.getLinkState(robot_fixed, fixed_ee_idx)[0]
            
            diff = np.array(pos_orig) - np.array(pos_fixed)
            distance = np.linalg.norm(diff)
            
            print("\n" + "="*70)
            print("END EFFECTOR COMPARISON")
            print("="*70)
            print(f"\nOriginal position: {pos_orig}")
            print(f"Fixed position:    {pos_fixed}")
            print(f"Difference:        {diff}")
            print(f"Distance:          {distance:.6f} meters")
            
            if distance < 0.001:  # 1mm tolerance
                print("\n✅ SUCCESS: Positions match within 1mm tolerance!")
                print("The fixed URDF produces the same kinematics as the original.")
            else:
                print(f"\n⚠️  WARNING: Position difference is {distance*1000:.3f}mm")
                print("This may indicate a problem with the conversion.")
        
        p.disconnect()
        
    except Exception as e:
        print(f"\n❌ Error during comparison: {e}")
        print("Make sure both URDF files exist in assets/jaka/")


if __name__ == "__main__":
    import sys
    
    print("="*70)
    print("DUAL ARM FIXED URDF - TESTING SCRIPT")
    print("="*70)
    
    if len(sys.argv) > 1 and sys.argv[1] == "--compare":
        compare_with_original()
    else:
        print("\nOptions:")
        print("  python test_dual_arm_fixed.py          - Interactive GUI test")
        print("  python test_dual_arm_fixed.py --compare - Compare original vs fixed")
        print()
        
        choice = input("Choose mode (press Enter for GUI test, 'c' for compare): ").strip().lower()
        
        if choice == 'c':
            compare_with_original()
        else:
            test_dual_arm_fixed()
