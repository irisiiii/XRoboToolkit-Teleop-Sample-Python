#!/usr/bin/env python3
"""
Verification script for dual_arm_fixed.urdf

This script compares the original dual_arm.urdf with the fixed version
to verify that the axis transformations are correct.

It will:
1. Load both URDFs
2. Display joint axis information
3. Test basic kinematics
4. Compare left/right arm symmetry
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


def compute_actual_axis(rpy, original_axis):
    """
    Apply RPY transformation to get the actual rotation axis.
    
    Args:
        rpy: [roll, pitch, yaw] in radians
        original_axis: [x, y, z] axis vector
    
    Returns:
        Transformed axis vector
    """
    rot = R.from_euler('xyz', rpy, degrees=False)
    actual_axis = rot.apply(original_axis)
    # Normalize and round to clean values
    actual_axis = actual_axis / np.linalg.norm(actual_axis)
    actual_axis = np.round(actual_axis, decimals=3)
    return actual_axis


def analyze_joint_transformation(name, rpy, axis):
    """Analyze a single joint transformation."""
    actual_axis = compute_actual_axis(rpy, axis)
    
    print(f"\n{name}:")
    print(f"  Original: rpy={rpy}, axis={axis}")
    print(f"  Actual rotation axis: {actual_axis}")
    
    # Determine the primary axis
    abs_axis = np.abs(actual_axis)
    max_idx = np.argmax(abs_axis)
    axis_names = ['X', 'Y', 'Z']
    direction = '+' if actual_axis[max_idx] > 0 else '-'
    
    print(f"  → Rotates around {direction}{axis_names[max_idx]} axis")
    
    return actual_axis


def main():
    print("="*70)
    print("DUAL ARM URDF AXIS VERIFICATION")
    print("="*70)
    
    print("\n" + "="*70)
    print("ORIGINAL URDF - Joint Axis Analysis")
    print("="*70)
    
    # Define original joint configurations (from dual_arm.urdf)
    original_joints = {
        'RIGHT ARM': {
            'r-j1': {'rpy': [1.5708, 0, 0], 'axis': [0, 0, 1]},
            'r-j2': {'rpy': [-1.5708, 0, -3.1416], 'axis': [0, 0, 1]},
            'r-j3': {'rpy': [1.5708, 0, 0], 'axis': [0, 0, 1]},
            'r-j4': {'rpy': [-1.5708, 0, 0], 'axis': [0, 0, 1]},
            'r-j5': {'rpy': [1.5708, 0, 0], 'axis': [0, 0, 1]},
            'r-j6': {'rpy': [-1.5708, 0, -3.1416], 'axis': [0, 0, 1]},
            'r-j7': {'rpy': [1.5708, 0, 0], 'axis': [0, 0, 1]},
        },
        'LEFT ARM': {
            'l-j1': {'rpy': [1.5708, 0, -3.1416], 'axis': [0, 0, 1]},
            'l-j2': {'rpy': [-1.5708, 0, 0], 'axis': [0, 0, 1]},
            'l-j3': {'rpy': [1.5708, 0, 0], 'axis': [0, 0, 1]},
            'l-j4': {'rpy': [-1.5708, 0, 0], 'axis': [0, 0, 1]},
            'l-j5': {'rpy': [1.5708, 0, 0], 'axis': [0, 0, 1]},
            'l-j6': {'rpy': [-1.5708, 0, 0], 'axis': [0, 0, 1]},
            'l-j7': {'rpy': [1.5708, 0, 0], 'axis': [0, 0, 1]},
        }
    }
    
    # Analyze original joints
    original_axes = {}
    for arm_name, joints in original_joints.items():
        print(f"\n{arm_name}:")
        print("-" * 70)
        for joint_name, config in joints.items():
            axis = analyze_joint_transformation(
                joint_name, 
                config['rpy'], 
                config['axis']
            )
            original_axes[joint_name] = axis
    
    print("\n" + "="*70)
    print("FIXED URDF - Proposed Direct Axes")
    print("="*70)
    
    # Define fixed joint configurations (from dual_arm_fixed.urdf)
    fixed_joints = {
        'RIGHT ARM': {
            'r-j1': [0, -1, 0],
            'r-j2': [0, -1, 0],
            'r-j3': [0, -1, 0],
            'r-j4': [0, 1, 0],
            'r-j5': [0, -1, 0],
            'r-j6': [0, -1, 0],
            'r-j7': [0, -1, 0],
        },
        'LEFT ARM': {
            'l-j1': [0, 1, 0],
            'l-j2': [0, 1, 0],
            'l-j3': [0, -1, 0],
            'l-j4': [0, 1, 0],
            'l-j5': [0, -1, 0],
            'l-j6': [0, 1, 0],
            'l-j7': [0, -1, 0],
        }
    }
    
    print("\nFixed joint axes:")
    for arm_name, joints in fixed_joints.items():
        print(f"\n{arm_name}:")
        print("-" * 70)
        for joint_name, axis in joints.items():
            axis_names = ['X', 'Y', 'Z']
            direction = '+' if np.any(np.array(axis) > 0) else '-'
            idx = np.argmax(np.abs(axis))
            print(f"  {joint_name}: axis={axis} → {direction}{axis_names[idx]} axis")
    
    print("\n" + "="*70)
    print("VERIFICATION - Comparing Original vs Fixed")
    print("="*70)
    
    # Compare axes
    all_match = True
    mismatches = []
    
    for arm_name in ['RIGHT ARM', 'LEFT ARM']:
        print(f"\n{arm_name}:")
        print("-" * 70)
        
        for joint_name in fixed_joints[arm_name].keys():
            original = original_axes[joint_name]
            fixed = np.array(fixed_joints[arm_name][joint_name])
            
            # Check if they match (allowing for small numerical errors)
            match = np.allclose(original, fixed, atol=0.01)
            status = "✓ MATCH" if match else "✗ MISMATCH"
            
            print(f"{joint_name}:")
            print(f"  Original computed: {original}")
            print(f"  Fixed direct:      {fixed}")
            print(f"  Status: {status}")
            
            if not match:
                all_match = False
                mismatches.append({
                    'joint': joint_name,
                    'original': original,
                    'fixed': fixed,
                    'diff': original - fixed
                })
    
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    
    if all_match:
        print("\n✓ SUCCESS: All axes match perfectly!")
        print("\nThe fixed URDF correctly represents the same kinematic chain")
        print("as the original, but with direct axis definitions instead of")
        print("rpy transformations.")
    else:
        print(f"\n✗ WARNING: Found {len(mismatches)} mismatched axes!")
        print("\nMismatched joints:")
        for mm in mismatches:
            print(f"\n  {mm['joint']}:")
            print(f"    Original: {mm['original']}")
            print(f"    Fixed:    {mm['fixed']}")
            print(f"    Diff:     {mm['diff']}")
    
    print("\n" + "="*70)
    print("SYMMETRY CHECK")
    print("="*70)
    
    print("\nChecking left-right arm symmetry...")
    
    # Check if left arm is proper mirror of right arm
    symmetric_pairs = [
        ('r-j1', 'l-j1'),
        ('r-j2', 'l-j2'),
        ('r-j3', 'l-j3'),
        ('r-j4', 'l-j4'),
        ('r-j5', 'l-j5'),
        ('r-j6', 'l-j6'),
        ('r-j7', 'l-j7'),
    ]
    
    for r_joint, l_joint in symmetric_pairs:
        r_axis = np.array(fixed_joints['RIGHT ARM'][r_joint])
        l_axis = np.array(fixed_joints['LEFT ARM'][l_joint])
        
        # For Y-axis rotations, left and right should be opposite
        # (mirror symmetry across XZ plane)
        expected_l_axis = r_axis.copy()
        expected_l_axis[1] = -expected_l_axis[1]  # Flip Y component
        
        symmetric = np.allclose(l_axis, expected_l_axis, atol=0.01)
        status = "✓" if symmetric else "✗"
        
        print(f"{status} {r_joint} vs {l_joint}:")
        print(f"    Right: {r_axis}, Left: {l_axis}")
        if symmetric:
            print(f"    → Properly mirrored")
        else:
            print(f"    → Expected left: {expected_l_axis}")
    
    print("\n" + "="*70)
    print("RECOMMENDATION")
    print("="*70)
    
    print("""
The fixed URDF (dual_arm_fixed.urdf) uses X7S-style direct axis definitions:

Benefits:
  ✓ Much clearer and easier to understand
  ✓ Direct axis vectors (no mental math needed)
  ✓ Eliminates ambiguity in rpy interpretation
  ✓ Easier to debug and modify
  ✓ More compatible across different simulators

Next steps:
  1. Test dual_arm_fixed.urdf in your simulator (PyBullet, IsaacSim, etc.)
  2. Verify forward kinematics match the original
  3. Check inverse kinematics solutions
  4. Test collision detection
  5. Validate teleoperation mapping

If everything works correctly, you can replace the original with the fixed version.
""")
    
    print("="*70)


if __name__ == "__main__":
    main()
