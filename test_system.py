#!/usr/bin/env python3
"""
Simple test script to verify the teleoperation system is working
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_imports():
    """Test all critical imports"""
    print("Testing imports...")
    
    try:
        # Test televuer
        from teleop.televuer.src.televuer import TeleVuerWrapper
        print("✅ TeleVuerWrapper imported successfully")
        
        # Test robot control
        from teleop.robot_control.robot_arm import G1_29_ArmController
        print("✅ G1_29_ArmController imported successfully")
        
        # Test hand retargeting
        from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
        print("✅ HandRetargeting imported successfully")
        
        # Test dex-retargeting
        from dex_retargeting import RetargetingConfig
        print("✅ dex_retargeting imported successfully")
        
        # Test unitree SDK
        from unitree_sdk2py.core.channel import ChannelPublisher
        print("✅ unitree_sdk2py imported successfully")
        
        # Test pinocchio
        import pinocchio as pin
        print("✅ pinocchio imported successfully")
        
        # Test other dependencies
        import torch
        import trimesh
        import numpy as np
        print("✅ All dependencies imported successfully")
        
        return True
        
    except ImportError as e:
        print(f"❌ Import error: {e}")
        return False

def test_hand_retargeting():
    """Test hand retargeting functionality"""
    print("\nTesting hand retargeting...")
    
    try:
        from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
        
        # Test creating hand retargeting for dex3
        retargeting = HandRetargeting(HandType.UNITREE_DEX3)
        print("✅ Dex3 hand retargeting created successfully")
        
        # Test creating inspire hand retargeting
        retargeting_inspire = HandRetargeting(HandType.INSPIRE_HAND)
        print("✅ Inspire hand retargeting created successfully")
        
        return True
        
    except Exception as e:
        print(f"❌ Hand retargeting error: {e}")
        return False

def main():
    print("🔧 Testing XR Teleoperation System\n")
    
    # Test imports
    if not test_imports():
        return 1
    
    # Test hand retargeting
    if not test_hand_retargeting():
        return 1
    
    print("\n🎉 All tests passed! The system is ready to run.")
    print("\nTo run the full system:")
    print("  python teleop/teleop_hand_and_arm.py --sim --arm=G1_29 --ee=dex3 --record")
    print("\nTo run without hand controllers:")
    print("  python teleop/teleop_hand_and_arm.py --sim --arm=G1_29 --record")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())