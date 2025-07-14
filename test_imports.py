#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Test script to debug import issues
"""

import sys
import os

# Add paths for local imports
sys.path.append(os.path.abspath('dynamixel/libgx'))
sys.path.append(os.path.abspath('damiao/DM_Control'))
sys.path.append(os.path.abspath('daimon/dmrobotics'))

print("Testing imports...")

try:
    print("1. Testing dynamixel_sdk import...")
    from dynamixel_sdk import *
    print("   ✓ dynamixel_sdk imported successfully")
except ImportError as e:
    print(f"   ✗ dynamixel_sdk import failed: {e}")

try:
    print("2. Testing motor import...")
    from motor import Motor as DynamixelMotor
    print("   ✓ motor imported successfully")
except ImportError as e:
    print(f"   ✗ motor import failed: {e}")

try:
    print("3. Testing DM_CAN import...")
    from DM_CAN import MotorControl, Motor as GripperMotor, DM_Motor_Type, Control_Type
    print("   ✓ DM_CAN imported successfully")
except ImportError as e:
    print(f"   ✗ DM_CAN import failed: {e}")

try:
    print("4. Testing dmrobotics import...")
    from dmrobotics import Sensor, put_arrows_on_image
    print("   ✓ dmrobotics imported successfully")
except ImportError as e:
    print(f"   ✗ dmrobotics import failed: {e}")

print("\nImport test completed.") 