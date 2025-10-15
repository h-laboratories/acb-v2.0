#!/usr/bin/env python3
"""
Test script for ACB v2.0 angle limits functionality.
This script tests the new angle limit features:
1. Setting and getting min/max angle limits
2. Validating position commands against limits
3. Monitoring angle limits in velocity/torque mode
"""

import serial
import time
import sys

def send_command(ser, command):
    """Send a command and return the response."""
    print(f"Sending: {command}")
    ser.write(f"{command}\n".encode())
    time.sleep(0.1)
    
    response = ""
    while ser.in_waiting > 0:
        response += ser.read(ser.in_waiting).decode()
        time.sleep(0.01)
    
    if response:
        print(f"Response: {response.strip()}")
    return response.strip()

def test_angle_limits():
    """Test the angle limits functionality."""
    # Configure serial connection
    try:
        ser = serial.Serial('/dev/ttyUSB0', 2000000, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        print("Connected to ACB v2.0")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return False
    
    try:
        print("\n=== Testing Angle Limits Functionality ===\n")
        
        # Test 1: Get current angle limits
        print("1. Getting current angle limits:")
        send_command(ser, "get_min_angle")
        send_command(ser, "get_max_angle")
        
        # Test 2: Set new angle limits
        print("\n2. Setting new angle limits:")
        send_command(ser, "set_min_angle -90")
        send_command(ser, "set_max_angle 90")
        send_command(ser, "get_min_angle")
        send_command(ser, "get_max_angle")
        
        # Test 3: Test invalid angle limit setting
        print("\n3. Testing invalid angle limit setting:")
        send_command(ser, "set_min_angle 100")  # Should fail (min > max)
        send_command(ser, "set_max_angle -100")  # Should fail (max < min)
        
        # Test 4: Test position commands within limits
        print("\n4. Testing position commands within limits:")
        send_command(ser, "set_position 0")     # Should work
        send_command(ser, "set_position 45")    # Should work
        send_command(ser, "set_position -45")   # Should work
        
        # Test 5: Test position commands outside limits
        print("\n5. Testing position commands outside limits:")
        send_command(ser, "set_position 100")   # Should fail (outside max)
        send_command(ser, "set_position -100")  # Should fail (outside min)
        
        # Test 6: Test velocity mode with angle limits
        print("\n6. Testing velocity mode with angle limits:")
        send_command(ser, "set_velocity 10")    # Set velocity mode
        send_command(ser, "enable")             # Enable motor
        print("Motor is now in velocity mode. If it moves outside [-90, 90], it should automatically switch to position mode.")
        print("Watch for 'set_position X' messages when limits are exceeded.")
        
        # Let it run for a few seconds
        time.sleep(5)
        
        # Test 7: Reset to default limits
        print("\n7. Resetting to default angle limits:")
        send_command(ser, "set_min_angle -180")
        send_command(ser, "set_max_angle 180")
        send_command(ser, "get_min_angle")
        send_command(ser, "get_max_angle")
        
        # Test 8: Save configuration
        print("\n8. Saving configuration:")
        send_command(ser, "save_config")
        
        print("\n=== Angle Limits Test Complete ===")
        return True
        
    except Exception as e:
        print(f"Test failed: {e}")
        return False
    finally:
        ser.close()

if __name__ == "__main__":
    success = test_angle_limits()
    sys.exit(0 if success else 1)
