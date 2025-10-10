import canopen
import time
from datetime import datetime

def print_can_message(message):
    """
    Print received CAN message with timestamp and formatted data.
    
    Args:
        message: CAN message object with id, data, and other attributes
    """
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # Include milliseconds
    
    print(f"[{timestamp}] ID: 0x{message.arbitration_id:03X} | "
          f"Length: {message.dlc} | "
          f"Data: {' '.join(f'{byte:02X}' for byte in message.data[:message.dlc])}")

def main():
    """
    Main function to receive and print CAN bus messages.
    """
    print("CAN Bus Message Receiver")
    print("========================")
    print("Waiting for CAN messages...")
    print("Press Ctrl+C to stop")
    print("Format: [timestamp] ID: 0xXXX | Length: X | Data: XX XX XX...")
    print()
    
    try:
        # Create and connect to CAN network
        network = canopen.Network()
        network.connect(
            channel=1,
            bustype="canalystii",
            bitrate=20000,
        )
        
        print("Connected to CAN bus successfully!")
        print("Listening for messages...\n")
        
        # Listen for messages
        while True:
            try:
                # Check for received messages
                for message in network.bus:
                    print_can_message(message)
                    
            except Exception as e:
                print(f"Error receiving message: {e}")
                time.sleep(0.1)  # Small delay to prevent excessive CPU usage
                
    except KeyboardInterrupt:
        print("\nStopping CAN message receiver...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            network.disconnect()
            print("Disconnected from CAN bus.")
        except:
            pass

if __name__ == "__main__":
    main()