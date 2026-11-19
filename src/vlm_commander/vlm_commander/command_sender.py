# vlm_commander/vlm_commander/command_sender.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandSender(Node):
    def __init__(self):
        super().__init__('command_sender')
        self.pub = self.create_publisher(String, '/user_command', 10)
    
    def send_command(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f'Sent: {text}')

def main():
    rclpy.init()
    node = CommandSender()
    
    print("\n=== VLM Robot Commander ===")
    print("Try commands like:")
    print("  - Collect the red cubes")
    print("  - Go to the blue cube on the left")
    print("  - Push all cubes to collection zone\n")
    
    try:
        while rclpy.ok():
            cmd = input("Command: ")
            if cmd.lower() in ['quit', 'exit', 'q']:
                break
            node.send_command(cmd)
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
