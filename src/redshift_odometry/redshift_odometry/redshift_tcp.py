import socket
import rclpy
import time
import socket
import struct
from rclpy.node import Node
from std_msgs.msg import String
from roborio_msgs.msg import RoborioOdometry

class TcpClientNode(Node):
    def __init__(self):
        super().__init__('tcp_client_node')
        
        # Set up TCP connection parameters
        self.server_ip = '192.168.1.204'     # 10.40.48.2
        self.server_port = 5806
        
        # Create a TCP socket
        self.socket_connected = False
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while (self.socket_connected == False):
           try:
              self.socket.connect((self.server_ip, self.server_port))
              self.socket_connected = True
              self.get_logger().info("Connected to socket...")
           except:
              self.socket_connected = False
              self.get_logger().warning("Could not connect to socket. Trying again")
              time.sleep(0.1)   
        
        # Subscribe to the /pose topic and use callback to send data over tcp
        self.pose_subscription = self.create_subscription(
            RoborioOdometry,
            '/pose',
            self.tcp_callback,
            10)

    def tcp_callback(self, pose_msg):
       if (self.socket_connected == True):
          # Create message buffer with POSE (x, y, theta), DISTANCE of robot to tag, and the TAG
          msg = [pose_msg.x, pose_msg.y, pose_msg.yaw, pose_msg.distance, pose_msg.tag]
          format_string = "!{}d{}i".format(len(msg) - 1, 1)
          data = struct.pack(format_string, *msg)
          self.socket.sendall(data)
          #print(data) # BZ - delete
       

def main(args=None):
    rclpy.init(args=args)

    tcp_client = TcpClientNode()

    rclpy.spin(tcp_client)

    tcp_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

