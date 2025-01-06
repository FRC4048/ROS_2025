import os
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode 
 	 	  
def generate_launch_description():
   ld = LaunchDescription()
     
   usb_cam_node = Node(
      package='usb_cam',
      executable='usb_cam_node_exe',
      remappings=[('/image_raw', '/image')],
      parameters=[
         {'--video_device': '/dev/video0'},
         {'camera_name': 'logitech_cam'},
         {'frame_id': 'logitech'},
      ],
      name='cam_driver',  
   )
   
   ld.add_action(usb_cam_node)  
       
   return ld
   

