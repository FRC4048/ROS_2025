import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from redshift_odometry.TagTable import *
 	 	  
def generate_launch_description():
   ld = LaunchDescription()

   parameter_file_path_cam = "/home/redshift/ros2_ws_2025/misc/apriltag_cam.yaml"

   
   cam_comp = ComposableNode(package='usb_cam',
                             plugin='usb_cam::UsbCamNode',
                             name='cam_driver',
                             remappings=[('/image_raw', '/image')],
                             parameters=[
                                {'video_device': '/dev/video2'},
                                {'camera_name': 'logitech_cam'},
                                {'frame_id': 'cam1'},
                                {'brightness': 133},
                                {'contrast': 256},
                                {'hue': 40.0},
                                {'image_width': 640},
                                {'image_height': 480},
                                {'framerate': 30.0}
                             ])

   rect_comp = ComposableNode(package='image_proc',
                             plugin='image_proc::RectifyNode',
                             name='rectify',
                             parameters=[
                                {'queue_size': 10}
                             ])
   image_processing_node = ComposableNodeContainer(namespace='',
                                                   name='image_processing_container',
                                                   package='rclcpp_components',
                                                   executable='component_container',
                                                   composable_node_descriptions=[cam_comp,rect_comp])

 
   apriltag_cam1_node = Node(
      package='apriltag_ros',
      executable='apriltag_node',
      parameters=[parameter_file_path_cam],
   )  

   robot_to_cam1_node = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='cam1',
      output='screen',
      arguments=[
         '--x', str(0),
         '--y', str(0),
         '--z', str(0),
         '--roll', str(-1.57),
         '--pitch', str(0),
         '--yaw', str(-1.57),
         '--frame-id', 'robot',
         '--child-frame-id', 'cam1'
      ],
      respawn=True,
      respawn_delay=2   
   )
   
   
   # Create a set of static transforms from world to each tag
   # The rotation is applied in a weired order.....Z-Y-X  Yaw-Pitch-Roll 
   # Positive is clockwise (right hand rule)
   
   for tag_entry in TagTable.tag_table:
      ld.add_action(create_transform_node(tag_entry))   



def create_transform_node(entry):
   tag   = entry["tagid"]
   x     = entry["x"]
   y     = entry["y"]
   z     = entry["z"]
   roll  = entry["roll"]
   pitch = entry["pitch"]
   yaw   = entry["yaw"]
               
   nd = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='tag'+str(tag),
      output='screen',
      arguments=[
         '--x', str(x),
         '--y', str(y),
         '--z', str(z),
         '--roll', str(roll),
         '--pitch', str(pitch),
         '--yaw', str(yaw),
         '--frame-id', 'world',
         '--child-frame-id', 'tag'+str(tag)
      ],
      respawn=True,
      respawn_delay=2   
   )
   return(nd)   
   
   
   
   
