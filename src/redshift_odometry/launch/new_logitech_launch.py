import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
 	 	  
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
   



   # the tf below is tag1 with fake position for testing...need to be deleted
   ld.add_action(create_transform_node(1, 0.5, 0, 0,      1.57, 0, 1.57))    # tag is 0.5m on x wrt world
   ld.add_action(create_transform_node(3, 1, 1, 0,      0   , 0, 0   ))    
   #
   # below we create a set of static transforms from world to each tag
   # The rotation is applied in a weired order.....Z-Y-X  Yaw-Pitch-Roll 
   # Positive is clockwise (right hand rule)
   #
   #ld.add_action(create_transform_node(1, 593.68, 9.68, 53.38, -2.62, 0.0, 1.57))
   ld.add_action(create_transform_node(2, 637.21, 34.79, 53.38, -2.62, 0.0, 1.57))
   #ld.add_action(create_transform_node(3, 652.73, 196.17, 57.13, -1.57, 0.0, 1.57))
   ld.add_action(create_transform_node(4, 652.73, 218.42, 57.13, -1.57, 0.0, 1.57))
   #ld.add_action(create_transform_node(5, 578.77, 323.00, 53.38, 0.0, 0.0, 1.57))
   #ld.add_action(create_transform_node(6, 72.5, 323.00, 53.38, 0.0, 0.0, 1.57))
   #ld.add_action(create_transform_node(7, -1.50, 218.42, 57.13, 1.57, 0.0, 1.57))
   #ld.add_action(create_transform_node(8, -1.50, 196.17, 57.13, 1.57, 0.0, 1.57))
   #ld.add_action(create_transform_node(9, 14.02, 34.79, 53.38, 2.62, 0.0, 1.57))
   #ld.add_action(create_transform_node(10, 57.54, 9.68, 53.38, 2.62, 0.0, 1.57))
   #ld.add_action(create_transform_node(11, 468.69, 146.19, 52.00, 0.5236, 0.0, 1.57))
   #ld.add_action(create_transform_node(12, 468.69, 177.10, 52.00, 2.62, 0.0, 1.57))
   #ld.add_action(create_transform_node(13, 441.74, 161.62, 52.00, -1.57, 0.0, 1.57))
   #ld.add_action(create_transform_node(14, 209.48, 161.62, 52.00, 1.57, 0.0, 1.57))
   #ld.add_action(create_transform_node(15, 182.73, 177.10, 52.00, -2.62, 0.0, 1.57))      
   #ld.add_action(create_transform_node(16, 182.73, 146.19, 52.00, 5.76, 0.0, 1.57))   
   
   
   ld.add_action(robot_to_cam1_node)   
   ld.add_action(image_processing_node)
   ld.add_action(apriltag_cam1_node)
       
   return ld
   
   
   
def create_transform_node(tag, x, y, z, roll, pitch, yaw):
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
   
   
   
   
