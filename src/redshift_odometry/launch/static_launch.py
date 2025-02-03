import os
import math
from launch_ros.actions import Node
from launch import LaunchDescription
from redshift_odometry.TagTable import *
from redshift_odometry.CamTable import *
 	 	  
def generate_launch_description():
   ld = LaunchDescription()   
      
   for tag_entry in TagTable.tag_table:
      ld.add_action(create_transform_node(tag_entry))   
      
   for cam_entry in CamTable.cam_table:
      ld.add_action(create_robot_to_cam_node(cam_entry))      
             
   return ld



def create_robot_to_cam_node(entry):
   cam   = entry["camid"]
   x     = entry["x"]
   y     = entry["y"]
   z     = entry["z"]
   roll  = math.radians(entry["roll"])
   pitch = math.radians(entry["pitch"])
   yaw   = math.radians(entry["yaw"])   
   
   nd = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='RobotTo' + cam,
      output='screen',
      arguments=[
         '--x', str(x),
         '--y', str(y),
         '--z', str(z),
         '--roll', str(roll),
         '--pitch', str(pitch),
         '--yaw', str(yaw),
         '--frame-id', 'robot',
         '--child-frame-id', cam
      ],
      respawn=True,
      respawn_delay=2   
   )
   return(nd)


   
def create_transform_node(entry):
   # Create a static transform from world to a tag
   # The rotation is applied in a weired order.....Z-Y-X  Yaw-Pitch-Roll 
   # Positive is clockwise (right hand rule)
   tag   = entry["tagid"]
   x     = entry["x"]
   y     = entry["y"]
   z     = entry["z"]
   roll  = math.radians(entry["roll"])
   pitch = math.radians(entry["pitch"])
   yaw   = math.radians(entry["yaw"])
               
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
   
   
   
   
