import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage


# this class is used to get all the April tag transformations.
# it is meant to be called from a ROS node.

class TagManager:

    def __init__(self, node, tags, logger):
       self.node = node
       self.number_of_tags = tags  ### BZ - this needs to change to cycle through the tag_table
       self.logger = logger
       self.received_all_static_tf = False
       self.__tag_dict = {}
       for i in range(1, self.number_of_tags+1):
          self.__tag_dict[i] = TFMessage()
       self.tf_buffer = Buffer()
       self.tf_listener = TransformListener(self.tf_buffer, self.node)   
    
    def start_tagging(self):
       self.static_timer = self.node.create_timer(0.5, self.check_static_transforms)
    
    def __stop_tagging(self):
       self.node.destroy_timer(self.static_timer)
       
    def get_tf_for_tag(self, tag):
       return self.__tag_dict[tag]
       
    def get_all_tags(self):
       return self.__tag_dict.keys()   
          
    # -----------------------------------------------------------------------------------------
    # This callback function is used to find and save all the static world->tag transforms.
    # Once all are received, the function kills itself.
    # -----------------------------------------------------------------------------------------
    def check_static_transforms(self):
       if (self.received_all_static_tf):
          return
          
       self.logger.info("Getting static transforms")
       tags_found = 0;
       for tag in range(1, self.number_of_tags + 1):
          try:
             temp_tf = self.tf_buffer.lookup_transform('world', 'tag'+str(tag), rclpy.time.Time())
             tags_found +=1;
             self.__tag_dict[tag] = temp_tf
          except Exception as e:
             self.logger.info(f'Could not get static transform for tag{tag}:' f' {e}')   
       
       if tags_found == self.number_of_tags:
          self.received_all_static_tf = True
          self.logger.info("static transforms done:")
          for tag in range(1, self.number_of_tags + 1):
              self.logger.info(f"   Frame ID:{self.__tag_dict[tag].header.frame_id}" f" , Child Frame ID: {self.__tag_dict[tag].child_frame_id}")
          self.__stop_tagging()  
          
          
          
           
