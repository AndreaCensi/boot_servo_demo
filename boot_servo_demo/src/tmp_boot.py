#!/usr/bin/env python
import roslib; roslib.load_manifest('boot_servo_demo')  # @IgnorePep8

from bootstrapping_olympics import get_boot_config
from rosstream2boot import get_rs2b_config
from rosstream2boot.programs import iterate_robot_observations

# def create_ros_node_for_robot(robot):
#     rospy.init_node('boot_interface', disable_signals=True)
# 
#     if isinstance(robot, EquivRobot):
#         orig_robot = robot.get_original_robot()
#     else:
#         orig_robot = robot
#    
#     if not isinstance(orig_robot, ROSRobot):
#         msg = 'Expected ROSRobot, got %s' % describe_type(robot)
#         raise ValueError(msg)
#    
#     orig_robot.connect_to_ros()
#     
# #     adapter = orig_robot.adapter
# #     node = ROSRobotAdapterNode()
# #     node.init_messages(adapter) 
# #     
# #     class Iterator:
# #         def next(self):
# #             pass
# #         
#     orig_robot.iterator = Iterator()
    
   
def main():
    # boot_root = '/data/work/1303-youbot-videos/20-grace-workspace/out/boot-root'
    config_dir = '/data/work/scm/boot12env/src/yc1304'
    rs2b_config = get_rs2b_config()
    boot_config = get_boot_config()
    boot_config.load(config_dir)
    rs2b_config.load(config_dir)
    id_robot = 'ldr21'
    robot = boot_config.robots.instance(id_robot)
    
#     create_ros_node_for_robot(robot)

#     spec = robot.get_spec()
#     commands = spec.get_commands().get_default_value()
#     robot.set_commands(commands, 'rest')     
#     
    for obs in iterate_robot_observations(robot):
        print obs.timestamp

#         
#         
# nraw = numpy_msg(Raw)
# 
# 
# class DisplayDemo(ROSNode):
#     
#     def __init__(self):
#         ROSNode.__init__(self, 'ServoDemo')        
# 
#     def main(self):
#         rospy.init_node('servo_demo')
#         
#         self.info('Started.')
# 
#         self.boot_root = rospy.get_param('~boot_root')
#         rs2b_config_dir = rospy.get_param('~config_dir')
#                 
#         rs2b_config = get_rs2b_config()
#         rs2b_config.load(rs2b_config_dir)
# 
#         self.data_central = DataCentral(self.boot_root)
#         bo_config = self.data_central.get_bo_config()
#         bo_config.load(rs2b_config_dir)
#         
# 
#         self.id_robot = rospy.get_param('~id_robot')
#         robot = bo_config.load(self.id_robot)
#         
#         self.info('Creating ROSRobotAdapterNode')
#         self.adapter_node = ROSRobotAdapterNode()
#         self.adapter_node.init_messages(ros_robot_adapter=self.ros_robot_adapter)
#         self.adapter_node.obs_callback(self.obs_ready)
#         
#         self.info('Finished initialization')
#            
#         rospy.spin()    
#             

if __name__ == '__main__':
    main()
    
