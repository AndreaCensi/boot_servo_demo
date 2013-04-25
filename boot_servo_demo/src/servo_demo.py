#!/usr/bin/env python
import roslib; roslib.load_manifest('boot_servo_demo')  # @IgnorePep8

from boot_servo_demo.msg import Raw
from bootstrapping_olympics.programs.manager.meat.data_central import (
    DataCentral)
from bootstrapping_olympics.programs.manager.meat.load_agent_state import (
    load_agent_state)
from bootstrapping_olympics.utils.safe_pickle import (safe_pickle_load,
    safe_pickle_dump)
from contracts import contract
from ros_node_utils import ROSNode
from rospy.numpy_msg import numpy_msg
from rosstream2boot.config import get_rs2b_config
from rosstream2boot.nodes.ros_adapter_node import ROSRobotAdapterNode
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import os
import rospy
import warnings
 

nraw = numpy_msg(Raw)


STATE_WAIT = 'wait'
STATE_SERVOING = 'servoing'

class ServoDemo(ROSNode):
    """ 
        First implementation of a servo demo; a big node does everything.
        
        Parameters:
            boot_root: Root directory for the
             
            id_agent: id_agent  
            id_robot: id_robot
            
            id_ros_robot_adapter: 
        
        Input:
        
        Output:
        
    """
    state_filename = 'y_goal.pickle'
    
    def __init__(self):
        ROSNode.__init__(self, 'ServoDemo')
        

    def main(self):
        rospy.init_node('servo_demo')
        
        self.info('Started.')

        self.boot_root = rospy.get_param('~boot_root')
        rs2b_config_dir = rospy.get_param('~config_dir')
        
        self.id_ros_robot_adapter = rospy.get_param('~id_ros_robot_adapter')
        self.id_agent = rospy.get_param('~id_agent')
        self.id_robot = rospy.get_param('~id_robot')
        self.error_threshold = float(rospy.get_param('~error_threshold'))
        self.K = np.array(eval(rospy.get_param('~K')))
        
        rs2b_config = get_rs2b_config()
        rs2b_config.load(rs2b_config_dir)
        self.data_central = DataCentral(self.boot_root)
        bo_config = self.data_central.get_bo_config()
        bo_config.load(rs2b_config_dir)
        
        self.ros_robot_adapter = rs2b_config.adapters.instance(self.id_ros_robot_adapter)
        
        
        self.agent, self.state = load_agent_state(self.data_central, self.id_agent, self.id_robot,
                                        reset_state=False, raise_if_no_state=True)
        
        self.info('Loaded state: %s' % self.state)
        
        self.servo_agent = self.agent.get_servo()

            
        self.publish_info_init()     
        
     
        self.y = None
        self.y_goal = None
        self.started_now = False
        self.e0 = 1
        self.e = 1         
        self.last_boot_data = None
        self.state = STATE_WAIT
        

        self.info('Defining services')
        rospy.Service('set_goal', Empty, self.srv_set_goal)
        rospy.Service('start_servo', Empty, self.srv_start_servo)
        rospy.Service('stop_servo', Empty, self.srv_stop_servo)
        
        self.info('Creating ROSRobotAdapterNode')
        self.adapter_node = ROSRobotAdapterNode()
        self.adapter_node.init_messages(ros_robot_adapter=self.ros_robot_adapter)
        self.adapter_node.obs_callback(self.obs_ready)
        
        self.info('Finished initialization')
           
        rospy.spin()    
        
    # Gui
    def srv_set_goal(self, req):  # @UnusedVariable
        self.info('called "set_goal"')
        if self.last_boot_data is not None:
            self.set_goal_observations(self.last_boot_data['observations'])
        else:
            self.info('I have no observations yet.')
        return EmptyResponse()
        
    def srv_start_servo(self, req):  # @UnusedVariable
        self.info('called "start_servo"')
        self.state = STATE_SERVOING
        self.started_now = True
        return EmptyResponse()
    
    def srv_stop_servo(self, req):  # @UnusedVariable
        self.info('called "stop_servo"')
        self.state = STATE_WAIT
        
        return EmptyResponse()

    def obs_ready(self, msg):  # @UnusedVariable
        # Callback called when ready
        buf = self.adapter_node.get_boot_observations_buffer()
        if len(buf) > 1:
            # self.info('Warning: dropping %d observations (too slow?).' % (len(buf) - 1))
            pass
        if not buf:
            # 
            pass
        try:
            boot_data = buf[-1]
        except:
            return
        
        self.last_boot_data = boot_data.copy()
        
        self.u = self.get_servo_commands(boot_data)
        
#         warnings.warn('removing linear')
#         self.u[0] = 0
#         self.u[1] = 0
        self.publish_info()
        
        pc = lambda x: ' '.join(['%+6.3f' % a for a in x]) 
        msg = 'u: %s ' % pc(self.u)
        
        
        msg += ('e0: %8f   e: %8f  e/e0: %8f' % (self.e0, self.e, self.e / self.e0))
        
        verbose = True
        if self.state == STATE_WAIT:
            if verbose:
                self.info('hold %s' % msg)
            
        elif self.state == STATE_SERVOING:
            if verbose:
                self.info('send %s' % msg)
            self.adapter_node.send_commands(self.u)
        
        
    def get_servo_commands(self, boot_data):
        self.y = boot_data['observations']
        
        if self.y_goal is None:
            if os.path.exists(ServoDemo.state_filename):
                self.info('Loading y_goal from %s' % ServoDemo.state_filename)
                y_goal = safe_pickle_load(ServoDemo.state_filename)
                self.set_goal_observations(y_goal)
            else:
                self.set_goal_observations(self.y)
            
        if self.started_now or self.e0 is None:
            # First iteration with new goal
            self.e0 = self.get_distance_to_goal(self.y)
            self.started_now = False

        self.servo_agent.process_observations(boot_data)
    
        u = self.servo_agent.choose_commands() 

        warnings.warn('This was what worked for rotation.')
#         res = self.servo_agent.choose_commands2(self.K) 
#         u = res['u']


        if self.state == STATE_SERVOING:    
            self.e = self.get_distance_to_goal(self.y)
            
            
            if self.e < self.error_threshold:
                print('stopping here')
                return u * 0 
             
        
#         warnings.warn('use param gain')
#         if u.size == 3:
#             u[2] *= 40
    
        return u


    def publish_info_init(self):
        self.pub_u = rospy.Publisher('~u', numpy_msg(Raw))
        self.pub_y = rospy.Publisher('~y', numpy_msg(Raw)) 
        self.pub_y_goal = rospy.Publisher('~y_goal', numpy_msg(Raw))
        self.pub_servo_state = rospy.Publisher('~servo_state', String)

    def publish_info(self):    
        
        msg_y = nraw()
        msg_y.values = self.y
        msg_y.importance = np.ones(self.y.shape)
        self.pub_y.publish(msg_y)

        msg_y_goal = nraw()
        msg_y_goal.values = self.y_goal
        msg_y_goal.importance = np.ones(self.y_goal.shape)
        self.pub_y_goal.publish(msg_y_goal)

        msg_u = nraw()
        msg_u.values = self.u
        msg_u.importance = np.ones(self.u.shape)
        self.pub_u.publish(msg_u)
        
        msg_state = String(self.state)
        self.pub_servo_state.publish(msg_state)
     
    def get_distance_to_goal(self, y):
        diff = y - self.y_goal
        disc = np.abs(y - self.y_goal)
#         print('L1 mean: %s' % np.mean(np.abs(diff)))
        L1_robust = np.percentile(disc, 80)
#         print('L1 robu: %s' % L1_robust)
        L2 = np.linalg.norm(y - self.y_goal)
        
        return L1_robust
    
    @contract(y='array')
    def set_goal_observations(self, y):
        self.y_goal = y.copy()
        
        safe_pickle_dump(self.y_goal, ServoDemo.state_filename)
        
        self.servo_agent.set_goal_observations(self.y_goal)
        
 

if __name__ == '__main__':
    ServoDemo().main()
