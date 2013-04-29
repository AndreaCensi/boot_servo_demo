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
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import os
import rospy
import warnings
from bootstrapping_olympics.interfaces.observations import ObsKeeper
from rosstream2boot.programs.rs2b_convert2 import iterate_robot_observations
import time
 

nraw = numpy_msg(Raw)


STATE_WAIT = 'wait'
STATE_SERVOING = 'servoing'

class ServoDemo2(ROSNode):
    """ 
        
        Parameters:
            boot_root: Root directory for the
             
            id_agent: id_agent  
            id_robot: id_robot
            
            id_robot_learned: 
        
        Input:
        
        Output:
        
    """
    state_filename = 'y_goal.pickle'
    
    def __init__(self):
        ROSNode.__init__(self, 'ServoDemo')
        

    def main(self):
        rospy.init_node('servo_demo', disable_signals=True)
        
        self.info('Started.')

        boot_root = rospy.get_param('~boot_root')
        rs2b_config_dir = rospy.get_param('~config_dir')
        id_robot_learned = rospy.get_param('~id_robot_learn')
        id_agent = rospy.get_param('~id_agent')
        id_robot = rospy.get_param('~id_robot')
        self.error_threshold = float(rospy.get_param('~error_threshold'))
        self.K = np.array(eval(rospy.get_param('~K')))
        
        rs2b_config = get_rs2b_config()
        rs2b_config.load(rs2b_config_dir)
        data_central = DataCentral(boot_root)
        bo_config = data_central.get_bo_config()
        bo_config.load(rs2b_config_dir)
        
        ag_st = load_agent_state(data_central, id_agent, id_robot_learned,
                                 reset_state=False, raise_if_no_state=True)
        self.agent, state = ag_st
        
        self.info('Loaded state: %s' % state)
        
        self.servo_agent = self.agent.get_servo()

        self.robot = bo_config.robots.instance(id_robot)
            
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
                
        self.info('Finished initialization')
        
        self.obs_keeper = ObsKeeper(boot_spec=self.robot.get_spec(),
                                id_robot=id_robot)
        
        self.count = 0
        self.go()
        
    def go(self):
        
        for obs in iterate_robot_observations(self.robot, sleep=0.005):
            self.count += 1
            timestamp = obs.timestamp
            print('delay: %10.5f' % (time.time() - timestamp))
            observations = obs.observations
            commands = obs.commands
            commands_source = obs.commands_source
            warnings.warn('fill these up')
            id_episode = 'XXX'  # XXX
            id_world = 'xxx'  # XXX
            boot_data = self.obs_keeper.push(timestamp, observations,
                                             commands, commands_source,
                                             id_episode, id_world)
            self.process(boot_data)
                    

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
 
    def process(self, boot_data):
        self.last_boot_data = boot_data.copy()
        
        self.u = self.get_servo_commands(boot_data)
        
        self.publish_info()
        
        pc = lambda x: ' '.join(['%+6.3f' % a for a in x]) 
        msg = 'u: %s ' % pc(self.u)
        
        
        msg += ('e0: %8f   e: %8f  e/e0: %8f' % 
                (self.e0, self.e, self.e / self.e0))
        
        verbose = True
        if self.state == STATE_WAIT:
            if verbose:
                self.info('hold %s' % msg)
            
        elif self.state == STATE_SERVOING:
            if verbose:
                self.info('send %s' % msg)
                
            if True:
                warnings.warn('Cehcking if we are getting overloaded')
                if self.count % 2 == 0:
                    self.robot.set_commands(self.u, 'servo')
        
        
    def get_servo_commands(self, boot_data):
        self.y = boot_data['observations']
        
        if self.y_goal is None:
            if os.path.exists(ServoDemo2.state_filename):
                self.info('Loading y_goal from %s' % ServoDemo2.state_filename)
                y_goal = safe_pickle_load(ServoDemo2.state_filename)
                try: 
                    obs_spec = self.robot.get_spec().get_observations()
                    obs_spec.check_valid_value(y_goal)
                    
                    self.set_goal_observations(y_goal)
                except Exception as e:
                    self.error(str(e))
                    self.info('Removing file; using current observations')
                    os.unlink(ServoDemo2.state_filename)
                    self.set_goal_observations(self.y)
            else:
                self.set_goal_observations(self.y)
            
        if self.started_now or self.e0 is None:
            # First iteration with new goal
            self.e0 = self.get_distance_to_goal(self.y)
            self.started_now = False

        self.servo_agent.process_observations(boot_data)
    
        u = self.servo_agent.choose_commands() 

#         warnings.warn('Some extra spice')
#         u[0] = u[0] * 0.2
# #         u[1] = 0
#         self.info('scaled: %s' % str(u))

        if self.state == STATE_SERVOING:    
            self.e = self.get_distance_to_goal(self.y)
            if self.e < self.error_threshold:
                print('stopping here')
                return u * 0 
    
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
        disc = np.abs(y - self.y_goal)
        L1_robust = np.percentile(disc, 80)
        return L1_robust
    
    @contract(y='array')
    def set_goal_observations(self, y):
        self.y_goal = y.copy()
        
        safe_pickle_dump(self.y_goal, ServoDemo2.state_filename)
        
        self.servo_agent.set_goal_observations(self.y_goal)
        

if __name__ == '__main__':
    ServoDemo2().main()
