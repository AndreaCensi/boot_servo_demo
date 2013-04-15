#!/usr/bin/env python
import roslib; roslib.load_manifest('boot_servo_demo')  # @IgnorePep8

from bootstrapping_olympics.programs.manager.meat.data_central import DataCentral
from bootstrapping_olympics.programs.manager.meat.load_agent_state import load_agent_state
import numpy as np
import rospy 
 
from ros_node_utils import ROSNode
from rosstream2boot.nodes.ros_adapter_node import ROSRobotAdapterNode
from rosstream2boot.config import get_rs2b_config
from contracts import contract
from bootstrapping_olympics.extra.ros.publisher.ros_publisher import ROSPublisher
from reprep.plot_utils.axes import y_axis_set
from procgraph_signals.history import HistoryInterval
from std_srvs.srv import Empty, EmptyResponse
import warnings

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

        
        self.adapter_node = ROSRobotAdapterNode()
        self.adapter_node.init_messages(ros_robot_adapter=self.ros_robot_adapter)

        self.adapter_node.obs_callback(self.obs_ready)
        
        self.y0 = None
        self.e0 = None
        
        self.publisher = ROSPublisher()
        self.info('Finished initialization')
        
        
        self.u_history = HistoryInterval(10)
        
        self.last_boot_data = None
         
        self.state = STATE_WAIT

        rospy.Service('set_goal', Empty, self.srv_set_goal)
        rospy.Service('start_servo', Empty, self.srv_start_servo)
        rospy.Service('stop_servo', Empty, self.srv_stop_servo)
   
        self.started_now = False
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
        boot_data = buf[-1]
        self.last_boot_data = boot_data.copy()
        
        commands = self.get_servo_commands(boot_data)
        
        pc = lambda x: ' '.join(['+%.3f' % a for a in x]) 
        msg = 'u: %s ' % pc(commands)
        
        if self.state == STATE_WAIT:
            self.info('hold %s' % msg)
            pass
        elif self.state == STATE_SERVOING:
            # self.info('send %s' % msg)
            self.adapter_node.send_commands(commands)
        
        
    def get_servo_commands(self, boot_data):
        t = boot_data['timestamp']
        y = boot_data['observations']
        
        if self.y0 is None:
            self.set_goal_observations(y)
            
        if self.started_now or self.e0 is None:
            # First iteration with new goal
            self.e0 = self.get_distance_to_goal(y)
            self.started_now = False

        if self.state == STATE_SERVOING:    
            e = self.get_distance_to_goal(y)
            print('e0: %8f   e: %8f  e/e0: %8f' % (self.e0, e, e / self.e0))
            
            if e < 1.6:
                print('stopping here')
                return np.array([0, 0]) 
            
# 
#         if False:
#             e_raw = y - self.y0
#             e_limit = 0.1 
#             too_far = np.abs(e_raw) > e_limit 
#     
#             boot_data['observations'][too_far] = self.y0[too_far]

        self.servo_agent.process_observations(boot_data)
    
        res = self.servo_agent.choose_commands2()  # repeated
        self.u_history.push(t, res['u_raw'])
        
#         msg = 'u_raw: %8.3f %8.3f ' % tuple(res['u_raw'])
#         msg += 'u    : %8.3f %8.3f  ' % tuple(res['u'])

        
        publish = False
        if publish:
            pub = self.publisher
            
            sec = pub.section('servo')
            sec.array('y', y)
            sec.array('y0', self.y0)
            
            with sec.plot('both') as pylab:
                pylab.plot(self.y0, 'sk')
                pylab.plot(y, '.g')
                
                y_axis_set(pylab, -0.1, 1.1)
                
            e = y - self.y0
            with sec.plot('error') as pylab:
                pylab.plot(e, '.m')
                # y_axis_set(pylab, -1.1, 1.1)
                y_axis_set(pylab, -e_limit - 0.1, e_limit + 0.1)
            
            ts, us = self.u_history.get_ts_xs()
            us = np.array(us)            
            with sec.plot('u') as pylab:
                pylab.plot(ts, us)
                y_axis_set(pylab, -1.1, 1.1)
         
#         warnings.warn('choosing u_raw')
        commands = res['u']
        
        # commands = -commands
#         commands[0] = 0
#         commands[1] = 0
# #         
#         warnings.warn('Pazza idea')
#         u = res['u'].copy()
#         commands[0] = u[1]
#         commands[1] = -u[0]
        
#         warnings.warn('0 linear')
#         commands[0] = 0
#         commands[1] = 0
#         
#          
#         warnings.warn('For the moment, theta is set to 0')
#         commands[2] = 0

        return commands
    

    def get_distance_to_goal(self, y):
        return np.linalg.norm(y - self.y0)
    
    @contract(y='array')
    def set_goal_observations(self, y):
        self.y0 = y.copy()
        self.servo_agent.set_goal_observations(self.y0)
        
 

if __name__ == '__main__':
    ServoDemo().main()
