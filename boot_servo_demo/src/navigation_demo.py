#!/usr/bin/env python
import roslib; roslib.load_manifest('boot_servo_demo')  # @IgnorePep8

from boot_servo_demo.msg import Raw
from bootstrapping_olympics import bd_sequence_from_robot, get_boot_config
from bootstrapping_olympics.programs.manager import (DataCentral,
    load_agent_state)
from bootstrapping_olympics.utils import (expand_environment, safe_pickle_dump,
    safe_pickle_load)
from conf_tools import GlobalConfig
from contracts import contract
from ros_node_utils import ROSNode
from rospy.numpy_msg import numpy_msg
from servo_demo2 import ServoDemo2
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
import contracts
import numpy as np
import rospy
 
nraw = numpy_msg(Raw)


STATE_WAIT = 'wait'
STATE_SERVOING = 'servoing'


class NavigationDemo(ROSNode):
    """ 
        
        Parameters:
            boot_root: Root directory for the
             
            id_agent: id_agent  
            id_robot: id_robot
            
            id_robot_learned: 
            
            map = out-exp29/compmake/cm:default:res:20130531195407-create_navigation_map_from_episode.pickle
            config_dir: list of configutation directories
                    
    """
    state_filename = 'y_goal.pickle'
    
    def __init__(self):
        ROSNode.__init__(self, 'ServoDemo')
        
    def main(self):
        rospy.init_node('servo_demo', disable_signals=True)
        
        self.info('Started.')
        contracts.disable_all()

        boot_root = rospy.get_param('~boot_root')
        boot_root = expand_environment(boot_root)
        
        config_dir = rospy.get_param('~config_dir')
        id_robot_learned = rospy.get_param('~id_robot_learn')
        
        self.info('loading %r' % config_dir)
        GlobalConfig.global_load_dir(config_dir)
        
        id_agent = rospy.get_param('~id_agent')
        self.id_robot = rospy.get_param('~id_robot')
        self.sleep = rospy.get_param('~sleep', 0.005)
        self.info('sleep: %s' % self.sleep)
#         self.error_threshold = float(rospy.get_param('~error_threshold'))
        
        self.ratio_threshold = float(rospy.get_param('~ratio_threshold'))  # 0.95
        self.d_next_threshold = float(rospy.get_param('~d_next_threshold'))  # 0.03
        self.d_next_threshold_alone = float(rospy.get_param('~d_next_threshold_alone'))  # 0.04
        
        
        nmap = expand_environment(rospy.get_param('~map'))
        self.nmap = safe_pickle_load(nmap)
        
        raise_if_no_state = rospy.get_param('~raise_if_no_state', True)
        
        data_central = DataCentral(boot_root)
        
        ag_st = load_agent_state(data_central, id_agent, id_robot_learned,
                                 reset_state=False,
                                 raise_if_no_state=raise_if_no_state)
        self.agent, state = ag_st
        
        self.info('Loaded state: %s' % state)
        
        self.servo_agent = self.agent.get_servo()
        
        bo_config = get_boot_config()
        self.robot = bo_config.robots.instance(self.id_robot)
        self.boot_spec = self.robot.get_spec()
            
        self.publish_info_init()     
        
        self.y = None
        self.y_goal = None
        self.started_now = False
        self.stopped_now = False
        self.e0 = 1
        self.e = 1         
        self.last_boot_data = None
        self.state = STATE_WAIT
        self.index_cur = None
        
        self.info('Defining services')
        rospy.Service('set_goal', Empty, self.srv_set_goal)
        rospy.Service('start_servo', Empty, self.srv_start_servo)
        rospy.Service('stop_servo', Empty, self.srv_stop_servo)
                
        self.info('Finished initialization') 
        self.count = 0
        
        self.go()
        
        
    def go(self):
        id_episode = 'XXX'  # XXX
        id_environment = 'xxx'  # XXX

        bd_seq = bd_sequence_from_robot(id_robot=self.id_robot,
                                     robot=self.robot,
                                     sleep_wait=self.sleep,
                                     id_episode=id_episode,
                                     id_environment=id_environment,
                                     check_valid_values=False)
        
        try:
            for bd in bd_seq: 
                self.process(bd)
        except KeyboardInterrupt:
            self.info('Keyboard interrupt-sending stop')
            self._send_rest()

    def srv_set_goal(self, req):  # @UnusedVariable
        self.info('called "set_goal"')
        # reset localization
        self.index_cur = None
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
        self.stopped_now = True
        return EmptyResponse()
 
    def check_delay(self, bd):
        ts = bd['timestamp'].item()
        then = rospy.Time.from_seconds(ts)
        now = rospy.Time.now()
        delay = (now - then).to_sec()
        if delay > 0.1:
            print('delay: %4.2fsec' % delay)
        
    def process(self, boot_data):
        self.check_delay(boot_data)
        
        self.last_boot_data = boot_data.copy()
        
        self.u = self.get_servo_commands(boot_data)
        self.publish_info()
        
        msg = '%s|' % self.state
        pc = lambda x: ' '.join(['%+6.3f' % a for a in x]) 
        msg += 'u: %s ' % pc(self.u)
        
        
        msg += ('e0: %8f   e: %8f  e/e0: %8f' % 
                (self.e0, self.e, self.e / self.e0))
        
        verbose = True
        if self.state == STATE_WAIT:
            if verbose:
                self.info('hold %s' % msg)
            
        elif self.state == STATE_SERVOING:
            if verbose:
                self.info('send %s' % msg)
            self.robot.set_commands(self.u, 'servo')
        
        if self.stopped_now:
            self._send_rest()
            self.stopped_now = False

    def _send_rest(self):
        print('sending rest command')
        rest = self.robot.get_spec().get_commands().get_default_value()
        self.robot.set_commands(rest, 'rest')
        
    def get_servo_commands(self, boot_data):
        self.y = boot_data['observations']
         
        plus = 1
        inst = self.get_closest_point(self.y)
        if self.index_cur is None:
            # self.index_cur = inst
            self.index_cur = self.get_initial_closest_point(self.y)
            self.index_target = self.index_cur + plus
            
        if self.index_cur >= self.nmap.npoints() - 1 - plus:
            self.info('starting from beginning')
            self.index_cur = 0
            
        index_next = self.index_cur + 1 
        # if we are closer to target
        d_cur = self.get_distance(self.y, self.nmap.get_observations_at(self.index_cur))
        d_next = self.get_distance(self.y, self.nmap.get_observations_at(index_next))
        ratio = d_next / d_cur
        if False:
            print('ratio: %1.4f <= %1.4f  d_next: %1.4f < %1.4f' % (ratio, self.ratio_threshold,
                                                                d_next, self.d_next_threshold))
        if (d_next < self.ratio_threshold * d_cur and d_next < self.d_next_threshold) or \
            d_next < self.d_next_threshold_alone:
            self.info('closer to next than to cur')
            self.index_cur += 1
            self.index_target = self.index_cur + plus
        
        indices = dict(inst=inst, cur=self.index_cur, next=index_next, target=self.index_target)
        print(self.get_navigation_status_string(self.y, indices))
        
        goal = self.nmap.get_observations_at(self.index_target)
        self.set_goal_observations(goal)
        safe_pickle_dump(goal, ServoDemo2.state_filename)
        
        if self.started_now or self.e0 is None:
            # First iteration with new goal
            self.e0 = self.get_distance_to_goal(self.y)
            self.started_now = False

        self.servo_agent.process_observations(boot_data)
    
        u = self.servo_agent.choose_commands()         
            
        self.boot_spec.get_commands().check_valid_value(u)

        return u

    def get_navigation_status_string(self, cur_obs, indices):
        s = ''
        for k, index in indices.items():
            if index >= self.nmap.npoints():
                s += ' %s=out' % k
            else:
                obs = self.nmap.get_observations_at(index)
                dist = self.get_distance(obs, cur_obs)
                s += ' %s=%5d (%10f)' % (k, index, dist)
        return s
    
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
        return self.get_distance(y, self.y_goal)
    
    def get_distance(self, y1, y2):
#         return self.servo_agent.get_distance(y1, y2)
        disc = np.abs(y1 - y2)
        L1_robust = np.percentile(disc, 80)
        return L1_robust

    @contract(y='array')
    def set_goal_observations(self, y):
        self.y_goal = y.copy()        
        self.servo_agent.set_goal_observations(self.y_goal)
        
    def get_closest_point(self, y):
        return self.nmap.get_closest_point(y, self.get_distance)
        
    def get_initial_closest_point(self, y):
        i = 0 
        r = 50
        return self.nmap.get_closest_point_around(y, i, r, self.get_distance)


if __name__ == '__main__':
    NavigationDemo().main()
