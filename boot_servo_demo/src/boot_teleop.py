#!/usr/bin/env python
# Initial code created by Graylin Trevor Jay (tjay@cs.brown.edu) an published under Crative Commens Attribution license.
# addition for signal interrupt by Koen Buys

from bootstrapping_olympics import get_boot_config
from quickapp import QuickAppBase
from rosstream2boot import get_rs2b_config
import numpy as np
import select
import signal
import sys
import termios
import tty
import time

moveBindings = {
    'u': (+1, +1),
    'i': (+1, +0),
    'o': (+1, -1),
    'j': (0., +1),
    'k': (0., +0),
    'l': (0., -1),
    'm': (-1, +1),
    ',': (-1, +0),
    '.': (-1, -1),
}
 
class TimeoutException(Exception): 
    pass 

def getKeys(timeout):
    settings = termios.tcgetattr(sys.stdin)

    try:
        while True:
            def timeout_handler(signum, frame):
                raise TimeoutException()
            old_handler = signal.signal(signal.SIGALRM, timeout_handler)
            signal.alarm(timeout)  # this is the watchdog timing
            tty.setraw(sys.stdin.fileno())
            select.select([sys.stdin], [], [], 0)
            try:
                key = sys.stdin.read(1)
            except TimeoutException:
                key = "-"
            finally:
                signal.signal(signal.SIGALRM, old_handler)
            yield key
            signal.alarm(0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    finally:    
        signal.alarm(0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
 
 
class TeleOp(QuickAppBase):
    
    def define_program_options(self, params):
        params.add_string('robot')
        params.add_string_list('config', default=[])
        params.add_float('rate', default=0.1)
        
    def go(self):
        options = self.get_options()
        rate = options.rate
        boot_config = get_boot_config()
        rs2b_config = get_rs2b_config()
        
        for c in options.config:
            boot_config.load(c)
            rs2b_config.load(c)
            
        id_robot = options.robot
        robot = boot_config.robots.instance(id_robot)
        
        rest = robot.get_spec().get_commands().get_default_value()
        robot.set_commands(rest, 'rest')
        
        try: 
            for key in getKeys(timeout=0):
                if key == 'q':
                    break
                if key in moveBindings.keys():
                    u = np.array(moveBindings[key])
                else:
                    u = rest
                robot.set_commands(u, 'teleop')
                time.sleep(rate)
        except Exception as e:
            print str(e)
            
        finally:
            robot.set_commands(rest, 'rest')

if __name__ == '__main__':
    TeleOp.get_sys_main()()
