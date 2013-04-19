
import roslib; roslib.load_manifest('boot_servo_demo')  # @IgnorePep8

import pylab
import Tkinter
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from rospy.numpy_msg import numpy_msg 
from boot_servo_demo.msg._Raw import Raw
import rospy

class ServoPlotter:
         
    def plot_stuff(self):
        if 'y' in self.data:
            y = self.data['y'].values            
            n = y.size
            x = range(n)
            self.line_y[0].set_data(x, y)
            
            
            if 'y_goal' in self.data:
                y_goal = self.data['y_goal'].values
                self.line_y_goal[0].set_data(x, y_goal)
            
            self.ax.axis([-1, n, -0.1, 1.1])
        else:
            print('y not received yet')
    
    def RealtimePloter(self):
        self.plot_stuff()
        self.canvas.draw()
        self.root.after(50, self.RealtimePloter)

    def _quit(self):
        self.root.quit()
        self.root.destroy()
        
    
    def main(self):
        root = Tkinter.Tk()
        self.root = root
        root.wm_title("Bootstrapping servo interface")
        
        
        xAchse = pylab.arange(0, 100, 1)
        yAchse = pylab.array([0] * 100)
        
        fig = pylab.figure(1)
        ax = fig.add_subplot(111)
        self.ax = ax
        ax.grid(True)
        ax.set_title("")
        ax.set_xlabel("sensel")
        ax.set_ylabel("")
        ax.axis([0, 100, -1.5, 1.5])
        
        self.line_y = ax.plot(xAchse, yAchse, 'b.')
        self.line_y_goal = ax.plot(xAchse, yAchse, 'r.')
        
        canvas = FigureCanvasTkAgg(fig, master=root)
        canvas.show()
        canvas.get_tk_widget().pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=1)
        
        toolbar = NavigationToolbar2TkAgg(canvas, root)
        toolbar.update()
        canvas._tkcanvas.pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=1)
        self.canvas = canvas
    
        self.button = Tkinter.Button(master=root, text='Quit', command=self._quit)
        self.button.pack(side=Tkinter.BOTTOM)
        
        self.wScale = Tkinter.Scale(master=root, label="View Width:", from_=3, to=1000, sliderlength=30,
                                     length=ax.get_frame().get_window_extent().width, orient=Tkinter.HORIZONTAL)
        self.wScale2 = Tkinter.Scale(master=root, label="Generation Speed:", from_=1, to=200, sliderlength=30,
                                      length=ax.get_frame().get_window_extent().width, orient=Tkinter.HORIZONTAL)
        self.wScale2.pack(side=Tkinter.BOTTOM)
        self.wScale.pack(side=Tkinter.BOTTOM)
        
        self.wScale.set(100)
        self.wScale2.set(self.wScale2['to'] - 10)
        
        self.root.protocol("WM_DELETE_WINDOW", self._quit)  # thanks aurelienvlg

        self.root.after(500, self.RealtimePloter)
        
        rospy.init_node('gui')
    
        rospy.Subscriber('/servo_manager/y', numpy_msg(Raw), self.callback_arrived, callback_args='y')
        rospy.Subscriber('/servo_manager/y_goal', numpy_msg(Raw), self.callback_arrived, callback_args='y_goal')        
#         rospy.Subscriber('/servo_manager/u', numpy_msg(Raw), self.callback_arrived, callback_args='u')        
#     
    
        self.data = {}
        #         rospy.spin()
        Tkinter.mainloop() 
        
    def callback_arrived(self, msg, name):
        self.data[name] = msg 


if __name__ == '__main__':
    ServoPlotter().main()
    
    
    
