#! /usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import threading
import rospy
from geometry_msgs.msg import Pose

class Visualizer:

    RATIO = 1279.305/517.136 # PNG:GMAPS
    IMAGE_SIZE = 3000 # size of X and Y (should be square aspect ratio)
    UTM_EAST_ZERO = 589702.87
    UTM_NORTH_ZERO = 4477172.947
    ZERO_LOC_UTM = (UTM_EAST_ZERO, UTM_NORTH_ZERO) # East North in meters UTM
    PNG_EAST_ZERO = 1866.1
    PNG_NORTH_ZERO = 1139.04
    UTM_ZONE_NUM = 17
    UTM_ZONE_LETTER = "T"

    def __init__(self):
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1,1,1)

        image_path = "/rb_ws/src/buggy/assets/satellite_unflip.png"
        im = plt.imread(image_path)
        self.image_arr = np.array(im)
        self.image_arr = np.rot90(np.rot90(self.image_arr))

        self.ax1.imshow(self.image_arr)

        self.e_utm = Visualizer.UTM_EAST_ZERO
        self.n_utm = Visualizer.UTM_NORTH_ZERO
        self.heading = 0
        
        self.lock = threading.Lock()

        self.plot_subscriber = rospy.Subscriber("sim_2d/utm", Pose, self.update_plot_var)
    
    def calculate_plot_from_utm(self, e_utm: float, n_utm: float, ratio: float) -> tuple:
        """Calculates the plot for matplotlib given a utm_coordinate

        Args:
            e_utm (float): utm input east
            n_utm (float): utm input north
            ratio (float): ratio PNG:GMAPS

        Returns:
            tuple: (x, y) for matplotlib
        """
        x = ((e_utm - Visualizer.UTM_EAST_ZERO) * ratio) + Visualizer.PNG_EAST_ZERO
        y = Visualizer.IMAGE_SIZE - (((n_utm - Visualizer.UTM_NORTH_ZERO) * ratio) + Visualizer.PNG_NORTH_ZERO)
        return (x, y)

    def get_arrow_vector(self, heading: float, size: float) -> tuple:
        """Get the dx and dy for input into plt.arrow()

        Args:
            heading (float): Heading, aligned with East to the right (degrees)
            size (float): size of arrow vector

        Returns:
            tuple: (dx, dy)
        """
        dx = size*np.cos(np.deg2rad(heading))
        dy = -size*np.sin(np.deg2rad(heading))
        return (dx, dy)
    
    def plot(self):
        """This function plots whatever is currently in self.e_utm, self.n_utm, and self.heading
        """
        with self.lock:
            e_utm = self.e_utm
            n_utm = self.n_utm
            heading = self.heading
        
        (x, y) = self.calculate_plot_from_utm(e_utm, n_utm, Visualizer.RATIO)
        (dx, dy) = self.get_arrow_vector(heading, 60)
        self.ax1.imshow(self.image_arr)
        self.ax1.arrow(x, y, dx, dy, width = 10)
        plt.pause(0.001)
        self.ax1.cla()

        
    
    def update_plot_var(self, utm: Pose):
        """Update the plotting variables so self.plot() can plot with the newest data

        Args:
            utm (Pose): Use the rospy Pose message to store UTM east + north and heading info
        """
        p = Pose()
        # x is East
        # y is North
        # z is heading (east aligned, + is CCW)
        with self.lock:
            self.e_utm = utm.position.x
            self.n_utm = utm.position.y
            self.heading = utm.position.z
        
        (x, y) = self.calculate_plot_from_utm(utm.position.x, utm.position.y, Visualizer.RATIO)


if __name__ == "__main__":
    vis = Visualizer()
    rospy.init_node("sim_2d_visualizer")
    while not rospy.is_shutdown():
        vis.plot()
