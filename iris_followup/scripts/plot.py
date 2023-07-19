#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped

class PosePlotter:
    def __init__(self):
        rospy.init_node('pose_plotter', anonymous=True)
        self.x_positions = []
        self.y_positions = []
        rospy.Subscriber('/iris/pose', PoseStamped, self.pose_callback)

    def pose_callback(self, msg):
        self.x_positions.append(msg.pose.position.x)
        self.y_positions.append(msg.pose.position.y)
        self.plot_data()

    def plot_data(self):
        plt.clf()
        plt.plot(self.x_positions, self.y_positions, 'b.')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Position Plot')
        plt.pause(0.01)  # Pause to allow real-time updates

if __name__ == '__main__':
    try:
        plotter = PosePlotter()
        plt.ion()  # Turn on interactive mode for live plotting
        plt.show()  # Show the plot window
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
