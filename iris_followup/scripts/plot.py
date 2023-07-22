#!/usr/bin/env python
import csv
import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class PosePlotter:
    def __init__(self):
        rospy.init_node('pose_plotter', anonymous=True)
        self.x_iris = []
        self.y_iris = []
        self.z_iris = []
        self.yaw_iris = []
        self.timestamp_iris = []

        self.x_magni = []
        self.y_magni = []
        self.z_magni = []
        self.yaw_magni = []
        self.timestamp_magni = []

        # rospy.Subscriber('/iris/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/mavros/global_position/local', Odometry, self.iris_callback)
        rospy.Subscriber('/exact_pose', Odometry, self.magni_callback)

        self.csv_iris_pose = 'iris_pose.csv'
        self.csv_magni_pose = 'magni_pose.csv'
        self.csv_headers = ['Time', 'X Position', 'Y Position', 'Z Position', 'yaw']

    def quaternion_to_yaw(self, orientation):
        # Convert quaternion to yaw angle
        import math
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        roll, pitch, yaw = math.atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y)), \
                           math.asin(2.0*(w*y - z*x)), \
                           math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
        return yaw

    def iris_callback(self, data):
        self.x_iris.append(data.pose.pose.position.x + 1)
        self.y_iris.append(data.pose.pose.position.y + 1)
        self.z_iris.append(data.pose.pose.position.z)
        self.timestamp_iris.append(data.header.stamp.to_sec())

        orientation = data.pose.pose.orientation
        self.yaw_iris.append(self.quaternion_to_yaw(orientation))
        
        self.save_csv_iris_pose(data.header.stamp.to_sec())
        # self.plot_data()

    def magni_callback(self, data):
        self.x_magni.append(data.pose.pose.position.x)
        self.y_magni.append(data.pose.pose.position.y)
        self.z_magni.append(data.pose.pose.position.z + 0.53)
        self.timestamp_magni.append(data.header.stamp.to_sec())

        orientation = data.pose.pose.orientation
        self.yaw_magni.append(self.quaternion_to_yaw(orientation))

        self.save_csv_magni_pose(data.header.stamp.to_sec())
        # self.plot_data()

    def plot_data(self):
        plt.clf()
        plt.plot(self.x_positions, self.y_positions, 'b.')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Position Plot')
        plt.axis([-2, 5, -0.1, 0.1])
        plt.pause(0.01)  # Pause to allow real-time updates

    def save_csv_iris_pose(self, timestamp):
        print(timestamp)
        with open(self.csv_iris_pose, 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.csv_headers)
            for i in range(len(self.x_iris)):
                writer.writerow([self.timestamp_iris[i], self.x_iris[i], self.y_iris[i], self.z_iris[i], self.yaw_iris[i]     ])

    def save_csv_magni_pose(self, timestamp):
        print(timestamp)
        with open(self.csv_magni_pose, 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.csv_headers)
            for i in range(len(self.x_magni)):
                writer.writerow([self.timestamp_magni[i], self.x_magni[i], self.y_magni[i], self.z_magni[i], self.yaw_magni[i]         ])

if __name__ == '__main__':
    try:
        plotter = PosePlotter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
