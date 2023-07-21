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

        self.x_magni = []
        self.y_magni = []
        self.z_magni = []
        # rospy.Subscriber('/iris/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/mavros/global_position/local', Odometry, self.iris_callback)
        rospy.Subscriber('/exact_pose', Odometry, self.magni_callback)

        self.csv_iris_pose = 'iris_pose.csv'
        self.csv_magni_pose = 'magni_pose.csv'
        self.csv_headers = ['Time', 'X Position', 'Y Position', 'Z Position']

    def iris_callback(self, data):
        self.x_iris.append(data.pose.pose.position.x + 1)
        self.y_iris.append(data.pose.pose.position.y + 1)
        self.z_iris.append(data.pose.pose.position.z)
        self.save_csv_iris_pose(data.header.stamp.to_sec())
        # self.plot_data()

    def magni_callback(self, data):
        self.x_magni.append(data.pose.pose.position.x)
        self.y_magni.append(data.pose.pose.position.y)
        self.z_magni.append(data.pose.pose.position.z + 0.53)
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
                writer.writerow([timestamp, self.x_iris[i], self.y_iris[i], self.z_iris[i]])

    def save_csv_magni_pose(self, timestamp):
        print(timestamp)
        with open(self.csv_magni_pose, 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.csv_headers)
            for i in range(len(self.x_magni)):
                writer.writerow([timestamp, self.x_magni[i], self.y_magni[i], self.z_magni[i]])

if __name__ == '__main__':
    try:
        plotter = PosePlotter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
