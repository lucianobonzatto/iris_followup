import os
import csv
import glob
import rospy
import rosbag
import rospkg 
import datetime
import subprocess
import tkinter as tk
from tkinter import Label, Entry, Button, Radiobutton, StringVar

class BagReader:
  def __init__(self):
    self.csv_headers = ['Time', 'X_vel_uav', 'Y_vel_uav', 'Z_vel_uav', 'yaw_vel_uav'
                        , 'X_cmd_vel', 'Y_cmd_vel', 'Z_cmd_vel', 'yaw_cmd_vel']
    self.x_vel_uav = []
    self.y_vel_uav = []
    self.z_vel_uav = []
    self.yaw_vel_uav = []
    self.timestamp_vel_uav = []

    self.x_cmd_vel = 0
    self.y_cmd_vel = 0
    self.z_cmd_vel = 0
    self.yaw_cmd_vel = 0

    self.x_cmd = []
    self.y_cmd = []
    self.z_cmd = []
    self.yaw_cmd = []
    
  def static_bag(self):
    folder = os.path.join("/home/lukn23/bag")
    data = glob.glob(folder + "/*")

    index = 0
    for bag_filename in data:
      index += 1
      bag_filename = folder + "/" + str(index) + ".bag"
      print(index, "\t-> ", bag_filename)

      self.x_vel_uav = []
      self.y_vel_uav = []
      self.z_vel_uav = []
      self.yaw_vel_uav = []
      self.timestamp_vel_uav = []

      self.x_cmd = []
      self.y_cmd = []
      self.z_cmd = []
      self.yaw_cmd = []
      
      self.x_cmd_vel = 0
      self.y_cmd_vel = 0
      self.z_cmd_vel = 0
      self.yaw_cmd_vel = 0

      try:
        with rosbag.Bag(bag_filename, 'r') as bag:
          topics = bag.get_type_and_topic_info().topics.keys()

          for topic, msg, t in bag.read_messages():
            print(t, end='\r')
            self.topic_treatment(topic, msg, t)
            if rospy.is_shutdown():
              bag.close()
              return -1

          print("\nfim")
          bag.close()
      except rosbag.ROSBagException as e:
        rospy.logerr("Erro ao reproduzir o arquivo de bag: %s", str(e))
    
      filename = "uav_" + str(index) + ".csv"
      with open(filename, 'w') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(self.csv_headers)
        for i in range(len(self.timestamp_vel_uav)):
          writer.writerow([ self.timestamp_vel_uav[i],
                            self.x_vel_uav[i],
                            self.y_vel_uav[i],
                            self.z_vel_uav[i],
                            self.yaw_vel_uav[i],
                            self.x_cmd[i],
                            self.y_cmd[i],
                            self.z_cmd[i],
                            self.yaw_cmd[i],
                          ])
      print(filename)

  def topic_treatment(self, topic, msg, t):
    if(topic == "/mavros/local_position/odom"):
      # rospy.loginfo("Reproduzindo mensagem em %s", topic)
      self.x_vel_uav.append(msg.twist.twist.linear.x)
      self.y_vel_uav.append(msg.twist.twist.linear.y)
      self.z_vel_uav.append(msg.twist.twist.linear.z)
      self.yaw_vel_uav.append(msg.twist.twist.angular.z)
      self.timestamp_vel_uav.append(msg.header.stamp.to_sec())

      self.x_cmd.append(self.x_cmd_vel)
      self.y_cmd.append(self.y_cmd_vel)
      self.z_cmd.append(self.z_cmd_vel)
      self.yaw_cmd.append(self.yaw_cmd_vel)

    elif(topic == "/mavros/setpoint_velocity/cmd_vel"):
      # rospy.loginfo("Reproduzindo mensagem em %s", topic)
      self.x_cmd_vel = msg.twist.linear.x
      self.y_cmd_vel = msg.twist.linear.y
      self.z_cmd_vel = msg.twist.linear.z
      self.yaw_cmd_vel = msg.twist.angular.z

def main():
    rospy.init_node('read_bag_node', anonymous=True)
    app = BagReader()
    app.static_bag()

if __name__ == "__main__":
    main()