#! /usr/bin/python


import rclpy
import rospy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data
import os
import cv2
import argparse
import rosbag
from cv_bridge import CvBridge
import csv

# source /opt/ros/foxy/setup.bash 까먹지 말자

class DataSubscriber():
    def __init__(self):
        self.img_front = rospy.Subscriber('/carla/flying_sensor/rgb_front/image', 
            Image, self.callback_img_front, 10)
        self.img_down = rospy.Subscriber('/carla/flying_sensors/rgb_down/image',
            Image, self.callback_img_down, 10)
        self.gnss_msg = rospy.Subscriber('/carla/flying_sensor/gnss',
            NavSatFix, self.callback_gnss, 10)

        self.img_f_msg = None
        self.img_d_msg = None
        self.gnss_msg  = None
    
    def callback_img_front(self, img_f_msg):
        self.img_f_msg = img_f_msg

    def callback_img_down(self, img_d_msg):
        self.img_d_msg = img_d_msg

    def callback_gnss(self, gnss_msg):
        self.gnss_msg = gnss_msg


class DataProcess():
    def __init__(self):
        pass
    def saveImg(self, args, bag, bridge):
        count_f = 0
        count_d = 0

        if args.image_topic_front == '/carla/flying_sensor/rgb_front/image':
            
            f_front = open('front_cam.csv', 'w', newline='')
            ff = csv.writer(f_front)    # front file : ff

            for topic, msg, t in bag.read_messages(topics=[args.image_topic_front]):
                cv_img_f = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                cv2.imwrite(os.path.join(args.output_dir_front, "00000000000000%i.png" % count_f), cv_img_f)
                if count_f % 50 == 0:
                    print("saved front image %i" %count_f) 

                ff.writerow([t, "00000000000000%i.png" % count_f])
                count_f += 1
            # bag.close()
            f_front.close()

        if args.image_topic_down == '/carla/flying_sensor/rgb_down/image':

            f_down = open('down_cam.csv', 'w', newline='')
            df = csv.writer(f_down)    #down file
            
            for topic, msg, t in bag.read_messages(topics=[args.image_topic_down]):
                cv_img_d = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                cv2.imwrite(os.path.join(args.output_dir_down, "00000000000000%i.png" % count_d), cv_img_d)
                if count_d % 50 == 0:
                    print("saved downward image %i" %count_d) 
                
                df.writerow([t, "00000000000000%i.png" % count_d])
                count_d += 1
            # bag.close()
            f_down.close()

def main():
    rospy.init_node('sensor_data_subscriber')
    data = DataSubscriber()
    process = DataProcess()
    bridge = CvBridge()
    """
    get sensor datas from rosbag
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag")
    parser.add_argument("--bag_file", help="Input ROS bag.")
    parser.add_argument("--output_dir_front", help="Output directory for front images")
    parser.add_argument("--output_dir_down", help="Output directory for down images")
    parser.add_argument("--image_topic_front", help="image topic for front image")
    parser.add_argument("--image_topic_down", help="image topic for down image")

    args = parser.parse_args()
    print("Extract front images from %s on topics %s into %s" %(args.bag_file, args.image_topic_front, args.output_dir_front))
    print("Extract downward images from %s on topics %s into %s" %(args.bag_file, args.image_topic_down, args.output_dir_down))
    
    bag = rosbag.Bag(args.bag_file, 'r')
    process.saveImg(args, bag, bridge)  # front Images
    

    bag.close()

if __name__ == '__main__':
    main()

