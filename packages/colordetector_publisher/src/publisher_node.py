#!/usr/bin/env python

import os
import pwd
import rospy
from duckietown import DTROS
from std_msgs.msg import String
import picamera
import picamera.array
from time import sleep
from pprint import pprint



class PublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(PublisherNode, self).__init__(node_name=node_name)
        # construct publisher
        self.pub = rospy.Publisher('MaxiColorDetector', String, queue_size=10)

    def run(self):
        # publish message every 5 second
        rate = rospy.Rate(5) # 1Hz

        print("before camera init")
        with picamera.PiCamera() as camera:
            w_x = 320
            w_y = 240
            camera.resolution = (w_x, w_y)
            print("after camera init")

            while not rospy.is_shutdown():
                with picamera.array.PiRGBArray(camera) as output:
                    try:
                        camera.capture(output, 'rgb')

                        message = output.array
                        self.pub.publish(message)

                        rospy.loginfo("Publishing message with image content.")

                        rate.sleep()
                    except Exception as e:
                        print("an exception occurred while capturing image and sending message: " +str(e))
                        print("close camera now")
                        camera.close()
        camera.close()


if __name__ == '__main__':
    # create the node
    node = PublisherNode(node_name='publisher_node')
    # run node
    node.run()