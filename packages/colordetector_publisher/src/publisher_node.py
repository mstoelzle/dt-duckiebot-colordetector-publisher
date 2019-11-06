#!/usr/bin/env python

import os
import pwd
import rospy
from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
import picamera
import picamera.array
from cv_bridge import CvBridge, CvBridgeError
from time import sleep
from pprint import pprint



class PublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(PublisherNode, self).__init__(node_name=node_name)
        # construct publisher
        self.pub = rospy.Publisher('MaxiColorDetector', CompressedImage, queue_size=10)

        self.bridge = CvBridge()

        # RaspberryPi Camera: mode = 0
        # sample image: mode = 1
        self.mode = 0

    def run(self):
        # publish message every 5 second
        rate = rospy.Rate(5) # 1Hz

        with picamera.PiCamera() as camera:
            w_x = 320
            w_y = 240
            camera.resolution = (w_x, w_y)

            while not rospy.is_shutdown():
                with picamera.array.PiRGBArray(camera) as output:
                    try:
                        camera.capture(output, 'rgb')

                        img = output.array

                        self.publish_image(img)

                        rate.sleep()
                    except Exception as e:
                        print("an exception occurred while capturing image and sending message: " +str(e))
                        print("close camera now")
                        camera.close()
        camera.close()

    def publish_image(self,img):
        try:
            self.pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

        rospy.loginfo("Publishing message with image content.")


if __name__ == '__main__':
    # create the node
    node = PublisherNode(node_name='publisher_node')
    # run node
    node.run()