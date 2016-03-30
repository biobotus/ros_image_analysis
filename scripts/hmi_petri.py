#!/usr/bin/python

# Imports
import rospy
from std_msgs.msg import String

class HMIPetri():
    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

        # ROS publishments
        self.capture = rospy.Publisher('Capture_2D', String, queue_size=10)


    def hmi_petri_analysis(self):
        self.run = 1

        while self.run:
            file_name = raw_input(" File name of petri picture: ")
            self.capture.publish(file_name)
            self.run = int(raw_input(" Analyze another file? (yes = 1/no = 0) "))


if __name__ == '__main__':
    try:
        h = HMIPetri()
        h.hmi_petri_analysis()

    except (rospy.ROSInterruptException, EOFError) as e:
        print(e)

