#!/usr/bin/python

import cv2
import img_analysis
import numpy as np
import os
import rospy
from std_msgs.msg import String


class ColonyAnalysis():
    """Analysis of bacteria colonies in Petri dish."""

    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('Done_Capture', String, \
                                             self.callback_analysis)

    def callback_analysis(self, data):
        """
        find_dish finds the petri dish in a picture using
        Circular Hough Transform (CHT).

        Inputs:
            - image_file : the image file name
            - radius_range : list of min and max radius of petri dish
        """

        # Filename
        image_file = data.data

        # Function variables
        img_dict = {}           # dictionnary of all images to print
        med_order = 3           # order for the median filter
        scale = 0.1             # scale factor to find circle faster
        #radius_range = [600, 800]
        radius_range = [550, 650]
        radius_range = tuple([int(scale*rad) for rad in radius_range])
        circle_line_width = 25  # line width in pixels
        circle_color = img_analysis.COLOR_GREEN

        # Read image file
        image = cv2.imread(image_file)
        if image is None:
            print("Requested image does not exist: {0}".format(image_file))
            return

        shape = image.shape
        max_x = shape[1]
        max_y = shape[0]
        size = (int(scale*max_x), int(scale*max_y)) # (size_x, size_y)
        im_scale = cv2.resize(image, dsize=size)
        img_dict["{0} - Original image".format(image_file)] = image

        # Transform image to grayscale
        im_gray = cv2.cvtColor(im_scale, cv2.COLOR_BGR2GRAY)
        #img_dict["Original image converted to grayscale"] = im_gray

        # Adjust grayscale image contrast
        # im_filt = cv2.equalizeHist(im_gray) # Does not work like imadjust of Matlab
        #img_dict["Grayscale image with better contrast"] = np.hstack((im_gray,im_filt))
        im_filt = im_gray   # Temporary workaround to not having imadjust

        # Slightly blur grayscale image
        im_filt_med = cv2.medianBlur(im_filt, med_order)
        # img_dict["Grayscale image filtered w/ median blurring (order {0})".format(\
        #                             med_order)] = np.hstack((im_filt, im_filt_med))

        # Find circles is the image with Hough Circle Transform
        # The algorithm returns a list of (x, y, radius) where (x, y) is center
        circles = cv2.HoughCircles(im_filt_med, cv2.cv.CV_HOUGH_GRADIENT, 2, \
                     20, minRadius=radius_range[0], maxRadius=radius_range[1])

        # Return nothing if no or more than one circle is found
        if not isinstance(circles, type(np.empty(0))):
            print("Error - no circle found")
            return
        elif len(circles) != 1:
            print("Error - {0} circles found, there should only be 1.".format(\
                                                                 len(circles)))
            return

        # Display found circle on original image
        c_x, c_y, radius = [int(i/scale) for i in circles[0,0,:]]
        im_circle = np.copy(image)
        cv2.circle(im_circle, (c_x, c_y), radius, circle_color, circle_line_width)
        #img_dict["Detected circle"] = im_circle

        # Crop original image around circle
        im_crop = image[c_y-radius:c_y+radius, c_x-radius:c_x+radius]
        #img_dict["Cropped image around petri dish"] = im_crop

        # Create boolean circle mask (True inside, False outside)
        nx = np.linspace(-c_x, max_x - c_x - 1, max_x)
        ny = np.linspace(-c_y, max_y - c_y - 1, max_y)
        mesh_x, mesh_y = np.meshgrid(nx, ny)
        c_mask = mesh_x ** 2 + mesh_y ** 2 <= radius ** 2
        mask_crop = c_mask[c_y-radius:c_y+radius, c_x-radius:c_x+radius]

        # Stack the mask three times in the Z axis to make it 3D
        mask_3d = np.dstack(3*(mask_crop,))

        # Apply the mask to the cropped picture
        im_crop_mask = im_crop*mask_3d
        img_dict["Cropped image with mask"] = im_crop_mask

        # Display all images stored in the dictionnary if desired
        img_analysis.imshow(img_dict)

    def listener(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        ca = ColonyAnalysis()
        ca.listener()

    except rospy.ROSInterruptException as e:
        print(e)

