#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image

from person_blob_ros.srv import *

import numpy as np

import cv2

from cv_bridge import CvBridge

#from pyefd import elliptic_fourier_descriptors

#from sklearn.cluster import DBSCAN


################################################################################
class ImageContainer(object):
    """Just container nothing else."""
    def __init__(self):
        super(ImageContainer, self).__init__()
        self.Image = None
        self._bridge = CvBridge()

    def setImage(self, image_msgs):
        self.Image = self._bridge.imgmsg_to_cv2(image_msgs, "passthrough")

class ParameterContainer(object):
    """docstring for ParameterContainer."""
    def __init__(self):
        super(ParameterContainer, self).__init__()
        self.min_distance = None
        self.max_distance = None
        self.depth_topic = None

        self.get_parameters_from_file()

    def get_parameters_from_file(self):
        self.min_distance = rospy.get_param("/person_blob_ros/min_distance")
        self.max_distance = rospy.get_param("/person_blob_ros/max_distance")
        self.depth_topic = rospy.get_param("/person_blob_ros/depth_topic")

################################################################################

# To make it fast I set some global variables sorry :/
ic = ImageContainer()
pc = ParameterContainer()

def check_if_person_exists(im, contours, area_threshold=50000):
    """
    Experimental function to check if a set of contours are person or not
    """
    for cnt in contours:
        # Get the area of a contour
        area = cv2.contourArea(cnt)

        if area > area_threshold:
            return True, area

    return False, 0

    #cv2.drawContours(im, contours, -1, 255, 3)

    # Show keypoints
    #cv2.imshow("Keypoints", im)

    #cv2.waitKey(0)



def find_blob(depth_image, human_thresh, min_thresh=600, max_thresh=1200):
    """
    Contains the logic to find the blob in front of the camera
    """

    global feature_list
    global cnts
    # Apply thresholding after 1.2 metres

    im = depth_image.copy()
    im[im >= max_thresh] = 0
    im[im <= min_thresh] = 0

    im = (im/(max_thresh/255)).astype('uint8')

    nonzero_count = np.count_nonzero(im)

    print "Ratio of blobs to image: {}"\
        .format(float(nonzero_count) / float(im.shape[0]*im.shape[1]))

    ret,thresh = cv2.threshold(im,30,255,0)
    kernel = np.ones((10,10),np.uint8)
    thresh = cv2.dilate(thresh,kernel,iterations = 1)

    contours, hierarchy = \
        cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    is_person, blob_area = check_if_person_exists(im, contours, human_thresh)

    return is_person, blob_area

#def efd_feature(contour, order=20, normalize=True):
#    """
#    Packs up the elliptic fourier descriptors
#    """
#    coeffs = \
#    elliptic_fourier_descriptors(\
#        np.squeeze(contour), order=order, normalize=normalize)
#
#    if normalize:
#        return coeffs.flatten()[3:]
#    else:
#        return coeffs.flatten()[:]


def callback(arg):
    """
    Callback for depth images
    """
    global ic

    ic.setImage(arg)

def check(req):
    """
    Checks if there is a blob in front of the camera
    """

    global ic
    global pc

    response = CheckResponse()

    try:
        is_person, blob_area = find_blob(ic.Image, req.blob_threshold, pc.min_distance, pc.max_distance)
    except:
        is_person = False
        blob_area = 0

    response.if_person_exists = is_person

    response.blob_area = blob_area

    return response

def main():
    global pc
    # Create the node
    rospy.init_node('check_if_person_exitsts_srv')
    # Subscribe to depth image
    rospy.Subscriber(pc.depth_topic, Image, callback)
    # Create the server
    s = rospy.Service('check_if_person_exists', Check, check)

    rospy.spin()

if __name__ == "__main__":
    main()
