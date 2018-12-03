#!/usr/bin/python
# -*- coding: utf-8 -*-
from sensor_msgs.msg import Image
import os
import pygame
import time
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from pdt_msgs.msg import BoundingBox


class MotionDetection(object):
    # parameters need to modify
    image_sub_topic = '/hk_video'
    # alarm_interval_sec = 30
    alarm_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'alarm.mp3')
    # alarm_threshold = 0.1
    alarm_pixel_num = 1000
    front_pixel_threshold = 50

    # parameters do not need to modify
    # box_sub = rospy.Subscriber()
    # decision_pub = rospy.Publisher()
    WINDOWNAME = "motion_detection"
    image_hight = -1
    image_width = -1
    is_first_frame = True
    alarm_begin = False
    cvi = CvBridge()
    src = np.array([])
    pre_pt = np.zeros(2, np.int32)
    end_pt = np.zeros(2, np.int32)
    cur_pt = np.zeros(2, np.int32)
    draw_tmp_box = False

    alarm_box = []
    run_detection = False
    alarm_src_image = np.array([])
    set_background = False
    background = np.array([])
    diff = np.array([])



    def __init__(self):
        # node
        self.image_sub = rospy.Subscriber(self.image_sub_topic, Image, self.image_callback, queue_size=1)

    def image_callback(self, msg):
        try:
            self.src = self.cvi.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
        if self.is_first_frame:
            self.image_hight = self.src.shape[0]
            self.image_width = self.src.shape[1]
            self.is_first_frame = False
        cv2.namedWindow(self.WINDOWNAME, cv2.WINDOW_NORMAL)
        # cv2.namedWindow(self.WINDOWNAME)
        cv2.setMouseCallback(self.WINDOWNAME, self.on_mouse, 0)

        if self.set_background:
            self.set_background = False
            self.background = self.crop_trans_image(self.background)
            self.run_detection = True

        if self.run_detection:
            self.alarm_src_image = self.crop_trans_image(self.alarm_src_image)
            self.diff = cv2.absdiff(self.background, self.alarm_src_image)
            self.diff = cv2.threshold(self.diff, self.front_pixel_threshold, 1, cv2.THRESH_BINARY)[1]
            # cv2.imshow("diff", self.diff*255)
            # cv2.waitKey(1)

        self.draw_boxes()
        cv2.imshow(self.WINDOWNAME, self.src)
        cv2.waitKey(1)

        self.alarm_decision()

    def crop_trans_image(self, image):
        image = cv2.cvtColor(self.src, cv2.COLOR_BGR2GRAY)
        image = image[self.alarm_box[0].ymin:self.alarm_box[0].ymax, self.alarm_box[0].xmin:self.alarm_box[0].xmax]
        image = cv2.GaussianBlur(image, (11, 11), 0)
        return image

    def draw_boxes(self):
        if self.draw_tmp_box:
            cv2.rectangle(self.src, tuple(self.pre_pt), tuple(self.end_pt), (0, 0, 255), 2)
        for box in self.alarm_box:
            cv2.rectangle(self.src, tuple([box.xmin, box.ymin]), tuple([box.xmax, box.ymax]), (0, 0, 255), 2)

    def on_mouse(self, event, x, y, flags, param):
        self.cur_pt = np.array([x, y], np.int32)
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pre_pt = np.array([x, y], np.int32)
        if flags == cv2.EVENT_FLAG_LBUTTON:
            self.draw_tmp_box = True
            self.end_pt = self.cur_pt
        if event == cv2.EVENT_LBUTTONUP:
            self.draw_tmp_box = False
            tmp_alarm_box = BoundingBox()
            tmp_alarm_box.ymin = min(self.pre_pt[1], self.end_pt[1])
            tmp_alarm_box.xmin = min(self.pre_pt[0], self.end_pt[0])
            tmp_alarm_box.ymax = max(self.pre_pt[1], self.end_pt[1])
            tmp_alarm_box.xmax = max(self.pre_pt[0], self.end_pt[0])
            tmp_alarm_box.ymin_normal = float(tmp_alarm_box.ymin) / self.image_hight
            tmp_alarm_box.xmin_normal = float(tmp_alarm_box.xmin) / self.image_width
            tmp_alarm_box.ymax_normal = float(tmp_alarm_box.ymax) / self.image_hight
            tmp_alarm_box.xmax_normal = float(tmp_alarm_box.xmax) / self.image_width
            if tmp_alarm_box.xmax - tmp_alarm_box.xmin > 2 and tmp_alarm_box.xmax - tmp_alarm_box.xmin > 2:
                self.alarm_box = [tmp_alarm_box]
                self.set_background = True
                # print(tmp_alarm_box)
        if event == cv2.EVENT_LBUTTONDBLCLK:
            self.alarm_box = []
            self.run_detection = False
            cv2.destroyWindow("diff")

    def alarm_decision(self):
        if self.run_detection:
            # if float(self.diff.sum()) / (self.diff.shape[0] * self.diff.shape[1]) > self.alarm_threshold:
            #     self.alarm_begin = True
            # print(self.diff.sum())
            if self.diff.sum() > self.alarm_pixel_num:
                self.alarm_begin = True

    def get_person_box(self, bboxes):
        box_person = []
        for box in bboxes:
            if box.Class == "person":
                box_person.append(box)
        return box_person

def play_alarm(td):
    while not rospy.is_shutdown():
        time.sleep(0.1)
        if td.alarm_begin:
            print("alarm")
            td.alarm_begin = False
            pygame.mixer.init()
            pygame.mixer.music.load(td.alarm_file)
            pygame.mixer.music.play()
            time.sleep(2)
            pygame.mixer.music.stop()

if __name__ == '__main__':
    rospy.init_node('motion_detection_node', anonymous=False)
    md = MotionDetection()
    while not rospy.is_shutdown():
        play_alarm(md)
        time.sleep(0.1)
    rospy.spin()
