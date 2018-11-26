#!/usr/bin/python
# -*- coding: utf-8 -*-
from pdt_msgs.msg import BoundingBox
from pdt_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
import os
import pygame
import time
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class TrackingDecision(object):
    # parameters need to modify
    box_sub_topic = '/pedstrian_bboxes'
    image_sub_topic = '/hk_video'
    # alarm_interval_sec = 30
    alarm_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'alarm.mp3')
    max_alarm_box_number = 5

    # parameters do not need to modify
    # box_sub = rospy.Subscriber()
    # decision_pub = rospy.Publisher()
    image_hight = -1
    image_width = -1
    is_first_frame = True
    begin_time = -1
    alarm_begin = False
    alarm_begin_time = -1
    cvi = CvBridge()
    src = np.array([])
    pre_pt = np.zeros(2, np.int32)
    end_pt = np.zeros(2, np.int32)
    cur_pt = np.zeros(2, np.int32)
    alarm_boxes = []
    person_boxes = []
    draw_tmp_box = False

    def __init__(self):
        # node
        self.box_sub = rospy.Subscriber(self.box_sub_topic, BoundingBoxes, self.box_callback, queue_size=1)
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
        cv2.namedWindow("img", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("img")
        cv2.setMouseCallback('img', self.on_mouse, 0)
        self.draw_boxes()


        cv2.imshow('img', self.src)
        cv2.waitKey(1)

        self.alarm_decision()

    def draw_boxes(self):
        if self.draw_tmp_box:
            cv2.rectangle(self.src, tuple(self.pre_pt), tuple(self.end_pt), (0, 0, 255), 2)
        for box in self.alarm_boxes:
            cv2.rectangle(self.src, tuple([box.xmin, box.ymin]), tuple([box.xmax, box.ymax]), (0, 0, 255), 2)
        for box in self.person_boxes:
            cv2.rectangle(self.src, tuple([box.xmin, box.ymin]), tuple([box.xmax, box.ymax]), (0, 255, 0), 2)

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
            if len(self.alarm_boxes) < self.max_alarm_box_number:
                self.alarm_boxes.append(tmp_alarm_box)
        if event == cv2.EVENT_RBUTTONUP:
            self.alarm_boxes = []

    def box_callback(self, msg):
        self.person_boxes = self.get_person_box(msg.bboxes)

    def alarm_decision(self):
        for alarm_box in self.alarm_boxes:
            for person_box in self.person_boxes:
                if self.boxes_intersect(alarm_box, person_box):
                    self.alarm_begin = True
                    break

    def boxes_intersect(self, box1, box2):
        if box1.xmin > box2.xmax:
            return False
        if box1.ymin > box2.ymax:
            return False
        if box2.xmin > box1.xmax:
            return False
        if box2.ymin > box1.ymax:
            return False
        return True


    def get_person_box(self, bboxes):
        box_person = []
        for box in bboxes:
            if box.Class == "person":
                box_person.append(box)
        return box_person

def play_alarm(td):
    print('alarm')
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
    rospy.init_node('alarm_setting_decision_node', anonymous=False)
    td = TrackingDecision()
    while not rospy.is_shutdown():
        play_alarm(td)
        time.sleep(0.1)
    rospy.spin()
