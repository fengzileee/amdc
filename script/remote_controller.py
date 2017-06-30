#!/usr/bin/env python2

import roslib
roslib.load_manifest('amdc')
import rospy
from amdc.msg import PropellerCmd

from Tkinter import *
from math import atan2
from math import pi

window_size = 400
knob_size = window_size * 0.35
knob_coord = [window_size / 2. - knob_size / 2.,
              window_size / 2. - knob_size / 2.,
              window_size / 2. + knob_size / 2.,
              window_size / 2. + knob_size / 2.,]
knob_bg_coord = [window_size / 2. - window_size / 2.,
                 window_size / 2. - window_size / 2.,
                 window_size / 2. + window_size / 2.,
                 window_size / 2. + window_size / 2.,]

pwm_max = 230

def knob2motor(dx, dy):
    max_dist = (window_size / 2. - knob_size / 2.)**2
    dist = dx**2 + dy**2
    dist = max_dist if dist > max_dist else dist
    dist_ratio = dist / float(max_dist)
    angle = atan2(dy, dx)

    if dx <= 0 and dy >= 0:
        angle = pi - angle
        left_pwm_forward = dist_ratio * angle / pi * 2
        right_pwm_forward = dist_ratio
        left_pwm_reverse = 0
        right_pwm_reverse = 0
    elif dx >= 0 and dy >= 0:
        left_pwm_forward = dist_ratio
        right_pwm_forward = dist_ratio * angle / pi * 2
        left_pwm_reverse = 0
        right_pwm_reverse = 0
    elif dx >= 0 and dy <= 0:
        angle = abs(angle)
        left_pwm_forward = 0
        right_pwm_forward = 0
        left_pwm_reverse = dist_ratio
        right_pwm_reverse = dist_ratio * angle / pi * 2
    elif dx <= 0 and dy <= 0:
        angle = pi - abs(angle)
        left_pwm_forward = 0
        right_pwm_forward = 0
        left_pwm_reverse = dist_ratio * angle / pi * 2
        right_pwm_reverse = dist_ratio

    left_pwm_forward = int(230 * left_pwm_forward)
    left_pwm_reverse = int(230 * left_pwm_reverse)
    right_pwm_forward = int(230 * right_pwm_forward)
    right_pwm_reverse = int(230 * right_pwm_reverse)

    return left_pwm_forward, left_pwm_reverse, \
           right_pwm_forward, right_pwm_reverse

class App:

    active = set()

    def __init__(self, root):
        # ros stuff
        self.pub = rospy.Publisher('propeller_cmd', PropellerCmd, queue_size=100)
        self.msg = PropellerCmd()
        # fixed window size
        # set up GUI
        self.canvas = Canvas(root, width=window_size, height=window_size)
        self.canvas.pack()
        self.canvas.bind('<B1-Motion>', self.mouse_move_callback)
        self.canvas.bind('<B1-ButtonRelease>', self.mouse_up_callback)

        knob_bg = self.canvas.create_oval(*knob_bg_coord)
        self.canvas.itemconfig(knob_bg, fill='white')

        self.knob = self.canvas.create_oval(*knob_coord)
        self.canvas.itemconfig(self.knob, fill='red')

    def mouse_move_callback(self, event):
        new_pos = [event.x - knob_size / 2.,
                   event.y - knob_size / 2.,
                   event.x + knob_size / 2.,
                   event.y + knob_size / 2.,]

        # check if out of knob background
        center = window_size / 2.
        diff_x = event.x - center
        diff_y = center - event.y
        pwm = knob2motor(diff_x, diff_y)

        # send pwm to motor_mcu
        self.msg.left_pwm = pwm[0] - pwm[1]
        self.msg.right_pwm = pwm[2] - pwm[3]
        self.msg.left_enable = 1
        self.msg.right_enable = 1
        self.pub.publish(self.msg)

        self.canvas.coords(self.knob, *new_pos)

    def mouse_up_callback(self, event):
        self.canvas.coords(self.knob, *knob_coord)

        # send 0,0,0,0 to motor_mcu
        self.msg.left_pwm = 0
        self.msg.right_pwm = 0
        self.msg.left_enable = 0
        self.msg.right_enable = 0
        self.pub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node('remote_controller')
    root = Tk()
    app = App(root)
    root.mainloop()
