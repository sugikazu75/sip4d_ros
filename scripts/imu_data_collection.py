#!/usr/bin/python3

import rospy
from spinal.msg import Imu
from std_srvs.srv import Trigger, TriggerResponse
import os, json, threading, collections, datetime
from roslib.message import get_message_class


class SlidingWindowBuffer:
    def __init__(self, robot_ns=""):
        # params
        self.topic = rospy.get_param("~topic", robot_ns + "/imu")
        self.msg_type = rospy.get_param("~type", "spinal/Imu")
        self.interval_sec = float(rospy.get_param("~interval_sec", 0.05))
        self.window_sec = float(rospy.get_param("~window_sec", 60.0))
        self.maxlen_hint = int(rospy.get_param("~maxlen", 2000))  # 20Hz * 60s

        # buffer
        self.buf = collections.deque(maxlen=self.maxlen_hint)
        self.lock = threading.Lock()
        self.last_time = rospy.Time.now().to_sec()

        cls = get_message_class(self.msg_type)
        if cls is None:
            raise RuntimeError("Unknown message type: %s" % self.msg_type)
        self.sub = rospy.Subscriber(self.topic, cls, self.imu_callback, queue_size=100)

        self.gc = rospy.Timer(rospy.Duration(1.0), self.garbage_collection)

    def imu_callback(self, msg):
        imu_acc_x = msg.acc[0]
        imu_acc_y = msg.acc[1]
        imu_acc_z = msg.acc[2]
        if rospy.Time.now().to_sec() - self.last_time < self.interval_sec:
            return
        with self.lock:
            self.buf.append(
                (rospy.Time.now().to_sec(), imu_acc_x, imu_acc_y, imu_acc_z)
            )
        self.last_time = rospy.Time.now().to_sec()

    def garbage_collection(self, event):
        with self.lock:
            now = rospy.Time.now().to_sec()
            while self.buf and (now - self.buf[0][0]) > self.window_sec:
                self.buf.popleft()

    def snapshot(self):
        cutoff = rospy.Time.now().to_sec() - self.window_sec
        with self.lock:
            data = [(t, x, y, z) for (t, x, y, z) in self.buf if t >= cutoff]
        return data

    def get_data(self):
        data = self.snapshot()
        return data


if __name__ == "__main__":
    rospy.init_node("imu_data_collection", anonymous=False)
    SlidingWindowBuffer()
    rospy.spin()
