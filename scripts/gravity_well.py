#!/usr/bin/env python
# Send projectiles through 3D space populated initially by fixed spherical
# gravity sources.
#
# Later add service for adding the gravity wells, initially just make
# a few randomly.

import math
import numpy as np
import random
import rospy

from visualization_msgs.msg import Marker, MarkerArray


def logn_uniform(low=0.0, high=1.0, size=None, base=np.e):
    return np.power(base, np.random.uniform(low, high, size))

class Sphere:
    def __init__(self, x, y, z, radius, mass):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius
        self.mass = mass

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.action = Marker.ADD
        self.marker.ns = "spheres"
        self.marker.type = Marker.SPHERE
        # self.marker.id
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = radius
        self.marker.scale.y = radius
        self.marker.scale.z = radius
        self.marker.color.r = random.uniform(0.0, 1.0)
        self.marker.color.g = random.uniform(0.0, 1.0)
        self.marker.color.b = random.uniform(0.0, 1.0)
        self.marker.color.a = 1.0
        self.marker.frame_locked = True

    def get_marker(self):
        return self.marker

class GravityWell:
    def __init__(self):
        self.update_dt = 0.05

        min_x = rospy.get_param("~min_x", -5.0)
        max_x = rospy.get_param("~max_x", 5.0)
        min_radius = rospy.get_param("~min_radius", 0.01)
        max_radius = rospy.get_param("~max_radius", 1.6)

        self.marker_pub = rospy.Publisher("marker_array", MarkerArray, queue_size=10)

        self.spheres = []
        for i in range(10):
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_x, max_x)
            z = random.uniform(min_x, max_x)
            radius = logn_uniform(min_radius, max_radius)
            mass = 4.0/3.0 * math.pi * radius**3
            sphere = Sphere(x, y, z, radius, mass)
            sphere.marker.id = i
            self.spheres.append(sphere)

        self.marker_array = MarkerArray()
        for sphere in self.spheres:
            self.marker_array.markers.append(sphere.get_marker())

        self.count = 0
        self.update_timer = rospy.Timer(rospy.Duration(self.update_dt), self.update)

    def pub_spheres(self):
        self.marker_pub.publish(self.marker_array)

    def update(self, event):
        if self.count % 20 == 0:
            self.pub_spheres()
        self.count += 1

if __name__ == '__main__':
    rospy.init_node('gravity_well')
    gravity_well = GravityWell()
    rospy.spin()
