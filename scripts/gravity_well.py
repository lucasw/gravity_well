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

from geometry_msgs.msg import Point
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
        self.marker.pose.position.x = self.x
        self.marker.pose.position.y = self.y
        self.marker.pose.position.z = self.z
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = radius * 2.0
        self.marker.scale.y = radius * 2.0
        self.marker.scale.z = radius * 2.0
        self.marker.color.r = random.uniform(0.0, 1.0)
        self.marker.color.g = random.uniform(0.0, 1.0)
        self.marker.color.b = random.uniform(0.0, 1.0)
        self.marker.color.a = 1.0
        self.marker.frame_locked = True

    def get_marker(self):
        return self.marker

class Projectile:
    def __init__(self, x, y, z, radius, mass, vx, vy, vz):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius
        self.mass = mass
        self.enabled = True

        self.vx = vx
        self.vy = vy
        self.vz = vz

        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

        self.markers = []

        marker = Marker()
        marker.header.frame_id = "map"
        marker.action = Marker.ADD
        marker.ns = "projectiles_traj"
        marker.type = Marker.LINE_STRIP
        # marker.id
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        sc = 0.05
        marker.scale.x = sc
        marker.scale.y = sc
        marker.scale.z = sc
        marker.color.r = random.uniform(0.0, 1.0)
        marker.color.g = random.uniform(0.0, 1.0)
        marker.color.b = random.uniform(0.0, 1.0)
        marker.color.a = 1.0
        marker.frame_locked = True
        self.markers.append(marker)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.action = Marker.ADD
        marker.ns = "projectiles"
        marker.type = Marker.SPHERE
        # marker.id
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = radius * 2.0
        marker.color.r = random.uniform(0.0, 1.0)
        marker.color.g = random.uniform(0.0, 1.0)
        marker.color.b = random.uniform(0.0, 1.0)
        marker.color.a = 1.0
        marker.frame_locked = True
        self.markers.append(marker)

    def get_markers(self):
        self.markers[1].pose.position.x = self.x
        self.markers[1].pose.position.y = self.y
        self.markers[1].pose.position.z = self.z
        return self.markers

    def add_force(self, fx, fy, fz):
        if not self.enabled:
            return
        self.ax += fx / self.mass
        self.ay += fy / self.mass
        self.az += fz / self.mass

    def update(self, dt):
        if not self.enabled:
            return
        self.vx += self.ax * dt
        self.vy += self.ay * dt
        self.vz += self.az * dt

        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        pt = Point()
        pt.x = self.x
        pt.y = self.y
        pt.z = self.z
        self.markers[0].points.append(pt)

        rospy.logdebug('{} pos {:0.4f} {:0.4f} {:0.4f}, '.format(self.markers[0].id, self.x,
                                                                 self.y, self.z))
        rospy.logdebug('{} vel {:0.4f} {:0.4f} {:0.4f}, '.format(self.markers[0].id, self.vx,
                                                                 self.vy, self.vz))
        rospy.logdebug('{} acc {:0.4f} {:0.4f} {:0.4f}'.format(self.markers[0].id, self.ax, self.ay, self.az))
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

class GravityWell:
    def __init__(self):
        self.update_dt = 0.05

        max_x = rospy.get_param("~max_x", 5.0)
        min_x = rospy.get_param("~min_x", -5.0)
        min_radius = rospy.get_param("~min_radius", 0.01)
        max_radius = rospy.get_param("~max_radius", 0.1)

        self.marker_pub = rospy.Publisher("marker_array", MarkerArray, queue_size=10)

        self.spheres = []
        for i in range(10):
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_x, max_x)
            z = random.uniform(min_x, max_x)
            radius = logn_uniform(min_radius, max_radius, base=2.0)
            mass = 2.0 * 4.0/3.0 * math.pi * radius**3
            sphere = Sphere(x, y, z, radius, mass)
            sphere.marker.id = i
            self.spheres.append(sphere)

        self.projectiles = []
        for i in range(5):
            x = random.uniform(min_x * 1.5, max_x * 1.5)
            y = random.uniform(min_x * 1.5, max_x * 1.5)
            z = random.uniform(min_x * 1.5, max_x * 1.5)
            max_v = 1.0
            vx = random.uniform(-max_v, max_v)
            vy = random.uniform(-max_v, max_v)
            vz = random.uniform(-max_v, max_v)
            radius = 0.2
            mass = 4.0/3.0 * math.pi * radius**3
            projectile = Projectile(x, y, z, radius, mass, vx, vy, vz)
            for marker in projectile.markers:
                marker.id = i
                marker.color.r *= 0.5
                marker.color.g = 1.0
                marker.color.b *= 0.5
            projectile.markers[0].color.a = 0.4
            self.projectiles.append(projectile)

        self.marker_array = MarkerArray()
        for sphere in self.spheres:
            self.marker_array.markers.append(sphere.get_marker())

        self.count = 0
        self.update_timer = rospy.Timer(rospy.Duration(self.update_dt), self.update)

    def pub_spheres(self):
        self.marker_pub.publish(self.marker_array)

    def pub_projectiles(self):
        marker_array = MarkerArray()
        for projectile in self.projectiles:
            if projectile.enabled and self.count % 60 == 0:
                projectile.markers[0].points = projectile.markers[0].points[1::2]
            marker_array.markers.extend(projectile.get_markers())
        self.marker_pub.publish(marker_array)

    def update(self, event):
        if event.last_expected is None:
            return
        gm = 1.0
        for projectile in self.projectiles:
            if not projectile.enabled:
                continue
            for sphere in self.spheres:
                dx = sphere.x - projectile.x
                dy = sphere.y - projectile.y
                dz = sphere.z - projectile.z
                r2 = (dx * dx + dy * dy + dz * dz)
                if r2 == 0.0:
                    continue
                if r2 < sphere.radius * sphere.radius:
                    projectile.vx = 0.0
                    projectile.vy = 0.0
                    projectile.vz = 0.0
                    projectile.enabled = False
                    rospy.logwarn("disabling projectile {} {}".format(r2, sphere.radius))
                    continue
                force = gm * (sphere.mass * projectile.mass) / r2
                r = math.sqrt(r2)
                # rospy.loginfo("r {:0.2f}, force {:0.6f}".format(r, force))
                dxn = dx / r
                dyn = dy / r
                dzn = dz / r
                projectile.add_force(dxn * force, dyn * force, dzn * force)

            projectile.update((event.current_expected - event.last_expected).to_sec())

        self.pub_projectiles()

        if self.count % 20 == 0:
            self.pub_spheres()
        self.count += 1

if __name__ == '__main__':
    rospy.init_node('gravity_well')
    gravity_well = GravityWell()
    rospy.spin()
