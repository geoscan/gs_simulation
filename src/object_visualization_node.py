#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rospkg, json
from visualization_msgs.msg import Marker, MarkerArray
from rospy import Publisher

class ObjectVisualizationNode:
    def __init__(self, path, rate):
        self.config_path = f"{path}/config/objects.json"
        self.rate = rate

        self.__array = []
        self.__marker_array_publisher = Publisher("visualization_marker_array", MarkerArray, queue_size=10)
        self.load()

    def load(self):
        with open(self.config_path, "r") as f:
            objects = json.load(f)
            for i in range(len(objects)):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.id = i + 1
                marker.scale.x = objects[i]["scale"]["x"]
                marker.scale.y = objects[i]["scale"]["y"]
                marker.scale.z = objects[i]["scale"]["z"]
                marker.pose.position.x = objects[i]["position"]["x"]
                marker.pose.position.y = objects[i]["position"]["y"]
                marker.pose.position.z = objects[i]["scale"]["z"] / 2 
                marker.color.r = 0.86
                marker.color.g = 0.86
                marker.color.b = 0.86
                marker.color.a = 1.0
                marker.type = marker.CUBE
                self.__array.append(marker)

    def spin(self):
        self.__marker_array_publisher.publish(self.__array)

        if self.rate is not None:
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("object_visualization_node")

    path = rospkg.RosPack().get_path("gs_simulation")
    rate = rospy.Rate(100)

    object_visualization_node = ObjectVisualizationNode(path, rate)

    while not rospy.is_shutdown():
        object_visualization_node.spin()