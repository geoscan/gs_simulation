#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rospkg
from math import sqrt
from rospy import Publisher, Service, ServiceProxy
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Empty, EmptyResponse
from threading import Thread
from copy import deepcopy

TIME = 30.0

class CarSimulationNode:
    def __init__(self, path, rate = None):

        self.rate = rate
        self.path = []
        self.run = False
        self.__point_index = 1
        self.__start_time = None
        with open(path + "/line.txt", "r") as f:
            for line in f:
                x, y = line.replace("\n", "").split(" ")
                x, y = float(x), float(y)
                self.path.append(Point(x, y, 0.0))

        self.position = deepcopy(self.path[0])
        
        self.__run_service = Service("run", SetBool, self.handle_run)
        self.__continue_service = Service("continue", Empty, self.handle_continue)

        rospy.wait_for_service("pause")
        self.__pause_service_proxy = ServiceProxy("pause", Empty)
        self.__position_publisher = Publisher("position", Point, queue_size=10)
        self.timer = Thread(target=self.timer_targer)
        self.timer.start()
        
    def handle_run(self, request):
        self.run = request.data
        if self.run:
            self.__start_time = rospy.Time.now().to_sec()
        return SetBoolResponse(True, "")

    def handle_continue(self, request):
        self.run = True
        self.__start_time = rospy.Time.now().to_sec()
        return EmptyResponse()

    def timer_targer(self):
        old_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            new_time = rospy.Time.now().to_sec()
            if abs(old_time - new_time) > 0.1:
                self.__position_publisher.publish(self.position)

            if self.__start_time is not None:
                if abs(self.__start_time - new_time) > TIME:
                    self.run = False
                    self.__pause_service_proxy()
                    self.__start_time = None

    def spin(self):
        delta_x = self.path[self.__point_index].x - self.position.x
        delta_y = self.path[self.__point_index].y - self.position.y
        l = sqrt(delta_x**2 + delta_y**2)
        for _ in range(0,int(l*100) - 1):
            while not rospy.is_shutdown() and not self.run:
                pass
            self.position.x += delta_x / l * 0.01
            self.position.y += delta_y / l * 0.01
            rospy.sleep(0.03)

        self.position.x += self.path[self.__point_index].x - self.position.x
        self.position.y += self.path[self.__point_index].y - self.position.y
        self.__point_index += 1
        if self.__point_index >= len(self.path):
            self.__point_index = 0

        if self.rate is not None:
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("car_simulation_node")

    path = rospkg.RosPack().get_path("gs_simulation") + "/path"
    rate = rospy.Rate(100)
    car_simulation_node = CarSimulationNode(path, rate)
    while not rospy.is_shutdown():
        car_simulation_node.spin()