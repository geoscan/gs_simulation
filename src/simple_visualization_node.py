#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rospkg
from rospy import Subscriber, Publisher
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, ColorRGBA
from visualization_msgs.msg import Marker
from math import radians, cos, sin

class SimpleVisualizationNode(): # класс ноды визуализации
    def __init__(self, path, rate = None):
        self.rate = rate
        self.path = path

        self.pioneer_marker_publisher = Publisher("visualization_marker", Marker, queue_size=10) # создаем издатель темы с маркерами для отображения, название темы задается в rviz

        self.copter_marker = Marker() # создаем объект класса маркеров
        self.copter_marker.header.frame_id = "map" # указываем имя протсранства для отображения, задается в rviz - в конкретно этом случае map
        self.copter_marker.id = 0 # заполянем айди маркера
        self.copter_marker.scale.x = 0.5 # устанавливаем размер по координате X
        self.copter_marker.scale.y = 0.1 # устанавливаем размер по координате Y
        self.copter_marker.scale.z = 0.1 # устанавливаем размер по координате Z
        self.copter_marker.pose.orientation.x = cos(radians(90)) # устанавливаем направление маркера по координате X
        self.copter_marker.pose.orientation.y = sin(radians(-90)) # устанавливаем направление маркера по координате Y
        self.copter_marker.color.r = 0.52 # устанавлием красный канал цвета маркера
        self.copter_marker.color.g = 0.52 # устанавлием зеленый канал цвета маркера
        self.copter_marker.color.b = 0.52 # устанавлием синий канал цвета маркера
        self.copter_marker.color.a = 1.0 # устанавливаем прозрачность маркера, 1 - не прозрачный
        self.copter_marker.type = self.copter_marker.MESH_RESOURCE # задаем тип маркера, MESH_RESOURCE - маркер ввиде 3D модели
        self.copter_marker.mesh_resource = f"file://{self.path}/model/drone.stl" # задаем файл 3D модели
        
        self.position_subscriber = Subscriber("geoscan/navigation/local/position", Point, self.__position_callback)
        self.yaw_subscriber = Subscriber("geoscan/navigation/local/yaw", Float32, self.__yaw_callback)
        self.color_subscriber = Subscriber("simulation/color", ColorRGBA, self.__color_callback)

    def __position_callback(self, position): # функция обработки новых сообщений топика позиции пионера в LPS
        self.copter_marker.pose.position.x = position.x # запоминаем координату X
        self.copter_marker.pose.position.y = position.y # запоминаем координату Y
        self.copter_marker.pose.position.z = position.z # запоминаем координату Z

    def __yaw_callback(self, angle):
        self.copter_marker.pose.orientation.x = cos(radians(angle.data))
        self.copter_marker.pose.orientation.y = sin(radians(angle.data))

    def __color_callback(self, color):
        color.a = 1.0
        self.copter_marker.color = color

    def spin(self):
        self.copter_marker.header.seq += 1
        self.copter_marker.header.stamp = rospy.Time.now()
        self.pioneer_marker_publisher.publish(self.copter_marker)

        if self.rate is not None:
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("simple_visualization_node") # инициализируем ноду визуализации

    gs_simulation_path = rospkg.RosPack().get_path("gs_simulation")

    rate = rospy.Rate(100)
    visualization_node = SimpleVisualizationNode(gs_simulation_path, rate) # создаем объект класса ноды визуализации

    while not rospy.is_shutdown(): # создаем бесконечный цикл
        visualization_node.spin()
