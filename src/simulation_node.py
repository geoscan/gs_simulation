#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from rospy import Publisher, Service
from gs_interfaces.srv import Live, LiveResponse
from gs_interfaces.srv import Led,LedResponse
from gs_interfaces.srv import Event,EventResponse
from gs_interfaces.srv import NavigationSystem,NavigationSystemResponse
from gs_interfaces.srv import SetNavigationSystem,SetNavigationSystemResponse
from gs_interfaces.srv import Position,PositionResponse
from gs_interfaces.srv import Yaw, YawResponse
from gs_interfaces.srv import ParametersList, ParametersListResponse
from gs_interfaces.srv import SetParametersList, SetParametersListResponse
from gs_interfaces.srv import Wait, WaitResponse
from gs_interfaces.msg import SimpleBatteryState, OptVelocity
from std_msgs.msg import Float32, Int32, ColorRGBA
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import SetBool, SetBoolResponse
from sensor_msgs.msg import CompressedImage
from threading import Thread
from math import degrees, sqrt
from time import sleep

TIME_FOR_RESTART = 5 # приблизительное время необходимое для перезапуска платы

class ROSSimNode(): # класс ноды ros_plaz_node
    def __init__(self, rate = None, start_x = 0.0, start_y = 0.0, start_z = 0.0):
        self.live = False # состояние подключение к базовой платы АП
        self.event_messages = (10, 12, 23, 2) # доступные события(команды) АП
        self.callback_event_messages = (255, 26, 31, 32, 42, 43, 51, 56, 65) # события, возвращаемые АП
        self.state_event = -1 # последнеее событие, отправленное в АП
        self.state_callback_event = -1 # полседнее событие пришедшее от АП
        self.state_position = [0., 0., 0., 0.] # последняя точка, на которую был отправлен коптер (в локальных координатах)  
        self.rate = rate # таймер
        self.camera_status = False

        self.__misssions = []

        self.preflight_state = False
        self.takeoff_state = False

        self.__start_x = start_x
        self.__start_y = start_y
        self.__start_z = start_z

        self.x = start_x
        self.y = start_y
        self.z = start_z
        self.yaw = -90.0

        self.alive = Service("geoscan/alive", Live, self.handle_live) # сервис, показывающий состояние подключения

        self.get_autopilot_params_service = Service("geoscan/board/get_parameters", ParametersList, self.handle_get_autopilot_params) # сервис, возвращающий параметры АП
        self.set_autopilot_params_service = Service("geoscan/board/set_parameters", SetParametersList, self.handle_set_autopilot_params) # сервис, устанавливающий параметры АП
        self.restart_service = Service("geoscan/board/restart", Empty, self.handle_restart) # сервиc перезапуска базововй платы
        
        self.get_navigation_service = Service("geoscan/navigation/get_system", NavigationSystem, self.handle_get_navigation_system) # сервис, возвращающий текущую систему позиционирования
        self.set_navigation_service = Service("geoscan/navigation/set_system", SetNavigationSystem, self.handle_set_navigation_system) # сервис, устанавливающий текущую систему позиционирования

        self.local_position_service = Service("geoscan/flight/set_local_position", Position, self.handle_local_pos) # сервис полета в локальную точку
        self.yaw_service = Service("geoscan/flight/set_yaw", Yaw, self.handle_yaw) # сервис управления рысканьем
        self.event_service = Service("geoscan/flight/set_event", Event, self.handle_event) # севрис управления событиями АП
        self.wait_service = Service("geoscan/flight/set_wait", Wait, self.handle_wait)

        self.module_led_service = Service("geoscan/led/module/set", Led, self.handle_led) # сервис управления светодиодами на LED-модуле

        self.camera_command_service = Service("geoscan/camera/command", SetBool, self.handle_camera_command)

        self.battery_publisher = Publisher("geoscan/battery_state", SimpleBatteryState, queue_size=10) # издатель темы состояния АКБ

        self.local_position_publisher = Publisher("geoscan/navigation/local/position", Point, queue_size=10) # издатель темы позиции в LPS
        self.local_yaw_publisher = Publisher("geoscan/navigation/local/yaw", Float32, queue_size=10) # издаетель темы рысканья в LPS
        self.local_status = Publisher("geoscan/navigation/local/status", Int32, queue_size=10) # издатель темы статуса LPS

        self.opt_velocity_publisher = Publisher("geoscan/navigation/opt/velocity", OptVelocity, queue_size=10) # издатель темы ускорения в OPT

        self.callback_event_publisher = Publisher("geoscan/flight/callback_event", Int32, queue_size=10) # издатель темы событий, возвращаемых АП

        self.led_publisher = Publisher("geoscan/led/module/color", ColorRGBA, queue_size=10)

        self.camera_publisher = Publisher("pioneer_max_camera/image_raw/compressed", CompressedImage, queue_size=10)

    def __preflight(self):
        sleep(0.5)
        self.preflight_state = True
        self.callback_event_publisher.publish(7)

    def __takeoff(self):
        if self.preflight_state and not self.takeoff_state:
            for _ in range(0, 200):
                self.z += 0.01
                sleep(0.05)
            self.takeoff_state = True
            self.callback_event_publisher.publish(6)

    
    def __landing(self, time=0.1):
        if self.takeoff_state:
            for _ in range(int(self.z * 100),0,-1):
                self.z -= 0.01
                sleep(time)
            self.callback_event_publisher.publish(1)
        else:
            self.callback_event_publisher.publish(0)
        self.takeoff_state = False
        self.preflight_state = False

    def __go_to_point(self, x, y, z):
        while len(self.__misssions) > 1:
            pass
        delta_x = x - self.x
        delta_y = y - self.y
        delta_z = z - self.z
        l = sqrt(delta_x**2 + delta_y**2 + delta_z**2)
        for _ in range(0,int(l*100) - 1):
            if len(self.__misssions) <= 1:
                self.x += delta_x / l * 0.01
                self.y += delta_y / l * 0.01
                self.z += delta_z / l * 0.01
                sleep(0.03)
            else:
                self.__misssions.pop(0)
                return
        self.callback_event_publisher.publish(5)
        self.x += x - self.x
        self.y += y - self.y
        self.z += z - self.z
        self.callback_event_publisher.publish(4)
        self.__misssions.pop(0)

    def __update_yaw(self, angle):
        if self.takeoff_state:
            old_angle = int(self.yaw)
            pri = 1
            if angle < 0.0:
                pri = -1
            for new_angle in range(old_angle, old_angle + int(angle / 2), pri):
                self.yaw = new_angle
                sleep(0.03)

    def __wait(self, seconds):
        sleep(seconds)
        self.callback_event_publisher.publish(9)

    def handle_restart(self, request): # функция обработки запроса на перезагрузку
        return EmptyResponse() # возвращаем пустой ответ

    def handle_live(self, request): 
        return LiveResponse(self.live)

    def handle_wait(self, request):
        Thread(target=self.__wait, args=(request.seconds,)).start()
        return WaitResponse(True)

    def handle_event(self, request): # функция обработки запроса на отправление события в АП
        if self.state_event != request.event:
                if request.event == 0:
                    Thread(target=self.__preflight).start()
                elif request.event == 1:
                    Thread(target=self.__takeoff).start()
                elif request.event == 2:
                    Thread(target=self.__landing).start()
                elif request.event == 3:
                    self.x = self.__start_x
                    self.y = self.__start_y
                    self.z = self.__start_z
                    self.takeoff_state = False
                    self.preflight_state = False
                self.state_event = request.event
        return EventResponse(1)

    def handle_local_pos(self, request): # функция обработки запроса на полет в локальную точку
        request_position = [request.position.x, request.position.y, request.position.z] # запоминаем координаты точки из запроса
        if self.takeoff_state: # сравниваем координаты точки с предыдущими координатами
            mission = Thread(target=self.__go_to_point, args = [request_position[0], request_position[1], request_position[2]])
            self.__misssions.append(mission)
            mission.start()
            self.state_position = request_position
        return PositionResponse(True) # возвращаем True - команда выполнена

    def handle_yaw(self, request): # функция обработки запроса на изменение угла рысканья
        Thread(target=self.__update_yaw, args=[degrees(request.angle), ]).start()
        return YawResponse(True) # возвращаем True - команда выполнена

    def handle_led(self, request): # функция обработки запроса на изменение цвета светодиодов на LED-модуле
        self.led_publisher.publish(request.leds[0])
        return LedResponse(True) # возвращаем True - команда выполнена

    def handle_get_navigation_system(self, request): # функция обработки запроса на получение текущей системы навигации
        return NavigationSystemResponse("LPS") # возвращаем имя системы позиционирования
    
    def handle_set_navigation_system(self, request):
        return SetNavigationSystemResponse(True) # возвращаем True - команда выполнена

    def handle_get_autopilot_params(self, request):
        return ParametersListResponse([])

    def handle_set_autopilot_params(self, request):
        return SetParametersListResponse(True) # возвращаем True - команда выполнена

    def handle_camera_command(self, request):
        return SetBoolResponse(True, '')

    def connect(self):
        rospy.loginfo("Try to connect ...")

        self.state_event = -1
        self.state_callback_event = -1

        rospy.loginfo("Board start connect - done")
        self.live = True

    def data_exchange(self):
        if self.live:
            battery_state = SimpleBatteryState()
            battery_state.header.stamp = rospy.Time.now()
            battery_state.charge = 8.33
            self.battery_publisher.publish(battery_state)

            local_point = Point()
            local_point.x = self.x
            local_point.y = self.y
            local_point.z = self.z
            self.local_position_publisher.publish(local_point)

            self.local_yaw_publisher.publish(self.yaw)

            self.local_status.publish(1)

    def spin(self):
        if self.live:
            self.data_exchange()
        else:
            self.connect()
        if self.rate is not None:
            self.rate.sleep()
        return True

if __name__ == "__main__":
    rospy.init_node("ros_plaz_node") # инициализируем ноду

    x = rospy.get_param(rospy.search_param("start_x")) # получение имени порта, как параметра ноды
    if type(x) == dict:
        x = 0.0

    y = rospy.get_param(rospy.search_param("start_y")) # получение скорости обмена данными, как параметра ноды
    if type(y) == dict:
        y = 0.0

    z = rospy.get_param(rospy.search_param("start_z")) # получение скорости обмена данными, как параметра ноды
    if type(z) == dict:
        z = 0.0

    rate = rospy.Rate(100)
    ros_plaz_node = ROSSimNode(rate, x, y, z)

    while not rospy.is_shutdown() and ros_plaz_node.spin():
        pass