# ------------------------------------------------------------
# Файл содержит пример кода управления ходьбой
# шагающего робота МОРС с Робо-Головы
# Внимание! При включении этого примера, убедитесь, что спереди
#           у робота есть 3 метра свободного пространства
# ------------------------------------------------------------

import rospy

from mors.srv import QuadrupedCmd, QuadrupedCmdResponse
from mors_driver.srv import TwistDuration, TwistDurationRequest
import time

# call robot_mode service
def set_mode_client(mode:int) -> QuadrupedCmdResponse:
    rospy.wait_for_service('SetMorsMode')
    try:
        set_mode = rospy.ServiceProxy('SetMorsMode', QuadrupedCmd)
        resp = set_mode(mode)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# call robot_action service
def set_action_client(action:int) -> QuadrupedCmdResponse:
    rospy.wait_for_service('SetMorsAction')
    try:
        set_action = rospy.ServiceProxy('SetMorsAction', QuadrupedCmd)
        resp = set_action(action)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def set_stride_height_client(height:float) -> QuadrupedCmdResponse:
    rospy.wait_for_service('SetMorsStrideHeight')
    try:
        set_height = rospy.ServiceProxy('SetMorsStrideHeight', QuadrupedCmd)
        resp = set_height(height)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    # инициализация ROS
    rospy.init_node("example_mors_driver_walk_node")
    srv_goal_vel = rospy.ServiceProxy('SetMorsVel', TwistDuration)

    # встаем
    set_action_client(1)

    # устанавливаем режим ходьбы
    set_mode_client(0)

    # устанавливаем высоту шага 4 см
    set_stride_height_client(0.04)

    request = TwistDurationRequest()

    # Идем вперед со скоростью 0.4 м/сек: разгоняемся до этой скорости за 1 сек, шагаем 2 сек
    request.data.linear.x = 0.4
    request.data.linear.y = 0
    request.data.angular.z = 0
    request.duration = 1
    srv_goal_vel(request)
    rospy.sleep(3) # Ждем время разгона + время шагания

    # Плавно останавливаемся
    request.data.linear.x = 0
    request.data.linear.y = 0
    request.data.angular.z = 0
    request.duration = 1
    srv_goal_vel(request)
    rospy.sleep(1) # Ждем время торможения

    # Идем назад со скоростью 0.4 м/сек: разгоняемся до этой скорости за 1 сек, шагаем 2 сек
    request.data.linear.x = -0.4
    request.data.linear.y = 0
    request.data.angular.z = 0
    request.duration = 1
    srv_goal_vel(request)
    rospy.sleep(3) # Ждем время разгона + время шагания

    # Плавно останавливаемся
    request.data.linear.x = 0
    request.data.linear.y = 0
    request.data.angular.z = 0
    request.duration = 1
    srv_goal_vel(request)
    rospy.sleep(1) # Ждем время торможения

    # # Идем влево со скоростью 0.2 м/сек: разгоняемся до этой скорости за 1.5 сек, шагаем 3 сек
    # request.data.linear.x = 0
    # request.data.linear.y = 0.2
    # request.data.angular.z = 0
    # request.duration = 1.5
    # srv_goal_vel(request)
    # rospy.sleep(4.5) # Ждем время разгона + время шагания

    # # Плавно останавливаемся
    # request.data.linear.x = 0
    # request.data.linear.y = 0
    # request.data.angular.z = 0
    # request.duration = 1
    # srv_goal_vel(request)
    # rospy.sleep(1) # Ждем время торможения
    
    # # Поворачиваем налево с радиусом поворота 0.7 метра: разгон 1.5 с, шагание 3 с
    # request.data.linear.x = 0.3
    # request.data.linear.y = 0.0
    # request.data.angular.z = 0.3
    # request.duration = 1.5
    # srv_goal_vel(request)
    # rospy.sleep(4.5) # Ждем время разгона + время шагания

    # # Плавно останавливаемся
    # request.data.linear.x = 0
    # request.data.linear.y = 0
    # request.data.angular.z = 0
    # request.duration = 1
    # srv_goal_vel(request)
    # rospy.sleep(1) # Ждем время торможения

    set_action_client(2)