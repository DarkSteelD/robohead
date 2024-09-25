# ------------------------------------------------------------
# Файл содержит пример кода для вызова скриптов действий  
# на шагающем роботе МОРС с Робо-Головы
# Внимание! При включении этого примера, убедитесь, что слева 
#           от робота есть 1.5 м свободного пространства
# ------------------------------------------------------------

import rospy
from mors.srv import QuadrupedCmd, QuadrupedCmdResponse

# call robot_action service
def set_action_client(action:int) -> QuadrupedCmdResponse:
    rospy.wait_for_service('SetMorsAction')
    try:
        set_action = rospy.ServiceProxy('SetMorsAction', QuadrupedCmd)
        resp = set_action(action)
        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    # инициализируем ноду
    rospy.init_node("example_mors_driver_actions_node")

    rospy.loginfo("Demo Actions: Start")
    # Встать
    set_action_client(1)
    # Дай лапу
    set_action_client(3)
    # При повторном вызове робот дает другую лапу
    set_action_client(3)
    # Кувырок
    set_action_client(4)
    # Помахай лапой
    set_action_client(5)
    # Сидеть
    set_action_client(6)
    # Лечь на пол и снять питание с двигателей
    set_action_client(2)

    rospy.loginfo("Demo Actions: Finish")
