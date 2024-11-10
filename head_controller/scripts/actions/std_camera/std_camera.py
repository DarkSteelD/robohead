# Пример создан для отладки, проверяет работу камеры, тачскрина и датчика напряжения
import os
import time
import cv2 # Библиотека OpenCV
from cv_bridge import CvBridge # Конвертация изображения ROS<->OpenCV
import rospy # Библиотека для работы с ROS
from sensor_msgs.msg import Image
from mors_driver.srv import TwistDuration, TwistDurationRequest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
import evdev, select
from ultralytics import YOLO

class STD_CAMERA():
    def __init__(self, srv_display_player=None, srv_set_neck=None, srv_set_ears=None,
                srv_play_sound=None, srv_mors_mode=None, srv_mors_action=None,
                srv_mors_cmd_vel=None, srv_mors_cmd_pos=None, srv_mors_ef_pos=None,
                srv_mors_joint_pos=None, srv_mors_joints_kp=None, srv_mors_joints_kd=None,
                srv_mors_stride_height=None, sound_direction=270):

        self.srv_display_player = srv_display_player
        self.srv_set_neck = srv_set_neck
        self.srv_set_ears = srv_set_ears
        self.srv_play_sound = srv_play_sound

        self.srv_mors_mode=srv_mors_mode
        self.srv_mors_action=srv_mors_action
        self.srv_mors_cmd_vel=srv_mors_cmd_vel
        self.srv_mors_cmd_pos=srv_mors_cmd_pos
        self.srv_mors_ef_pos=srv_mors_ef_pos
        self.srv_mors_joint_pos=srv_mors_joint_pos
        self.srv_mors_joints_kp=srv_mors_joints_kp
        self.srv_mors_joints_kd=srv_mors_joints_kd
        self.srv_mors_stride_height=srv_mors_stride_height
        self.sound_direction=sound_direction
        self.model = YOLO("yolov5s.pt")  
        self._script_path = os.path.dirname(os.path.abspath(__file__))
        
    def start_action(self)->int:
        
        # Добавить остановку текущего аудио, и его запоминание

        if self.srv_display_player!=None:
            self.srv_display_player('__BLANK__') # Останавливаем воспроизведение любого видео

        if self.srv_set_neck!=None:
            neck_angle_v = 15 # Change it
            neck_angle_h = 0 # Change it
            duration=1 # Change it
            self.srv_set_neck(neck_angle_v, neck_angle_h, duration)

        if self.srv_set_ears!=None:
            ear_angle_l = 10 # Change it
            ear_angle_r = 10 # Change it
            self.srv_set_ears(ear_angle_l, ear_angle_r)
        self.cvBridge = CvBridge()
        image_sub = rospy.Subscriber("cv_camera/image_raw", Image, callback=self.img_proc) # Подписчик на топик потока с камеры
        self.image_pub = rospy.Publisher("head/display/raw_image", Image, queue_size=1)
        bat_sub = rospy.Subscriber("/head/bat", BatteryState, self.__battery_state_callback)


        for i in [0,1]:
            touch = evdev.InputDevice(f"/dev/input/event{i}")
            print(touch.name, f"/dev/input/event{i}")
            if ("waveshare" in str(touch.name).lower()):
                break

        touch.grab()
        
        start_time = time.time()
        self.p = (0,0)
        flag = 0
        self.battery = '0'
        while ((time.time()-start_time)<=5):
            r,w,x = select.select([touch], [], [])
            for event in touch.read():
                if event.type == evdev.ecodes.EV_ABS:
                    if event.code == 0:
                        X = event.value
                    elif event.code == 1:
                        Y = event.value
                        flag = 1
                if flag==1:
                    flag = 0
                    self.p = self.getPixelsFromCoordinates((X, Y))
                    # lcd.fill((50, 50, 50))
                    # pygame.draw.circle(lcd, (255, 255, 255), p, 50, 10)
                    # refresh()
        image_sub.unregister()
        self.image_pub.unregister()
        bat_sub.unregister()


    def getPixelsFromCoordinates(self, coords):
        x = coords[0]/4095*1079
        y = coords[1]/4095*1079

        return (1079-int(y), int(x))

    def __battery_state_callback(self, msg:BatteryState):
        self.battery = "{:.3f}|{:.3f}".format(msg.voltage, msg.current)

    def img_proc(self, image_msg:Image):
        cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv_image = cv2.resize(cv_image, (1080, 1080))
        results = self.model(cv_image)  # Выполнение распознавания
        for result in results:
            for obj in result.boxes:
                x1, y1, x2, y2 = map(int, obj.xyxy[0])
                label = obj.cls  # Имя класса объекта
                conf = obj.conf  # Уверенность
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, f"{label} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.circle(cv_image, self.p, 200, (255,255,255), -1)
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (self.p[0]-70, self.p[1]-70)
        fontScale              = 1
        fontColor              = (0,0,0)
        thickness              = 2
        lineType               = 2
        cv2.putText(cv_image, self.battery,
            bottomLeftCornerOfText,
            font,
            fontScale,
            fontColor,
            thickness,
            lineType)
        self.image_pub.publish(self.cvBridge.cv2_to_imgmsg(cv_image, encoding="bgr8")) 
        return 0