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
import requests
import numpy as np
from std_msgs.msg import Int32

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
        rospy.logwarn("Got image")
        cv_image = cv2.resize(cv_image, (1080, 1080))
        _, encoded_image = cv2.imencode('.jpg', cv_image)
        rospy.logerr("Got image")
        # Convert the encoded image to bytes
        image_bytes = encoded_image.tobytes()
        self.srv_mors_action(1)
        # Make the POST request to your server
        url = "http://192.168.1.128:5000/upload"
        try:
            response = requests.post(url, files={"image": ("image.jpg", image_bytes, "image/jpeg")}, timeout=5)
            if response.status_code == 200:
                data = response.json()
                direction = data.get('direction', 1)  # По умолчанию назад
                rospy.loginfo(f"Received direction: {direction}")
                # Публикуем направление в топик /walk_direction
                status_pub = rospy.Publisher("/head/status", true, queue_size=10)
                rospy.Subscriber("/walk_direction", Int32(direction), self.walk_direction_callback)
            else:
                rospy.logerr(f"Server returned status code {response.status_code}")
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Failed to send image: {e}")

        rospy.loginfo("Sent image", response)
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
        
        time.sleep(10)
        return 0