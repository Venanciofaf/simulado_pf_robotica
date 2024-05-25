import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from my_package.odom import Odom
from my_package.laser import Laser
from sensor_msgs.msg import CompressedImage
from my_package.goto import GoTo
from my_package.rotate2 import Rotate2
import cv2
import numpy as np 

import time
from cv_bridge import CvBridge
# Adicione aqui os imports necessários

class Seguidores(Node , Odom , Laser): # Mude o nome da classe

    def __init__(self):
        Node.__init__(self , 'circuito_node') # Mude o nome do nó
        Odom.__init__(self)
        Laser.__init__(self)
        self.subcomp = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        time.sleep(5)

        self.timer = self.create_timer(0.1, self.control)
        self.bridge = CvBridge()
        self.robot_state = 'rotaciona'
        self.state_machine = {
            'stop': self.stop,
            'segue_linha': self.segue_linha,
            'goto': self.goto,
            'rotaciona': self.rotaciona
        }

        # Inicialização de variáveis
        self.GOTO = 0
        self.rotate2 = Rotate2(90 , True)

        self.twist = Twist()
        self.kp_ang = 0.007
        self.ordem_cores = ['vermelho' , 'verde' , 'azul']
        self.start = (0, 0)
        self.i = 0
        self.j = 0
        self.cor_desejada = self.ordem_cores[self.i]
        self.comecou = False
        self.cores = {
    'vermelho' : [np.array([0, 55, 55]) , np.array([8, 255, 255])],
    'verde' : [np.array([50, 55, 55]) , np.array([75, 255, 255])],
    'azul' : [np.array([105, 55, 55]) , np.array([135, 255, 255])],
        }
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

    #Processamentos de Subscribers
    def image_callback(self , msg):
        
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.copia_camera = self.cv_image.copy()

        self.mascara_camera = self.faz_mascara(self.copia_camera , self.ordem_cores[self.i])
        
        self.imagem_redefinida = self.redefine_dimensão(self.copia_camera)

        self.h , self.w , _ = self.cv_image.shape        
        self.w = self.w/2

        
        self.acha_cm(self.mascara_camera)


        if self.i < 2:
            self.mask_prox = self.faz_mascara(self.copia_camera , self.ordem_cores[self.i+1])
            area = self.acha_area(self.mask_prox)
            print('A área do próximo é de: ' , area)
            print('O j é : ' , self.j)
            cv2.imshow('Mascara do Proximo'  , self.mask_prox)
            if area > 10000:
                self.j += 1
        
            if self.j > 11:
                self.i += 1 
                self.j = 0
        else:
            self.mask_prox = self.faz_mascara(self.copia_camera , self.ordem_cores[self.i-1])
            area = self.acha_area(self.mask_prox)
            print('A área do próximo é de: ' , area)
            print('O j é : ' , self.j)
            cv2.imshow('Mascara do Proximo'  , self.mask_prox)
            if area > 10000:
                self.j += 1
        cv2.imshow('Mask_Camera' , self.mascara_camera)
        cv2.imshow('Image', self.cv_image)
        cv2.waitKey(1)
    
    #Funções Auxiliares 
    def faz_mascara(self , imagem , cor_desejada): 
        
        img_copy = cv2.cvtColor(imagem , cv2.COLOR_BGR2HSV)

        kernel = np.ones((10,10), np.uint8)

        limite_inferior = self.cores[cor_desejada][0]
        limite_superior = self.cores[cor_desejada][1]

        mascara = cv2.inRange(img_copy , limite_inferior , limite_superior)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, kernel)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)

        return mascara

    def acha_area(self , mask):
        area = 0     
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)
        return area
    

    def redefine_dimensão(self , imagem):
        height = self.cv_image.shape[0]
        imagem[:int(height/2)] = 0
        
        return imagem
        
    def acha_cm(self , mascara):

        contours, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(self.cv_image, contour, -1, [255, 0, 0], 3)

            M = cv2.moments(contour)
            self.cx = int(M["m10"] / M["m00"])
            self.cy = int(M["m01"] / M["m00"])
            cv2.circle(self.cv_image, (self.cx, self.cy), 5, (0, 0, 255), -1)
        else:
            self.cx = -1
            self.cy = -1

    def calcula_erro(self):

        self.erro = self.w - self.cx   
    

    #Estados
    def segue_linha(self): 
        if not(self.comecou):
            self.x0 = self.x
            self.y0 = self.y
            self.start = (self.x0 , self.y0)
            self.comecou = True
            self.GOTO = GoTo(self.start)

        if self.cx == -1:
            self.twist.angular.z = -0.5
            self.twist.linear.x = 0.
        else: 

            self.calcula_erro()
            self.twist.angular.z = self.erro * self.kp_ang
            self.twist.linear.x = 0.4
        
        if self.i == 2 and self.j > 20:
            self.robot_state = 'goto'
    
    def rotaciona(self):
        rclpy.spin_once(self.rotate2)
        self.twist = self.rotate2.twist

        if self.rotate2.robot_state == 'para':
            self.robot_state = 'segue_linha'
    def goto(self): 
        
        rclpy.spin_once(self.GOTO)
        self.twist = self.GOTO.twist

        if self.GOTO.robot_state == 'stop':
            self.robot_state = 'stop'

    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        if self.comecou:
            print('O Start é:' , self.start)
        print(f'Estado Atual: {self.robot_state}')
        
        
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)
    
def main(args=None):
    rclpy.init(args=args)
    ros_node = Seguidores() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()