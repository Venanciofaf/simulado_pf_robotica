import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from my_package.odom import Odom
from my_package.laser import Laser
from sensor_msgs.msg import CompressedImage
import cv2
import random
import numpy as np 
from random import randint
import time
from cv_bridge import CvBridge
# Adicione aqui os imports necessários

class Aleatorio(Node , Odom , Laser): # Mude o nome da classe

    def __init__(self):
        Node.__init__(self , 'aleatorio_node')
        Odom.__init__(self) 
        Laser.__init__(self)
        self.timer = self.create_timer(0.1, self.control)

        self.robot_state = 'peao'
        self.state_machine = {
            'stop': self.stop,
            'peao': self.peao,
            'andador':self.andador,
            'center': self.center,
            'goto': self.goto
        }
        # Subscribers
        ## Coloque  aqui os subscribers
        self.bridge = CvBridge()
        self.subcomp = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        time.sleep(5)
        # Inicialização de variáveis
        self.twist = Twist()
        self.inicio = self.get_clock().now().to_msg().sec
        self.sorteado = random.randint( 0 , 1)
        self.kp_ang = 0.003
        self.kp_linear = 0.2
        self.kp_angular = 0.5
        self.centralizou = False
        self.comecou = False

        self.controle = 0
        self.cores = {
            'mangeta': [np.array([135, 55, 55]) , np.array([165, 255, 255])],
            'amarelo' : [np.array([15, 55, 55]) , np.array([35, 255, 255])]
        }
        

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

    #Subscribers
    def image_callback(self , msg):
        
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.copia_camera = self.cv_image.copy()
        
        self.mask_mangeta = self.faz_mascara(self.copia_camera , 'mangeta')
        self.mask_amerelo = self.faz_mascara(self.copia_camera , 'amarelo')


        self.h , self.w , _ = self.cv_image.shape
        width = self.w        
        self.w = self.w/2

        
        if self.centralizou:
            self.mask_maior = self.faz_mascara(self.copia_camera , self.cor_desejada)
            #self.mask_maior[:, :int(width/3)] = 0
            #self.mask_maior[:, :int(2*width/3)] = 0
            self.acha_cm(self.mask_maior)
            cv2.imshow('Mascara_prepoderante', self.mask_maior)
        
                

        cv2.imshow('Image', self.cv_image)
        #cv2.imshow('Mascara Amarelo', self.mask_amerelo)
        #cv2.imshow('Mascara Mangeta', self.mask_mangeta)
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
    
    def define_prevalente(self , mask1 , mask2 , nome_cor1 , nome_cor2):

        contours1, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours2, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        maior1 = 0
        maior2 = 0 
        area1 = 0 
        area2 = 0

        if len(contours1) > 0:
            maior1  = max(contours1 , key=cv2.contourArea)
            area1 = cv2.contourArea(maior1)
        if len(contours2) > 0:
            maior2 = max(contours2 , key=cv2.contourArea)
            area2 = cv2.contourArea(maior2)
        
        if area1 > area2:
            return nome_cor1
        elif area2 > area1:
            return nome_cor2
        elif area2 == area1:     
            
            cor_usada = random.randint(0 , 1)
            if cor_usada == 0:
                return nome_cor1
            else:
                return nome_cor2
            
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

    def error(self):
        if self.cx == -1:
            self.erro = 100
        else:
            self.erro = self.w - self.cx   

    def get_angular_erro(self , point):

        #Calculo da distancia 

        self.err_x = point[0] - self.x
        self.err_y = point[1] - self.y
        self.dist = np.sqrt(self.err_x ** 2 + self.err_y ** 2)


        #Calculo do Angulo Desejado 

        self.theta = np.arctan2(point[1]-self.y , point[0]-self.x)
        self.ang_erro1 = self.theta - self.yaw
        self.ang_erro1 = np.arctan2(np.sin(self.ang_erro1), np.cos(self.ang_erro1))

    #Estados 
    
                
    def peao(self):

        if not(self.comecou):
            self.point = (self.x , self.y)
            self.comecou = True

        self.atual = self.get_clock().now().to_msg().sec
        
        if self.sorteado == 0:
            self.twist.angular.z = 0.5
        if self.sorteado == 1:
            self.twist.angular.z = -0.5
        dif = self.atual - self.inicio

        if dif > 7:
            self.cor_desejada = self.define_prevalente(self.mask_amerelo , self.mask_mangeta , 'amarelo' , 'mangeta')
            self.centralizou = True
            self.twist.angular.z = 0.
            self.robot_state = 'andador'
    def andador(self):
        self.error()
        self.twist.angular.z = self.erro * self.kp_ang
        print('O erro é: ' , self.erro)
        if self.cx == -1:
            self.twist.linear.x = 0.
        else:
            if self.cor_desejada == 'mangeta':
                self.twist.linear.x = 0.2
                if min(self.front) < 0.5:
                    self.robot_state = 'center'
                    self.twist.linear.x = 0.
            else:
                self.twist.linear.x = -0.2
                if min(self.back) < 0.5:
                    self.robot_state = 'center'
                    self.twist.linear.x = 0.
    
    def center(self):

        self.twist.linear.x = 0.
        self.get_angular_erro(self.point)
        self.twist.angular.z = self.kp_angular * self.ang_erro1
          
        if abs(self.ang_erro1) < np.deg2rad(1):
            self.robot_state = 'goto'
            self.twist.angular.z = 0.
    
    def goto(self):
        
        self.get_angular_erro(self.point)
        
        self.twist.angular.z = self.kp_angular * self.ang_erro1

        self.twist.linear.x = self.kp_linear * self.dist
        if self.dist < 0.05:
            self.robot_state = 'stop'

    def stop(self):
        self.twist = Twist()

    def control(self):
        
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        print('O Ponto inicial é:' , self.point)
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = Aleatorio() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()