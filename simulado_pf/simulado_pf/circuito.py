import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from my_package.odom import Odom
from my_package.laser import Laser
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np 

import time
from cv_bridge import CvBridge
# Adicione aqui os imports necessários

class Circuito(Node , Odom , Laser): # Mude o nome da classe

    def __init__(self , cor_desejada):
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
        self.robot_state = 'segue_linha'
        self.state_machine = {
            'stop': self.stop,
            'segue_linha': self.segue_linha,
            'girar': self.giro,
            'centraliza': self.centraliza,
            'aproxima': self.aproxima,
            'goto': self.goto
        }

        # Inicialização de variáveis
        self.twist = Twist()
        self.kp_ang = 0.007
        self.kp_ang3 = 0.001
        self.kp_ang2 = 0.5
        self.kp_linear = 0.1
        self.comecou = False
        self.volta = False
        self.inicio = self.get_clock().now().to_msg().sec
        self.cor_desejada = cor_desejada
        self.cores = {
    'vermelho' : [np.array([0, 55, 55]) , np.array([8, 255, 255])],
    'verde' : [np.array([50, 55, 55]) , np.array([75, 255, 255])],
    'azul' : [np.array([105, 55, 55]) , np.array([135, 255, 255])],
    'amarelo' : [np.array([15, 110, 170]) , np.array([35, 255, 255])]
}
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

    #Processamentos de Subscribers
    def image_callback(self , msg):
        
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.copia_camera = self.cv_image.copy()

        self.mascara_camera = self.faz_mascara(self.copia_camera , 'amarelo')
        self.imagem_redefinida = self.redefine_dimensão(self.copia_camera)

        self.mask_color = self.faz_mascara(self.cv_image , self.cor_desejada)
        
    
        self.h , self.w , _ = self.cv_image.shape        
        self.w = self.w/2

        #self.mascara_camera[:int(self.h/2),:] = 0
        
        if self.robot_state == 'segue_linha':
            self.acha_cm(self.mascara_camera)
            cv2.imshow('Mask_Camera' , self.mascara_camera)

        else: 
            self.acha_cm(self.mask_color)
            cv2.imshow('Mask_Camera' , self.mask_color)

        cv2.imshow('Image', self.cv_image)
        #cv2.imshow('Image_Reduced' , self.imagem_redefinida)
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
    
    def get_angular_erro(self , point):

        #Calculo da distancia 

        self.err_x = point[0] - self.x
        self.err_y = point[1] - self.y
        self.dist = np.sqrt(self.err_x ** 2 + self.err_y ** 2)


        #Calculo do Angulo Desejado 

        self.theta = np.arctan2(point[1]-self.y , point[0]-self.x)
        self.erro = self.theta - self.yaw
        self.erro = np.arctan2(np.sin(self.erro), np.cos(self.erro))

    #Estados
    def segue_linha(self): 
        
        if not(self.comecou):
            self.x0 = self.x
            self.y0 = self.y
            self.comecou = True

        self.now = self.get_clock().now().to_msg().sec
        
        if self.cx == -1:
            self.twist.angular.z = -0.5
            self.twist.linear.x = 0.
        else: 
            self.calcula_erro()
            self.twist.angular.z = self.erro * self.kp_ang
            self.twist.linear.x = 0.6
            dist = np.sqrt((self.x0 - self.x)**2 + (self.y0 - self.y)**2)

            if dist < 5 and self.now - self.inicio > 10:
                self.twist.linear.x = dist * self.kp_linear
            
            print('A distância é: ' , dist)
            print('A diferença de tempo é: ' , self.now - self.inicio)
            if dist < 0.05 and self.now - self.inicio > 30:
                self.twist.linear.x = 0.
                self.twist.angular.z = 0.
                self.robot_state = 'girar'
                self.goal_yaw = (self.yaw_2pi + (np.pi/2)) % (2 * np.pi) 

    def giro(self):
        
        
        erro_ang = self.goal_yaw - self.yaw_2pi
        self.twist.angular.z = erro_ang * self.kp_ang2
        
        print('O goal yaw é :  ' , self.goal_yaw)
        print('O erro angular é: ' , erro_ang % (2*np.pi))
        
        if abs(erro_ang)  < 0.07:
            if not(self.volta):
                self.robot_state = 'centraliza'
            else:
                self.robot_state = 'goto'

    def centraliza(self):
        self.calcula_erro()
        if self.cx == -1:
            self.twist.angular.z = -0.3
        else:
            self.twist.angular.z = self.kp_ang3 * self.erro
        
        
        if self.erro < 1:
            self.robot_state = 'aproxima'
            
    def aproxima(self):

        self.calcula_erro()
        self.twist.angular.z = self.kp_ang3 * self.erro
        if min(self.front) != np.inf:
            self.twist.linear.x = self.kp_linear * min(self.front)
        else:
            self.twist.linear.x = 0.6

        if min(self.front) < 0.4:
            self.twist.linear.x = 0.
            self.goal_yaw = (self.yaw_2pi + np.pi) % (2 * np.pi) 
            self.robot_state = 'girar'
            self.point = (self.x0 , self.y0)
            self.volta = True
    
    def goto(self):
        self.get_angular_erro(self.point)
        
        self.twist.angular.z = self.kp_linear * self.erro
        self.twist.linear.x = self.kp_linear * self.dist
        if self.dist < 0.05:
            self.robot_state = 'stop'
        
    
    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        if self.comecou:
            print('O X0 é :' , self.x0)
            print('O Y0 é :' , self.y0)
        
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)
def main(args=None):
    rclpy.init(args=args)
    ros_node = Circuito('azul') # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()