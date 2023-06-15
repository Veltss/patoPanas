#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

# Definir los valores de umbral HSV para el color amarillo y el color blanco
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([40, 255, 255])
lower_white = np.array([0, 0, 200])
upper_white = np.array([180, 20, 255])

class Template(object):
    def __init__(self):
        super(Template, self).__init__()

        self.bridge = CvBridge()
        self.publisher= rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = "x")
        self.vel = Twist()
        self.sub = rospy.Subscriber("/duckiebot/camera/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        # Convertir la imagen de un mensaje ROS al formato OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Aplicar umbral de color para detectar regiones amarillas y blancas
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        white_mask = cv2.inRange(hsv_image, lower_white, upper_white)

        # Aplicar operaciones morfológicas para eliminar ruido
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

        # Encontrar contornos en las máscaras binarias
        contours_yellow, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_white, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Verificar si se detectaron obstáculos amarillos
        if len(contours_yellow) > 0:
            # Si hay obstáculos amarillos, girar el robot
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.5  # Ajustar la velocidad angular de giro

        else:
            # Si no hay obstáculos amarillos, seguir la línea blanca
            if len(contours_white) > 0:
                # Calcular el centroide del contorno blanco más grande
                white_contour = max(contours_white, key=cv2.contourArea)
                M = cv2.moments(white_contour)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)

                # Ajustar la velocidad del robot en función de la posición del centroide
                error = cx - cv_image.shape[1] / 2
                self.vel.linear.x = 0.2
                self.vel.angular.z = -float(error) / 100

            else:
                # Si no se detecta línea blanca, detener el robot
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0

        # Publicar la velocidad del robot
        self.publisher.publish(self.vel)

        # Mostrar la imagen procesada en una ventana
        cv2.imshow("Imagen", cv_image)
        cv2.waitKey(1)

def main():
    rospy.init_node('template_node')
    template = Template()
    rospy.spin()

if __name__ == '__main__':
    main()