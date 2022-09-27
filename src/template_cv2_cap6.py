#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
from geometry_msgs.msg import Point
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy


class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub  = rospy.Subscriber('duckiebot/camera_node/image/rect', Image, self.procesar_img)
		self.pub = rospy.Publisher('duckiebot/camera_node/pov', Image, queue_size=1 )
		self.pubDist = rospy.Publisher('duckiebot/dist', Point, queue_size = 1)
		self.min_area=400



	#def publicar(self):

	#def callback(self,msg):

	def procesar_img(self, img):
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(img, "bgr8")
		
		# Cambiar espacio de color
		
		image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		# Filtrar rango util
		limL = np.array([15, 150,  120])
		limU = np.array([80, 255, 255])
	
		mask = cv2.inRange(image_hsv, limL, limU)

		# Aplicar mascara

		

		# Aplicar transformaciones morfologicas
		kernel = np.ones((5,5), np.uint8)
		img_out= cv2.erode(mask, kernel, iterations= 5)
		img_out= cv2.dilate(mask, kernel, iterations= 5)
		
		image_out = cv2.bitwise_and(image, image, mask= mask)	
		# Definir blobs
		
			
		
		_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	
		for cnt in contours:
			x,y,w,h=cv2.boundingRect(cnt)
			if w*h > self.min_area:
				x2 = x+w
				y2 = y+h
				cv2.rectangle(image,(x,y),(x2,y2),(255,0,0),2)
				point = Point()
				f = 101.85916357881302 ## pixeles
				dpatox = 4 ## cm
				dpatoy = 3.1 ## cm
				Dz = dpatox*f/w
				point.z = Dz
				print(point)
				self.pubDist.publish(point)

		# Publicar imagen final
		#image_out3 = cv2.cvtColor(image_out, cv2.COLOR_HSV2BGR)
		#msg = bridge.cv2_to_imgmsg(image_out, "bgr8")
		msg = bridge.cv2_to_imgmsg(image, "bgr8")
		self.pub.publish(msg)
		
		
		

def main():
	rospy.init_node('cv2') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
