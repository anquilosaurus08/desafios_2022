#!/usr/bin/env python

import rospy #importar ros para python
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy


class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.sub  = rospy.Subscriber('/duckiebot/camera_node/image/rect', Image, self.procesar_img)
		self.pub = rospy.Publisher('/duckiebot/detectar', Image, queue_size = 1)
		self.detector = cv2.CascadeClassifier("/home/duckiebot/duckietown/catkin_ws/src/desafios_2022/src/cascade3_LBP_.xml")
		self.bridge = CvBridge()
		
	#def callback(self,msg):
		
	def procesar_img(self, img):

		image = self.bridge.imgmsg_to_cv2(img, "bgr8")
		
		# Cambiar espacio de color
		if self.detector.empty():
			print("data mala")
			return
		img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		print(img_gray.shape)
		dets = self.detector.detectMultiScale(img_gray, 1.3, 10)
		
		for x,y,w,h in dets:
			x2 = x+w
			y2 = y+h
			cv2.rectangle(image,(x,y),(x2,y2),(255,0,0),2)

		# Publicar imagen final
		#image_out3 = cv2.cvtColor(image_out, cv2.COLOR_HSV2BGR)
		#msg = bridge.cv2_to_imgmsg(image_out, "bgr8")
		msg = bridge.cv2_to_imgmsg(image, "bgr8")
		self.pub.publish(msg)
		
		
		

def main():
	rospy.init_node('detectar') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
