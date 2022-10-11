#!/usr/bin/env python

import rospy #importar ros para python
from sensor_msgs.msg import Joy # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from duckietown_msgs.msg import Twist2DStamped
from geometry_msgs.msg import Point


class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.pato = False
		self.sub1  = rospy.Subscriber('duckiebot/possible_cmd', Twist2DStamped, self.callback)
		self.sub2 = rospy.Subscriber('duckiebot/dist', Point, self.callback2)
		self.pub = rospy.Publisher('duckiebot/wheels_driver_node/car_cmd',Twist2DStamped, queue_size = 10 )

	#def publicar(self):
	

	def callback(self,msg):
		Datos = msg
		if self.pato == True and Datos.v > 0:
			Datos.v = 0		
			self.pub.publish(Datos)
		else: 
			self.pub.publish(Datos)
		
			

	def callback2(self,msg):
		
		dist = msg.z
		minDist = 22
		print(dist, self.pato)
		if dist <= minDist and dist >= 0:
			self.pato = True
		elif msg.z == -1:
			self.pato = False
		else:
			self.pato = False
			
		
	

	
def main():
	rospy.init_node('niuetemplate') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()

