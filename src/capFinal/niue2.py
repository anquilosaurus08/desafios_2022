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
		self.sub  = rospy.Subscriber('duckiebot/joy', Joy, self.callback)
		self.pub = rospy.Publisher('duckiebot/possible_cmd', Twist2DStamped, queue_size = 10 )

	#def publicar(self):

	def callback(self,msg):
		msg.axes = list(msg.axes)
		if msg.buttons[1] == 1:
			msg.axes[0] = 0
			msg.axes[1] = 0
			print("CUAC")
		
		Y = msg.axes[0]
		X = msg.axes[1]
		
		print(msg.axes)
		
		Datos = Twist2DStamped()
		Datos.v = X*1.5
		Datos.omega = Y*(7.5)
		self.pub.publish(Datos)
	
def mapeo(inf, sup, val):
	delta = (abs(sup) - abs(inf))*0.5
	escala = sup - delta
	return 1.0*escala*val + 1.0*delta
	
def main():
	rospy.init_node('niue') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()

