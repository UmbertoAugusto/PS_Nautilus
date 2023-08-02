#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

def callback(vetores):
    '''Funcao para calcular os modulos das velocidades e publica-los em outros topicos.'''
    #Define variaveis que sao os valores das componentes dos vetores.
    x_linear = vetores.linear.x
    y_linear = vetores.linear.y
    z_linear = vetores.linear.z
    x_angular = vetores.angular.x
    y_angular = vetores.angular.y
    z_angular = vetores.angular.z
    #Calcula o modulo de cada vetor.
    modulo_linear = (x_linear**2 + y_linear**2 + z_linear**2)**(1/2)
    modulo_angular = (x_angular**2 + y_angular**2 + z_angular**2)**(1/2)
    #Define os topicos nos quais serao publicados os modulos.
    pub_lin = rospy.Publisher('modulo_linear', Float64, queue_size=10)
    pub_ang = rospy.Publisher('modulo_angular', Float64, queue_size=10)
    #Publica as mensagens que sao os modulos das velocidades.
    pub_lin.publish(modulo_linear)
    pub_ang.publish(modulo_angular)
    

    
    
def listener():
    '''Funcao para subscrever as informacoes das componentes das velocidades e manda-las para funcao callback.'''
    #define o noh "modulos" que vai subscrever as informacoes das componentes das velocidades.
    rospy.init_node('modulos', anonymous=True)
    #define qual vai ser o topico do qual as informacao vao ser subscritas.
    rospy.Subscriber("componentes_vetores", Twist, callback)
    #spin() para que essa funcao continue rodando.
    rospy.spin()

if __name__ == '__main__':
    listener()