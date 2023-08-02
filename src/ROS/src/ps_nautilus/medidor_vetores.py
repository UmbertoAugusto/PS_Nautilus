#!/usr/bin/env python3

#importa randint pra gerar os valores aleatorios pros componentes dos vetores das velocidades
from random import randint
import rospy
from geometry_msgs.msg import Twist

def publisher():
    '''Funcao para publicar as informacoes das componentes dos vetores velocidade linear e velocidade angulas.'''
    #define o topico no qual publichser vai publicar os valores dos componentes dos vetores.
    pub = rospy.Publisher('componentes_vetores', Twist, queue_size=10)
    #define o noh vetores que vai publicar as informacoes dos componentes.
    rospy.init_node('vetores', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #cria "velocidade", do tipo Twist, que vai ser a mensagem.
        velocidade = Twist()
        #cria valores pseudoaleatorios pras componentes das velocidades.
        velocidade.linear.x = randint(-100,100)
        velocidade.linear.y = randint(-100,100)
        velocidade.linear.z = randint(-100,100)
        velocidade.angular.x = randint(-100,100)
        velocidade.angular.y = randint(-100,100)
        velocidade.angular.z = randint(-100,100)
        #publica "velocidade" no "topic1"
        pub.publish(velocidade)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass