#!/usr/bin/python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
#importa math para modelar os movimentos dos planetas e satelites.
import math

class FrameFixo():
    '''class para criar um frame fixo que vai ser a estrela.'''
    def __init__(self):
        '''metodo init para definir o broadcaster da estrela.'''
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    def publicarTransformadaEstatica(self):
        '''metodo para enviar a posicao da estrela.'''
        #define a transformada.
        transformada = TransformStamped()
        #define parent e child.
        transformada.header.frame_id = 'centro_observacao' #parente eh um centro de observacao
        transformada.child_frame_id = 'estrela' #child eh a propria estrela
        transformada.header.stamp = rospy.Time.now()
        #define a posicao da estrela.
        transformada.transform.translation.x = 0.0
        transformada.transform.translation.y = 0.5
        transformada.transform.translation.z = 0.0
        #define a orientacao da estrela.
        transformada.transform.rotation.x = 0.0
        transformada.transform.rotation.y = 0.0
        transformada.transform.rotation.z = 0.0
        transformada.transform.rotation.w = 1.0
        #envia a transformada com a informacao da posicao da estrela.
        self.broadcaster.sendTransform(transformada)

class FramePlaneta():
    '''class para criar os planetas.'''
    def __init__(self,planeta):
        '''metodo que define as informacoes dos planetas.'''
        #nome do planeta eh passado por argumento.
        self.planeta = planeta
        #o raio de orbita do planeta eh pego de um parametro.
        self.raio = rospy.get_param('planetas/'+planeta+'/raio')
        #define o broadcaster do planeta.
        self.br = tf2_ros.TransformBroadcaster()

    def publicar(self,k):
        '''Metodo para fazer o planeta girar em torno da estrela. Recebe um numero k que sera usado para o calculo do seu movimento.'''
        #define a transformada que vai ser enviada.
        transformada = TransformStamped()
        #define parent e child.
        transformada.header.frame_id = "estrela" #parent eh a estrela central
        transformada.child_frame_id = self.planeta #child eh o proprio planeta
        transformada.header.stamp = rospy.Time.now()
        #usa seno e cosseno para modelar o movimento de translacao orbital.
        transformada.transform.translation.x = self.raio*math.sin(k)
        transformada.transform.translation.y = self.raio*math.cos(k)
        transformada.transform.translation.z = 0.0
        #define as coordenadas de rotacao.
        transformada.transform.rotation.x = 0.0
        transformada.transform.rotation.y = 0.0
        transformada.transform.rotation.z = 0.0
        transformada.transform.rotation.w = 1.0
        #envia a transformada.
        self.br.sendTransform(transformada)
            

class FrameSatelite():
    '''class para criar os satelites.'''
    def __init__(self,planeta,satelite):
        '''metodo que define as informacoes dos satelites.'''
        #nome do satelite e do planeta que ele orbita são passados por argumentos.
        self.planeta = planeta
        self.satelite = satelite
        #o raio da orbita do satelite em torno do planeta eh pego de um parametro.
        self.raio = rospy.get_param('satelites/'+planeta+'/'+satelite+'/raio')
        #define o broadcaster do satelite
        self.br = tf2_ros.TransformBroadcaster()

    def publicar(self,k):
        '''Metodo para fazer o satelite girar em torno do planeta. Recebe um numero k que sera usado para o calculo do seu movimento.'''
        #define a transformada que sera enviada.
        transformada = TransformStamped()
        #define parent e child.
        transformada.header.frame_id = self.planeta #parente eh o planete no qual o satelite orbita
        transformada.child_frame_id = self.satelite #child eh o proprio satelite
        transformada.header.stamp = rospy.Time.now()
        #usa seno e cosseno para modelar o movimento de translacao orbital.
        transformada.transform.translation.x = self.raio*math.sin(k)
        transformada.transform.translation.y = self.raio*math.cos(k)
        transformada.transform.translation.z = 0.0
        #define coordenadas de rotacao.
        transformada.transform.rotation.x = 0.0
        transformada.transform.rotation.y = 0.0
        transformada.transform.rotation.z = 0.0
        transformada.transform.rotation.w = 1.0
        #envia a transformada.
        self.br.sendTransform(transformada)
    


    

if __name__ == '__main__':
    #inicia um no.
    rospy.init_node("tf_broadcaster")
    #define a estrela e publica sua transformada.
    estrela = FrameFixo()
    estrela.publicarTransformadaEstatica()

    #define os planetas que aparecerao.
    p1 = FramePlaneta('mercurio')
    p2 = FramePlaneta('venus')
    p3 = FramePlaneta('terra')
    p4 = FramePlaneta('marte')
    #define os satelites que aparecerao.
    s1 = FrameSatelite('terra','lua')
    s2 = FrameSatelite('marte','phobos')
    s3 = FrameSatelite('marte','deimos')
    #laco de repeticao com frequencia de 10hz para fazer os movimentos dos planetas e satelites.
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        #define variavel k que vai ser usada no calculo do movimento de todos os corpos.
        k = rospy.Time.now().to_sec()
        #chama metodos para realizar o movimento dos corpos, cada um com um fator diferente multiplicando k para diferenciar suas velocidades.
        #planetas:
        p1.publicar(1*k)
        p2.publicar(0.9*k)
        p3.publicar(0.7*k)
        p4.publicar(0.6*k)
        #satelites:
        s1.publicar(4*k)
        s2.publicar(5*k)
        s3.publicar(2*k)

        r.sleep()