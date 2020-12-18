# -*- coding: utf-8 -*-
import numpy as np
from numpy import array as ar
from spatialmath import SE2, base
from coppeliaSincro import Cliente
import math
from math import pi,cos,sin,atan2
from random import random

class Robot(Cliente):
    
    def __init__(self,conectar=True):
        super().__init__(conectar)

        # Velocidades actuales
        self.v_act = 0
        self.w_act = 0
        # Velocidades de ruedas
        self.wL = 0
        self.wR = 0

        # Velocidad máxima
        self.v_max = 1

        # Dimensiones del robot
        self.d = 1
        self.R = 1

        # Handles de los joints
        self.ML = 0
        self.MR = 0

        # Lista de Sonares
        self.sonares = []
        self.sonar_max = 0.66

        # Odometría
        self.postura = ar([[0],[0],[0]])

        self.h_posturas = []
        
        self.last_time = 0

        # Handle del Centro cinemático (eje) del robot
        self.centro = 0
        # Handle de Goal para visualización
        self.goal_dummy_handle = 0
        self.goal_dummy_actualizado = False
        #Atributos para implementar funciones fuera de la clase
        self.actuar_func = None
        self.detectar_func = None
        self.init_func = None
    
    def getHandle(self, nombre):
        r, h = self.client.simxGetObjectHandle(nombre, self.call()) # llamadas Bloquean la Sim
        if r:
            return h 
        return -1
    
    def getObjectPosition(self, obj, rel_obj=-1):
        r = self.client.simxGetObjectPosition(obj, rel_obj, self.call())
        if r[0]: # success
            return r[1]
    
    def setObjectPosition(self, obj, pos, rel_obj=-1):
        self.client.simxSetObjectPosition(obj, rel_obj, pos, self.pub())
        

    def reset(self):
        self.postura = ar([[0],[0],[0]])
        self.h_posturas = [self.postura]
        self.last_time = 0
        self.v_act = 0
        self.w_act = 0
        self.wL = 0
        self.wR = 0
        self.goal_dummy_actualizado = False
    
    def init(self):
        ''' Inicializar los valores internos a los del robot.'''
        # Dimensiones del Robot
        self.d = 0.092 # 9.2 cm
        self.R = 0.032224 #3.2 cm

        self.v_max= 0.30 #m/s
        
        self.ML = self.getHandle("ML")
        self.MR = self.getHandle("MR") 
        self.centro = self.getHandle("centro_cine")
        self.goal_dummy_handle = self.getHandle("Goal")
        
        for i in range(1,6):
            self.sonares.append(self.getHandle("Sonar%d"%i))
        
        self.reset() # TODO:¿Hacer opcional?
        # Llamar la función modificable (notebook)
        
        if self.init_func:
          self.init_func(self)
            
    def actuation(self):

        if self.actuar_func:
          self.actuar_func(self)

    def sensing(self):
        if self.detectar_func:
          self.detectar_func(self)

        dt = self.getDeltaT()
        self.nueva_odometriaVel(dt)
        self.last_time = self.simTime # Actualizar antes que todo

    #### Movimiento
    def avanzar(self, vel):
        self.desplazar(vel,0)
    
    def mover(self, wR=0, wL=0):
        # Guardar
        self.wL = wL
        self.wR = wR
        # Enviar
        self.client.simxSetJointTargetVelocity(self.MR, wR, self.pub())
        self.client.simxSetJointTargetVelocity(self.ML, wL, self.pub())
    
    def desplazar(self, v, w):
        wR = (2*v+self.d*w)/(2*self.R) 
        wL = (2*v-self.d*w)/(2*self.R)
        # Guardar
        self.v_act = v
        self.w_act = w

        self.mover(wR=wR, wL=wL)

    def combinar_vel(self, v,w):
        '''Combina las velocidades para dar una final.'''
        if not hasattr(self,'velocidades'):
            self.velocidades = [0,0]
        
        return self.velocidades

    def arco(self, v, radio):
        w = v/radio
        self.desplazar(v,w)

    def ms_a_rad(self, vel):
        return vel/self.R
    
    #### Detección
    def leerSonar(self, disp):
        # Devuelve la distancia desde el sonar 
        r = self.client.simxReadProximitySensor(disp, self.call())
        #print(r)
        if r[0] and r[1]>0:
            return r[2]
        return self.sonar_max 
    
    def leer_distancias(self):
        '''Lee todos los sonares y devuelve un array con los valores.'''
        d = []
        for sonar in self.sonares:
            d.append(self.leerSonar(sonar))
        return np.array(d)

    #### Odometria
    def getDeltaT(self):
        return self.simTime - self.last_time

    def getPostura(self):
        return self.postura

    def histPostura(self):
        return self.h_posturas

    def actualizar_postura(self, dx, dth):
        pk_n = ar([[dx], [0.0], [dth]])
        T = base.trot2(self.postura[2]+dth)
        p_n = T @ pk_n
        self.postura = self.postura + p_n
        # Guardar
        self.h_posturas.append(self.postura)

    def forzar_postura(self,pos):
        # TODO: revisar si pos es array()
        self.postura = pos
        self.h_posturas.append(self.postura)

    def nueva_odometriaVel(self, dt):
        dx = self.v_act*dt
        dth = self.w_act*dt

        self.actualizar_postura(dx,dth)

    def nueva_odometriaRuedas(self, dt):
        dx = self.R/2*(self.wR+self.wL)
        dth = self.R/self.d*(self.wR-self.wL)

        self.actualizar_postura(dx,dth)

    #TODO: implementar odometria a partir de los "sensores" en las ruedas

    def actualizar_goal_pos(self, pos):
        p0 = self.getObjectPosition(self.goal_dummy_handle, rel_obj=self.centro)
        p0[0:2] = pos[0:2]
        self.setObjectPosition(self.goal_dummy_handle,p0, rel_obj=self.centro)

    #### Controladores
    def go2goal(self,goal, v=None):
        '''Controla para llegar a goal (x,y) controlando el arco.'''
        completo  = False
        if not self.goal_dummy_actualizado:
          self.actualizar_goal_pos(goal.flatten().tolist())
          self.goal_dummy_actualizado = True

        if not hasattr(self,'go2goal_gains'):
            self.go2goal_gains = {'K_w': 0.5, 'K_v':0.2}
        
        K_w = self.go2goal_gains['K_w']
        K_v = self.go2goal_gains['K_v']

        d_min = 0.01 # metros

        pos = self.postura
        p = pos[0:2]
        th = pos[2].item()
        #print(p, goal)
        # TODO: revisar que goal sea array()

        e_pos = goal - p 
        e_dist = np.linalg.norm(e_pos)

        # Error de orientación
        angle = atan2(e_pos[1], e_pos[0])  # atan va de [-pi/2, pi]

        e_ang = angle - th
        e_ang = atan2(sin(e_ang), cos(e_ang)) # Correr asegurar el ángulo adecuao

        # Ley de control
        w = K_w*e_ang
        #if v == None:
        #  v = self.v_act
        #print("G2G D: ", e_dist)
        v = K_v*e_dist
        if e_dist < d_min:
          v = 0
          w = 0
          completo  = True
          self.goal_dummy_actualizado = False # Débil aquí, pero blah

        self.desplazar(v,w)
        return completo
        
    def go2pose(self, pose_g, v=None):
        '''Controla para llegar a goal (x,y) controlando el arco.'''

        completo  = False

        if not self.goal_dummy_actualizado:
          self.actualizar_goal_pos(pose_g.flatten().tolist())
          self.goal_dummy_actualizado = True

        if not hasattr(self,'go2pose_gains'):
            self.go2pose_gains = {'K_w1': 0.5, 'K_w2': 0.3, 'K_v':0.2}
        
        K_w1 = self.go2pose_gains['K_w1']
        K_w2 = self.go2pose_gains['K_w2']
        K_v = self.go2pose_gains['K_v']

        d_min = 0.01 # metros

        pos = self.postura
        p_robot = pos[0:2]
        th_robot = pos[2].item()
        #print(p, goal)
        # TODO: revisar que goal sea array()
        goal = pose_g[0:2]
        th_goal = pose_g[2].item()
        
        # Errores: Ecuación 3.65 de Correll (con ajustes)
        e_pos = goal - p_robot
        e_dist = np.linalg.norm(e_pos) #Rho
        
        # Dirección del punto: 
        angle = atan2(e_pos[1], e_pos[0])  

        # Error de dirección (al goal)
        alpha = angle - th_robot
        alpha = atan2(sin(alpha), cos(alpha)) # Correr para asegurar el ángulo adecuado
        
        # Error de orientación (a postura)
        eta = th_goal - th_robot
        eta = atan2(sin(eta), cos(eta))

        # Ley de control
        # Implementar ecuaciones 3.66 y 3.67 de Correll
        #print("angle: %0.2f, a: %0.2f, n: %0.2f" %(angle, alpha, eta))
        w = K_w1*alpha + K_w2*eta
        #if v == None:
        #  v = self.v_act

        v = K_v*e_dist
        if e_dist < d_min:
          v = 0
          w = 0
          completo  = True
          self.goal_dummy_actualizado = False # Débil aquí, pero blah

        self.desplazar(v,w)
        return completo

    # Caso 1: según el ángulo del sensor
    def evitarObsAng(self):
        '''Si alguno de los sensores detecta un obstáculo, evitarlo.'''        
        completo  = False
        v = 0
        w = 0

        self.desplazar(v,w)
        return completo
    
    # Caso 2: según vector que "apunta" al obstáculo
    def evitarObs(self):
        '''Evitar un obstáculo según su ubicación ante el vehículo.'''
        completo  = False
        v = 0
        w = 0

        self.desplazar(v,w)
        return completo

    def seguirPared(self):
        '''Si encuentra un obstáculo, recorrer su perímetro desde una distancia adecuada, manteniendolo a su izquierda.'''
        # Si tiene obstáculos a ambos lados, prefiere el de la izquierda.
        completo  = False
        v = 0
        w = 0

        self.desplazar(v,w)
        return completo
    

    # Comportamientos
    def miedo(self):
        '''Se aleja de cualquier objeto.'''
        # Leer sensores
        # Cambiar la dirección lejos de la detección

        dis = self.leer_distancias()

        lados = ar([ dis[0], d[1], d[3], d[4] ])
        frente =  dis[3]

        ganancias_lados = ar([ 0.1, 0.1, -0.1, -0.1  ])
        ganancia_frente = -0.1

        w = np.dot(lados, ganancias)
        v = ganancia_frente*frente 

        self.desplazar(v,w)

    def curioso(self):
        '''Se acerca a todo lo que ve.'''
        # Leer sensores
        # Cambiar la dirección hacia la detección

        dis = self.leer_distancias()

        lados = ar([ dis[0], d[1], d[3], d[4] ])
        frente =  dis[3]

        ganancias_lados = ar([ -0.1, -0.1, 0.1, 0.1  ])
        ganancia_frente = 0.1

        w = np.dot(lados, ganancias)
        v = ganancia_frente*frente 

        self.desplazar(v,w)

    def interes(self, dir_g, v=0):
        '''Intenta ir en una dirección.'''
        if v == 0:
            v = self.v_max*0.3
        
        pos = self.postura ## Idealmente se haría con algún tipo de faro
        #p_robot = pos[0:2]
        th_robot = pos[2].item()
        
        e = dir_g - th_robot
        e = atan2(sin(e), cos(e))
        
        Ke = 0.2
        w = Ke*e

        self.desplazar(v,w)
        
    def deambular(self, v=0):
        '''Anda sin rumbo específico.'''
        if v == 0:
            v = self.v_max*0.3
        # Cambiar la direccion paulatinamente.
        
        alpha = 0.8
        a = 3
        b = a/2
        dth = random()*a - b

        w = alpha*self.w_act + (1-alpha)*dth

        self.desplazar(v,w)

if __name__ == "__main__":
  
  # Celda 1: Función externa para extender la inicialización del Robot
  # Ver init() en la clase Robot.

  def m_init(self):
    pass
  # Fin Celda 1

  # Celda 2: Puntos objetivo para los recorridos del robot
  goal0 = ar([ [-0.88], [1.2] ])
  goal1 = ar([ [0.34], [-1.6] ])
  goal2 = ar([ [1.44], [0.33] ])
  goal3 = ar([ [0.32], [1.67] ])
  # Fin Celda 2

  # Celda 3: Función externa para extender la actuación del Robot
  # Ver actuation() en la clase Robot.

  # Función que hace el trabajo de sysCall_actuation()
  def actuar(self):
      '''Alcanzar cada uno de los puntos goal#, evitando obstáculos que se interpongan.'''

      self.go2goal(goal2)
      #self.go2pose(ar([[0.48], [0.98],[1/6*math.pi]]))
      p = self.postura
      print("%0.3f, v:%0.3f, w:%0.3f , (%0.3f,%0.3f,%0.3f)" % (self.simTime,self.v_act, self.w_act, p[0], p[1], p[2]))

  # Fin Celda 3

  # Celda 4: Función externa para extender la detección del Robot
  # Ver sensing() en la clase Robot.

  # Función que hace el trabajo de sysCall_sensing()
  def detectar(self):
      

      print(self.leer_distancias())
  # Fin Celda 4

  # Celda 5: Creación de la instancia del robot, borrar si existe.
  try:
      del(duckie)
      
  except NameError:
      pass

  duckie = Robot() 

  # Fin Celda 5

  # Celda 6: Preparación de la instancia del robot
  duckie.actuar_func = actuar
  duckie.detectar_func = detectar 
  duckie.go2goal_gains = {'K_w': 1.2, 'K_v':0.2} 
  #duckie.go2pose_gains = {'K_w1': 0.3, 'K_w2': 0.1, 'K_v':0.1}
  # Fin Celda 6
  
  # Celda 7: Correr simulación 
  print ("Corriendo Sim")
  duckie.correrSim(15) # Agregar pausar=True para terminar en pausa

  #Fin Celda 7

  duckie.cerrar()
