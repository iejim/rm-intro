# -*- coding: utf-8 -*-
import numpy as np
from numpy import array
from spatialmath import SE2, base
from coppeliaSincro import Cliente
import math
from math import pi,cos,sin,atan2
from random import random
from msgpack import packb

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
        self.postura_de_sim = False
        self.postura = array([[0],[0],[0]])

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

        self.trayectoria = dict()
    
    def getHandle(self, nombre):
        r, h = self.client.simxGetObjectHandle(nombre, self.call()) # llamadas Bloquean la Sim
        if r:
            return h 
        return -1
    
    # TODO: Devolver un vector columna (como la Postura)
    def getObjectPosition(self, obj, rel_obj=-1):
        r = self.client.simxGetObjectPosition(obj, rel_obj, self.call())
        if r[0]: # success
            return r[1]
    
    def setObjectPosition(self, obj, pos, rel_obj=-1):
        self.client.simxSetObjectPosition(obj, rel_obj, pos, self.pub())

    def getObjectOrientation(self, obj, rel_obj=-1, deg=False):
        r = self.client.simxGetObjectOrientation(obj, rel_obj, self.call())
        if r[0]: # success
            if deg: 
                d = array(r[1])*180.0/pi
                return d.tolist()
            else:
                return r[1] 
    
    def vector(self, lista):
        v = array(lista).reshape(len(lista), 1)
        return v

    def reset(self):
        self.postura = array([[0],[0],[0]])
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
        
        self.deambular_gain = 0.9
        self.sonares = []
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
        # Guardar
        self.v_act =  v
        self.w_act =  w
        
        wR = (2*v+self.d*w)/(2*self.R) 
        wL = (2*v-self.d*w)/(2*self.R)
        
        #Filtrar aqui

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

    def nueva_postura(self, dx, dth):
        pk_n = array([[dx], [0.0], [dth]])
        T = base.trot2(self.postura[2]+dth)
        p_n = T @ pk_n
        return self.postura + p_n

    def postura_en_sim(self):
        pos = self.getObjectPosition(self.centro)
        ort = self.getObjectOrientation(self.centro)
        return self.vector([pos[0], pos[1], ort[2]]) # Vector postura 2D

    def actualizar_postura(self, dx, dth):
        
        if self.postura_de_sim:
            self.postura = self.postura_en_sim()
        else: 
            self.postura = self.nueva_postura(dx, dth)
        # Guardar
        self.h_posturas.append([self.simTime, self.postura])

    def forzar_postura(self,pos):
        # TODO: revisar si pos es array()
        self.postura = pos
        self.h_posturas.append([self.simTime, self.postura])

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
        
        if type(pos) is np.ndarray:
            nueva_p = pos[0:2].flatten().tolist()
        else:
            nueva_p = pos[0:2]
        p0[0:2] = nueva_p
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
    

    # Comportamientos Pasivos (no mueven el robot)
    def atraccion(self, goal, v=0):
        '''Controla para llegar a goal (x,y) controlando el arco.'''

        if not hasattr(self,'atraccion_gains'):
            self.atraccion_gains = {'K_w': 0.5, 'K_v':0.2}
        
        K_w = self.atraccion_gains['K_w']
        K_v = self.atraccion_gains['K_v']

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

        return (v,w)
    
    def evasion(self, v=0, d_min = 0.15):
        '''Cambia su direccion ante cualquier objeto.'''
        # Leer sensores
        # Cambiar la dirección lejos de la detección
        dis = self.leer_distancias()

        #lados = self.sonar_max - array([ dis[0], dis[1], dis[3], dis[4] ]) #no incluir el frente
        lados = self.sonar_max - dis #usar el frente para lograr girar si es solo este.
        
        frente = self.sonar_max - dis[2]
        #print(lados)
        #print("F: %0.3f, L: %s" % (frente, lados))
        ganancias_lados = array([ 1, 2, 0.75, -2.2, -1 ]) * 0.5 #darle poco peso al frente.
        
        # Detiene al llegar a una d_min
        ganancia_frente = v/(self.sonar_max-d_min)

        # Para combinarlos con otros, deberían ser solo el delta (sin suma)
        w = np.dot(lados, ganancias_lados)
        v = v - ganancia_frente*frente

        return (v,w)

    
    # Comportamientos Activos (sí mueven el robot)
    def miedo(self, v=0):
        '''Se aleja de cualquier objeto.'''

        # Alejarse si es necesario.
        if v == 0:
            v = self.v_max*0.3

        # Cambiar la dirección lejos de la detección
        v, w = self.evasion(v)
        v = v 
        
        self.desplazar(v,w)

    def curioso(self, v=0):
        '''Se acerca a todo lo que ve.'''
        # Leer sensores
        # Cambiar la dirección hacia la detección
        if v == 0:
            v = self.v_max*0.3

        dis = self.leer_distancias()

        lados = self.sonar_max - array([ dis[0], dis[1], dis[3], dis[4] ])
        frente =  self.sonar_max - dis[3]

        ganancias_lados = array([ -2, -1, 1, 2 ]) * 0.5
        ganancia_frente = -0.4

        w = np.dot(lados, ganancias_lados)
        v = v + ganancia_frente*frente 

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
        
        Ke = self.deambular_gain
        w = Ke*e

        self.desplazar(v,w)
        
    def deambular(self, v=0):
        '''Anda sin rumbo específico.'''
        if v == 0:
            v = self.v_max*0.3
        # Cambiar la direccion paulatinamente.
        
        alpha = self.deambular_gain
        a = 6
        b = a/2
        dth = random()*a - b
        # Moving average
        w = alpha*self.w_act + (1-alpha)*dth

        self.desplazar(v,w)

    # Trayectorias
    def distancia(self,a,b):
        ''' Norma euclidiana entre dos listas (array-like), usando la dimensión de la primera.'''
        
        if len(a) == 0  or len(b) ==0:
            return 0 # TODO: por ahora
        s = 0
        if type(a) is np.ndarray:
            a = a.flatten()
        if type(b) is np.ndarray:
            b = b.flatten()

        for i in range(len(a)):
            s = s + (a[i]-b[i])**2
        return math.sqrt(s)
    
    def pathLength_sim(self, path):
        data = self.client.simxCallScriptFunction('sim.getPathLength',"sim.scripttype_sandboxscript",path, self.call())
        if data[0]:
            return data[1]
        return 0

    def analizarTrayectoria_sim(self):
        path = self.trayectoria["path"] #TODO: trayectoria pudera ser una dict con [path, paso, lookahead]

        L = self.pathLength_sim(path) # Extrae la longitud del path

        # Longitudes relativas
        paso = self.trayectoria["paso"]/L # Salto (cm) entre un punto y otro a buscar a lo largo del path
        lookahead = self.trayectoria["lookahead"]/L

        # Guardar en self.trayectoria (para uso interno sin crear nuevas variables)
        self.trayectoria["paso_rel"] = paso
        self.trayectoria["lookahead_rel"] = lookahead
        self.trayectoria["length"] = L

    def puntoEnPath_sim(self, path, rel_d):
        # simxCallScriptFunction(string funcAtObjName, number/string scriptType, anyType funcArgs, string topic)
        data = self.client.simxCallScriptFunction('posOnPath@Duckie',"sim.scripttype_childscript",dict(path=path,rel_d=rel_d), self.call())
        # data = self.client.simxCallScriptFunction('sim.getPositionOnPath',"sim.scripttype_sandboxscript",[[path], [rel_d]], self.call())
        if data[0]:
            #print(data[1])
            return self.vector(data[1][0:2])
        return array([])

    def goalEnTrayectoria_sim(self):
        '''Busca un punto en una trayectoria en CoppeliaSim. Usa distancias relativas en vez de indices para encontrar punto.'''
        
        #TODO: hay que establecer cuándo se empieza a buscar otra vez,
        #      tanto para agilizar, como para no agotar los puntos.
        # Inicializar memorias ¿aquí?
        if not "paso_rel" in self.trayectoria.keys():
            self.analizarTrayectoria_sim()

        if not "l_cercano" in self.trayectoria.keys():
            self.trayectoria["l_cercano"] = 0
            # self.trayectoria["l_goal"] = 0 # No es necesario si usamos Lookahead
        
        pos = self.postura[0:2]

        path = self.trayectoria["path"]
        paso = self.trayectoria["paso_rel"]
        DL = self.trayectoria["lookahead_rel"]

        ultima_l = self.trayectoria["l_cercano"] # Empezar donde nos quedamos
        # Buscar Punto más cercano
        #   Paso  a paso buscar el punto cuya distancia sea menor a la anterior
        dist_vieja = 0
        dist_nueva = -1
        while dist_nueva < dist_vieja: 
            # Buscar punto, clc distancias
            ultima_l = ultima_l+paso
            punto = self.puntoEnPath_sim(path,ultima_l) 
            #TODO Revisar si punto es []
            dist_vieja = dist_nueva
            dist_nueva = self.distancia(pos,punto)

        # Buscar goal
        #   Buscar punto usando ultima_l + lookahead_relativo
        goal = self.puntoEnPath_sim(path,ultima_l+DL)
        # print(goal)
        self.trayectoria["l_cercano"] = ultima_l #Actualizar
        self.actualizar_goal_pos(goal)
        return goal

    def goalEnTrayectoria(self):
        
        tray = self.trayectoria["path"]

        # Inicializar memorias ¿aquí?
        if not "ind_cercano" in self.trayectoria.keys():
            self.trayectoria["ind_cercano"] = 0
            self.trayectoria["ind_goal"] = 0

        total = self.trayectoria["puntos"]

        ultimo_ind = self.trayectoria["ind_cercano"] # Ultimo sitio donde buscamos 
        ultimo_goal = self.trayectoria["ind_goal"] # Ultimo punto usado para goal


        if ultimo_goal >= total-1:
            return tray[-1] # como señal de que no hay que buscar; (0,0) puede ser un goal

        # Buscar más cercano
        dist_ant = 0
        dist_nueva = 0
        while dist_nueva < dist_ant:
            ind = ultimo_ind + 1
            if ind >= total:
                ind = total - 1  
            punto = tray[ind]
            dist_ant = dist_nueva
            dist_nueva = self.distancia(pos,punto)
            ultimo_ind = ind
        
        cerca = punto
        self.trayectoria["ind_cercano"] = ultimo_ind

        # Buscar el goal:
        #   Empezando en el cercano, buscar el goal adecuado
        #   TODO: Pudiera ser que nos ahorremos lo del cercano 
        #   y solo busquemos un nuevo goal a una distancia L
        dist_nueva = 0
        L = self.trayectoria["lookahead"]
        while dist_nueva < L:
            ind = ultimo_goal + 1
            if ind >= total:
                ind = total - 1
            punto = tray[ind]
            dist_nueva = self.distancia(cerca,punto)
            ultimo_goal = ind
        
        goal = punto
        self.trayectoria["ind_goal"] = ultimo_goal

        return goal


    def seguirTrayectoria(self, v=0):
        '''Sigue una trayectoria usando Pure Pursuit'''
        #
        if v == 0:
            v = self.v_max*0.3
        
        # print(self.trayectoria)
        pos = self.postura[0:2]
        theta = self.postura[2]

        goal = self.goalEnTrayectoria_sim()
        # print("g ", goal)
        # print("p ", pos)
        #Generar el arco a un punto en la trayectoria
        e = (goal - pos).flatten().tolist() # usa lista estandar
        # print("e ", e)
        # D = np.linalg.norm(e)
        D2 = e[0]*e[0]+e[1]*e[1]  # Escalar (no array)
        dY = e[0]*sin(theta) + e[1]*cos(theta)

        gamma = -2*dY/D2

        v = v
        w = v*gamma

        return (v,w)

if __name__ == "__main__":
  
  # Celda 1: Función externa para extender la inicialización del Robot
  # Ver init() en la clase Robot.

  def m_init(self):
    pass
  # Fin Celda 1

  # Celda 2: Puntos objetivo para los recorridos del robot
  goal0 = array([ [-0.88], [1.2] ])
  goal1 = array([ [0.34], [-1.6] ])
  goal2 = array([ [1.55], [1.14] ])
  goal3 = array([ [0.32], [1.67] ])
  # Fin Celda 2

  # Celda 3: Función externa para extender la actuación del Robot
  # Ver actuation() en la clase Robot.

  # Función que hace el trabajo de sysCall_actuation()
  def actuar(self):
      '''Alcanzar cada uno de los puntos goal#, evitando obstáculos que se interpongan.'''

      self.go2goal(goal2)
      #self.go2pose(array([[0.48], [0.98],[1/6*math.pi]]))
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