# -*- coding: utf-8 -*-
## Simulación Coppelia B0

# Agregar /Applications/coppeliaSim.app/Contents/Resources/programming/b0RemoteApiBindings/python al path
#import sys
#sys.path.append("/Applications/coppeliaSim.app/Contents/Resources/programming/b0RemoteApiBindings/python")



from B0 import b0RemoteApi
import time



class Cliente:
  def __init__(self,conectar=True, channel='b0RemoteApi'):
    self.channel = channel
    self.inicializado = False
    self.conectado = False
    self.simTime = 0
    if conectar:
      self.conectar()

  def conectar(self):
    self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient',self.channel)
    if self.client:
      self.conectado = True
      self.pub = self.client.simxDefaultPublisher # le pasa info a Coppelia
      self.sub = self.client.simxDefaultSubscriber # Recibe info de Coppelia
      self.call = self.client.simxServiceCall # Le pide a coppelia que corra una func
    else:
        print('No conectado')
  
  def cerrar(self):
    self.client.__exit__()
    self.client._node.shutdown_requested()
    self.client._node.shutdown()

  def inicializar(self):

    self.doNextStep=True

    # Callbacks
    # Hace el trabajo de consumir la información recibida
    self.activarCallbackStepStarted()
    self.activarCallbackStepDone()

    # Setear como asincrono
    self.client.simxSynchronous(True)
    self.inicializado = True

  def cb_simulationStepStarted(self, msg):
        simTime=msg[1][b'simulationTime']
        self.simTime = simTime
        #print('Simulation step started. Simulation time: ',simTime)

  def cb_simulationStepDone(self, msg):
        simTime=msg[1][b'simulationTime']
        #print('Simulation step done. Simulation time: ',simTime);
        self.doNextStep=True


  def activarCallbackStepStarted(self):
    self.client.simxGetSimulationStepStarted(self.sub(self.cb_simulationStepStarted))
  
  def activarCallbackStepDone(self):
    self.client.simxGetSimulationStepDone(self.sub(self.cb_simulationStepDone))
  
  def getTime(self):
    return self.simTime

  def init(self):
    pass

  def actuation(self):
    pass

  def sensing(self):
    pass

  def cleanup(self):
    pass

  def correrSim(self, tiempo=None, usarSim=False, pausar=False):
    if not self.inicializado:
      self.inicializar()

    # TODO: Implementar uso del tiempo de simulacion
    # if usarSim: # ...

    # Correr Sim
    self.simTime = 0
    self.client.simxStartSimulation(self.pub())
    if tiempo is None:
      tiempo = 1000
    startTime=time.time()
    
    self.init() # Analogo a sysCall_init()

    try:
      while time.time()<startTime+tiempo: 
          if self.doNextStep:
              self.doNextStep=False
              self.client.simxSynchronousTrigger()
              self.actuation() # Analogo a sysCall_actuation()
              self.sensing() # Analogo a sysCall_sensing()
          self.client.simxSpinOnce()
          r = self.client.simxGetSimulationState(self.call())
          if r[0] and r[1] ==0 : # Simulación detenida en Coppelia
            print("Detenida en el Simulador")
            break
    except Exception as e:
        self.client.simxStopSimulation(self.pub())
        raise(e)

    if pausar:
        self.client.simxPauseSimulation(self.pub())
        return 
    self.client.simxStopSimulation(self.pub())



