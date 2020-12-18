function sysCall_init()
  MR = sim.getObjectHandle("MR")
  ML = sim.getObjectHandle("ML")
  B = sim.getObjectHandle("Bombillo")  

  Fr = sim.getObjectHandle("Vision_Frente")
  Iz = sim.getObjectHandle("Vision_Izq")
  Dr = sim.getObjectHandle("Vision_Der")
  
  -- Memorias
  t_inicio = 0
  t_espera = 0
  
  estado = "inicio"
  
  local D = 1.6881e-02
  R = D/2
  
  v= 0.01 --m/s
  w = v/R
  
  -- Hue del color deseado
  Azul = 240
  Verde = 120
  Amarillo = 60
  Rojo = 0
  
  threshold = 10 -- maxima distancia del color
  color = Verde
  
end


function esperar(tiempo)
  -- Activa la memoria para esperar un tiempo
  -- usando esperando()

  t_espera = tiempo
  t_inicio = sim.getSimulationTime()
end

function esperando()
  -- Espera a  que haya pasado el tiempo
  -- marcado en t_espera

  -- Devuelve:
  -- True si está en proceso de espera
  -- False si ya no se está esperando

  -- Si no han activado la espera, salir
  if t_espera == 0 then
    return false
  end

  -- Calcula el tiempo que ha pasado
  t = sim.getSimulationTime()
  delta = t - t_inicio

  if delta >= t_espera then
      -- Reiniciar memorias
      t_inicio = 0
      t_espera = 0

      return false
  end

  return true
end


function sysCall_actuation()

 --if not esperando () then
 -- sim.setJointTargetVelocity(MR,0.4)
 -- sim.setJointTargetVelocity(ML,0.4)
 --end
 
    if estado == "inicio" then
      esperar(3)
      estado = "iniciando"
      
    elseif estado == "iniciando" then

      if not esperando() then
        estado = "arrancando"
        esperar(2)
      end

    elseif estado == "arrancando" then
      avanzar(w)
      
      --if not esperando() then
        
      --end
      
      --[[if frente() then
        estado = "detener"
      end
      --]]

    else
      avanzar(0) -- detener
    end
 
end

function sysCall_sensing()
  -- put your sensing code here


end

function sysCall_cleanup()
  -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
function avanzar(vel)

  sim.setJointTargetVelocity(MR,vel)
  sim.setJointTargetVelocity(ML,vel)
end

function leerSonar(disp)
  -- Devuelve la distancia desde el sonar 
  local r, d = sim.readProximitySensor(disp)
  local pista = nil
  
  if r == 1 then
      return d
  else
      return -1
  end
  
end 

function ms_a_rads(vel)
  -- convierte de velocidad lineal a velocidad angular
  
  local omega = vel/Radio
  return omega
end


--[===[
function frente()
    return color_detectado(extraer_hue(Fr), color)
end

function izq()
    return color_detectado(extraer_hue(Iz), color)
end

function der()
    return color_detectado(extraer_hue(Dr), color)
end

function leerSensorVis(sen)
    r, d = sim.readVisionSensor(sen)
    
    if r then
        local int = d[11]
        local hue = rgb2hue(d[12],d[13],d[14],true)
        return int,hue
    else 
        return 0,0
    end
end



function extraer_hue(sen)
    local a,b = leerSensorVis(sen)
    
    if a>0 then
        return b
    end
    
    return 0
end
function color_detectado(hue, col)
    -- Decidir si vemos el color que esperamos
    
    local diff = col - hue
    
    if col ~= Red then
        
        if math.abs(diff) <= threshold then
            return true
        end
    else
        -- Si rojo, centrar en 180
        diff = 180-math.abs(diff+180)
        if diff <= threshold then
            return true
        end
    end
    
    return false
    
end

function rgb2hue(r,g,b, degs)
        degs = degs or false -- Default parameter
        -- Prestado de https://www.geeksforgeeks.org/program-change-rgb-color-model-hsv-color-model/
        --h = hue 
        local cmax = math.max(r, math.max(g, b)); -- maximum of r, g, b 
        local cmin = math.min(r, math.min(g, b)); -- minimum of r, g, b 
        local diff = cmax - cmin; -- diff of cmax and cmin. 
        local h = -1 
          
        -- if cmax and cmax are equal then h = 0 
        if (cmax == cmin) then
            h = 0; 
  
        -- if cmax equal r then compute h 
        elseif (cmax == r) then
            h = (60 * ((g - b) / diff) + 360) % 360; 
  
        -- if cmax equal g then compute h 
        elseif (cmax == g) then
            h = (60 * ((b - r) / diff) + 120) % 360; 
  
        -- if cmax equal b then compute h 
        elseif (cmax == b) then
            h = (60 * ((r - g) / diff) + 240) 
        end
        
        if degs then
            return h
        else
            return h/360
        end
end

--]===]