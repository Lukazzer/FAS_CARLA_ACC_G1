import carla
import time
import math
import pygame
import argparse

#Fesnter für Infos
pygame.init()
screen = pygame.display.set_mode((700, 150))  
pygame.display.set_caption("CARLA Infos")
myfont = pygame.font.SysFont("monospace", 20)
font = pygame.font.Font(None, 36)


#Variablen für Einstellung des Abstandes, der vom fahrer geändert werden kann
lowDistance = True
mediumDistance = False
highDistance = False


last_time = time.time()
distance = 0
current_speed = 0
max_speed_mps = 0


# Alle Fahrzeuge zerstören
def destroy_vehicles():
    actors = world.get_actors()
    vehicles = actors.filter('vehicle.*')
      
    for vehicle in vehicles:
        vehicle.destroy()      

def steer_vehicle(vehicle, steerFloat):
    vehicle.apply_control(carla.VehicleControl(steer=steerFloat))

def brake_vehicle(vehicle, brakeFloat):
    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=brakeFloat))

def cancelBrake_vehicle(vehicle):
    vehicle.apply_control(carla.VehicleControl(brake=0.0))
    
def speedUp_vehicle(vehicle, throttle_value):
    vehicle.apply_control(carla.VehicleControl(throttle=throttle_value, steer=0.0, brake=0.0))


client = carla.Client('localhost', 2000)
client.set_timeout(10.0)


world = client.get_world()
blueprint_library = world.get_blueprint_library()

# Wetter einstellen
weather = carla.WeatherParameters(
    cloudiness=70.0,    # Viele Wolken
    precipitation=0.0,      # Kein Regen
    sun_azimuth_angle=33.0,  # Sonne im Osten
    sun_altitude_angle=33.0   # Sonne hoch am Himmel
)


world.set_weather(weather)

#Alle Autos von der Map löschen
destroy_vehicles()

# Fahrzeug erstellen und spawnen
vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
spawn_point = world.get_map().get_spawn_points()[120]
#spawn_point.rotation.yaw += 180 
#Bei Town10
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
steer_vehicle(vehicle, 0.0)

radar_bp = blueprint_library.find('sensor.other.radar')
radar_location = carla.Transform(carla.Location(x=2.3, z=0.5))
radar = world.spawn_actor(radar_bp, radar_location, attach_to=vehicle)

# Kamera auf das Fahrzeug setzen
spectator = world.get_spectator()
transform = vehicle.get_transform()

# Zweites Fahrzeug 20 Meter vor dem ersten Fahrzeug spawnen
# Bei TOWN10 ---> ..x +20
second_spawn_point = carla.Transform(
    carla.Location(x=spawn_point.location.x - 20, y=spawn_point.location.y, z=spawn_point.location.z),
    spawn_point.rotation)

second_vehicle_bp = blueprint_library.find('vehicle.audi.a2')  # Beispiel eines zweiten Fahrzeugmodells
second_vehicle = world.spawn_actor(second_vehicle_bp, second_spawn_point)


target_distance = 0

# Daten des vorausfahrenden Fahrzeugs
detected_vehicle = {
    'distance': 0, 
    'relative_velocity': 0 
}

# sensor registrieren, um die daten zu bekommen
radar.listen(lambda radar_data: radar_callback(radar_data))

#Display schwarz machen
def noRendering(activateRendering):
    argparser = argparse.ArgumentParser(
    description=__doc__)
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        default=False,
        help='Activate no rendering mode')

    args = argparser.parse_args()
    settings = world.get_settings()

    if args.no_rendering:
        settings.no_rendering_mode = True
    else:
        settings.no_rendering_mode = False
        
    if(activateRendering == True):
        settings.no_rendering_mode = True
    world.apply_settings(settings)  

#Info-Bildschirm in einem zweiten Fenster
def setText():
    text = f"Geschwindigkeit: {(current_speed*3.6):06.2f}  Abstand: {distance:06.2f} Modus:"    
    if lowDistance == True:
        text = text + " Low"
    if mediumDistance == True:
        text = text + " Medium"
    if highDistance == True:
        text = text + " High"        
            
    screen.fill((0, 0, 0))        
    text_surface = myfont.render(text, True, (255, 255, 0))  
    screen.blit(text_surface, (20, 20))  
    pygame.display.flip()
    
def listenKeys():
    global lowDistance, mediumDistance, highDistance
    for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    print("Fahrzeug 2 bremsen")
                    brake_vehicle(second_vehicle, 1.0)
                if event.key == pygame.K_1:
                    lowDistance = True
                    mediumDistance = False
                    highDistance = False
                if event.key == pygame.K_2:
                    lowDistance = False
                    mediumDistance = True
                    highDistance = False
                if event.key == pygame.K_3:
                    lowDistance = False
                    mediumDistance = False
                    highDistance = True     
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_SPACE:
                    print("Bremse von Fahrzeug 2 aufgehoben")
                    speedUp_vehicle(second_vehicle, 0.9)
                    
                    
def setCorrectDistance():
    global target_distance
    if lowDistance == True:
        target_distance = 20
    if mediumDistance == True:
        target_distance = 40
    if highDistance == True:
        target_distance = 60  
                    
def setSpecatator():
    transform = vehicle.get_transform()    
    camera_transform = carla.Transform(
    transform.location + carla.Location(x=+14, z=10, y= +5), 
    carla.Rotation(pitch=-25, yaw=transform.rotation.yaw + 0))
    #transform.location + carla.Location(x=-14, z=10, y= -5), carla.Rotation(pitch=-25))
    # Spectator auf die neue Position setzen
    spectator.set_transform(camera_transform)
    
# PID-Regler für die Geschwindigkeitsregelung
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        # Proportionaler Anteil
        proportional = self.Kp * error

        # Integraler Anteil
        self.integral += error * dt
        integral = self.Ki * self.integral

        # Differenzialer Anteil
        derivative = self.Kd * (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error

        return proportional + integral + derivative    

# PID-Regler initialisieren
pid_controller = PIDController(Kp=0.4, Ki=0.02, Kd=0.6)

def radar_callback(radar_data):
    
    
    global distance, detected_vehicles
    # den ort des sensors farblich markieren
    radar_transform = radar.get_transform()
    world.debug.draw_point(
        radar_transform.location,  # position des sensors
        size=0.1,
        color=carla.Color(0, 255, 0),  # grün für den sensor
        life_time=0.1  # die zeit, wie lange die markierung sichtbar bleibt
    )

    # daten für jedes erkannte objekt durchgehen
    for detection in radar_data:
        #winkel
        azimuth = detection.azimuth
        altitude = detection.altitude
        depth = detection.depth  # abstand zum objekt
        relative_velocity = detection.velocity  # relative geschwindigkeit zum objekt
        
        max_azimuth_angle = 0.01
        max_elevation_angle = 0.01
        max_depth = 500

        # wir ignorieren objekte, die stillstehen (relative_velocity = 0)
        if abs(relative_velocity) < 0.1:  # kleine schwellenwerte für genauigkeit
            continue

        # fahrzeuge bewegen sich, daher interessieren uns objekte mit bewegung
        if -max_azimuth_angle < azimuth < max_azimuth_angle and depth <= max_depth and -max_elevation_angle < detection.altitude < max_elevation_angle:  # winkel begrenzt auf ±0.1 rad und max. 25 meter
            # position des erkannten objekts berechnen
            x = depth * math.cos(altitude) * math.cos(azimuth)
            y = depth * math.cos(altitude) * math.sin(azimuth)
            z = depth * math.sin(altitude)
            detection_location = radar.get_transform().transform(carla.Location(x=x, y=y, z=z))

            # markiere das erkannte objekt als roten punkt
            world.debug.draw_point(
                detection_location,
                size=0.1,
                color=carla.Color(255, 0, 0),  # rot für erkannte objekte
                life_time=0.1
            )

            # zeichne eine linie vom radar zum erkannten objekt
            world.debug.draw_line(
                radar_transform.location,  # vom sensor
                detection_location,  # zum erkannten objekt
                thickness=0.05,
                color=carla.Color(0, 0, 255),  # blau für die linie
                life_time=0.1
            )

            detected_vehicle['distance'] = depth
            detected_vehicle['relative_velocity'] = relative_velocity
                   
            distance = depth
                     
            
# Eingabe der maximalen Geschwindigkeit ermöglichen
def get_max_speed():
    while True:
        try:
            max_speed_kmh = float(input("Gib die maximale Geschwindigkeit in km/h ein (z.B. 140): "))
            if max_speed_kmh > 0:
                return max_speed_kmh
            else:
                print("Bitte eine positive Zahl eingeben.")
        except ValueError:
            print("Ungültige Eingabe. Bitte eine Zahl eingeben.")
                     
                        
def setVehicleSpeed():
    global last_time, current_speed, max_speed_mps
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    
    
    error = (distance - target_distance) + detected_vehicle['relative_velocity']
 
    # Aktuelle Geschwindigkeit des Fahrzeugs holen
    vehicle_velocity = vehicle.get_velocity()
    current_speed = math.sqrt(vehicle_velocity.x ** 2 + vehicle_velocity.y ** 2 + vehicle_velocity.z ** 2)

    
    control = carla.VehicleControl()
    
    # Throttle- und Brake-Kontrolle
    if error > 0:  # Fahrzeug ist zu weit weg, also beschleunigen
        control.throttle = 1.0
        control.brake = 0
    else:  # Fahrzeug ist zu nah, also bremsen
        control.throttle = 0
        control.brake = 1.0

    control.steer = 0.0  
    vehicle.apply_control(control)


def setVehicleSpeedPID():
    global last_time, current_speed
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    
    
    error = (distance - target_distance) + detected_vehicle['relative_velocity']
    
    # Kleinere Fehler (unter 1 Meter) ignorieren, um ständiges Hin-und-Her-Bewegen zu vermeiden
    if abs(error) > 1.0:  
        adjustment = pid_controller.update(error, dt)
    else:
        adjustment = 0  

    # Aktuelle Geschwindigkeit des Fahrzeugs berechnen (in m/s)
    vehicle_velocity = vehicle.get_velocity()
    current_speed = math.sqrt(vehicle_velocity.x ** 2 + vehicle_velocity.y ** 2 + vehicle_velocity.z ** 2)
    
    # Maximalgeschwindigkeit festlegen (20 km/h umgerechnet in m/s)
    max_speed_kmh = 100
    max_speed_mps = max_speed_kmh / 3.6  

    # Neue Zielgeschwindigkeit durch Daten des PID-Regler anpassen
    new_speed = current_speed + adjustment
    
    # Zielgeschwindigkeit begrenzen, damit das Fahrzeug nicht zu schnell fährt
    new_speed = max(0, min(new_speed, max_speed_mps))

    # Fahrzeugsteuerung initialisieren
    control = carla.VehicleControl()

    # Wenn das vorausfahrende Fahrzeug langsamer ist, passe die Geschwindigkeit darauf an
    target_speed = max(0, detected_vehicle['relative_velocity'])

    # Wenn wir zu nah am vorausfahrenden Fahrzeug sind, reduziere die Geschwindigkeit auf deren Niveau
    if distance < target_distance:
        new_speed = target_speed  
    
    # Wenn der Abstand zu gering ist, bremsen
    if distance < target_distance:
        control.throttle = 0  # Kein Gas geben
        control.brake = min(1.0, -adjustment / max_speed_mps)  # Bremsen entsprechend dem Fehler

    # Wenn der Abstand groß genug ist, beschleunigen
    else:
        control.throttle = min(1.0, new_speed / max_speed_mps)  # Gas proportional zur Zielgeschwindigkeit
        control.brake = 0 if adjustment > 0 else min(1.0, -adjustment / max_speed_mps)  # Leichtes Bremsen, falls nötig

    # Steuerung anwenden (Gaspedal, Bremse und Lenkrad)
    control.steer = 0.0  # Keine Lenkbewegung
    vehicle.apply_control(control)

  
try:
    
    #max_speed_kmh = get_max_speed()
    #max_speed_mps = max_speed_kmh / 3.6  # Umrechnung in m/s
    
    #print(max_speed_mps)
    noRendering(False)
    #setSpecatator() 
    
    # Fahrzeug 2 fahren lassen
    speedUp_vehicle(second_vehicle, 0.5)  # throttle_value = 0.5
    while True:
               
        listenKeys()
        setCorrectDistance()              
        setVehicleSpeed()  
        #setVehicleSpeedPID()         
        setSpecatator()                               
        setText()                   
        #time.sleep(0.01)

finally:
    noRendering(True)
    vehicle.destroy()
    second_vehicle.destroy()
    radar.destroy()
    pygame.quit()
