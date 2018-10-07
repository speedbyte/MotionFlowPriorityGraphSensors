# coding=utf-8

import carla_client as carla
import datetime

print dir(datetime)
print dir(carla)

#Camera

for i in dir(carla):
    print i,"  ",type(getattr(carla,i))

camera = carla.sensor.Camera('MyCamera', PostProcessing='SceneFinal')
camera.set(FOV=90.0)
camera.set_image_size(800, 600)
camera.set_position(x=0.30, y=0, z=1.30)
camera.set_rotation(pitch=0, yaw=0, roll=0)

carla_settings.add_sensor(camera)

#[CARLA/Sensor/MyCamera]
#SensorType=CAMERA
#PostProcessing=SceneFinal
#ImageSizeX=800
#ImageSizeY=600
#FOV=90
#PositionX=0.30
#PositionY=0
#PositionZ=1.30
#RotationPitch=0
#RotationRoll=0
#RotationYaw=0


