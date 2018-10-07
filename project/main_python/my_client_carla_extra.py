# coding=utf-8



# Depth
camera = carla.sensor.Camera('MyCamera', PostProcessing='Depth')
camera.set(FOV=90.0)
camera.set_image_size(800, 600)
camera.set_position(x=0.30, y=0, z=1.30)
camera.set_rotation(pitch=0, yaw=0, roll=0)

carla_settings.add_sensor(camera)


[CARLA/Sensor/MyCamera]
SensorType=CAMERA
PostProcessing=Depth
ImageSizeX=800
ImageSizeY=600
FOV=90
PositionX=0.30
PositionY=0
PositionZ=1.30
RotationPitch=0
RotationRoll=0
RotationYaw=0



camera = carla.sensor.Camera('MyCamera', PostProcessing='SemanticSegmentation')
camera.set(FOV=90.0)
camera.set_image_size(800, 600)
camera.set_position(x=0.30, y=0, z=1.30)
camera.set_rotation(pitch=0, yaw=0, roll=0)

carla_settings.add_sensor(camera)

CarlaSettings.ini

[CARLA/Sensor/MyCamera]
SensorType=CAMERA
PostProcessing=SemanticSegmentation
ImageSizeX=800
ImageSizeY=600
FOV=90
PositionX=0.30
PositionY=0
PositionZ=1.30
RotationPitch=0
RotationRoll=0
RotationYaw=0


lidar = carla.sensor.Lidar('MyLidar')
lidar.set(
    Channels=32,
    Range=50,
    PointsPerSecond=100000,
    RotationFrequency=10,
    UpperFovLimit=10,
    LowerFovLimit=-30)
lidar.set_position(x=0, y=0, z=1.40)
lidar.set_rotation(pitch=0, yaw=0, roll=0)

carla_settings.add_sensor(lidar)

CarlaSettings.ini

[CARLA/Sensor/MyLidar]
SensorType=LIDAR_RAY_CAST
Channels=32
Range=50
PointsPerSecond=100000
RotationFrequency=10
UpperFOVLimit=10
LowerFOVLimit=-30
PositionX=0
PositionY=0
PositionZ=1.40
RotationPitch=0
RotationYaw=0
RotationRoll=0
