#!/usr/bin/env python

import sys

sys.path.append(
    'PythonAPI/carla-0.9.0-py%d.%d-linux-x86_64.egg' % (sys.version_info.major,
                                                        sys.version_info.minor))

import carla

import os
import random
import time



# This function is here because this functionality haven't been ported to the
# new API yet.
def save_to_disk(image):
    """Save this image to disk (requires PIL installed)."""

    filename = '_images/{:0>6d}_{:s}.png'.format(image.frame_number, image.type)

    try:
        from PIL import Image as PImage
    except ImportError:
        raise RuntimeError(
            'cannot import PIL, make sure pillow package is installed')

    image = PImage.frombytes(
        mode='RGBA',
        size=(image.width, image.height),
        data=image.raw_data,
        decoder_name='raw')
    color = image.split()
    image = PImage.merge("RGB", color[2::-1])

    folder = os.path.dirname(filename)
    if not os.path.isdir(folder):
        os.makedirs(folder)
    image.save(filename)



def


def prepare_settings(sensor_type):

    if (sensor_type == "camera"):

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
    elif (sensor_type == "semantic_segmentation"):

        camera = carla.sensor.Camera('MyCamera', PostProcessing='SemanticSegmentation')
        camera.set(FOV=90.0)
        camera.set_image_size(800, 600)
        camera.set_position(x=0.30, y=0, z=1.30)
        camera.set_rotation(pitch=0, yaw=0, roll=0)

        carla_settings.add_sensor(camera)

        #CarlaSettings.ini

        #[CARLA/Sensor/MyCamera]
        #SensorType=CAMERA
        #PostProcessing=SemanticSegmentation
        #ImageSizeX=800
        #ImageSizeY=600
        #FOV=90
        #PositionX=0.30
        #PositionY=0
        #PositionZ=1.30
        #RotationPitch=0
        #RotationRoll=0
        #RotationYaw=0
    elif (sensor_type == "lidar"):

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

        #CarlaSettings.ini
        #[CARLA/Sensor/MyLidar]
        #SensorType=LIDAR_RAY_CAST
        #Channels=32
        #Range=50
        #PointsPerSecond=100000
        #RotationFrequency=10
        #UpperFOVLimit=10
        #LowerFOVLimit=-30
        #PositionX=0
        #PositionY=0
        #PositionZ=1.40
        #RotationPitch=0
        #RotationYaw=0
        #RotationRoll=0


def prepare_environment(add_a_camera, enable_autopilot):
    client = carla.Client('localhost', 2000)
    client.set_timeout(2000)

    print('client version: %s' % client.get_client_version())
    print('server version: %s' % client.get_server_version())

    world = client.get_world()

    blueprint_library = world.get_blueprint_library();

    vehicle_blueprints = blueprint_library.filter('vehicle');


    actor_list = []

    try:

        index = 0

        rendering = True
        while True:

            index = index + 1

            if ( rendering is True ):
                bp = random.choice(vehicle_blueprints)

                if bp.contains_attribute('number_of_wheels'):
                    n = bp.get_attribute('number_of_wheels')
                    print('spawning vehicle %r with %d wheels' % (bp.id, n))

                color = random.choice(bp.get_attribute('color').recommended_values)
                bp.set_attribute('color', color)

                transform = carla.Transform(
                    carla.Location(x=180.0, y=199.0, z=40.0),
                    carla.Rotation(yaw=0.0))

                vehicle = world.try_spawn_actor(bp, transform)

                if vehicle is None:
                    continue

                actor_list.append(vehicle)

                print(vehicle)

                if add_a_camera:
                    add_a_camera = False

                    camera_bp = blueprint_library.find('sensor.camera')
                    # camera_bp.set_attribute('post_processing', 'Depth')
                    camera_transform = carla.Transform(carla.Location(x=0.4, y=0.0, z=1.4))
                    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
                    camera.listen(save_to_disk)

                if enable_autopilot:
                    vehicle.set_autopilot()
                else:
                    vehicle.apply_control(carla.VehicleControl(throttle=1, steer=0.0))

                time.sleep(3)

                print('vehicle at %s' % vehicle.get_location())
                vehicle.set_location(carla.Location(x=0, y=0, z=0))
                print('is now at %s' % vehicle.get_location())

                time.sleep(2)
            if ( index > 3  ):
                rendering = False

    finally:

        for actor in actor_list:
            actor.destroy()


if __name__ == '__main__':

    prepare_environment(add_a_camera=True, enable_autopilot=True)

