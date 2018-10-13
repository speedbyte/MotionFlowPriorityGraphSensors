#!/usr/bin/env python

import sys

sys.path.append(
    'PythonAPI/carla-0.9.0-py%d.%d-linux-x86_64.egg' % (sys.version_info.major,
                                                        sys.version_info.minor))
sys.path.append('client_files')

import carla
import sensor
import settings as client_settings
import client
#from carla import sensor

#from util import make_connection


import os
import random
import time

import logging

import shutil

try:
    import pygame
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
START_POSITION = carla.Transform(carla.Location(x=180.0, y=199.0, z=40.0))
MAX_ENVIRONMENT_ACTORS = 3

class CarlaWrapper(object):
    def __init__(self, add_a_camera, enable_autopilot):

        self.rendering = True
        logging.info('connecting...')

        #self._client = client.CarlaClient('localhost', 2000)
        #self._client.connect()
        #print("client connected is " + str(self._client.connected()))

        self._client = carla.Client('localhost', 2000)
        self._client.set_timeout(2000)
        print('client version: %s' % self._client.get_client_version())
        print('server version: %s' % self._client.get_server_version())

        logging.info('listening to server %s:%s', 'localhost', 2000)


        self._display = None
        self._surface = None
        self._camera = None
        self._vehicle = None
        self._autopilot_enabled = enable_autopilot
        self.add_a_camera = add_a_camera
        self._is_on_reverse = False
        self.pygame_initalised = False
        self.kill_script = False


    def initialise_pygame(self):
        pygame.init()
        try:
            self._display = pygame.display.set_mode(
                (WINDOW_WIDTH, WINDOW_HEIGHT),
                pygame.HWSURFACE | pygame.DOUBLEBUF)
            logging.debug('pygame started')
            self.pygame_initalised = True

        except:
            self.kill_script = True

    # This function is here because this functionality haven't been ported to the
    # new API yet.
    def save_to_disk(self, image_raw):
        """Save this image to disk (requires PIL installed)."""

        filename = '_images/{:0>6d}_{:s}.png'.format(image_raw.frame_number, image_raw.type)

        try:
            from PIL import Image as PImage
        except ImportError:
            raise RuntimeError(
                'cannot import PIL, make sure pillow package is installed')

        image = PImage.frombytes(
            mode='RGBA',
            size=(image_raw.width, image_raw.height),
            data=image_raw.raw_data,
            decoder_name='raw')
        color = image.split()
        image = PImage.merge("RGB", color[2::-1])

        image.save(filename)
        if ( self.pygame_initalised is True ):
            self._parse_image(image_raw)


    def _parse_image(self, image_raw):
        array = np.frombuffer(image_raw.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image_raw.height, image_raw.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))


    def prepare_environment(self):

        # not CarlaClient

        world = self._client.get_world()

        blueprint_library = world.get_blueprint_library();
        vehicle_blueprints = blueprint_library.filter('vehicle');

        cam_blueprint = world.get_blueprint_library().find('sensor.camera')
        camera_transform_position = carla.Transform(carla.Location(x=1.4, y=0.0, z=1.4))

        actor_list = []

        print(__doc__)

        try:
            index = 0
            while True:

                if ( self.kill_script is True  ):
                    break

                time.sleep(0.1)

                if ( self.rendering is True ):
                    #choose any vehicle
                    bp = random.choice(vehicle_blueprints)
                    bp_ego = vehicle_blueprints.find('ford.mustang')

                    if bp.contains_attribute('number_of_wheels'):
                        n = bp.get_attribute('number_of_wheels')
                        print('spawning vehicle %r with %d wheels' % (bp.id, n))

                    color = random.choice(bp.get_attribute('color').recommended_values)
                    bp.set_attribute('color', color)

                    if ( index <  MAX_ENVIRONMENT_ACTORS  ):
                        transform = carla.Transform(
                            carla.Location(x=300.0, y=199.0, z=40.0),
                            carla.Rotation(yaw=0.0))
                        vehicle = world.try_spawn_actor(bp, transform)
                        if self._autopilot_enabled:
                            vehicle.set_autopilot()
                        else:
                            vehicle.apply_control(carla.VehicleControl(throttle=1, steer=0.0))
                    else:
                        transform = carla.Transform(
                            carla.Location(x=335.0, y=179.0, z=40.0),
                            carla.Rotation(yaw=90.0))
                        vehicle = world.try_spawn_actor(bp_ego, transform)
                        self.ego_vehicle = vehicle
                        vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0.0))

                    # the last vehicle is ego vehicle
                    if ( index == MAX_ENVIRONMENT_ACTORS  ):
                        if self.add_a_camera:
                            try:
                                self._camera = world.spawn_actor(cam_blueprint, camera_transform_position, attach_to=self.ego_vehicle)
                                #self._camera.listen(lambda image: self._parse_image(image))
                                self._camera.listen(lambda image: self.save_to_disk(image))

                            except:
                                if self._camera is not None:
                                    self._camera.destroy()
                                    self._camera = None
                                if self._vehicle is not None:
                                    self._vehicle.destroy()
                                    self._vehicle = None

                            self.add_a_camera = False
                            #initialize pygame window to show the camera window
                            self.initialise_pygame()

                    if vehicle is None:
                        continue
                    actor_list.append(vehicle)
                    print(vehicle)


                    time.sleep(3)

                    print('vehicle at %s' % vehicle.get_location())

                    time.sleep(2)

                # successfully spawned a vehicle
                index = index + 1
                if ( index > MAX_ENVIRONMENT_ACTORS  ):
                    self.rendering = False

                if self.rendering is False and self.pygame_initalised is True:
                    try:
                        for event in pygame.event.get():
                            if event.type == pygame.QUIT:
                                return
                        #self._on_loop()
                        self._on_render()
                    except Exception as error:
                        logging.error(error)
                        time.sleep(1)
            if ( self.kill_script is True ):
                raise

        finally:
            pygame.quit()
            for actor in actor_list:
                actor.destroy()
            if self._camera is not None:
                self._camera.destroy()
                self._camera = None
            if self._vehicle is not None:
                self._vehicle.destroy()
                self._vehicle = None


    def set_new_vehicle_position(self, vehicle):

        vehicle.set_location(carla.Location(x=220, y=199, z=38))
        print('is now at %s' % vehicle.get_location())


    def _on_loop(self):
        autopilot = self._autopilot_enabled
        control = self._get_keyboard_control(pygame.key.get_pressed())
        if autopilot != self._autopilot_enabled:
            self._vehicle.set_autopilot(autopilot)
            self._autopilot_enabled = autopilot
        if not self._autopilot_enabled:
            self._vehicle.apply_control(control)


    def _get_keyboard_control(self, keys):
        control = carla.VehicleControl()
        if keys[K_LEFT] or keys[K_a]:
            control.steer = -1.0
        if keys[K_RIGHT] or keys[K_d]:
            control.steer = 1.0
        if keys[K_UP] or keys[K_w]:
            control.throttle = 1.0
        if keys[K_DOWN] or keys[K_s]:
            control.brake = 1.0
        if keys[K_SPACE]:
            control.hand_brake = True
        if keys[K_q]:
            self._is_on_reverse = not self._is_on_reverse
        if keys[K_p]:
            self._autopilot_enabled = not self._autopilot_enabled
        control.reverse = self._is_on_reverse
        return control

    def _on_render(self):
        if self._surface is not None:
            self._display.blit(self._surface, (0, 0))
        pygame.display.flip()


    def disconnect_world_client(self):
        self._client.disconnect()

    def prepare_settings(self, sensor_type):

        settings = client_settings.CarlaSettings()
        settings.set(SendNonPlayerAgentsInfo=True, SynchronousMode=True)
        settings.randomize_seeds()

        if (sensor_type == "camera"):

            camera = sensor.Camera('DefaultCamera', PostProcessing='SceneFinal')
            camera.set(FOV=90.0)
            camera.set_image_size(WINDOW_WIDTH, WINDOW_HEIGHT)
            camera.set_position(x=0.30, y=0, z=1.30)
            camera.set_rotation(pitch=0, yaw=0, roll=0)

            settings.add_sensor(camera)

            logging.debug('sending CarlaSettings:\n%s', settings)
            logging.info('new episode requested')

            scene = self._client.load_settings(settings)

            #carla_settings.add_sensor(camera)
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
        elif (sensor_type == "se"):

            camera = carla.sensor.Camera('MyCamera', PostProcessing='SemanticSegmentation')
            camera.set(FOV=90.0)
            camera.set_image_size(800, 600)
            camera.set_position(x=0.30, y=0, z=1.30)
            camera.set_rotation(pitch=0, yaw=0, roll=0)

            #carla_settings.add_sensor(camera)

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

            lidar = sensor.Lidar('DefaultLidar')
            lidar.set(
                Channels=32,
                Range=50,
                PointsPerSecond=100000,
                RotationFrequency=10,
                UpperFovLimit=10,
                LowerFovLimit=-30)
            lidar.set_position(x=0, y=0, z=1.40)
            lidar.set_rotation(pitch=0, yaw=0, roll=0)
            settings.add_sensor(lidar)

            #carla_settings.add_sensor(lidar)

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



if __name__ == '__main__':



    log_level = logging.NOTSET
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
    #log.setLevel(log_level)

    # Remove directory
    filename_dummy = '_images/abcd.png'
    folder = os.path.dirname(filename_dummy)

    if ( os.path.exists(folder)):
        print("removing folder " + folder)
        shutil.rmtree(folder)
    print("creating folder " + folder)
    os.makedirs(folder)

    try:
        carla_wrapper = CarlaWrapper(add_a_camera=True, enable_autopilot=True)
        #carla_wrapper.prepare_settings("camera")
        carla_wrapper.prepare_environment()

    except Exception as exception:
        print(str(exception))

    finally:
        pass
        #carla_wrapper.disconnect_world_client()



