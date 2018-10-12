#!/usr/bin/env python

import sys

sys.path.append(
    'PythonAPI/carla-0.9.0-py%d.%d-linux-x86_64.egg' % (sys.version_info.major,
                                                        sys.version_info.minor))

import carla

import math
import time

# Nice spot on Town01.
LOCATION = carla.Location(x=155.5, y=55.8, z=39)


def get_transform(angle, d=6.5):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + LOCATION
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))


def main():
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    spectator = world.get_spectator()
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle');

    for blueprint in vehicle_blueprints:
        transform = carla.Transform(LOCATION, carla.Rotation(yaw=-45.0))
        vehicle = world.spawn_actor(blueprint, transform)

        try:

            print(vehicle.type_id)
            for x in range(2, 360, 2):
                spectator.set_transform(get_transform(x - 90))
                time.sleep(0.02)

        finally:

            vehicle.destroy()


if __name__ == '__main__':

    main()
