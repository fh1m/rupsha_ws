"""urc_autonomy -- the URC Autonomous Navigation shape, driven by the executive.

Legs come from a YAML file so the field day changes coordinates, not code:

    legs:
      - {type: position, lat: ..., lon: ..., radius: 3}
      - {type: marker,   lat: ..., lon: ..., tag_id: 1, radius: 20}
      - {type: object,   lat: ..., lon: ..., target_name: mallet, radius: 20}
    obstacles:
      - {lat: ..., lon: ..., radius: 2}

    RUPSHA_LEGS=legs.yaml ros2 run rupsha_planner mission urc_autonomy
"""

import os
import time

import yaml


def run(rupsha, log):
    path = os.environ.get('RUPSHA_LEGS', '')
    if not path:
        log.error('set RUPSHA_LEGS to a legs YAML')
        return
    with open(path) as f:
        spec = yaml.safe_load(f)
    rupsha.require('arm', 'clear_legs', 'add_position_leg', 'add_marker_leg',
                   'add_object_leg', 'add_obstacle', 'start_autonomy')
    if not rupsha.arm().success:
        return
    rupsha.clear_legs()
    rupsha.clear_obstacles()
    for o in spec.get('obstacles', []):
        rupsha.add_obstacle(lat=o['lat'], lon=o['lon'], radius=o['radius'])
    for leg in spec['legs']:
        kind = leg['type']
        if kind == 'position':
            rupsha.add_position_leg(lat=leg['lat'], lon=leg['lon'], radius=leg.get('radius', 3.0))
        elif kind == 'marker':
            rupsha.add_marker_leg(lat=leg['lat'], lon=leg['lon'], tag_id=leg.get('tag_id', -1),
                                  radius=leg.get('radius', 20.0))
        elif kind == 'object':
            rupsha.add_object_leg(lat=leg['lat'], lon=leg['lon'], target_name=leg['target_name'],
                                  radius=leg.get('radius', 20.0))
        else:
            log.error(f'unknown leg type {kind!r}')
            return
    if rupsha.start_autonomy().success:
        log.info('autonomy started; watch /rupsha/autonomy_status')
        while True:
            time.sleep(1.0)
