#!/usr/bin/env python3

import argparse
import nexstar
import sys

# wraps an angle in degrees to the range [-180,+180)
def wrap_error(e):
    return (e + 180.0) % 360.0 - 180.0

parser = argparse.ArgumentParser()
parser.add_argument('--scope', help='serial device for connection to telescope', default='/dev/ttyUSB0')
parser.add_argument('--axis', help='axis to test (az or alt)')
parser.add_argument('--slew-rate', help='slew rate (arcseconds/second)', default=1.0, type=float)
args = parser.parse_args()

NUM_DELTAS = 10

if args.axis not in ['az', 'alt']:
    print('axis argument must be "az" or "alt"')
    sys.exit(1)

mount = nexstar.NexStar(args.scope)

print('Slewing at ' + str(args.slew_rate) + ' arcseconds per second in azimuth')
mount.slew_var(args.axis, args.slew_rate)

position_start = mount.get_azalt()
position_prev = position_start
deltas = []
while True:
    position = mount.get_azalt()
    if args.axis == 'az':
        position_change = wrap_error(position[0] - position_start[0]) * 3600.0
        position_delta = wrap_error(position[0] - position_prev[0]) * 3600.0
    else:
        position_change = wrap_error(position[1] - position_start[1]) * 3600.0
        position_delta = wrap_error(position[1] - position_prev[1]) * 3600.0
    position_prev = position
    if position_delta != 0:
        deltas.append(position_delta)
    print('Position change, delta (arcseconds): ' + str(position_change) + ', ' + str(position_delta))
    if len(deltas) >= NUM_DELTAS:
        break

print('\nDeltas for ' + str(args.axis) + ' axis:')
for delta in deltas:
    print(str(delta))