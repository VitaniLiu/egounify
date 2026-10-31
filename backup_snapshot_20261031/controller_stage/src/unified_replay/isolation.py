"""Process-local guard for offline replay and all mock workers."""
import os
from pathlib import Path
import sys


def guard(event, args):
    if event in ('socket.connect', 'socket.bind'):
        raise PermissionError('Offline replay cannot open network transports')
    if event == 'import' and args[0].split('.')[0] in ('rclpy', 'sharpa_driver', 'litchi_hardware'):
        raise PermissionError('ROS and hardware SDK imports forbidden in mock replay')
    if event == 'open' and isinstance(args[0], (str, bytes)):
        path = Path(os.fsdecode(args[0]))
        if path.suffix in ('.mcap', '.db3', '.h5', '.hdf5') or set(path.parts) & {'raw_data', 'gento_replay_bags', 'hdf5'}:
            raise PermissionError('Only LeRobot trajectory sources are allowed')
        if str(path).startswith('/dev/') and str(path) not in ('/dev/null', '/dev/urandom'):
            raise PermissionError('Hardware device access forbidden')


def install():
    sys.addaudithook(guard)
