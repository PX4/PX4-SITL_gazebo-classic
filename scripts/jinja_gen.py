#!/usr/bin/env python3


import jinja2
import argparse
import os
import fnmatch
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', help="file that the sdf file should be generated from")
    parser.add_argument('env_dir')
    parser.add_argument('--mavlink_addr', default="INADDR_ANY", help="IP address for PX4 SITL")
    parser.add_argument('--mavlink_tcp_port', default=4560, help="TCP port for PX4 SITL")
    parser.add_argument('--mavlink_udp_port', default=14560, help="Mavlink UDP port for mavlink access")
    parser.add_argument('--serial_enabled', default=0, help="Enable Serial device for HITL")
    parser.add_argument('--serial_device', default="/dev/ttyACM0", help="Serial device for FMU")
    parser.add_argument('--serial_baudrate', default=921600, help="Baudrate of Serial device for FMU")
    parser.add_argument('--qgc_addr', default="INADDR_ANY", help="IP address for QGC")
    parser.add_argument('--hil_mode', default=0, help="Enable HIL mode for HITL simulation")
    parser.add_argument('--use_tcp', default=0, help="Use TCP instead of UDP for PX4 SITL")
    parser.add_argument('--gstudphost', default="127.0.0.1", help="udpHost for Gst Camera plugin")
    parser.add_argument('--output-file', help="sdf output file")
    parser.add_argument('--stdout', action='store_true', default=False, help="dump to stdout instead of file")
    args = parser.parse_args()
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(args.env_dir))
    template = env.get_template(os.path.relpath(args.filename, args.env_dir))

    # create dictionary with useful modules etc.
    try:
        import rospkg
        rospack = rospkg.RosPack()
    except ImportError:
        pass
        rospack = None

    d = {'np': np, 'rospack': rospack, \
         'mavlink_addr': args.mavlink_addr, \
         'mavlink_tcp_port': args.mavlink_tcp_port, \
         'mavlink_udp_port': args.mavlink_udp_port, \
         'serial_enabled': args.serial_enabled, \
         'serial_device': args.serial_device, \
         'serial_baudrate': args.serial_baudrate, \
         'qgc_addr': args.qgc_addr, \
         'hil_mode': args.hil_mode, \
         'use_tcp': args.use_tcp, \
         'gstudphost': args.gstudphost}

    result = template.render(d)
    if args.output_file:
        filename_out = args.output_file
    else:
        filename_out = args.filename.replace('.sdf.jinja', '.sdf')

    if args.stdout:
        print(result)
    else:
        with open(filename_out, 'w') as f_out:
            print(('{:s} -> {:s}'.format(args.filename, filename_out)))
            f_out.write(result)
