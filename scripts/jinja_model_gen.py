#!/usr/bin/env python3
"""
Generate Models
@author: Benjamin Perseghetti
@email: bperseghetti@rudislabs.com
"""
import jinja2
import argparse
import os
import fnmatch
import numpy as np
import json

rel_px4_gazebo_path = ".."
rel_model_path ="../models"
script_path = os.path.realpath(__file__).replace("jinja_model_gen.py","")
default_env_path = os.path.relpath(os.path.join(script_path, rel_px4_gazebo_path))
default_model_path = os.path.relpath(os.path.join(script_path, rel_model_path))
json_path = os.path.relpath(os.path.join(script_path, "gen_params.json"))
default_sdf_dict = {
    "iris": 1.6,
    "plane": 1.5,
    "standard_vtol": 1.5,
    "r1_rover": 1.6,
    "cupcar": 1.5
}
hitl_base_model_list = ["iris", "plane", "standard_vtol"]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--json_gen', default=0, help="Use JSON (gen_params.json) for world params, on [1] or off [0]")
    parser.add_argument('--base_model', default="NotSet", help="Base model jinja file EX: iris")
    parser.add_argument('--sdf_version', default="NotSet", help="SDF format version to use for interpreting model file")
    parser.add_argument('--mavlink_tcp_port', default=4560, help="TCP port for PX4 SITL")
    parser.add_argument('--mavlink_udp_port', default=14560, help="Mavlink UDP port for mavlink access")
    parser.add_argument('--serial_enabled', default="NotSet", help="Enable serial device for HITL")
    parser.add_argument('--serial_device', default="/dev/ttyACM0", help="Serial device for FMU")
    parser.add_argument('--serial_baudrate', default=921600, help="Baudrate of Serial device for FMU")
    parser.add_argument('--enable_lockstep', default="NotSet", help="Enable lockstep for simulation")
    parser.add_argument('--hil_mode', default=0, help="Enable HIL mode for HITL simulation")
    parser.add_argument('--model_name', default="NotSet", help="Model to be used in jinja files")
    parser.add_argument('--output_path', help="Path for generated files")
    parser.add_argument('--config_file', default=0, help="Generate config file on [1] or off [0]")
    args = parser.parse_args()


    if args.json_gen:
        with open(json_path) as json_file:
            json_params = json.load(json_file)["model_params"]
        
        args.sdf_version=json_params["sdf_version"]
        if args.model_name == "NotSet":
            args.model_name=json_params["model_name"]
        if args.base_model == "NotSet":
            args.base_model=json_params["base_model"]
        if not args.output_path:
            args.output_path=json_params["output_path"]
            args.mavlink_tcp_port=json_params["mavlink_tcp_port"]
            args.mavlink_udp_port=json_params["mavlink_udp_port"]
        args.serial_enabled=json_params["serial_enabled"]
        args.serial_device=json_params["serial_device"]
        args.serial_baudrate=json_params["serial_baudrate"]
        args.enable_lockstep=json_params["enable_lockstep"]
        args.config_file=json_params["config_file"]
        args.hil_mode=json_params["hil_mode"]


    if args.base_model not in default_sdf_dict:
        print("\nWARNING!!!")
        print('Base model: "{:s}" DOES NOT MATCH any entries in default_sdf_dict.\nTry base model name:'.format(args.base_model))
        for model_option in default_sdf_dict:
            print('\t{:s}'.format(model_option))
        print("\nEXITING jinja_model_gen.py...\n")
        exit(1)

    if args.model_name == "NotSet":
        args.model_name = args.base_model
        print('Model name is NOT EXPLICITLY SET, setting to base model name: "{:s}"'.format(args.model_name))
    
    if args.sdf_version == "NotSet":
        args.sdf_version = default_sdf_dict.get(args.base_model)
        print('SDF version is NOT EXPLICITLY SET, base model name: "{:s}" is using default SDF version: {:s}'.format(args.base_model, str(args.sdf_version)))
        
    input_filename = os.path.relpath(os.path.join(default_model_path, '{:s}/{:s}.sdf.jinja'.format(args.base_model,args.base_model)))
    env = jinja2.Environment(loader=jinja2.FileSystemLoader(default_env_path))
    template_model = env.get_template(os.path.relpath(input_filename, default_env_path))
    
    if (args.base_model not in hitl_base_model_list) and args.hil_mode:
        print("\nWARNING!!!")
        print('Model name: "{:s}" DOES NOT MATCH any entries for HITL in hitl_base_model_list.\nTry HITL capable model name:'.format(args.model_name))
        for hitl_model_option in hitl_base_model_list:
            print('\t{:s}'.format(hitl_model_option))
        print("\nEXITING jinja_model_gen.py...\n")
        exit(1)

    if args.hil_mode:
        args.model_name='temp_{:s}_hitl'.format(args.model_name)
        args.config_file=1
    
    if args.serial_enabled=="NotSet":
        if args.hil_mode:
            args.serial_enabled=1
        else:
            args.serial_enabled=0

    if args.enable_lockstep=="NotSet":
        if args.hil_mode:
            args.enable_lockstep=0
        else:
            args.enable_lockstep=1

    if args.config_file: 
        input_config = os.path.relpath(os.path.join(script_path, "model.config.jinja"))
        template_config = env.get_template(os.path.relpath(input_config, default_env_path))

    try:
        import rospkg
        rospack = rospkg.RosPack()
    except ImportError:
        pass
        rospack = None

    d = {'np': np, 'rospack': rospack, \
         'sdf_version': args.sdf_version, \
         'mavlink_tcp_port': args.mavlink_tcp_port, \
         'mavlink_udp_port': args.mavlink_udp_port, \
         'serial_enabled': args.serial_enabled, \
         'serial_device': args.serial_device, \
         'serial_baudrate': args.serial_baudrate, \
         'enable_lockstep': args.enable_lockstep, \
         'model_name': args.model_name, \
         'hil_mode': args.hil_mode}

    model_result = template_model.render(d)
    if args.output_path and not args.hil_mode:
        model_out = os.path.relpath(os.path.join(args.output_path, '{:s}.sdf'.format(args.model_name)))
    else:
        if args.hil_mode:
            print(default_model_path)
            print(args.model_name)
            rel_hitl_path = os.path.relpath(os.path.join(default_model_path, args.model_name))
            print(rel_hitl_path)
            if not os.path.exists(rel_hitl_path):
                os.makedirs(rel_hitl_path, exist_ok=True)
            model_out = os.path.relpath(os.path.join(rel_hitl_path, '{:s}.sdf'.format(args.model_name)))
        else:
            model_out = input_filename.replace('.sdf.jinja', '.sdf')

    with open(model_out, 'w') as m_out:
        print(('{:s} -> {:s}'.format(input_filename, model_out)))
        m_out.write(model_result)
    

    if args.config_file:
        config_result = template_config.render(d)
        if args.output_path and not args.hil_mode:
            config_out = os.path.relpath(os.path.join(args.output_path, 'model.config'))
        else:
            if args.hil_mode:
                rel_hitl_path = os.path.relpath(os.path.join(default_model_path, args.model_name))
                if not os.path.exists(rel_hitl_path):
                    os.makedirs(rel_hitl_path, exist_ok=True)
                config_out = os.path.relpath(os.path.join(rel_hitl_path, 'model.config'))
            else:
                config_out = input_filename.replace('{:s}.sdf.jinja'.format(args.base_model), 'model.config')

        with open(config_out, 'w') as c_out:
            print(('{:s} -> {:s}'.format(input_config, config_out)))
            c_out.write(config_result)
