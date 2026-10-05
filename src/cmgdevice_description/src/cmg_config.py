#!/usr/bin/python3

import sys, os
import numpy as np
from pathlib import Path

# *****************************************************************************
# **                                                                         **
# **                     User defined                                        **
# **                                                                         **
# *****************************************************************************

# YAML config file prefix name
yaml_prefix = 'initial_CMG_prototype'

# Aluminum material density est in Kg/M^3
density_Al =2700
density_disk = density_Al
density_annulus = density_Al

# flywheel parameters (inches):
disk_rad = 3.5
disk_thick = 0.5
annulus_radout = 3.5
annulus_radin = 3.0
annulus_thick = 0.5

# motor parameters
flywheel_servo_mass = 0.559 # Kg mass
flywheel_max_speed = 5670 # RPM
flywheel_Kv = 270 # RPM/Volt, battery 18.5V nominal, 21 V max full charge
flywheel_Kt = 0.000525 # N*m/Amp, torque constant, 1/Kv, 1/(Kv*2*pi/60) = 0.000525 N*m/Amp
flywheel_servo_diam = 2.48031 # inches, 63.0 mm
flywheel_servo_length = 2.027559 # inches, 51.5 mm, 2.027559 in
torque_servo_mass = 1.089 # Kg mass
torque_servo_Tnominal = 11.3 # N*m continuous operating design torque
torque_servo_Tstall = 22.6 # N*m stall point (this is the max torque that can be applied to the CMG flywheel, but not sustained for long periods of time)

# battery parameters
battery_mass = 2.0 # Kg mass
battery_length = 12.0 # inches
battery_width = 3.0 # inches
battery_height = 3.0 # inches
battery_nominal_voltage = 18.5 # Volts

# ** END CMG DEVICE DEFS *** DO NOT REMOVE THIS COMMENTED LINE! ***************
# *****************************************************************************

def inch2meter(inches):
    return inches*0.0254

def meter2ft(meters):
    return meters/3.048

def lbs2kg(lbs):
    return(lbs*0.4536)

def kg2lbs(kg):
    return(kg*2.20462)

def flywheel_mass( Rinner, Router, thick, density ):
    return( np.pi*thick*(Router*Router - Rinner*Rinner)*density )

# def: X-axis => longitudinal axis of bike pointing forward direction.
#      Y-axis => lateral axis pointing +CCW RH rule, +ive, left.
#      Z-axis => vertical axis aligned with gravity +ive, up.
#      In this case, the fly wheel local principal axes are aligned with these axes...
def flywheel_Inertia(Rinner,Router, mass_IN, h=0.0):
    Izz = 0.5*mass_IN*(Rinner*Rinner + Router*Router)
    Iyy = mass_IN*((Rinner*Rinner + Router*Router)/4.0 + h*h/12.0) 
    Ixx = Iyy
    return Ixx,Iyy,Izz

def create_YAML( param_def ):

    print('Creating CMG YAML config...')
    # self documentation
    full_src_path = Path(__file__).resolve()

    # store the representative parameter config yaml file under $(find cmgdevice_description)/config
    folder =full_src_path.parent
    folder_upone = folder.parent
    config_subdir = "config"
    config_path1 = folder / config_subdir
    config_path2 = folder_upone / config_subdir

    if config_path1.is_dir():
        config_folder = str(config_path1)
    elif config_path2.is_dir():
        config_folder = str(config_path2)
    else:
        print(f'[ERROR:{str(full_src_path)}]: not able to find "{config_subdir}" subdirectory in either:\n\t{config_path1}\n\t{config_path2}\n')
        sys.exit(-2)


    # get intended name of YAML. 
    # TODO: don't let it overwrite pre-existing one
    YAML_file_name_found = False
    with open(str(full_src_path), "r", encoding='ascii') as file:
        for line in file:
            if "yaml_prefix" in line:
                quote_indices = [index for index, char in enumerate(line) if char in ('"', "'")]
                if (len(quote_indices) < 2 ):
                    print(f'ERROR: the line, >{line}<, must = an enclosed string for the YAML prefix name!')
                    break
                YAML_file_name = line[ quote_indices[0]+1:quote_indices[1] ] + '.yaml'
                YAML_file_name_found = True
                YAML_file_name = os.path.join(config_folder,YAML_file_name)
                break

    file.close()
    if not YAML_file_name_found:
        print(f'[ERROR:{str(full_src_path)}]: yaml_prefix = \'some_prefix_name\' missing!')
        sys.exit(-1)

    print(f'updating cmgparams.yaml to list the generated CMG YAML config file, "{YAML_file_name}"')  
    with open(str(full_src_path), "r", encoding='ascii') as file, \
         open(str(YAML_file_name), "w", encoding="ascii") as outfile:
        for line in file:
            if "END CMG DEVICE DEFS" in line:
                file.close()
                break
            comment_line = '# ' + line
            outfile.write(comment_line)
    file.close()
    outfile.close()

    # the urdf.xacro file will obtain the generated cmg param config using cmgparams.yaml.
    # This to mitigate the need to modify the urdf.xacro description ;-)
    cmgparams_ref_file = open( os.path.join(config_folder,"cmgparams.yaml"),"w", encoding="ascii")
    cmgparams_ref_file.write(f'yaml_param_file: \'{YAML_file_name}\'\n')
    cmgparams_ref_file.close()
    
    outfile = open(str(YAML_file_name), "a", encoding="ascii")
    for k, v in param_def.items():
        if v=='parent':
            outfile.write(f'\n{k}:\n')
            continue
        outfile.write(f'    {k}: {v:.8f}\n')
    outfile.close()

    print(f'finished building "{YAML_file_name}"...')
    return

def  main():
    global disk_rad, disk_thick, annulus_radout, annulus_radin, annulus_thick
    # motor parameters
    global flywheel_servo_mass, torque_servo_mass, flywheel_max_speed, flywheel_Kv
    global torque_servo_Tnominal, torque_servo_Tstall

    disk_rad =inch2meter(disk_rad)
    disk_thick = inch2meter(disk_thick)
    annulus_radout = inch2meter(annulus_radout)
    annulus_radin = inch2meter(annulus_radin)
    annulus_thick = inch2meter(annulus_thick)

    disk1_mass   = flywheel_mass(0.0,disk_rad, disk_thick, density_disk)
    annulus_mass = flywheel_mass(annulus_radin, annulus_radout, annulus_thick, density_annulus)
    
    Ixx_disk, Iyy_disk, Izz_disk = flywheel_Inertia(0.0, disk_rad, disk1_mass, disk_thick)
    Ixx_ann,  Iyy_ann,  Izz_ann  = flywheel_Inertia(annulus_radin, annulus_radout, annulus_mass, annulus_thick)

    cmg_def = {}
    cmg_def['links'] = 'parent'
    cmg_def['disk_rad'] = disk_rad
    cmg_def['disk_thick'] = disk_thick
    cmg_def['annulus_radout'] = annulus_radout
    cmg_def['annulus_radin'] = annulus_radin
    cmg_def['annulus_thick'] = annulus_thick
    cmg_def['disk1_mass'] = disk1_mass
    cmg_def['annulus_mass'] = annulus_mass
    cmg_def['Ixx_disk'] = Ixx_disk
    cmg_def['Iyy_disk'] = Iyy_disk
    cmg_def['Izz_disk'] = Izz_disk
    cmg_def['Ixx_ann'] = Ixx_ann
    cmg_def['Iyy_ann'] = Iyy_ann
    cmg_def['Izz_ann'] = Izz_ann

    cmg_def['actuators'] = 'parent'
    cmg_def['flywheel_servo_mass'] = flywheel_servo_mass
    cmg_def['torque_servo_mass'] =torque_servo_mass
    cmg_def['flywheel_max_speed'] = flywheel_max_speed
    cmg_def['flywheel_Kv'] = flywheel_Kv
    cmg_def['torque_servo_Tnominal'] = torque_servo_Tnominal
    cmg_def['torque_servo_Tstall'] = torque_servo_Tstall
    cmg_def['torque_servo_mass'] = torque_servo_mass
    cmg_def['flywheel_servo_diam'] = flywheel_servo_diam
    cmg_def['flywheel_servo_length'] = flywheel_servo_length
    
    cmg_def['battery'] = 'parent'
    cmg_def['battery_mass'] = battery_mass
    cmg_def['battery_length'] = battery_length
    cmg_def['battery_width'] = battery_width
    cmg_def['battery_height'] = battery_height
    cmg_def['battery_nominal_voltage'] = battery_nominal_voltage


    create_YAML(cmg_def)

    print('Calculations:')
    print(f'solid disk mass   = {disk1_mass:.3f} Kg ({kg2lbs(disk1_mass):.3f} lbs)')
    print(f'annulus disk mass = {annulus_mass:.3f} Kg ({kg2lbs(annulus_mass):.3f} lbs)')
    print('--------------------------------------------')
    print(f'       Total mass = {disk1_mass+annulus_mass:0.3f} ({kg2lbs(disk1_mass+annulus_mass):.3f} lbs)')
    print(f'\n')
    print(f'solid disk:   Ixx={Ixx_disk:.5f},Iyy={Iyy_disk:.5f},Izz={Izz_disk:.5f}')
    print(f'annular disk: Ixx={Ixx_ann:.5f},Iyy={Iyy_ann:.5f},Izz={Izz_ann:.5f}')

if __name__ == "__main__":
    main()

