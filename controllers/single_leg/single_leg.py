from controller import Supervisor
import numpy as np

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

motor_r = supervisor.getDevice('lf_right_motor')
motor_l = supervisor.getDevice('lf_left_motor')
motor_r.enableTorqueFeedback(1)
motor_l.enableTorqueFeedback(1)

encoder_r = motor_r.getPositionSensor()
encoder_l = motor_l.getPositionSensor()
encoder_r.enable(1)
encoder_l.enable(1)

force_sensor_node = supervisor.getFromDef('LF_force_sensor')
force_sensor = supervisor.getDevice('force_lf')
force_sensor.enable(1)

dist_sensor = supervisor.getDevice('dst_lf')
dist_sensor.enable(1)

save_file = True

if save_file: 
    with open(f'output_leg.csv', 'w', newline='') as file:
        file.write(f'Time,phi_r,phi_l,trq_r,trq_l\n')

supervisor.step(1000)

loop_count = 0.0
while supervisor.step(timestep) != -1:
    print(f'\n= = = = = Loop Count: {int(loop_count)} = = = = =')
    
    # if loop_count < 2000:
        # theta = 17 + 100 / 2000 * loop_count
        # beta = 0
    # else:
        # theta = 17 + 50 * (np.cos((loop_count-2000)/1000*np.pi)+1)
        # beta = 50 * (-np.cos((loop_count-2000)/2000*np.pi)+1)
    
    theta = 60
    beta = 0
    
    phi_r = np.deg2rad(beta+theta-17)
    phi_l = np.deg2rad(beta-theta+17)
    
    motor_r.setPosition(phi_r)
    motor_l.setPosition(phi_l)
    
    force_local = np.array(force_sensor.getValues()[:3])
    orientation = np.array(force_sensor_node.getOrientation()).reshape(3, 3)
    force_world = orientation.dot(force_local)
    
    print(f'    phi = [{encoder_r.getValue()}, {encoder_l.getValue()}]')
    print(f'    trq = [{motor_r.getTorqueFeedback()}, {motor_l.getTorqueFeedback()}]')
    
    print(f'Dist: {dist_sensor.getValue()}')
    
    if save_file: 
        with open(f'output_leg.csv', 'a', newline='') as file:
            file.write(f'{round(supervisor.getTime(), 3)},{encoder_r.getValue()},{encoder_l.getValue()},{motor_r.getTorqueFeedback()},{motor_l.getTorqueFeedback()}\n')

    # print(f'World Force = [{force_world[0]}, {force_world[1]}, {force_world[2]}]')
    
    loop_count += 1
    pass
