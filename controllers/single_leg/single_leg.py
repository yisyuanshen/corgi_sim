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
theta = 17
beta = 0

while supervisor.step(timestep) != -1:
    print(f'\n= = = = = Loop Count: {int(loop_count)} = = = = =')
    
    # if loop_count < 1000:
        # theta = 50
        # beta = 0
    # elif loop_count < 5000:
        # theta = 50 #+ 100 / 4000 * (loop_count-1000)
        # beta = 100 / 4000 * (loop_count-1000)
    # elif loop_count < 15000:
        # theta = 50
        # beta = 100
    # else:
        # break
    
    time_interval = 2000
    
    if loop_count < time_interval * 1: pass
    elif loop_count < time_interval * 2: theta += 30 / time_interval
    elif loop_count < time_interval * 3: pass
    elif loop_count < time_interval * 4: theta -= 30 / time_interval
    elif loop_count < time_interval * 5: beta += 30 / time_interval
    elif loop_count < time_interval * 6: theta += 30 / time_interval
    elif loop_count < time_interval * 7: pass
    elif loop_count < time_interval * 8: beta += 60 / time_interval
    elif loop_count < time_interval * 9: theta += 30 / time_interval
    elif loop_count < time_interval * 10: pass
    else: break
    
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
