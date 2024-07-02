from controller import Supervisor
import numpy as np

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())


motor_r = supervisor.getDevice('lf_right_motor')
motor_l = supervisor.getDevice('lf_left_motor')
motor_r.enableTorqueFeedback(1)
motor_l.enableTorqueFeedback(1)
motor_r.setControlPID(30, 0, 0)
motor_l.setControlPID(30, 0, 0)

encoder_r = motor_r.getPositionSensor()
encoder_l = motor_l.getPositionSensor()
encoder_r.enable(1)
encoder_l.enable(1)

dist_sensor = supervisor.getDevice('dst_lf')
dist_sensor.enable(1)

supervisor.getFromDef('LF_rl4').getField('physics').getSFNode().getField('mass').setSFFloat(0.0001)
supervisor.getFromDef('LF_rl5l6').getField('physics').getSFNode().getField('mass').setSFFloat(0.0001)
supervisor.getFromDef('LF_rdownframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.0001)
supervisor.getFromDef('LF_rupframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.0001)
supervisor.getFromDef('LF_rmotorbar').getField('physics').getSFNode().getField('mass').setSFFloat(0.0001)
supervisor.getFromDef('LF_ll4').getField('physics').getSFNode().getField('mass').setSFFloat(0.0001)
supervisor.getFromDef('LF_ll5l6').getField('physics').getSFNode().getField('mass').setSFFloat(0.0001)
supervisor.getFromDef('LF_ldownframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.0001)
supervisor.getFromDef('LF_lupframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.0001)
supervisor.getFromDef('LF_lmotorbar').getField('physics').getSFNode().getField('mass').setSFFloat(0.0001)

# supervisor.getFromDef('LF_rl4').getField('physics').getSFNode().getField('mass').setSFFloat(0.0145704)
# supervisor.getFromDef('LF_ll4').getField('physics').getSFNode().getField('mass').setSFFloat(0.0145704)
# supervisor.getFromDef('LF_rl5l6').getField('physics').getSFNode().getField('mass').setSFFloat(0.0364259)
# supervisor.getFromDef('LF_ll5l6').getField('physics').getSFNode().getField('mass').setSFFloat(0.0364259)
# supervisor.getFromDef('LF_rupframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.112321)
# supervisor.getFromDef('LF_lupframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.112321)
# supervisor.getFromDef('LF_rdownframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.052707)
# supervisor.getFromDef('LF_ldownframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.052707)
# supervisor.getFromDef('LF_rmotorbar').getField('physics').getSFNode().getField('mass').setSFFloat(0.047)
# supervisor.getFromDef('LF_lmotorbar').getField('physics').getSFNode().getField('mass').setSFFloat(0.046)

filename = '../data/output_leg.csv'

with open(filename, 'w', newline='') as file:
    file.write(f'time,phi_r,phi_l,trq_r,trq_l\n')

theta = 17
beta = 0
state = 0

supervisor.step(1000)

loop_count = 0
while supervisor.step(timestep) != -1:
    print(f'= = = = = Loop Count: {loop_count} = = = = =')
    
    '''
    theta = 120
    
    if state == 0:
        if loop_count % 3000 >= 2000: beta += 0.02
        if beta > 120: state += 1
    elif state == 1:
        if loop_count % 3000 >= 2000: beta -= 0.02
        if beta < -120: state += 1
    elif state == 2:
        if loop_count % 3000 >= 2000: beta += 0.02
        if beta >= 0: state += 1
    else:
        break
    '''
    
    # '''
    beta = 40
    
    if state == 0:
        if loop_count % 3000 >= 2000: theta += 0.02
        if theta > 117: state += 1
    elif state == 1:
        if loop_count % 3000 >= 2000: theta -= 0.02
        if theta <= 17: state += 1
    else:
        break
    # '''
    
    
    phi_r = np.deg2rad(beta+theta-17)
    phi_l = np.deg2rad(beta-theta+17)
    
    motor_r.setPosition(phi_r)
    motor_l.setPosition(phi_l)
    
    print('Trq =', [motor_r.getTorqueFeedback(), motor_l.getTorqueFeedback()])

    with open(filename, 'a', newline='') as file:
        file.write(f'{round(supervisor.getTime(),3)},{encoder_r.getValue()},{encoder_l.getValue()},{motor_r.getTorqueFeedback()},{motor_l.getTorqueFeedback()}\n')

    loop_count += 1
