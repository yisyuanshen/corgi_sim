from controller import Supervisor
import numpy as np

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

leg_node = supervisor.getFromDef("LEG")
leg_node.enableContactPointsTracking(1)

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
imu = supervisor.getDevice("imu")
imu.enable(1)
gyro = supervisor.getDevice("gyro")
gyro.enable(1)
ang_vel = supervisor.getDevice("ang_vel")
ang_vel.enable(1)

# supervisor.getFromDef('LF_rl4').getField('physics').getSFNode().getField('mass').setSFFloat(0.001)
# supervisor.getFromDef('LF_rl5l6').getField('physics').getSFNode().getField('mass').setSFFloat(0.001)
# supervisor.getFromDef('LF_rdownframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.001)
# supervisor.getFromDef('LF_rupframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.001)
# supervisor.getFromDef('LF_rmotorbar').getField('physics').getSFNode().getField('mass').setSFFloat(0.001)
# supervisor.getFromDef('LF_ll4').getField('physics').getSFNode().getField('mass').setSFFloat(0.001)
# supervisor.getFromDef('LF_ll5l6').getField('physics').getSFNode().getField('mass').setSFFloat(0.001)
# supervisor.getFromDef('LF_ldownframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.001)
# supervisor.getFromDef('LF_lupframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.001)
# supervisor.getFromDef('LF_lmotorbar').getField('physics').getSFNode().getField('mass').setSFFloat(0.001)

supervisor.getFromDef('LF_rl4').getField('physics').getSFNode().getField('mass').setSFFloat(0.0145704)
supervisor.getFromDef('LF_ll4').getField('physics').getSFNode().getField('mass').setSFFloat(0.0145704)
supervisor.getFromDef('LF_rl5l6').getField('physics').getSFNode().getField('mass').setSFFloat(0.0364259)
supervisor.getFromDef('LF_ll5l6').getField('physics').getSFNode().getField('mass').setSFFloat(0.0364259)
supervisor.getFromDef('LF_rupframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.112321)
supervisor.getFromDef('LF_lupframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.112321)
supervisor.getFromDef('LF_rdownframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.052707)
supervisor.getFromDef('LF_ldownframe').getField('physics').getSFNode().getField('mass').setSFFloat(0.052707)
supervisor.getFromDef('LF_rmotorbar').getField('physics').getSFNode().getField('mass').setSFFloat(0.047)
supervisor.getFromDef('LF_lmotorbar').getField('physics').getSFNode().getField('mass').setSFFloat(0.046)

filename = '../data/output_leg.csv'

with open(filename, 'w', newline='') as file:
    file.write(f'time,phi_r,phi_l,trq_r,trq_l,dist\n')

theta = 17
beta = 0
state = 0

supervisor.step(1000)

loop_count = 0
while supervisor.step(timestep) != -1:
    print(f'= = = = = Loop Count: {loop_count} = = = = =')
    
    if state == 0:
        if theta >= 120: state += 1
        else: theta += 0.005
    elif state == 1:
        if theta <= 17: state += 1
        else: theta -= 0.005
    else:
        break
        
    # theta = 120
    
    # if state == 0:
    #     if loop_count % 3000 >= 2000: beta += 0.02
    #     if beta > 120: state += 1
    # elif state == 1:
    #     if loop_count % 3000 >= 2000: beta -= 0.02
    #     if beta < -120: state += 1
    # elif state == 2:
    #     if loop_count % 3000 >= 2000: beta += 0.02
    #     if beta >= 0: state += 1
    # else:
    #     break
    
    
    # if loop_count < 1000: pass
    # elif loop_count < 2000: theta += 0.02
    # elif loop_count < 4000: pass
    # elif loop_count < 5000: beta += 0.03
    # elif loop_count < 7000: pass
    # elif loop_count < 8000: beta += 0.03
    # elif loop_count < 10000: pass
    # elif loop_count < 11000: beta += 0.03
    # elif loop_count < 13000: pass
    # elif loop_count < 14000: beta += 0.03
    # elif loop_count < 16000: pass
    # elif loop_count < 17000: theta += 0.02
    # elif loop_count < 19000: pass
    # elif loop_count < 20000: theta += 0.02
    # elif loop_count < 22000: pass
    # elif loop_count < 23000: theta += 0.02
    # elif loop_count < 25000: pass
    # elif loop_count < 26000: beta -= 0.03
    # elif loop_count < 28000: pass
    # elif loop_count < 29000: beta -= 0.03
    # elif loop_count < 31000: pass
    # else: break
    
    phi_r = np.deg2rad(beta+theta-17)
    phi_l = np.deg2rad(beta-theta+17)
    
    motor_r.setPosition(phi_r)
    motor_l.setPosition(phi_l)
    
    print('Trq =', [motor_r.getTorqueFeedback(), motor_l.getTorqueFeedback()])

    with open(filename, 'a', newline='') as file:
        file.write(f'{round(supervisor.getTime(),3)},{encoder_r.getValue()},{encoder_l.getValue()},{motor_r.getTorqueFeedback()},{motor_l.getTorqueFeedback()},{dist_sensor.getValue()}\n')

    loop_count += 1
