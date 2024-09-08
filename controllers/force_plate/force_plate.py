from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

if robot.getDevice('force_plate_leg'):
    force_plates = [robot.getDevice('force_plate_leg')]
else:
    force_plates = [robot.getDevice('force_plate_1'),
                    robot.getDevice('force_plate_2'),
                    robot.getDevice('force_plate_3'),
                    robot.getDevice('force_plate_4')]

[force_plate.enable(1) for force_plate in force_plates]

filename = f'../data/{robot.getName()}.csv'

with open(filename, 'w', newline='') as file:
    data = 'time'
    for force_plate in force_plates:
        force_plate_name = force_plate.getName()
        data += f',{force_plate_name}_fx,{force_plate_name}_fy,{force_plate_name}_fz'
    data += '\n'
    
    file.write(data)

while robot.step(timestep) != -1:
    forces = [force_plate.getValues() for force_plate in force_plates]
    
    data = [str(round(robot.getTime(),3))]
    for i in range(len(force_plates)):
        forces[i][2] -= 9.81
        # print(f'{force_plates[i].getName()}: [{forces[i][0]:.8f}, {forces[i][1]:.8f}, {forces[i][2]:.8f}]')
        
        data.extend([str(forces[i][j]) for j in range(3)])
        
    with open(filename, 'a', newline='') as file:
        file.write(','.join(data)+'\n')
