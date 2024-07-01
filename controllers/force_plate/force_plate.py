from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())
force_plate = robot.getDevice('force_plate_leg')
force_plate.enable(1)

filename = f'../data/{robot.getName()}.csv'

with open(filename, 'w', newline='') as file:
    file.write(f'time,force_x,force_y,force_z\n')

while robot.step(timestep) != -1:
    force = force_plate.getValues()
    force[2] -= 9.80997889
    
    print(f'{robot.getName()}: [{force[0]:.8f}, {force[1]:.8f}, {force[2]:.8f}]')
    
    with open(filename, 'a', newline='') as file:
        file.write(f'{round(robot.getTime(),3)},{force[0]},{force[1]},{force[2]:.8f}\n')
