from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())
force_plate = robot.getDevice("force plate")
force_plate.enable(1)

save_file = True

if save_file: 
    with open(f'{robot.getName()}.csv', 'w', newline='') as file:
        file.write(f"Time,force_x,force_y,force_z\n")

while robot.step(timestep) != -1:
    force = force_plate.getValues()
    print(f'{robot.getName()}: [{force[0]:.8f}, {force[1]:.8f}, {(force[2]-10):.8f}]')
    # robot.step(1)
    
    if save_file:
        with open(f'{robot.getName()}.csv', 'a', newline='') as file:
            file.write(f"{round(robot.getTime(),3)},{force[0]},{force[1]},{force[2]-10}\n")
    