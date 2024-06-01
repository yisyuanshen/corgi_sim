from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())
force_plate = robot.getDevice("force plate")
force_plate.enable(1)

while robot.step(timestep) != -1:
    force = force_plate.getValues()
    print(f'{robot.getName()}: [{force[0]:.8f}, {force[1]:.8f}, {(force[2]-98.1):.8f}]')
    # robot.step(1)