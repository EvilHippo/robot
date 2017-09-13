import ev3dev.ev3 as ev3
import time

import math

"""
m = ev3.LargeMotor('outA')
print("testonetwo")
m.run_timed(time_sp=3000, speed_sp=500)
"""

from ev3dev import ev3
import time

class Robot:
    def __init__(self):
        self.timeToCross = 9.2
        self.motor_left = ev3.LargeMotor("outA")
        self.motor_right = ev3.LargeMotor("outD")
        self.motor_left.reset()
        self.motor_right.reset()
        self.btn = ev3.Button()
        self.currentDir = 1 #1:N, 2:W, 3:S, 4:E
        self.currentPos = [0, 0]
        self.labMap = []
        self.labMap.append([(0,0), "0"])
        self.visited = []

        self.sonar = ev3.UltrasonicSensor("in1")
        self.sonar2 = ev3.UltrasonicSensor("in4")
        self.sonar3 = ev3.UltrasonicSensor("in3")

        self.gyro = ev3.GyroSensor("in2")
        self.gyro.mode = "GYRO-ANG"

    def set_robot_speed(self, speed):
        self.set_speed(self.motor_left, speed)
        self.set_speed(self.motor_right, speed)
        self.robot_speed = speed

    def set_speed(self, motor, speed):
        motor.duty_cycle_sp = speed

    def driveBack(self):
        self.motor_left.run_timed(time_sp=100000, speed_sp=-50)
        self.motor_right.run_timed(time_sp=100000, speed_sp=-50)

    def drive(self):
        #print("Sending command")
        #self.motor_left.run_forever()
        #self.motor_right.run_forever()
        self.motor_left.run_timed(time_sp=100000, speed_sp=50)
        self.motor_right.run_timed(time_sp=100000, speed_sp=50)

    def turnNew(self, dir):
        #print("********************************* IN NEW TURN *************************")
        if(dir == "right"):
            self.motor_left.run_timed(time_sp=100000, speed_sp=100)
            self.motor_right.run_timed(time_sp=100000, speed_sp=25)

        if(dir == "left"):
            self.motor_right.run_timed(time_sp=100000, speed_sp=100)
            self.motor_left.run_timed(time_sp=100000, speed_sp=25)

    def turnSameSpot(self, dir):
        self.motor_left.stop()
        self.motor_right.stop()

        if(dir == "right"):
            self.motor_left.run_timed(time_sp=100000, speed_sp=100)

        if(dir == "left"):
            self.motor_right.run_timed(time_sp=100000, speed_sp=100)

    def sleep(self, duration):
        for _ in range(duration):
            print("Sleeping 1 sec")
            time.sleep(1)

    def stop(self):
        self.motor_left.stop()
        self.motor_right.stop()

    def timed_drive(self, duration):
        self.drive()
        self.sleep(duration)
        self.stop()


    def turn(self, direction, duration):
        if direction == "Right":
            self.set_speed(self.motor_right, 0)
            self.timed_drive(duration)
            self.set_speed(self.motor_right, self.robot_speed)
        if direction == "Left":
            self.set_speed(self.motor_left, 0)
            self.timed_drive(duration)
            self.set_speed(self.motor_left, self.robot_speed)

    def turnSharp(self, direction):
        if direction == "Right":
            self.set_speed(self.motor_right, 0)
            self.set_speed(self.motor_right, self.robot_speed)
        if direction == "Left":
            self.set_speed(self.motor_left, 0)
            self.set_speed(self.motor_left, self.robot_speed)

"""
#Find empty spot to go to from map
#Searches for an empty spot to drive to that hasn't been visited
#If all visited, return end and the mapping ends.
"""
def driveToUnsearched(robot):
    #TODO implement
    return "end"

"""
Changes direction of robot according to the turn that Has Already been made
"""
def getNewDir(turnDir, robot):
    if(turnDir == "left"):
        robot.currentDir -= 1
    if(turnDir == "right"):
        robot.currentDir += 1
    if(robot.currentDir > 4):
        robot.currentDir = 1
    if(robot.currentDir < 1):
        robot.currentDir = 4


"""
#Drives according to directions made (ex: r,r,l,l)
"""
def driveWithDirections():
    # TODO should check map to see which way makes sense to map first
    #TODO implement
    return

"""
When facing a wall tries to turn left or right if there's room(redundant maybe? could use driveToUnsearched)
note: might actually be bad and cause robot to go in circles in a clearing(doesnt check if new location is visited)
"""
def tryTurnNow(robot, rightSonar, leftSonar):
    # TODO remove function , use driveToUnsearched instead
    print("IN TRY TURN, R: ", rightSonar, "  L: ", leftSonar)
    if (rightSonar > 100 and rightSonar != -1):
        robot.turnSameSpot("right")
        startGyro = robot.gyro.value()
        while (abs(startGyro - abs(robot.gyro.value())) < 90):
            #print(abs(startGyro - abs(robot.gyro.value())))
            time.sleep(0.01)
        robot.stop()
        return "right"
    elif (leftSonar > 100 and leftSonar != -1):
        robot.turnSameSpot("left")
        startGyro = abs(robot.gyro.value())
        while(abs(startGyro - abs(robot.gyro.value())) < 90):
            #print(abs(startGyro - abs(robot.gyro.value())))
            time.sleep(0.01)
        robot.stop()
        return "left"
    else:
        return "none"

"""
Mark the direction seen on the map as empty

If direction is front, also mark the new location as current location
"""
def markEmpty(robot, dir):
    print("*******************  In markEmpty  **************")
    x = robot.currentPos[0]
    y = robot.currentPos[1]
    #TODO make this better..
    if(robot.currentDir == 4):
        if(dir == "left"):
            robot.labMap.append([(x, y - 1), "0"])
        if(dir == "right"):
            robot.labMap.append([(x, y + 1), "0"])
        if(dir == "front"):
            robot.labMap.append([(x - 1, y), "0"])
            robot.currentPos[0] -= 1

    if (robot.currentDir == 3):
        if (dir == "left"):
            robot.labMap.append([(x + 1, y), "0"])
        if (dir == "right"):
            robot.labMap.append([(x - 1, y), "0"])
        if (dir == "front"):
            robot.labMap.append([(x, y - 1), "0"])
            robot.currentPos[1] -= 1

    if (robot.currentDir == 2):
        if (dir == "left"):
            robot.labMap.append([(x, y + 1), "0"])
        if (dir == "right"):
            robot.labMap.append([(x, y - 1), "0"])
        if (dir == "front"):
            robot.labMap.append([(x + 1, y), "0"])
            robot.currentPos[0] += 1

    if (robot.currentDir == 1):
        if (dir == "left"):
            robot.labMap.append([(x - 1, y), "0"])
        if (dir == "right"):
            robot.labMap.append([(x + 1, y), "0"])
        if (dir == "front"):
            robot.labMap.append([(x, y + 1), "0"])
            robot.currentPos[1] += 1




"""
check if sides are empty, if true, add them to map
check if wall in front is near, if true:
    check if a side is empty, if true, move there
    check if an already explored loc has a spot that hasnt been visited, if it does, go there, otherwise end
check if time moved equals time to pass through 1 square, if true
    reset time and mark location as visited
#not done# Try to stay in the middle
#
"""
def makeMap(robot):
    frontSonar = -1
    rightSonar = -1
    leftSonar = -1
    wallCloseFront = 130
    wallCloseSide = 150

    start_time = time.time()

    robot.visited.append((0, 0))

    while(not robot.btn.any()):
        robot.drive()
        #TODO add buffer for sonar values
        frontSonar = robot.sonar.value()
        rightSonar = robot.sonar2.value()
        leftSonar = robot.sonar3.value()

        if (rightSonar > wallCloseSide):
            markEmpty(robot, "right")
        if (leftSonar > wallCloseSide):
            markEmpty(robot, "left")

        if(frontSonar <= wallCloseFront):

            if (time.time() - (robot.timeToCross / 2) >= start_time):
                # it is likely that another square has been reached
                markEmpty(robot, "front")
                robot.visited.append((robot.currentPos[0], robot.currentPos[1]))

            robot.stop()
            turnedDir = tryTurnNow(robot, rightSonar, leftSonar)
            print()
            print(robot.labMap)
            print("current: " , robot.currentPos)
            print()
            if(turnedDir == "none"):
                print("ENTERING TURNEDDIR ::::::::::::::::::: O NOOOOOOOOOOOOOOOES! ITS NOT DONE!")
                turnedDir = driveToUnsearched(robot)
            if(turnedDir == "left"):
                getNewDir("left", robot)
            elif(turnedDir == "right"):
                getNewDir("right", robot)
            elif(turnedDir == "end"):
                print("NO MORE VALID PATHS FOUND, RETURNING")
                robot.stop()
                return
            else:
                print(" INVALID VALUE IN TURN ")
            start_time = time.time()
            robot.drive()

        if(time.time() - robot.timeToCross >= start_time):
            markEmpty(robot, "front")
            robot.visited.append((robot.currentPos[0], robot.currentPos[1]))
            start_time = time.time()


def main():
    robot = Robot()

    print("starting up")
    start_time = time.time()
    robot.set_robot_speed(70)
    new_time = 0

    while(new_time < (start_time + 3)):
        new_time = time.time()

    robot.sonar.mode = "US-DIST-CM"


    # for gyro and sonar testing (should probably just use a seperate file..)
    """
    c = 0
    #robot.turnSameSpot("left")

    start_time = time.time()
    new_time = 0
    robot.drive()
    while(not robot.btn.any()):
        if c == 15:
            print("sonar1: ", robot.sonar.value())
            #print("sonar2: ", robot.sonar2.value())
            print()
            c = 0
        else:
            c += 1
        if(robot.sonar.value() == 0 or robot.sonar.value() == 2550):
            print("TIME: ", time.time() - start_time)
            robot.stop()
            return
    robot.stop()
    return

    """

    """
    robot.gyro.value()
    c = 0
    while (not robot.btn.any()):
        if c == 20:
            print("Gyro: ", robot.gyro.value())
            print()
            c = 0
        else:
            c += 1
    robot.stop()
    return
    """

    """
    # for testing the distance traveled
    count = 0
    start_time = time.time()
    robot.drive()
    while(count < 3 and not robot.btn.any()):
        if (time.time() - robot.timeToCross >= start_time):
            count += 1
            print("Count: ", count)
            t = time.time() - start_time
            print("Time: ", t)
            print()
            start_time = time.time()

    print("END, count: ", count)
    robot.stop()
    return
    """

    makeMap(robot)

    robot.stop()


if __name__ == "__main__":
    main()
