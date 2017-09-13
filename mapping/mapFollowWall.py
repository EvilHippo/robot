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
        self.lastAdjust = None
        self.timeToCross = 7
        self.motor_left = ev3.LargeMotor("outA")
        self.motor_right = ev3.LargeMotor("outD")
        self.turret = ev3.MediumMotor("outB")
        self.motor_left.reset()
        self.motor_right.reset()
        self.turret.reset()
        self.btn = ev3.Button()
        self.currentDir = 1 #1:N, 2:W, 3:S, 4:E
        self.currentPos = [0, 0]
        self.labMap = []
        self.labMap.append([(0,0), "0"])
        self.visited = []
        self.connections = []
        self.normalWall = 120
        self.turretPos = "front"

        self.sonar = ev3.UltrasonicSensor("in1")
        #self.sonar2 = ev3.UltrasonicSensor("in4")
        #self.sonar3 = ev3.UltrasonicSensor("in3")

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
        self.motor_left.run_timed(time_sp=100000, speed_sp=70)
        self.motor_right.run_timed(time_sp=100000, speed_sp=70)

    def driveSonar(self):
        if(self.sonar.value() > 200):
            self.drive()
        else:
            normal = 70
            dif = (self.sonar.value() - self.normalWall) / 2
            self.motor_left.run_timed(time_sp=100000, speed_sp=normal)
            self.motor_right.run_timed(time_sp=100000, speed_sp=normal + dif)

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
            self.motor_right.run_timed(time_sp=100000, speed_sp=-100)

        if(dir == "left"):
            self.motor_right.run_timed(time_sp=100000, speed_sp=100)
            self.motor_left.run_timed(time_sp=100000, speed_sp=-100)

    def turnTurret(self, dir):
        self.turret.stop()

        if (dir == "right"):
            self.turret.run_timed(time_sp=100000, speed_sp=50)

        if (dir == "left"):
            self.turret.run_timed(time_sp=100000, speed_sp=-50)

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

    def turnAround(self):
        start = self.gyro.value()
        self.turnSameSpot("right")
        while(abs(start - self.gyro.value()) <= 180):
            pass
        self.stop()



def getCords(robot):
    #[[robot.currentPos[0], robot.currentPos[1]],
    #[robot.currentPos[0], robot.currentPos[1] - 1]]

    #labMap.append([(0,0), "0"])
    start = robot.currentPos
    end = None
    for x in range(len(robot.labMap)):
        if(robot.labMap[x][0] not in robot.visited):
            # finds last location that has not been visited.. loop through list in reverse and find last instead myb?..
            end = robot.labMap[x][0]
    if(end == None):
        return None


    d = {}
    i = 0
    explore = []
    explore.append(list(robot.currentPos))
    current = explore[0]
    while(current != end):
        current = explore[0]
        for x in range(len(robot.connections)):
            if(robot.connections[x][0] == current):
                if(robot.connections[x][1] not in d):
                    d[robot.connections[x][1]] = i
                    explore.append(robot.connections[x][1])
            elif(robot.connections[x][1] == current):
                if (robot.connections[x][0] not in d):
                    d[robot.connections[x][0]] = i
                    explore.append(robot.connections[x][0])
        i += 1
        explore.pop(0)

    cordList = []
    cordList.append(end)

    if(i == 0):
        print("Something went horribly, Horribly wrong!")
        return None

    while(i != 0):
        i -= 1
        for key, value in d.items():
            if(value == i):
                cordList.append(key)
    return cordList



"""
#Find empty spot to go to from map
#Searches for an empty spot to drive to that hasn't been visited
#If all visited, return end and the mapping ends.
"""
def driveToUnsearched(robot):
    return "end"
    cordList = getCords(robot)
    if(cordList == None):
        return "end"
    robot.turnAround()
    getNewDir("right", robot)
    getNewDir("right", robot)
    #TODO implement
    return "end"


def turn(robot, dir):
    return

"""
#Drives forward, in turns checks wh
"""
def driveWithDirections(robot, list):
    count = getSonar(robot, True)
    # TODO should check map to see which way makes sense to map first
    #TODO implement

    robot.visited.append(tuple(robot.currentPos))
    return


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
When facing a wall tries to turn left or right if there's room(redundant maybe? could use driveToUnsearched)
note: might actually be bad and cause robot to go in circles in a clearing(doesnt check if new location is visited)
"""
def tryTurnNow(robot, rightSonar, leftSonar):
    # TODO remove function , use driveToUnsearched instead
    print("IN TRY TURN, R: ", rightSonar, "  L: ", leftSonar)
    if(leftSonar > 220):
        print("in left")
        robot.turnSameSpot("left")
        startGyro = abs(robot.gyro.value())
        while (abs(abs(startGyro) - abs(robot.gyro.value())) < 90):
            # print(abs(startGyro - abs(robot.gyro.value())))
            time.sleep(0.01)
        robot.stop()
        return "left"
    elif (rightSonar > 220):
        print("in right")
        robot.turnSameSpot("right")
        startGyro = robot.gyro.value()
        while (abs(abs(startGyro) - abs(robot.gyro.value())) < 90):
            #print(abs(startGyro - abs(robot.gyro.value())))
            time.sleep(0.01)
        robot.stop()
        return "right"
    else:
        return "none"

def tryTurnLeft(robot, leftSonar):
    robot.stop()
    if (leftSonar > 220 and leftSonar != -1):
        robot.turnSameSpot("left")
        startGyro = abs(robot.gyro.value())
        while (abs(startGyro - abs(robot.gyro.value())) < 90):
            # print(abs(startGyro - abs(robot.gyro.value())))
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
    print("Going: ", dir)
    print("current: ", robot.currentPos)
    #TODO make this better..
    if(robot.currentDir == 4):
        if(dir == "left"):
            robot.labMap.append([(x, y - 1), "0"])
        if(dir == "right"):
            robot.labMap.append([(x, y + 1), "0"])
        if(dir == "front"):
            robot.labMap.append([(x - 1, y), "0"])
            robot.connections.append([
                [robot.currentPos[0], robot.currentPos[1]],
                [robot.currentPos[0] - 1, robot.currentPos[1]]
            ])
            robot.currentPos[0] -= 1


    if (robot.currentDir == 3):
        if (dir == "left"):
            robot.labMap.append([(x + 1, y), "0"])
        if (dir == "right"):
            robot.labMap.append([(x - 1, y), "0"])
        if (dir == "front"):
            robot.labMap.append([(x, y - 1), "0"])
            robot.connections.append([
                [robot.currentPos[0], robot.currentPos[1]],
                [robot.currentPos[0], robot.currentPos[1] - 1]
            ])
            robot.currentPos[1] -= 1

    if (robot.currentDir == 2):
        if (dir == "left"):
            robot.labMap.append([(x, y + 1), "0"])
        if (dir == "right"):
            robot.labMap.append([(x, y - 1), "0"])
        if (dir == "front"):
            robot.labMap.append([(x + 1, y), "0"])
            robot.connections.append([
                [robot.currentPos[0], robot.currentPos[1]],
                [robot.currentPos[0] + 1, robot.currentPos[1]]
            ])
            robot.currentPos[0] += 1

    if (robot.currentDir == 1):
        if (dir == "left"):
            robot.labMap.append([(x - 1, y), "0"])
        if (dir == "right"):
            robot.labMap.append([(x + 1, y), "0"])
        if (dir == "front"):
            robot.labMap.append([(x, y + 1), "0"])
            robot.connections.append([
                [robot.currentPos[0], robot.currentPos[1]],
                [robot.currentPos[0], robot.currentPos[1] + 1]
            ])
            robot.currentPos[1] += 1

    print("new pos: ", robot.currentPos)
    print()

def getSonar(robot, count = False):
    print("in get sonar")
    print("R wheel: ", robot.motor_right.position, " %190: ", robot.motor_right.position % 190)
    print("L wheel: ", robot.motor_left.position, " %190: ", robot.motor_left.position % 190)
    robot.stop()
    c = 0

    # reset view
    if(robot.turretPos == "right"):
        robot.turnTurret("left")
        while (robot.turret.position >= 0) and not robot.btn.any():
            # print(robot.turret.position)
            pass
        robot.turret.stop()
    elif(robot.turretPos == "left"):
        robot.turnTurret("right")
        while (robot.turret.position <= 0) and not robot.btn.any():
            # print(robot.turret.position)
            pass
        robot.turret.stop()


    # look forward
    front = robot.sonar.value()

    # look right
    robot.turnTurret("right")
    while (robot.turret.position <= 90) and not robot.btn.any():
        #print(robot.turret.position)
        pass
    robot.turret.stop()
    right = robot.sonar.value()

    # look left
    robot.turnTurret("left")
    while(robot.turret.position >= -90) and not robot.btn.any():
        #print(robot.turret.position)
        pass
    robot.turret.stop()
    left = robot.sonar.value()

    # reset to middle
    robot.turnTurret("right")
    while(robot.turret.position <= 0) and not robot.btn.any():
        #print(robot.turret.position)
        pass
    robot.turret.stop()

    wallCloseSide = 150
    if (right > wallCloseSide):
        c += 1
        markEmpty(robot, "right")

    if (left > wallCloseSide):
        c += 1
        markEmpty(robot, "left")

    if(count):
        return c
    return [front, right, left]



def adjust(robot, rightSonar, leftSonar, wallCloseSide):
    #self.motor_left.run_timed(time_sp=100000, speed_sp=70)
    #self.motor_right.run_timed(time_sp=100000, speed_sp=70)
    # TODO maybe make it still adjust if a wall that is seen is too close

    # empty space on either side, do nothing
    #if(rightSonar > wallCloseSide or leftSonar > wallCloseSide):
     #   robot.lastAdjust = None
      #  return

    a = 73
    dif = abs(rightSonar - leftSonar)

    if(dif > 40):
        a = 76
    if(dif > 60):
        a = 80


    if(abs(rightSonar - leftSonar) > 10):
        if(rightSonar < leftSonar and robot.lastAdjust != "left"):
            robot.lastAdjust = "left"
            robot.motor_right.run_timed(time_sp=100000, speed_sp=a)
            print("Adjusting to left")
        elif (robot.lastAdjust != "right"):
            robot.lastAdjust = "right"
            robot.motor_left.run_timed(time_sp=100000, speed_sp=a)
            print("Adjusting to right")
    else:
        robot.lastAdjust = None


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
    wallCloseFront = 60
    wallCloseSide = 300

    robot.visited.append((0, 0))


    start_time = time.time()

    while(not robot.btn.any()):

        temp = getSonar(robot)
        frontSonar = temp[0]
        rightSonar = temp[1]
        leftSonar = temp[2]

        test = tryTurnLeft(robot, leftSonar)
        if(test == "left"):
            getNewDir("left", robot)
            temp = getSonar(robot)
            frontSonar = temp[0]
            rightSonar = temp[1]
            leftSonar = temp[2]
        """
        if (rightSonar > wallCloseSide):
            markEmpty(robot, "right")
        if (leftSonar > wallCloseSide):
            markEmpty(robot, "left")
        """

        # TURNING
        if(frontSonar <= wallCloseFront):
            if (time.time() - (robot.timeToCross / 2) >= start_time):
                # it is likely that another square has been reached
                markEmpty(robot, "front")
                robot.visited.append((robot.currentPos[0], robot.currentPos[1]))

            robot.stop()
            turnedDir = tryTurnNow(robot, rightSonar, leftSonar)

            if(turnedDir == "none"):
                robot.turnAround()
                getNewDir("right", robot)
                getNewDir("right", robot)
                #print("ENTERING driveToUnsearched ::::::::::::::::::: O NOOOOOOOOOOOOOOOES! ITS NOT DONE!")
                #turnedDir = driveToUnsearched(robot)

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

            temp = getSonar(robot)
            frontSonar = temp[0]
            rightSonar = temp[1]
            leftSonar = temp[2]

        if(frontSonar < 200): # if too close, just drive forward and dont turn sonar
            start_time = time.time()
            robot.drive()
            robot.turretPos = "front"
            while(time.time() - robot.timeToCross <= start_time):
                if(robot.sonar.value() <= wallCloseFront):
                    break

        else:
            # turns sonar left
            robot.turnTurret("left")
            while (robot.turret.position >= -90) and not robot.btn.any():
                # print(robot.turret.position)
                pass
            robot.turret.stop()
            robot.turretPos = "left"

            start_time = time.time()
            while(time.time() - robot.timeToCross <= start_time):
                time.sleep(0.1)
                robot.driveSonar()
            robot.stop()
            markEmpty(robot, "front")
            robot.visited.append((robot.currentPos[0], robot.currentPos[1]))



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
    """             Double sonar testing
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


    """               Gyro value testing
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

    """               Aprox distance traveled testing
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

    """              Turret testing
    #robot.turret.position_sp = 90
    #print("Turret position sp is:  ", robot.turret.position_sp)
    #robot.turret.run_to_abs_pos()

    robot.turnTurret("right")

    while (robot.turret.position < 90 and robot.turret.position > -90) and not robot.btn.any():
        print(robot.turret.position)
        pass
    robot.turret.stop()

    while (not robot.btn.any()):
        print(robot.turret.position)
    return

    """

    """       Time measuring for how long it takes to move 1 square
    start_time = time.time()
    robot.drive()
    while(robot.sonar.value() != 2550):
        pass

    robot.stop()
    print(time.time() - start_time)
    """

    """      Turning testing

    robot.motor_right.run_timed(time_sp=100000, speed_sp=25)
    robot.motor_left.run_timed(time_sp=100000, speed_sp=-25)

    while (not robot.btn.any()):
        pass

    robot.stop()
    return
    """

    """      Wheel position testing
    c = 0
    while (not robot.btn.any()):

        if c == 20:
            print("R: ", robot.motor_right.position)
            print("L: ", robot.motor_left.position)
            print()
            c = 0
        else:
            c += 1
    robot.stop()
    return

    """

    """                Turn Measuring
    robot.turnSameSpot("right")
    startGyro = abs(robot.gyro.value())
    while (abs(startGyro - abs(robot.gyro.value())) < 90):
        # print(abs(startGyro - abs(robot.gyro.value())))
        pass
    robot.stop()
    print(robot.gyro.value())
    print("R: ", robot.motor_right.position)
    print("L: ", robot.motor_left.position, "\n")
    time.sleep(5)

    robot.turnSameSpot("left")
    startGyro = abs(robot.gyro.value())
    while (abs(startGyro - abs(robot.gyro.value())) < 90):
        # print(abs(startGyro - abs(robot.gyro.value())))
        pass
    robot.stop()
    print(robot.gyro.value())
    print("R: ", robot.motor_right.position)
    print("L: ", robot.motor_left.position, "\n")
    return
    """







    #temp = getSonar(robot)
    #print("Sonar values: ", temp)

    #robot.turnAround()

    makeMap(robot)

    for x in robot.connections:
        print(x)

    #self.labMap = []
    #self.labMap.append([(0, 0), "0"])
    #self.visited = []

    xM = 0
    xL = 0
    yM = 0
    yL = 0

    for x in robot.labMap:
        if(x[0][0] < xL):
            xL = x[0][0]
        if(x[0][0] > xM):
            xM = x[0][0]
        if (x[0][1] < yL):
            yL = x[0][1]
        if (x[0][1] > yM):
            yM = x[0][1]
    xL = abs(xL)
    yL = abs(yL)
    xM += xL
    yM += yL
    empties = []

    for x in robot.labMap:
        empties.append([x[0][0]+xL, x[0][1]+yL])
    temp = []
    for x in range(0, xM*3 + 3):
        temp.append([])
        for y in range(0, yM*3 + 3):
            temp[x].append("X")
    for x in empties:
        temp[x[0]*3+1][x[1]*3+1] = "0"

    for x in robot.connections:
        for y in x:
            for z in range(len(y)):
                y[z] = y[z]*3 +1

    for x in robot.connections:
        if(x[0][0] == x[1][0]):
            m = max([x[0][1], x[1][1]])
            l = min([x[0][1], x[1][1]])
            for y in range(len(temp)):
                for z in range(len(temp[y])):
                    if(y == x[0][0] and z >= l and z <= m):
                        temp[y][z] = "0"

        if (x[0][1] == x[1][1]):
            m = max([x[0][0], x[1][0]])
            l = min([x[0][0], x[1][0]])
            for y in range(len(temp)):
                for z in range(len(temp[y])):
                    if (z == x[0][1] and y >= l and y <= m):
                        temp[y][z] = "0"

    # start pos

    for x in temp:
        print(x)

    print("\nConnections: ")
    for x in robot.connections:
        print(x)

    robot.stop()


if __name__ == "__main__":
    main()
