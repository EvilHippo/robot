import ev3dev.ev3 as ev3
import time
"""
m = ev3.LargeMotor('outA')
print("testonetwo")
m.run_timed(time_sp=3000, speed_sp=500)
"""

from ev3dev import ev3
import time

class Robot:
    def __init__(self):
        self.motor_left = ev3.LargeMotor("outA")
        self.motor_right = ev3.LargeMotor("outD")
        self.motor_left.reset()
        self.motor_right.reset()
        self.color_sensor = ev3.ColorSensor("in3")
        self.color_sensor.mode = 'COL-REFLECT'
        self.btn = ev3.Button()
        self.sonar = ev3.UltrasonicSensor("in1")

    def set_robot_speed(self, speed):
        self.set_speed(self.motor_left, speed)
        self.set_speed(self.motor_right, speed)
        self.robot_speed = speed

    def set_speed(self, motor, speed):
        motor.duty_cycle_sp = speed

    def drive(self):
        #print("Sending command")
        #self.motor_left.run_forever()
        #self.motor_right.run_forever()
        self.motor_left.run_timed(time_sp=100000, speed_sp=50)
        self.motor_right.run_timed(time_sp=100000, speed_sp=50)
    def driveEmpty(self, t, lM, rM):
        self.motor_left.run_timed(time_sp=t, speed_sp=lM)
        self.motor_right.run_timed(time_sp=t, speed_sp=rM)

    def turnNew(self, dir):
        #print("********************************* IN NEW TURN *************************")
        if(dir == "right"):
            self.motor_left.run_timed(time_sp=100000, speed_sp=100)
            self.motor_right.run_timed(time_sp=100000, speed_sp=25)

        if(dir == "left"):
            self.motor_right.run_timed(time_sp=100000, speed_sp=100)
            self.motor_left.run_timed(time_sp=100000, speed_sp=25)

    def sleep(self, duration):
        for _ in range(duration):
            print("Sleeping 1sec")
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

    def sense_color(self):
            return self.color_sensor.value()

def brickEncounter(robot):
    roadFound = False
    # last action
    lastA = "drive_brick"
    # action count, if actions too low, black line is ignored(start of search may encounter it)
    aCount = 0
    sonar = robot.sonar.value()
    if(sonar >= 250):
        print("INVALID SONAR VALUE IN BRICK (brick more than 250 away)")
        #return
    emptyTime = 0

    print("********************************************* FOUND BRICK ****************************")
    # duration in seconds to keep driving after getting to next step(to avoid collision)
    FORWARD_DUR = 6
    TURN_DUR = 2
    robot.color_sensor.mode = 'COL-COLOR'
    randomCount = 0
    while(roadFound is False and not robot.btn.any()):
        randomCount += 1
        if(randomCount == 50):
            print(sonar)
            randomCount = 0
        lastS = sonar
        sonar = robot.sonar.value()
        #print(sonar)
        timeCount = time.time()
        if(lastA == "drive_brick" and (sonar < 350 or aCount == 0)):
            aCount += 1
            print("IN FIND EMPTY")
            robot.turnNew("right")
            lastA = "find_empty"
        """
        if(robot.sense_color() < 25 and aCount > 3):
            robot.stop()
            robot.color_sensor.mode = 'COL-REFLECT'
            return
        """
        #if(robot.sense_color() == 1):
            #print("SEEING BLACK**************************************************************************")
        if (lastA == "find_empty" and sonar > 500):
            print("IN WAIT")
            lastA = "find_emptyW"
            emptyTime = time.time()
        # keep turning for a while after seeing empty space
        elif(lastA == "find_emptyW" and timeCount > (emptyTime + TURN_DUR) and sonar > 500):
            print("IN DRIVE EMPTY")
            robot.stop()
            start_sonar = sonar
            aCount += 1
            robot.driveEmpty(10000, 100, 100)
            lastA = "drive_empty"
            emptyTime = time.time()

        # drive into the sunset for some time
        elif(lastA == "drive_empty" and timeCount > (emptyTime + FORWARD_DUR)):
            print("IN FIND BRICK")
            robot.stop()
            aCount += 1
            robot.turnNew("left")
            lastA = "find_brick"

        # new value is way closer than last, probably edge of brick
        elif(lastA == "find_brick" and abs(sonar - lastS) > 500 and sonar < 1000):
            print("IN DRIVE_BRICK")
            robot.stop()
            aCount += 1
            robot.drive()
            lastA = "drive_brick"



def main():
    robot = Robot()

    print("starting up")
    start_time = time.time()
    robot.set_robot_speed(70)
    new_time = 0

    while(new_time < (start_time + 2)):
        new_time = time.time()

    robot.drive()
    robot.color_sensor.mode = 'COL-REFLECT'
    robot.sonar.mode = "US-DIST-CM"
    countTo = 3
    count = 0
    col = robot.sense_color()
    last_turn = "right"
    bCount = countTo

    robot.turnNew("right")

    while(new_time < (start_time + 4)):
        new_time = time.time()
    robot.stop()
    robot.drive()
    # **************************************** goes straight into brick
    brickEncounter(robot)
    while not robot.btn.any():
    #while(new_time < (start_time + 30)):
        print(robot.sonar.value())
        if(robot.sonar.value() < 250):
            print("SONAR LESS THAN 15")
            brickEncounter(robot)
            last_turn = "left"
            robot.driveEmpty(2000, 75, 25)
            time.sleep(2)
            robot.stop()

        time.sleep(0.05)
        lastcol = col
        col = robot.sense_color()
        # print("COLOR:" + col)
        if(col >= 25 and lastcol >= 25):
            count += 1
        else:
            count = 0
        if(col < 25 and lastcol < 25):
            bCount += 1

        if (last_turn == "left" and bCount > 0 and count >= countTo):
            robot.stop()
            bCount = 0
            print("GOING RIGHT")
            robot.turnNew("right")
            last_turn = "right"
        elif (last_turn == "right" and bCount > 0 and count >= countTo):
            robot.stop()
            bCount = 0
            print("GOING LEFT")
            robot.turnNew("left")
            last_turn = "left"

       # new_time = time.time()
    robot.stop()


if __name__ == "__main__":
    main()


