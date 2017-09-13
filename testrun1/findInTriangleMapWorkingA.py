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
        self.motor_left = ev3.LargeMotor("outA")
        self.motor_right = ev3.LargeMotor("outD")
        self.motor_left.reset()
        self.motor_right.reset()
        self.color_sensor = ev3.ColorSensor("in3")
        self.color_sensor.mode = 'COL-REFLECT'
        self.btn = ev3.Button()
        self.sonar = ev3.UltrasonicSensor("in1")

        # TODO comment this out later
        self.sonar2 = ev3.UltrasonicSensor("in4")

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

    def turnSameSpot(self, dir):
        self.motor_left.stop()
        self.motor_right.stop()

        if(dir == "right"):
            self.motor_left.run_timed(time_sp=100000, speed_sp=100)

        if(dir == "left"):
            self.motor_right.run_timed(time_sp=100000, speed_sp=100)

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

    def driveClose(self):
        sonar = self.sonar.value()
        lastSeen = sonar
        startValue = sonar
        print("SONAR AT START OF SEARCH: ", startValue)
        targetLost = False
        self.drive()
        count = 0
        while not self.btn.any():
            if count >= 10:
                count = 0
                print("Sonar at count: ", sonar)
                print()
            else:
                count += 1
            lastS = sonar
            sonar = self.sonar.value()
            if(sonar < 150):
                print("**********TARGET REACHED, EXITING DRIVECLOSE*********")
                self.stop()
                return

            if (abs(sonar - lastS) > 200) and ((targetLost is False and lastS < startValue) or (sonar > startValue and targetLost is False)):
                targetLost = True
                lastSeen = lastS
                print("driveClose: TARGET LOST, sonar: ", lastSeen)
            if (targetLost is True):
                self.stop()
                self.turnSameSpot("left")
                while(not(sonar - 20 <= lastSeen and sonar + 20 >= lastSeen) and sonar > startValue):
                    sonar = self.sonar.value()
                self.stop()
                print("driveClose: TARGET FOUND, sonar: ", sonar)
                self.drive()
                targetLost = False

            time.sleep(0.2)


def main():
    robot = Robot()

    print("starting up")
    start_time = time.time()
    robot.set_robot_speed(70)
    new_time = 0

    while(new_time < (start_time + 3)):
        new_time = time.time()

    robot.color_sensor.mode = 'COL-REFLECT'
    robot.sonar.mode = "US-DIST-CM"


    # for gyro and sonar testing (should probably just use a seperate file..)
    """
    c = 0
    #robot.turnSameSpot("left")
    son = robot.sonar.value()
    while(not robot.btn.any()):
        lastson = son
        son = robot.sonar.value()
        #print(son)
        #if(abs(son - lastson) > 100) and son < 700:
         #   print("Change ammount: " , abs(son - lastson))
        if c == 15:
            print("sonar1: ", robot.sonar.value())
            print("sonar2: ", robot.sonar2.value())
            c = 0
        else:
            c += 1
        #print(robot.gyro.value() % 360)
    robot.stop()
    return
    """

    # ############################################## FIND 3 CLOSEST OBJECTS #################################
    sonar = robot.sonar.value()
    robot.turnSameSpot("left")
    count = 0
    oldFindTime = 0
    startGyro = robot.gyro.value()
    oldFindGyro = -1
    newFindGyro = -1
    while not robot.btn.any():
        lastS = sonar
        sonar = robot.sonar.value()
        gyro = robot.gyro.value()

        if count >= 10:
            #print("sonar: ", sonar)
            #print("gyro: ", gyro)
            count = 0
        else:
            count += 1

        # find start of close object
        if (abs(sonar - lastS) > 100 and sonar <= 700 and oldFindGyro == -1):
            oldFindGyro = gyro % 360
            print("found close object, gyro: ", oldFindGyro)

        # find end of close object
        elif (abs(sonar - lastS) > 100 and sonar > 700) and oldFindGyro != -1:
            newFindGyro = gyro % 360
            print("found far away object, gyro: ", newFindGyro)

        # if both start and end of object have been found, drive right until middle spot between value found
        #   (actual direction to object)
        if(oldFindGyro != -1 and newFindGyro != -1):
            robot.stop()
            robot.turnSameSpot("right")
            middle = (oldFindGyro + newFindGyro) / 2
            print("Middle: ", middle)
            while not(gyro - 2 <= middle and gyro + 2 >= middle) and not robot.btn.any():
                gyro = robot.gyro.value() % 360
            print("Found middle, gyro: ", gyro)
            robot.stop()
            robot.drive()
            # once middle is found, drive close to object
            while(sonar > 100 and not robot.btn.any()):
                sonar = robot.sonar.value()
            robot.stop()
            # once close, drive back a little
            robot.driveBack()
            while(sonar < 150 and not robot.btn.any()):
                sonar = robot.sonar.value()
            robot.stop()
            # once drive back is done, forcefully drive left for X time to ignore same object
            oldFindTime = time.time()
            robot.turnSameSpot("left")
            while(time.time() - 3 < oldFindTime) and not robot.btn.any():
                continue
            # once drive is done, reset old gyro value and keep driving left, repeating the pattern
            oldFindGyro = -1
            newFindGyro = -1
        time.sleep(0.1)


    """
    # TODO should probably remove this. Maybe. Possibly.
    # Probably not nessesary for this task. But needed if i decide to solve this with gyro

    targetLocked = False
    finding = False

    while not robot.btn.any():
    #while(new_time < (start_time + 30)):
        lastS = sonar
        sonar = robot.sonar.value()

        print(robot.sonar.value())

        if(robot.sonar.value() < 150):
            print("CLOSE TO TARGET")
            start_time = time.time()
            robot.stop()
            robot.turnNew("left")
            while(True):
                new_time = time.time()
                if(new_time < (start_time + 6)):
                    last_turn = "right"
                    targetLocked = False
                    break
                elif (new_time < (start_time + 4)):
                    robot.stop()
                    robot.turnNew("right")
                elif (new_time < (start_time + 2)):
                    robot.stop()
                    robot.drive()


        if(abs(sonar - lastS) > 200 and abs(sonar - lastS) < 500):
            print("TARGET CHANGED")
            if(targetLocked):
                if(last_turn == "right"):
                    last_turn = "left"
                else:
                    last_turn = "right"
                targetLocked = False
                finding = False
                forward = False
            elif(sonar < 1500):
                targetLocked = True


        if(targetLocked is True and forward is not True):
            robot.stop()
            forward = True
            robot.drive()
        elif (last_turn == "left" and targetLocked is False and finding is False):
            finding = True
            robot.stop()
            print("GOING RIGHT")
            robot.turnNew("right")
            last_turn = "right"
        elif (last_turn == "right" and targetLocked is False and finding is False):
            finding = True
            robot.stop()
            print("GOING LEFT")
            robot.turnNew("left")
            last_turn = "left"

        time.sleep(0.2)
        lastcol = col
        col = robot.sense_color()

       # new_time = time.time()
       """
    robot.stop()


if __name__ == "__main__":
    main()
