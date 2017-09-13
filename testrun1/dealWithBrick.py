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

    def turnNew(self, dir):
        #print("********************************* IN NEW TURN *************************")
        if(dir == "right"):
            self.motor_left.run_timed(time_sp=100000, speed_sp=75)
            self.motor_right.run_timed(time_sp=100000, speed_sp=25)

        if(dir == "left"):
            self.motor_right.run_timed(time_sp=100000, speed_sp=75)
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

    while not robot.btn.any():
    #while(new_time < (start_time + 30)):

        time.sleep(0.05)
        lastcol = col
        col = robot.sense_color()
        print(col)
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


