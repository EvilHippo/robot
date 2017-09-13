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
        # print("Sending command")
        # self.motor_left.run_forever()
        # self.motor_right.run_forever()
        self.motor_left.run_timed(time_sp=100000, speed_sp=50)
        self.motor_right.run_timed(time_sp=100000, speed_sp=50)

    def turnNew(self, dir):
        # print("********************************* IN NEW TURN *************************")
        if (dir == "right"):
            self.motor_left.run_timed(time_sp=100000, speed_sp=75)
            self.motor_right.run_timed(time_sp=100000, speed_sp=10)

        if (dir == "left"):
            self.motor_right.run_timed(time_sp=100000, speed_sp=75)
            self.motor_left.run_timed(time_sp=100000, speed_sp=10)

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
    last_turn = "right"
    while (new_time < (start_time + 2)):
        new_time = time.time()

    robot.drive()
    robot.color_sensor.mode = 'COL-REFLECT'
    moved = False
    turning = False
    # while(True):
    # robot.turnNew("right")
    countTo = 5
    count = 0
    col = robot.sense_color()
    while not robot.btn.any():
        # while(new_time < (start_time + 60)):

        time.sleep(0.1)
        lastcol = col
        col = robot.sense_color()
        print(col)
        if ((col < 25 and turning is True) or (turning is False and col >= 25)):
            if ((lastcol >= 25 and col >= 25) or (lastcol < 25 and col < 25)):
                count += 1
            else:
                count = 0
        if (((col >= 25 and turning is False) or (turning is True and col < 25)) and count > countTo):
            moved = False
            turning = False
            robot.stop()
            count = 0
            print("STOPPING")
        if (col >= 25 and moved is False):
            count = 0
            moved = True
            turning = True
            if (last_turn == "left"):
                print("GOING RIGHT")
                robot.turnNew("right")
                last_turn = "right"
            elif (last_turn == "right"):
                print("GOING LEFT")
                robot.turnNew("left")
                last_turn = "left"
            else:
                print("INVALID VALUE IN TURN")
        """
        elif(moved is False):
            count = 0
            print("GOING STRAIGHT")
            robot.drive()
            moved = True
        """
        new_time = time.time()

    """
    robot.set_robot_speed(15)

    for _ in range(7):
        robot.drive()

        while robot.sense_color() != 1:
            pass

        robot.turn("Left", 5)
    """
    robot.stop()


if __name__ == "__main__":
    main()


