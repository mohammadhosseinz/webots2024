# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            # Sensor
            # heading = math.degrees(self.compass.getValues()[2])
            heading = math.degrees(self.get_compass_heading())
            if heading > 180 and heading < 270:
                heading = heading - 360

            # motor
            ## 1 heading
            # if heading > -1 and heading < 1:
            #     self.left_motor.setVelocity(0)
            #     self.right_motor.setVelocity(0)
            # elif heading > 0:
            #     self.left_motor.setVelocity(-10)
            #     self.right_motor.setVelocity(10)
            # elif heading < 0:
            #     self.left_motor.setVelocity(10)
            #     self.right_motor.setVelocity(-10)

            ## 2 heading
            # left_motor = -heading * (1 / 18)
            # right_motor = heading * (1 / 18)
            # self.left_motor.setVelocity(left_motor)
            # self.right_motor.setVelocit(right_motor)

            # 3 heading

            gps = self.get_gps_coordinates()
            gps = [-gps[0], -gps[1]]
            position = [0 - gps[0], 0 - gps[1]]
            teta = math.degrees(math.atan2(abs(position[1]), abs(position[0])))
            distance = math.sqrt(position[0] ** 2 + position[1] ** 2)

            if distance < 0.1:
                left_motor = -(heading**1) * (10 / (180**1)) + 1
                right_motor = (heading**1) * (10 / (180**1)) + 1
            else:
                if position[0] >= 0 and position[1] >= 0:
                    angle = teta - 90
                elif position[0] <= 0 and position[1] >= 0:
                    angle = 90 - teta
                elif position[0] >= 0 and position[1] <= 0:
                    angle = -90 - teta
                elif position[0] <= 0 and position[1] <= 0:
                    angle = 90 + teta

                heading = heading - angle
                if heading < -180:
                    heading = heading + 360
                left_motor = -(heading**1) * (10 / (90**1)) + 7
                right_motor = (heading**1) * (10 / (90**1)) + 7
            print(left_motor, " ==== ", right_motor)

            if left_motor > 10:
                left_motor = 10
            if left_motor < -10:
                left_motor = -10
            if right_motor > 10:
                right_motor = 10
            if right_motor < -10:
                right_motor = -10
            self.left_motor.setVelocity(left_motor)
            self.right_motor.setVelocity(right_motor)
