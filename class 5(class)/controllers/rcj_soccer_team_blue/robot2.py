# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot2(RCJSoccerRobot):
    def run(self):
        # print("hello1")
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                angel = 0
                # print("hello2")
                # sensor
                heading = math.degrees(self.get_compass_heading())
                if heading >= 0 and heading <= 90:
                    heading = heading + 90
                elif heading > 90 and heading <= 270:
                    heading = heading - 270
                elif heading >= -90 and heading <= 0:
                    heading = heading + 90

                sin_value = math.sin(heading)
                cos_value = math.cos(heading)
                # print("sin:",sin_value)
                # print("cos:",cos_value)

                left_motor = -(heading + 90) * 1 / 13.5
                right_motor = (heading + 90) * 1 / 13.5

                # self.left_motor.setVelocity(left_motor)
                # self.right_motor.setVelocity(right_motor)

                # Set the speed to motors

                # Send message to team robots

                gps = self.get_gps_coordinates()
                gps = [-gps[0], -gps[1]]
                # print(gps)
                position = [0 - gps[0], 0 - gps[1]]
                # print(position, " ", type(position[0]))
                # print(position)
                teta = math.degrees(
                    math.atan2(abs(position[1]), abs(position[0]))
                )
                # print(teta)
                distance = math.sqrt(position[0] ** 2 + position[1] ** 2)
                # print(distance)
                if distance < 0.1:
                    left_motor = -(heading**1) * (10 / (180**1)) + 10
                    right_motor = -(heading**1) * (10 / (180**1)) + 10
                else:
                    if position[0] >= 0 and position[1] >= 0:
                        angel = teta - 90
                    elif position[0] <= 0 and position[1] >= 0:
                        angel = 90 - teta
                    elif position[0] >= 0 and position[1] <= 0:
                        angel = -90 - teta
                    elif position[0] <= 0 and position[1] <= 0:
                        angel = 90 + teta

                heading = heading - angel
                if heading < -180:
                    heading = heading + 360
                left_motor = -(heading**1) * (10 / (90**1)) + 7
                right_motor = -(heading**1) * (10 / (90**1)) + 7
                # print(angel, "---", teta)

                if left_motor > 10:
                    left_motor = 10
                if left_motor < -10:
                    left_motor = -10
                if right_motor > 10:
                    right_motor = 10
                if right_motor < -10:
                    right_motor = -10

                # Send message to team robots
            self.send_data_to_team(self.player_id)
