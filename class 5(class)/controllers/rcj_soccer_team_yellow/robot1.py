# rcj_soccer_player controller - ROBOT Y1

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

            # print(heading)
            gps = self.get_gps_coordinates()
            # print(gps)
