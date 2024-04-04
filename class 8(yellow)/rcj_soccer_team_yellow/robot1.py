# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from utils import POINT
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):
    def run(self):
        robot = utils.ROBOT()
        pos = [0, 0]
        p1 = POINT(1, 1)
        p1.set(2, 2)
        line = utils.LINE(POINT(0, 0), POINT(1, 1))
        robot.readSide(self.team)
        while self.robot.step(TIME_STEP) != -1:
            # print(robot.ballPosServer)
            while self.is_new_team_data():
                team_data = self.get_new_team_data()  # noqa: F841
                robot.readPacket(team_data)
                # Do something with team data

            # just robot 1
            robot.server()
            # line.print()
            # print(line.line_to_point(POINT(0, 0)))
            robot.readGPS(self.get_gps_coordinates())
            robot.readAngle(self.get_compass_heading())
            robot.readSonar(self.get_sonar_values())
            # print(robot.sonar)
            ballData = 0
            if self.is_new_ball_data():
                robot.isNewBallData = True
                ballData = self.get_new_ball_data()
                # print(ballData["direction"])
                pos = robot.ball_location(
                    [ballData["direction"][1], ballData["direction"][0]],
                    ballData["strength"],
                )
                # print(robot.gps, pos)
            else:
                robot.isNewBallData = False
            # print("robot1:", robot.ballPosServer)
            robot.goPosition(robot.ballPosServer)
            # print("yellow:", robot.compass)
            robot.checkMotor()
            # print(robot.leftMotor)
            self.left_motor.setVelocity(robot.leftMotor)
            self.right_motor.setVelocity(robot.rightMotor)
            self.send_data_to_team(
                self.player_id,
                robot.isNewBallData,
                robot.ballDist,
                robot.ballPosServer,
                robot.gps,
            )
