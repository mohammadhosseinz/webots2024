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
        robot.readSide(self.team)
        while self.robot.step(TIME_STEP) != -1:
            while self.is_new_team_data():
                team_data = self.get_new_team_data()  # noqa: F841
                robot.readPacket(team_data)
                # Do something with team data
            if self.is_new_ball_data():
                robot.isNewBallData = True
            else:
                robot.isNewBallData = False
            robot.readGPS(self.get_gps_coordinates())
            robot.readAngle(self.get_compass_heading())
            robot.readSonar(self.get_sonar_values())
            # just robot 1
            robot.server()

            # attack
            if self.player_id == robot.decision[0]:
                robot.goPosition(robot.ballPosServer)
            # attack
            elif self.player_id == robot.decision[1]:
                robot.goPosition(robot.ballPosServer)
            # defence
            elif self.player_id == robot.decision[2]:
                robot.goPosition(robot.goali)

            robot.checkMotor()
            self.left_motor.setVelocity(robot.leftMotor)
            self.right_motor.setVelocity(robot.rightMotor)
            self.send_data_to_team(
                self.player_id,
                robot.isNewBallData,
                robot.ballDist,
                robot.ballPosServer,
                robot.gps,
                robot.makeDecision,
            )
