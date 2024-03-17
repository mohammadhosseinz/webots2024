# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):
    def run(self):
        robot = utils.ROBOT()
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_ball_data():
                ballData = self.get_new_ball_data()["direction"]
                pos = robot.ball_location([ballData[1], ballData[0]])
                print(pos)
                direction = utils.get_direction(ballData)
                if direction == 0:
                    robot.leftMotor = 10
                    robot.rightMotor = 10
                if direction == 1:
                    robot.leftMotor = 10
                    robot.rightMotor = 4
                if direction == -1:
                    robot.leftMotor = 4
                    robot.rightMotor = 10
            else:
                robot.leftMotor = 0
                robot.rightMotor = 0
            robot.checkMotor()
            # print(robot.leftMotor)
            self.left_motor.setVelocity(robot.leftMotor)
            self.right_motor.setVelocity(robot.rightMotor)
