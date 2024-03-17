import math
from math import sin, degrees, radians, cos


def get_direction(ball_vector: list) -> int:
    """Get direction to navigate robot to face the ball

    Args:
        ball_vector (list of floats): Current vector of the ball with respect
            to the robot.

    Returns:
        int: 0 = forward, -1 = right, 1 = left
    """
    if -0.13 <= ball_vector[1] <= 0.13:
        return 0
    return -1 if ball_vector[1] < 0 else 1


class ROBOT:
    def __init__(self) -> None:
        self.compass = 0
        self.gps = [0, 0]
        self.angle2go = 0
        self.position2go = [0, 0]
        self.leftMotor = 0
        self.rightMotor = 0
        self.ballPos = [0, 0]
        self.ballDist = 0

    def readAngle(self, compass: float) -> float:
        heading = math.degrees(compass)
        if heading > 180 and heading < 270:
            heading = heading - 360
        self.compass = heading
        return heading

    def readGPS(self, gps: list) -> list:
        self.gps = [-gps[0], -gps[1]]
        return gps

    def goAngle(self, angle: float) -> list:
        heading = self.compass - angle
        if heading < -180:
            heading = heading + 360
        left_motor = -heading * (1 / 18)
        right_motor = heading * (1 / 18)
        self.leftMotor = left_motor
        self.rightMotor = right_motor
        return [left_motor, right_motor]

    def checkMotor(self) -> None:
        if self.leftMotor > 10:
            self.leftMotor = 10
        if self.leftMotor < -10:
            self.leftMotor = -10
        if self.rightMotor > 10:
            self.rightMotor = 10
        if self.rightMotor < -10:
            self.rightMotor = -10

    def distance(self, position: list) -> float:
        distance = math.sqrt(position[0] ** 2 + position[1] ** 2)
        return distance

    def goPosition(self, position: list) -> list:
        position = [position[0] - self.gps[0], position[1] - self.gps[1]]
        teta = math.degrees(math.atan2(abs(position[1]), abs(position[0])))
        distance = distance(self, position)
        if distance < 0.1:
            self.leftMotor = 0
            self.rightMotor = 0
        else:
            if position[0] >= 0 and position[1] >= 0:
                angle = teta - 90
            elif position[0] <= 0 and position[1] >= 0:
                angle = 90 - teta
            elif position[0] >= 0 and position[1] <= 0:
                angle = -90 - teta
            elif position[0] <= 0 and position[1] <= 0:
                angle = 90 + teta

            self.goAngle(self, angle)
            self.leftMotor = self.leftMotor + 7
            self.rightMotor = self.rightMotor + 7
            return [self.leftMotor, self.rightMotor]

    def ball_distance(self, strength) -> float:
        distance = math.sqrt(1 / strength)
        self.ballDist = distance
        return self.ballDist

    def ball_location(self, ball) -> list:
        xball = (
            sin(radians(self.compass)) * ball[1]
            + cos(radians(self.compass)) * ball[0]
        )
        yball = (
            cos(radians(self.compass)) * ball[1]
            - sin(radians(self.compass)) * ball[0]
        )
        self.ballPos = [xball, yball]
        return self.ballPos
