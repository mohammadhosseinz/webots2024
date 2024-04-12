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


class POINT:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

    def print(self):
        print("x:", self.x, "y:", self.y)

    def set(self, x, y):
        self.x = x
        self.y = y
        return [x, y]


class LINE:
    def __init__(self, point1, point2) -> None:
        self.x0 = point1.x
        self.y0 = point1.y
        self.x1 = point2.x
        self.y1 = point2.y
        self.m = (self.y1 - self.y0) / (self.x1 - self.x0)
        self.c = self.y0 - self.m * self.x0

    def print(self) -> None:
        print("y =", self.m, "x +", self.c)

    def line_to_point(self, point) -> float:
        dist = abs(point.y - self.m * point.x - self.c) / math.sqrt(
            1 + self.m**2
        )
        return dist

    def line_to_line(self, line):
        if self.m == line.m:
            if line.m * self.x0 + line.c == self.y0:
                print("inf ans")
                return "inf"
            else:
                print("zero ans")
                return "zero"
        else:
            x = (line.c - self.c) / (self.m - line.m)
            y = self.m * x + self.c
            point = POINT(x, y)
            return point


class ROBOT:
    def __init__(self) -> None:
        self.makeDecision = [0, 0, 0]
        self.decision = [0, 0, 0]
        self.goali = [0, -0.67]
        self.ballPosServer = [0, 0]
        self.data = [
            0,
            {
                "ball": False,
                "dist": 0,
                "ballpos": [0, 0],
                "gps": [0, 0],
                "decision": [3, 2, 1],
            },
            {
                "ball": False,
                "dist": 0,
                "ballpos": [0, 0],
                "gps": [0, 0],
                "decision": [3, 2, 1],
            },
            {
                "ball": False,
                "dist": 0,
                "ballpos": [0, 0],
                "gps": [0, 0],
                "decision": [3, 2, 1],
            },
        ]
        self.isNewBallData = False
        self.Cposition = 1
        self.Ccompass = 0
        self.compass = 0
        self.gps = [0, 0]
        self.angle2go = 0
        self.position2go = [0, 0]
        self.leftMotor = 0
        self.rightMotor = 0
        self.ballPos = [0, 0]
        self.ballDist = 0
        self.sonar = {
            "left": 0,
            "right": 0,
            "front": 0,
            "back": 0,
        }

    def readSide(self, team):
        if team == "Y":
            self.Cposition = 1
            self.Ccompass = 180
        else:
            self.Cposition = -1
            self.Ccompass = 0

    def readAngle(self, compass: float) -> float:
        heading = math.degrees(compass)

        if heading > 180 and heading < 270:
            heading = heading - 360
        heading = heading - self.Ccompass
        if heading < -180:
            heading = heading + 360
        self.compass = heading
        return heading

    def readSonar(self, sonar: dict) -> dict:
        self.sonar = sonar
        return sonar

    def readGPS(self, gps: list) -> list:
        self.gps = [self.Cposition * gps[0], self.Cposition * gps[1]]
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
        distance = self.distance(position)
        if distance < 0.001:
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

            self.goAngle(angle)
            self.leftMotor = self.leftMotor + 7
            self.rightMotor = self.rightMotor + 7
            return [self.leftMotor, self.rightMotor]

    def ball_distance(self, strength) -> float:
        distance = math.sqrt(1 / float(strength))
        self.ballDist = distance
        return self.ballDist

    def find_decision(self, id):
        if id != 1:
            self.decision = self.data[1]["decision"]

    ## just server
    def sorting(self, input):
        return input["dist"]

    def find_min_dist2point(self, point):
        robot2Point = [
            {"dist": self.point2point_distance(self.gps, point), "id": 1},
            {
                "dist": self.point2point_distance(self.data[2]["gps"], point),
                "id": 2,
            },
            {
                "dist": self.point2point_distance(self.data[3]["gps"], point),
                "id": 3,
            },
        ]
        robot2Point.sort(key=self.sorting)
        return robot2Point

    def point2point_distance(self, pos1, pos2):
        dist = math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)
        return dist

    def ball_location(self, ball, strength) -> list:
        xball = (
            -sin(radians(self.compass))
            * ball[1]
            * self.ball_distance(strength)
            - cos(radians(self.compass))
            * ball[0]
            * self.ball_distance(strength)
            + self.gps[0]
        )

        yball = (
            +cos(radians(self.compass))
            * ball[1]
            * self.ball_distance(strength)
            - sin(radians(self.compass))
            * ball[0]
            * self.ball_distance(strength)
            + self.gps[1]
        )
        self.ballPos = [float(xball), float(yball)]
        return self.ballPos

    def readPacket(self, packet):
        # data = {"id": id, "ball": ball, "ballpos": ballpos, "gps": gps}
        self.data[packet["id"]] = packet
        print(self.data)

    def server(self):

        if (
            self.isNewBallData
            and self.ballDist <= self.data[2]["dist"]
            and self.ballDist <= self.data[3]["dist"]
        ):
            self.ballPosServer = self.ballPos
        if (
            self.data[2]["ball"]
            and self.data[2]["dist"] <= self.ballDist
            and self.data[2]["dist"] <= self.data[3]["dist"]
        ):
            self.ballPosServer = self.data[2]["ballpos"]
        if (
            self.data[3]["ball"]
            and self.data[3]["dist"] <= self.ballDist
            and self.data[3]["dist"] <= self.data[2]["dist"]
        ):
            self.ballPosServer = self.data[3]["ballpos"]
        sortedRobot2Ball = self.find_min_dist2point(self.ballPosServer)
        sortedRobot2Goali = self.find_min_dist2point(self.goali)
        decision = [0, 0, 0]
        decision[0] = sortedRobot2Ball[0]["id"]
        decision[2] = sortedRobot2Goali[0]["id"]
        if decision[0] == decision[2]:
            decision[2] = sortedRobot2Goali[1]["id"]
        if 1 in decision and 2 in decision:
            decision[1] = 3
        elif 1 in decision and 3 in decision:
            decision[1] = 2
        elif 2 in decision and 3 in decision:
            decision[1] = 1

        self.makeDecision = decision
        self.decision = decision
