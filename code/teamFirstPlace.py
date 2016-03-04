import create2
import math
import odometry
import pd_controller2
import pid_controller


class Run:
    def __init__(self, create, time, sonar, servo):
        self.create = create
        self.time = time
        self.sonar = sonar
        self.servo = servo
        self.odometry = odometry.Odometry()
        # self.pidTheta = pd_controller2.PDController(500, 100, -200, 200, is_angle=True)
        self.pidTheta = pid_controller.PIDController(500, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(500, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.pidObstacle = pid_controller.PIDController(500,10,75,[-1,1], [-200, 200], is_angle=False)
        self.waypoints = [[2 ,0], [2 ,1], [0 ,1], [0 ,0]]

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        print("Ready, Set, GO!")
        atObstacle = False
        distanceFromObstacle = 3.3
        scanAngle = 0

        for goal_x, goal_y in self.waypoints:
            base_speed = 100
            start_time = self.time.time()
            while True:
                state = self.create.update()
                if state is not None:
                    base_speed = 100
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)

                    #Scan environment for obstacle
                    distanceFromObstacle = self.sonar.get_distance()
                    if distanceFromObstacle <= 1.0:
                        goal_theta = math.atan2(-45 - self.odometry.y, -45 - self.odometry.x)
                    else:
                        goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                    # improved version 2: fuse with velocity controller
                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    output_distance = self.pidDistance.update(0, distance, self.time.time())
                    self.create.drive_direct(int(output_theta + output_distance)+base_speed, int(-output_theta + output_distance)+base_speed)
                    if distance < 0.1:
                        break
