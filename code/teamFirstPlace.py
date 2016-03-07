import create2
import math
import odometry
import p_controller
import pd_controller
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
        self.pidTheta = pid_controller.PIDController(200, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.pd_controller = pd_controller.PDController(1000, 100, -75, 75)
        self.waypoints = [[2.0, 0.0], [3.0, 2.0], [2.5, 2.0], [0.0, 1.5], [0.0, 0.0]]
        
    def goTowardWaypoint(self, goal_x, goal_y, state):
        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
        goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
        theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
        output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

        # improved version 2: fuse with velocity controller
        self.distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
        output_distance = self.pidDistance.update(0, self.distance, self.time.time())
        self.create.drive_direct(int(output_theta + output_distance)+self.base_speed, int(-output_theta + output_distance)+self.base_speed)
        
    def goTowardMidWayPoint(self, goal_x, goal_y, state, midX, midY):
        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
        goal_theta = math.atan2(midY - self.odometry.y, midX - self.odometry.x)
        theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
        output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

        # improved version 2: fuse with velocity controller
        self.distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
        self.distanceToMidPoint = math.sqrt(math.pow(midX - self.odometry.x, 2) + math.pow(midY - self.odometry.y, 2))
        output_distance = self.pidDistance.update(0, self.distance, self.time.time())
        self.create.drive_direct(int(output_theta + output_distance)+self.base_speed, int(-output_theta + output_distance)+self.base_speed)
        
    def turnCreate(self, goalTheta, state, pos):
        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
        theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
        output_theta = self.pidTheta.update(self.odometry.theta, goalTheta, self.time.time())

        # improved version 2: fuse with velocity controller
        if pos:
            self.create.drive_direct(int(output_theta), int(-output_theta))
        else:
            self.create.drive_direct(-int(output_theta), int(output_theta))
        return output_theta
    
    def turnCreateAllTheWay(self, goalTheta, state, pos):
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                output_theta = self.pidTheta.update(self.odometry.theta, goalTheta, self.time.time())

                # improved version 2: fuse with velocity controller
                if pos:
                    self.create.drive_direct(int(output_theta), int(-output_theta))
                else:
                    self.create.drive_direct(-int(output_theta), int(output_theta))
                if output_theta < 10:
                    break
    
    def followObstacle(self):
        self.servo.go_to(70)
        self.time.sleep(2)

        goal_distance = 0.5
        base_speed = 100

        while True:
            distance = self.sonar.get_distance()
            if distance is not None:
                print(distance)
                #output = self.p_controller.update(distance, goal_distance)
                output = self.pd_controller.update(distance, goal_distance, self.time.time())
                self.create.drive_direct(int(base_speed - output), int(base_speed + output))
                self.time.sleep(0.01)
        
    def goAroundObstacle(self, startTheta):
        startTime = self.time.time()
        goalTheta = math.pi / 2
        prevDist = 0
        countToTen = 0
        pos = False
        Started = False
        while True:
            countToTen += 1
            state = self.create.update()
            if state is not None:
                #Turn create around 360 degrees, and scan continuously
                if self.turnCreate(goalTheta+startTheta, state, pos) < 10:
                    goalTheta += math.pi / 2
                #Get the obstacle distance
                obstacleDist = self.sonar.get_distance()
                if obstacleDist > 1.2 and not Started:
                    Started = True
                    startTime = self.time.time()
                elif obstacleDist > 1.2 and Started and self.time.time() - startTime > 0.5:
                    break
                if obstacleDist <= 1.2:
                    Started = False
                if countToTen >= 10:
                    countToTen = 0
                    if obstacleDist < prevDist:
                        #reverse directions
                        pos = not pos
                    prevDist = obstacleDist
                    
    def getWaypointOutsideObstacle(self, startTheta):
        #scan servo every 15 degrees until we get a place that has a standard value
        #calculate the waypoint, which should be at (x,y) = (distCosTheta,distSinTheta)
        startTime = self.time.time()
        goalTheta = math.pi / 2
        prevDist = 0
        countToTen = 0
        pos = False
        Started = False
        print(startTheta)
        while True:
            countToTen += 1
            state = self.create.update()
            if state is not None:
                #Turn create around 360 degrees, and scan continuously
                if self.turnCreate(goalTheta+startTheta, state, pos) < 10:
                    goalTheta += math.pi / 2
                #Get the obstacle distance
                obstacleDist = self.sonar.get_distance()
                if obstacleDist > 1.2 and not Started:
                    Started = True
                    startTime = self.time.time()
                elif obstacleDist > 1.2 and Started and self.time.time() - startTime > 0.1:
                    break
                if obstacleDist <= 1.2:
                    Started = False
                if countToTen >= 10:
                    countToTen = 0
                    if obstacleDist < prevDist:
                        #reverse directions
                        pos = not pos
                    prevDist = obstacleDist
        #now create the waypoint
        #dist on sim is 1.4
        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
        theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
        print([1.4 * math.cos(theta) + self.odometry.x, 1.4 * math.sin(theta) + self.odometry.y])
        print(theta)
        return [1.4 * math.cos(theta) + self.odometry.x, 1.4 * math.sin(theta) + self.odometry.y]

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
		
        print("Ready, Set, GO!")

        for goal_x, goal_y in self.waypoints:
            self.base_speed = 100
            start_time = self.time.time()
            midWayPoint = False
            midX = 0
            midY = 0
            self.distanceToMidPoint = 999
            self.distance = 999
            #Rotate towards the waypoint
#            while True:
#                state = self.create.update()
#                if state is not None:
#                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
#                    newTheta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
#                    self.turnCreateAllTheWay(newTheta, state, True)
#                    break
            while True:
                state = self.create.update()
                if state is not None:
                    #Check in front for an immediate obstacle
                    obstacleDist = self.sonar.get_distance()
                    if obstacleDist < 0.75 and obstacleDist < self.distance:
                        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                        theta = math.atan2(self.odometry.y, self.odometry.x)
#                        self.goAroundObstacle(theta)
#                        self.followObstacle()
                        midWayPoint = True
                        self.distanceToMidPoint = 999
                        midX, midY = self.getWaypointOutsideObstacle(theta)
                    elif midWayPoint:
                        self.goTowardMidWayPoint(goal_x, goal_y, state, midX, midY)
                    else:
                        self.goTowardWaypoint(goal_x, goal_y, state)
                    if self.distance < 0.1:
                        break
                    elif self.distanceToMidPoint < 0.1 and midWayPoint:
                        midWayPoint = False
                        self.distanceToMidPoint = 999
                        #Rotate towards the waypoint
#                        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
#                        newTheta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
#                        self.turnCreateAllTheWay(newTheta, state, True)