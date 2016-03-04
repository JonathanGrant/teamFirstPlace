import create2
import math
import odometry
from p_controller import p_controller
from pd_controller import pd_controller
from pid_controller import pid_controller

class Run:
    def __init__(self, create, time, sonar, servo):
        """Constructor.

        Args:
            create (robot.Create2Driver)
            time (time)
            sonar (robot.Sonar)
            servo (robot.Servo)
        """
        self.create = create
        self.time = time
        self.sonar = sonar
        self.servo = servo
        self.odometry = odometry.Odometry()
        
    def run(self):
        self.create.start()
        self.create.safe()
        
        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        
        self.state = self.create.update()
        if self.state is not None:
            stdVel = 0
            self.time.sleep(0.1)
            betterControl = pid_controller(100, 10, 10, self.time.time(), 10)

            speedControl = pid_controller(200, 10, 10, self.time.time(), 0.1)
            
            initAngle = 0

            self.odometry.update(self.state.leftEncoderCounts, self.state.rightEncoderCounts)

            theta = self.odometry.theta
            
            goalX = 1
            goalY = 1
            deltaX = goalX - self.odometry.x
            deltaY = goalY - self.odometry.y
            
            goalAngle = math.atan2(deltaY, deltaX)
            
            while True:
                self.state = self.create.update()
                if self.state is not None:
                    self.odometry.update(self.state.leftEncoderCounts, self.state.rightEncoderCounts)
                    theta = self.odometry.theta
                    
                    deltaX = goalX - self.odometry.x
                    deltaY = goalY - self.odometry.y
                    
                    goalAngle = math.atan2(deltaY, deltaX)
                    
                    betterVel = betterControl.update(goalAngle, theta, self.time.time())
                    
                    eucDist = math.sqrt(pow(deltaX, 2) + pow(deltaY, 2))
                    
                    stdVel = speedControl.update(0, eucDist, self.time.time())
                    
                    print(theta, goalAngle)
                    
                    if abs(eucDist) < 0.05:
                        break
                    
                    rVel = -betterVel - stdVel
                    lVel = (betterVel) - stdVel
                    if rVel > 200:
                        rVel = 200
                    elif rVel < -200:
                        rVel = -200
                    if lVel > 200:
                        lVel = 200
                    elif lVel < -200:
                        lVel = -200
                    self.create.drive_direct(int(lVel), int(rVel))
                    self.time.sleep(0.1)