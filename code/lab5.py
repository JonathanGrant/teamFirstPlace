from p_controller import p_controller
from pd_controller import pd_controller

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
        
    def run(self):
        self.create.start()
        self.create.safe()
        
        stdVel = 150
        betterControl = pd_controller(30, 300)
        
        self.servo.go_to(75)
        self.time.sleep(3)
        initDist = self.sonar.get_distance()
        betterVel = betterControl.update(initDist, self.sonar.get_distance(), self.time.time())
        self.time.sleep(0.01)
        while True:
            if self.sonar.get_distance() > 1.5:
                pass
            betterVel = betterControl.update(initDist, self.sonar.get_distance(), self.time.time())
            rVel = betterVel + stdVel
            lVel = (-betterVel) + stdVel
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