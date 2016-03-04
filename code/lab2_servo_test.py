"""
Sample Code for Lab2 for testing the servo motor
Use "run.py [--sim] lab2_servo_test" to execute
"""


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
        self.servo.go_to(90)
        self.time.sleep(2)
        self.servo.go_to(-90)
        self.time.sleep(2)
        self.servo.go_to(0)
        self.time.sleep(1)
