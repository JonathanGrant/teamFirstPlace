"""
Sample Code for Lab2 for testing the sonar
Use "run.py [--sim] lab2_sonar_test" to execute
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
        while True:
            print(self.sonar.get_distance())
            self.time.sleep(0.1)
