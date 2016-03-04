"""
Sample Code for Lab1
Use "run.py [--sim] lab1" to execute
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
        self.create.start()
        self.create.safe()

        self.create.drive_direct(50,50)

        self.create.stop()
