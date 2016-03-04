"""
Sample Code for Lab3
Use "run.py [--sim] lab3" to execute
"""

import create2
import math
import csv

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
        
    def sleepnprint(self, t):
        #directionState = 0
        counter = 0
        
        tempTime = self.time.time()
        while True:
            counter += 1
            if counter % 10 == 0:
                self.state = self.create.update()
                if self.state is not None:

                    if self.prevR > 0 and self.state.rightEncoderCounts < 0:
                        if self.prevR > 32000:
                            self.prevR = self.state.rightEncoderCounts -((32767 - self.prevR) + (-32768 - self.state.rightEncoderCounts))
                    if self.prevR < 0 and self.state.rightEncoderCounts > 0:
                        if self.prevR < -32000:
                            self.prevR = self.state.rightEncoderCounts + ((-32768 - self.prevR) + (32767 - self.state.rightEncoderCounts))
                    if self.prevL > 0 and self.state.leftEncoderCounts < 0:
                        if self.prevL > 32000:
                            self.prevL = self.state.rightEncoderCounts -((32767 - self.prevL) + (-32768 - self.state.leftEncoderCounts))
                    if self.prevL < 0 and self.state.leftEncoderCounts > 0:
                        if self.prevL < -32000:
                            self.prevL = self.state.rightEncoderCounts + ((-32768 - self.prevL) + (32767 - self.state.leftEncoderCounts))

                    #Calculate new right and left
                    rightEncoderCounts = self.state.rightEncoderCounts - self.prevR
                    leftEncoderCounts = self.state.leftEncoderCounts - self.prevL
                    self.prevR = self.state.rightEncoderCounts
                    self.prevL = self.state.leftEncoderCounts

                    #pi * D * R / N
                    deltaRight = math.pi * rightEncoderCounts * (72 / 1000) / 508.8  #Diameter in meters
                    deltaLeft = math.pi * leftEncoderCounts * (72 / 1000) / 508.8

                    self.distanceRight += deltaRight
                    self.distanceLeft += deltaLeft

                    distance = (deltaRight + deltaLeft) / 2
                    self.x += distance * math.cos(self.theta)
                    self.y += distance * math.sin(self.theta)

                    #Theta = (dR - dL) / W
                    self.theta = (self.distanceRight - self.distanceLeft) / (235.0 / 1000.0) #Wheel distance in meters

                    self.ppr = self.prevR
                    self.ppl = self.prevL

                    self.tempx += distance * math.cos(self.theta)
                    self.tempy += distance * math.sin(self.theta)


                    print(self.x, ",", self.y)
                    if t < self.time.time() - tempTime:
                        break
    
    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        
        #Create state and other vars
        self.state = None
        self.distanceRight = 0
        self.distanceLeft = 0
        self.theta = 0
        self.prevR = 0
        self.prevL = 0
        self.x=0
        self.y=0
        self.state = self.create.update()
            
        self.prevR = self.state.rightEncoderCounts
        self.prevL = self.state.leftEncoderCounts
        self.ppr = self.prevR
        self.ppl = self.prevL
        
        self.newTheta = 0
        self.tempx = 0
        self.tempy = 0
        #directionState = 0
        
        counter = 0
        
        while True:
            #Lets have the robot move a bit
#            if directionState == 0:
#                self.create.drive_direct(100, 100)
#            elif directionState == 1:
#                self.create.drive_direct(50, -50)
            self.create.drive_direct(300, 300)
            self.sleepnprint(3)
            self.create.drive_direct(300,-300)
            self.sleepnprint(3.3/3)
            self.create.drive_direct(300, 300)
            self.sleepnprint(4.5/3)
            self.create.drive_direct(30,-30)
            self.sleepnprint(3.3/3)
            self.create.drive_direct(300, 300)
            self.sleepnprint(9/3)
            self.create.drive_direct(300,-300)
            self.sleepnprint(3.3/3)
            self.create.drive_direct(300, 300)
            self.sleepnprint(4.5/3)
            self.create.drive_direct(300,-300)
            self.sleepnprint(3.3/3)