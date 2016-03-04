class pd_controller:
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.prevOutput = 0
        self.prevError = 0
        self.currErr = 0

        self.prevTime = 0

    def update(self, goal, output, time):
        #calculate vel
        self.currErr = goal - output
        vel = (self.currErr * self.Kp) + (((self.currErr - self.prevError) / (time - self.prevTime)) * self.Kd)
        self.prevTime = time
        self.prevError = self.currErr
        return vel