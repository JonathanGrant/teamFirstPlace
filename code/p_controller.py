class p_controller:
    def __init__(self, Kp):
        self.Kp = Kp

    def update(self, goal, output):
        #calculate vel
        vel = (goal - output) * self.Kp
        return vel