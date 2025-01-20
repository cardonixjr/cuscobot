class encoder():
    def __init__(self, ticks_p_revolution, radius):
        self.counter = 0
        self.ticks_p_revol = ticks_p_revolution
        self.radius = radius

    def count(self, value=1):
        self.counter += value

    def reset(self):
        self.counter = 0