# arm_class.py

class Arm:
    def __init__(self, L=1.0):
        # State variables
        self.theta = 0.0       # position (rad)
        self.theta_d = 0.0     # velocity (rad/s)
        self.theta_dd = 0.0    # acceleration (rad/s²)
        self.jerk = 0.0        # jerk (rad/s³)
        self.L = L             # link length

        self.dt_us = 0         # microseconds between updates

    def update(self, dt_us):
        """Integrate jerk → acceleration → velocity → position."""
        self.dt_us = dt_us
        dt = dt_us / 1_000_000.0  # convert to seconds

        # integrate
        self.theta_dd += self.jerk * dt
        self.theta_d  += self.theta_dd * dt
        self.theta    += self.theta_d * dt

    def set_jerk(self, j):
        """Set a new constant jerk value."""
        self.jerk = j