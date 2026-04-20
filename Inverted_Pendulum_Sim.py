import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class InvertedPendulum:
    """
    A simulation of a PD controlled inverted cartpole pendulum.
    Uses equations of motion derived from the Lagrangian and Euler-Lagrange formula.
    Gains were derived analytically from the second order ODE standard form 
    with a deired settling time of 2s and overshoot of 5%, then tuned experimentally.
    """
    def __init__(
        self,
        theta,
        mass_c,
        mass_p,
        length,
        g,
        controller,
        dt
    ):
        
        self.theta = theta
        self.theta_dot = 0
        self.theta_ddot = None
        self.sin_theta = np.sin(self.theta)
        self.cos_theta = np.cos(self.theta)
        self.x = 0
        self.x_dot = 0
        self.x_ddot = None
        self.mass_c = mass_c
        self.mass_p = mass_p
        self.length = length
        self.g = g
        self.dt = dt
        self.position = None
        self.input = None
        self.controller = controller

    def PD_control(self):
        self.input = 70*self.theta + 4.2*self.theta_dot
    
    def full_state_control(self):
        K = np.array([
            -25.71121762,
            -21.67234442,
            -127.71021537,
            -36.67234442
        ])

        X = np.array([
            [float(self.x)],
            [float(self.x_dot)],
            [float(self.theta)],
            [float(self.theta_dot)]
            ])
        
        self.input = -float((K @ X))
    
    def LQR(self):
        K = np.array([
            -3.16227766,
            -5.27615175,
            -61.18556668,
            -16.14107595
        ])
    
        X = np.array([
            [float(self.x)],
            [float(self.x_dot)],
            [float(self.theta)],
            [float(self.theta_dot)]
            ])
        
        self.input = -float((K @ X))


    def compute_theta_ddot(self):
        self.theta_ddot = (
            ((self.mass_p + self.mass_c)*self.g*self.sin_theta
            - self.input*self.cos_theta
            - self.mass_p*self.length*self.cos_theta*self.sin_theta*self.theta_dot**2)
            / (self.length*(self.mass_c + self.mass_p*self.sin_theta**2))
        )
    
    def compute_theta_dot(self):
        self.theta_dot = self.theta_dot + self.theta_ddot*self.dt

    def compute_theta(self):
        self.theta = self.theta + self.theta_dot*self.dt
    
    def compute_x_ddot(self):
        self.x_ddot = (
            (self.input
            - self.mass_p*self.g*self.cos_theta*self.sin_theta
            + self.mass_p*self.length*self.sin_theta*self.theta_dot**2)
            / (self.mass_c + self.mass_p*self.sin_theta**2)
        )
    
    def compute_x_dot(self):
        self.x_dot = self.x_dot + self.x_ddot*self.dt

    def compute_x(self):
        self.x = self.x + self.x_dot*self.dt

    def compute_position(self):
        self.position = ((self.x + self.length*self.sin_theta, self.length*self.cos_theta), self.x)

    def start_sim(self):
        self.compute_position()
        self.trajectory = [self.position]
        for _ in range(int((1/self.dt)*10)):
            self.sin_theta = np.sin(self.theta)
            self.cos_theta = np.cos(self.theta)
            if self.controller == "full state":
                self.full_state_control()
            elif self.controller == "PD":
                self.PD_control()
            elif self.controller == "LQR":
                self.LQR()
            else:
                break
            self.compute_theta_ddot()
            self.compute_x_ddot()
            self.compute_theta_dot()
            self.compute_x_dot()
            self.compute_theta()
            self.compute_x()
            self.compute_position()
            self.trajectory.append(self.position)
    

    
    def animate(self):
        interval = 1000 * self.dt
        fig = plt.figure()
        axes = fig.add_subplot(111)

        axes.set_aspect('equal')
        axes.set_xlim(-(self.length * 2), (self.length * 2))
        axes.set_ylim(-(self.length * 0.2), (self.length * 1.1))

        line1, = axes.plot([], [], ".-", lw=4, color='#66FF66')

        def update(frame):
            (x2, y2), x1 = self.trajectory[frame]
            line1.set_data([x1, x2], [0, y2])
            return line1
        
        ani = FuncAnimation(fig, update, frames = len(self.trajectory), interval = interval, blit = False)
        ani.save("Inverted_Pendulum_LQR.gif")

        plt.show()

    
inverted_pendulum = InvertedPendulum(-0.15, 1, 1, 1, 9.8,"LQR", 0.05)
inverted_pendulum.start_sim()
inverted_pendulum.animate()