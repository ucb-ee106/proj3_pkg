import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = ['FreeSans', 'Helvetica', 'Arial']
plt.rcParams['font.size'] = 18


class Estimator:
    """A base class to represent an estimator.

    This module contains the basic elements of an estimator, on which the
    subsequent DeadReckoning, Kalman Filter, and Extended Kalman Filter classes
    will be based on. A plotting function is provided to visualize the
    estimation results in real time.

    Attributes:
    ----------
        u : list
            A list of system inputs
        x : list
            A list of system states
        y : list
            A list of system outputs
        x_hat : list
            A list of estimated system states
        dt : float
            Update frequency of the estimator
        fig : Figure
            matplotlib Figure for real-time plotting
        ax : Axis
            matplotlib Axis for real-time plotting
        ln_x : Line
            matplotlib Line object for ground truth states
        ln_x_hat : Line
            matplotlib Line object for estimated states
        canvas_title : str
            Title of the real-time plot, which is chosen to be estimator type
        sub_u : rospy.Subscriber
            ROS subscriber for system inputs
        sub_x : rospy.Subscriber
            ROS subscriber for system states
        sub_y : rospy.Subscriber
            ROS subscriber for system outputs
        tmr_update : rospy.Timer
            ROS Timer for periodically updating the estimator's state
    """
    # noinspection PyTypeChecker
    def __init__(self):
        self.u = []
        self.x = []
        self.y = []
        self.x_hat = []  # Your estimates go here!
        self.dt = 0.1
        self.fig = plt.figure(figsize=(10, 10), dpi=128)
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ln_x, = self.ax.plot([], 'k', linewidth=2, label='x')
        self.ln_x_hat, = self.ax.plot([], 'o-y', label='x_hat')
        self.canvas_title = 'N/A'
        self.sub_u = rospy.Subscriber('u', Float32MultiArray, self.callback_u)
        self.sub_x = rospy.Subscriber('x', Float32MultiArray, self.callback_x)
        self.sub_y = rospy.Subscriber('y', Float32MultiArray, self.callback_y)
        self.tmr_update = rospy.Timer(rospy.Duration(self.dt), self.update)

    def callback_u(self, msg):
        self.u.append(msg.data)

    def callback_x(self, msg):
        self.x.append(msg.data)

    def callback_y(self, msg):
        self.y.append(msg.data)

    def update(self, _):
        raise NotImplementedError

    def plot_init(self):
        self.ax.set_title(self.canvas_title)
        self.ax.set_xlabel('x (m)')
        self.ax.set_ylabel('y (m)')
        self.ax.set_aspect('equal', adjustable='box')
        plt.tight_layout()

    def plot_update(self, _):
        self.plot_line(self.ln_x, self.x)
        self.plot_line(self.ln_x_hat, self.x_hat)
        self.ax.legend()
        return self.ln_x, self.ln_x_hat

    def plot_line(self, ln, data):
        if len(data):
            x = [data[1] for data in data]
            y = [data[2] for data in data]
            ln.set_data(x, y)
            self.ax.set_xlim([min(x) * 1.05, max(x) * 1.05])
            self.ax.set_ylim([min(y) * 1.05, max(y) * 1.05])


class OracleObserver(Estimator):
    """Oracle observer which has access to the true state.

    This class is intended as a bare minimum example for you to understand how
    to work with the code.

    Example
    ----------
    To run the oracle observer:
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=oracle_observer \
            noise_injection:=true \
            freeze_bearing:=false
    """
    def __init__(self):
        super().__init__()
        self.canvas_title = 'Oracle Observer'

    def update(self, _):
        self.x_hat.append(self.x[-1])


class DeadReckoning(Estimator):
    """Dead reckoning estimator.

    Your task is to implement the update method of this class using only the
    u attribute and x0. You will need to build a model of the unicycle model
    with the parameters provided to you in the lab doc. After building the
    model, use the provided inputs to estimate system state over time.

    The method should perfectly predict the state evolution if the system is
    free of noise. You may use this knowledge to test your implementation.

    Example
    ----------
    To run dead reckoning:
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=dead_reckoning \
            noise_injection:=true \
            freeze_bearing:=false
    For debugging, you can simulate a noise-free unicycle model by setting
    noise_injection:=false.
    """
    def __init__(self):
        super().__init__()
        self.canvas_title = 'Dead Reckoning'

    def update(self, _):
        # TODO: You implementation goes here!
        # You may ONLY use self.u and self.x[0] for estimation
        raise NotImplementedError


class KalmanFilter(Estimator):
    """Kalman filter estimator.

    Your task is to implement the update method of this class using the u
    attribute, y attribute, and x0. You will need to build a model of the
    linear unicycle model at the default bearing of pi/4. After building the
    model, use the provided inputs and outputs to estimate system state over
    time via the recursive Kalman filter update rule.

    Hint: You may want to reuse your code from DeadReckoning class.

    Example
    ----------
    To run the Kalman filter:
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=kalman_filter \
            noise_injection:=true \
            freeze_bearing:=true
    """
    def __init__(self):
        super().__init__()
        self.canvas_title = 'Kalman Filter'

    def update(self, _):
        # TODO: You implementation goes here!
        # You may use self.u, self.y, and self.x[0] for estimation
        raise NotImplementedError


class ExtendedKalmanFilter(KalmanFilter):
    """Extended Kalman filter estimator.

    Your task is to implement the update method of this class using the u
    attribute, y attribute, and x0. You will need to build a model of the
    unicycle model and linearize it at every operating point. After building the
    model, use the provided inputs and outputs to estimate system state over
    time via the recursive extended Kalman filter update rule.

    Hint: You may want to reuse your code from DeadReckoning class and
    KalmanFilter class.

    Example
    ----------
    To run the extended Kalman filter:
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=extended_kalman_filter \
            noise_injection:=true \
            freeze_bearing:=false
    """
    def __init__(self):
        super().__init__()
        self.canvas_title = 'Extended Kalman Filter'

    def update(self, _):
        # TODO: You implementation goes here!
        # You may use self.u, self.y, and self.x[0] for estimation
        raise NotImplementedError
