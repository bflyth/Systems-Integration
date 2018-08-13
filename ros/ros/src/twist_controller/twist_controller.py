import rospy
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, yaw_controller, vehicle_mass, decel_limit, accel_limit, wheel_radius):
        # TODO: Implement
        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.yaw_controller = yaw_controller
        self.throttle_pid = PID(0.3,0.1,0.,0.,0.2)
        self.throttle_filt = LowPassFilter(0.5,.02)
        self.steer_filt = LowPassFilter(1.,1.)
        self.last_time = rospy.get_time()

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_pid.reset()
            return 0., 0., 0.

        current_velocity = self.throttle_filt.filt(current_velocity)
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
        steer = self.steer_filt.filt(steer)

        err = linear_velocity - current_velocity
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        throttle = self.throttle_pid.step(err, sample_time) 
        brake = 0

        if linear_velocity == 0 and current_velocity < .1:
            throttle = 0.
            brake = 700
        elif throttle < .1 and err < 0:
            throttle = 0.
            decel = max(err, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        rospy.loginfo('throttle={}, brake={}, steer={}, current_vel={}, err={}'.format(throttle, brake, steer, current_velocity, err))

        return throttle, brake, steer