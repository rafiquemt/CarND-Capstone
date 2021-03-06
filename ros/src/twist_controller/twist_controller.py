from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        kp = 0.4
        ki = 0.0
        kd = 0.2
        mn = 0. # minimum throttle value
        mx = 0.2 # maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

        pass

    def control(self, linear_vel, angular_vel, current_vel, dbw_enabled):
        # Return throttle, brake, steer

        # filtering goes on always. Doesn't matter if dbw is off
        current_vel = self.vel_lpf.filt(current_vel)
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0.0

        if abs(linear_vel) <= 0.05 and current_vel < 0.15:
            throttle = 0.0
            brake = 700 #N*m - to hold the car in place if we are stopped at a red light
        elif throttle >= 0.1:
            brake = 0.0
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0.0
            # decel_limit < 0, so select less negative deceleration
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius
        # rospy.logwarn('Throttle: %s', throttle)
        return throttle, brake, steering    
