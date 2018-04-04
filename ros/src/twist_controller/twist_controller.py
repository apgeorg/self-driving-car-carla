import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, params):
	self.params = params
    	self.yaw_controller = YawController(wheel_base=params['wheel_base'], 
					steer_ratio=params['steer_ratio'], 
					min_speed=params['min_speed'], 
					max_lat_accel=params['max_lat_accel'], 
					max_steer_angle=params['max_steer_angle'])
	self.throttle_controller = PID(kp=0.3, ki=0.1, kd=0., mn=0., mx=0.2)
	self.vel_lpf = LowPassFilter(tau=0.5, ts=0.02)
	self.previous_vel = None
	self.previous_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
	if not dbw_enabled:
		self.reset()
		return 0., 0., 0.
			
	current_vel = self.vel_lpf.filt(current_vel)
	steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
	
	vel_error = linear_vel - current_vel
	self.previous_vel = current_vel

	current_time = rospy.get_time()
	sample_time = current_time - self.previous_time
	self.previous_time = current_time
	
	throttle = self.throttle_controller.step(vel_error, sample_time)
	brake = 0.

	#rospy.loginfo()
	
	if linear_vel == 0. and current_vel < self.params['min_speed']:
		throttle = 0.
		brake = 400 #N*m
	elif throttle < 0.1 and vel_error < 0.:
		throttle = 0
		decel = max(vel_error, self.params['decel_limit'])
		brake = abs(decel) * self.params['vehicle_mass'] * self.params['wheel_radius']
        
	return throttle, brake, steering

    def reset(self):
	self.throttle_controller.reset()
