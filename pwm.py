from gpiozero import PWMOutputDevice
from time import sleep
from enum import Enum
import RPi.GPIO as GPIO
import sys
# [START pin_config]
class Pin(Enum):
	# Motor A
	pwm_forward_left_pin = 26
	pwm_reverse_left_pin = 19
	# Motor B
	pwm_forward_right_pin = 13
	pwm_reverse_right_pin = 6
	# Pin encoder Motor A
	encoder_channel_a_left_pin = 23
	encoder_channel_b_left_pin = 24
	# Pin encoder Motor B
	encoder_channel_a_right_pin = 25
	encoder_channel_b_right_pin = 8
# [END pin_config]

# [START gpio setup]
GPIO.setmode(GPIO.BCM)
GPIO.setup(Pin.encoder_channel_a_left_pin.value, GPIO.IN)
GPIO.setup(Pin.encoder_channel_b_left_pin.value, GPIO.IN)
# [END gpio setup]

# [START initialise objects for H-Bride PWM pins
forward_left = PWMOutputDevice(Pin.pwm_forward_left_pin.value, True, 0, 500)
reverse_left = PWMOutputDevice(Pin.pwm_reverse_left_pin.value, True, 0, 500)

forward_right = PWMOutputDevice(Pin.pwm_forward_right_pin.value, True, 0, 500)
reverse_right = PWMOutputDevice(Pin.pwm_reverse_right_pin.value, True, 0, 500)
# [END initialise objects for H-Bride PWM pins]

# [START pid_config_parameters]
pulse = 0
pre_pulse = 0
kp = 0.000152941; ki = 0.000027451; kd = 0.000011765
P = 0; I = 0; D = 0
error = 0
pre_error = 0
des_rSpeed = float(sys.argv[1])
dir = int(sys.argv[2])
sp_time = 50 #(ms)
res_encoder = 334
rate = 34 #1:34
output = 0
# [END pid_config_parameters]

def interruptLeftFunc(channel):
	global pulse
	if GPIO.input(Pin.encoder_channel_b_left_pin.value):
		pulse += 1
	else:
		pulse -= 1

def allStop():
	forward_left.value = 0
	reverse_left.value = 0

def forwardDrive():
	forward_left.value = 0.1
	reverse_left.value = 0.0

def reverseDrive():
	forward_left.value = 0.0
	reverse_left.value = 1.0

def main():
	GPIO.add_event_detect(Pin.encoder_channel_a_left_pin.value, GPIO.RISING, callback=interruptLeftFunc)
	#print(des_rSpeed)
	while True:
		des_pSpeed = ConvertRpmToPulse(des_rSpeed * rate)
		Rotate(MotorPID(des_pSpeed), dir)
		#print(des_rSpeed, "***", des_pSpeed)
		sleep(sp_time/1000.0)

def ConvertRpmToPulse(rpm_value):
	return (rpm_value * sp_time * res_encoder * 0.001) / 60.0

def ConvertPulseToRpm(pulse_value):
	return (pulse_value * 60.0) / (sp_time * res_encoder * 0.001)


def MotorPID(des_pSpeed):
	global P, I, D, error, pre_error, pulse, pre_pulse, output
	act_pSpeed = pulse - pre_pulse
	pre_pulse = pulse
	error = des_pSpeed - abs(act_pSpeed)
	P = kp * error
	I += ki * error * sp_time / 1000.0
	D = kd * (error - pre_error) * 1000.0 / sp_time
	output += P + I + D
	#print(error, "--", output, "--", pulse,"--", pre_pulse, "--", act_pSpeed)
	if output >= 1.0:
		output = 1.0
	elif output <= 0:
		output = 0.0
	pre_error = error
	print(ConvertPulseToRpm(act_pSpeed)/rate)
#	print(pulse)
	return output

def Rotate(pwm, dir):
	forward_left.value = pwm if dir else 0
	reverse_left.value = 0 if dir else pwm

if __name__ == "__main__":
	main()
