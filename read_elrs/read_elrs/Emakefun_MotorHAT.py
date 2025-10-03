#!/usr/bin/python

from read_elrs.Emakefun_MotorDriver import PWM


class Emakefun_DCMotor:
	def __init__(self, controller, num):
		self.MC = controller
		self.motornum = num
		in1 = in2 = 0
		self._speed = 0
		if num == 0:
			in1 = 13
			in2 = 11
		elif num == 1:
			in1 = 10
			in2 = 8
		elif num == 2:
			in1 = 2
			in2 = 4
		elif num == 3:
			in1 = 7
			in2 = 5
		else:
			raise NameError('MotorHAT Motor must be between 1 and 4 inclusive')
		self.IN1pin = in1
		self.IN2pin = in2

	def run(self, command):

		if not self.MC:
			print ("Motor controller not initialized, cannot run motor")
			return
		if (command == Emakefun_MotorHAT.FORWARD):
			self.MC.setPWM(self.IN1pin, self._speed*16)
			self.MC.setPin(self.IN2pin, 0)
		if (command == Emakefun_MotorHAT.BACKWARD):
			self.MC.setPin(self.IN1pin, 0)
			self.MC.setPWM(self.IN2pin, self._speed*16)
		if (command == Emakefun_MotorHAT.RELEASE):
			self.MC.setPin(self.IN1pin, 0)
			self.MC.setPin(self.IN2pin, 0)
		if (command == Emakefun_MotorHAT.BRAKE):
			self.MC.setPWM(self.IN1pin, 255)
			self.MC.setPWM(self.IN2pin, 255)

	def setSpeed(self, speed):
		if (speed < 0):
			speed = 0
		if (speed > 255):
			speed = 255

		self._speed = speed

class Emakefun_MotorHAT:
	FORWARD = 1
	BACKWARD = 2
	RELEASE = 3
	BRAKE = 4
	

	def __init__(self, addr = 0x60, freq = 50):
		self._i2caddr = addr            # default addr on HAT
		self._frequency = freq		# default @1600Hz PWM freq
		self.motors = [ Emakefun_DCMotor(self, 0), Emakefun_DCMotor(self, 1), Emakefun_DCMotor(self, 2), Emakefun_DCMotor(self, 3) ]
		self._pwm =  PWM(addr, debug=False)
		self._pwm.setPWMFreq(self._frequency)

	def setPin(self, pin, value):

		if (pin < 0) or (pin > 15):
			raise NameError('PWM pin must be between 0 and 15 inclusive')
		if (value != 0) and (value != 1):
			raise NameError('Pin value must be 0 or 1!')
		
		if (value == 0):
			self._pwm.setPWM(pin, 0, 4096)

		if (value == 1):
			self._pwm.setPWM(pin, 4096, 0)

	def setPWM(self, pin, value):
		if (value > 4095):
			self._pwm.setPWM(pin, 4096, 0)
		else:
			self._pwm.setPWM(pin, 0, value)

	def getMotor(self, num):
		if (num < 1) or (num > 4):
			raise NameError('MotorHAT Motor must be between 1 and 4 inclusive')
		return self.motors[num-1]