#!/usr/bin/python3

import wiringpi as wiringpi
import time
from time import sleep 

pwmPin = 12 #HW PWM works on GPIO 12, 13, 18 and 19 on RPi4B
pwmRange = 5000
tachoPin = 6
lowTemp = 55 # Lowest temperature, if lowest of this, the FAN is Off
maxTemp = 60 # Higher than it, the FAN is on full speed
check_sec = 2 #Time to check temperature and set FAN speed

percentTemp = (maxTemp-lowTemp)/100.0

rpmChkStartTime=None
rpmPulse = 0

###PID Parameters###
KP = 2
KI = 1
KD = 1
TAU = 1
PID_MIN = 0
PID_MAX = 100


class PID_Controller:
	def __init__(self, kp, ki, kd, tau, limMin, limMax):
		self.kp=kp
		self.ki=ki
		self.kd=kd
		self.tau=tau
		self.limMin=limMin
		self.limMax=limMax
		self.time=check_sec
		self.integrator=0
		self.prevError=0
		self.differentiator=0
		self.prevMeasure=0
		self.out=0
	def update(self, setpoint, measure):
		error=setpoint-measure
		#error=measure-setpoint
		#Proportional gain
		proportional=self.kp*error
		#Integral gain
		self.integrator=self.integrator+0.5*self.ki*self.time*(error+self.prevError)
		#Anti-wind-up
		if self.limMax>proportional:
			intLimMax=self.limMax-proportional
		else:
			intLimMax=0
		if self.limMin<proportional:
			intLimMin=self.limMin-proportional
		else:
			intLimMin=0
		#Clamp integrator
		if self.integrator>intLimMax:
			self.integrator=intLimMax
		else:
			self.integrator=intLimMin
		#Differentiator gain
		self.differentiator=(2*self.kd*measure-self.prevMeasure)+(2*self.tau-self.time)*self.differentiator/(2*self.tau+self.time)
		#Calculate output
		self.out=proportional+self.integrator+self.differentiator
		#Apply limits
		if self.out > self.limMax:
			self.out=self.limMax
		elif self.out < self.limMin:
			self.out=self.limMin
		#Store data
		print(self.prevError)
		self.prevError=error
		print(self.prevError)
		self.prevMeasure=measure

myPID=PID_Controller(KP,KI,KD,TAU,PID_MIN,PID_MAX)

def getCPUTemp():
	f=open('/sys/class/thermal/thermal_zone0/temp', 'r')
	temp=f.readline()
	f.close()
	ret=float(temp)/1000
	return ret

def tachoISR():
	global rpmPulse
	#print("interruption!!!")
	rpmPulse+=1
	return

def setupTacho():
	global rpmChkStartTime

	print("Setting up Tacho input pin")
	wiringpi.wiringPiSetupGpio()
	wiringpi.pinMode(tachoPin,wiringpi.INPUT)
	wiringpi.pullUpDnControl(tachoPin,wiringpi.PUD_UP)
	rpmChkStartTime=time.time()
	#print("{:4d}".format(wiringpi.INT_EDGE_FALLING))
	wiringpi.wiringPiISR(tachoPin,wiringpi.INT_EDGE_FALLING,tachoISR)
	return

def readRPM():
	global rpmPulse, rpmChkStartTime
	fanPulses=2

	duration=time.time()-rpmChkStartTime
	frequency=rpmPulse/duration
	ret=int(frequency*60/fanPulses)
	rpmChkStartTime=time.time()
	rpmPulse=0
	print("Frequency {:3.2f} | RPM:{:4d}".format(frequency,ret))
#	with open('/tmp/adf-fanspeed', 'w') as f:
#		f.write(str(ret)+'\n')
#		f.close();
	return ret

def fanOn():
	wiringpi.pwmWrite(pwmPin,pwmRange)
	return

def updateFanSpeed():
	temp=getCPUTemp()
	myPID.update(lowTemp,temp)
	#percentDiff = 45


	if myPID.out < 0:
		percentDiff = 0
	else:
                percentDiff = myPID.out

	with open('/tmp/adf-fanspeed', 'w') as f:
		f.write(str(percentDiff)+'\n')
		f.close();

        #percentDiff = 100-myPID.out
	#diff=temp-lowTemp
	#percentDiff = 0
	#if diff > 0:
	#	percentDiff=diff/percentTemp
	pwmDuty=int(percentDiff*pwmRange/100.0)

	print(myPID.out)
	wiringpi.pwmWrite(pwmPin, pwmDuty)
	#print("currTemp {:4.2f} tempDiff {:4.2f} percentDiff {:4.2f} pwmDuty {:5.0f}".format(temp, diff, percentDiff, pwmDuty))
	return

def setup():
	wiringpi.wiringPiSetupGpio()
	#wiringpi.pinMode(pwmPin, 2) #HW PWM works on GPIO 12, 13, 18 and 19 on RPi4B
	wiringpi.pinMode(pwmPin,wiringpi.PWM_OUTPUT)

	wiringpi.pwmSetClock(768) #Set PWM divider of base clock 19.2Mhz to 25Khz (Intel's recommendation for PWM FANs)
	wiringpi.pwmSetRange(pwmRange) #Range setted

	wiringpi.pwmWrite(pwmPin, pwmRange) # Setting to the max PWM
	return

def main():
	print("PWM FAN control starting")
	setup()
	setupTacho()
	#fanOn()

	while True:
		try:
			updateFanSpeed()
			readRPM()
			sleep(check_sec)
		except KeyboardInterrupt:
			fanOn()
			break
		except e:
			print("Something went wrong")
			print(e)
			fanOn()

if __name__ == "__main__":
	main()

