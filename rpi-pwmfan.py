#!/usr/bin/python3

import wiringpi as wiringpi
import time
from time import sleep 

pwmPin = 12 #HW PWM works on GPIO 12, 13, 18 and 19 on RPi4B
pwmRange = 5000
tachoPin = 16
lowTemp = 45 # Lowest temperature, if lowest of this, the FAN is Off
maxTemp = 60 # Higher than it, the FAN is on full speed
check_sec = 2 #Time to check temperature and set FAN speed

percentTemp = (maxTemp-lowTemp)/100.0

rpmChkStartTime=None
rpmPulse = 0

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
	return ret

def fanOn():
	wiringpi.pwmWrite(pwmPin,pwmRange)
	return

def updateFanSpeed():
	temp=getCPUTemp()
	diff=temp-lowTemp
	percentDiff = 0
	if diff > 0:
		percentDiff=diff/percentTemp
	pwmDuty=int(percentDiff*pwmRange/100.0)
	
	wiringpi.pwmWrite(pwmPin, pwmDuty)
	print("currTemp {:4.2f} tempDiff {:4.2f} percentDiff {:4.2f} pwmDuty {:5.0f}".format(temp, diff, percentDiff, pwmDuty))
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
		except:
			print("Something went wrong")
			fanOn()

if __name__ == "__main__":
	main()

