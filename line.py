#!/usr/bin/python3
# coding: utf-8
from time import time, sleep
from ev3dev.auto import *
leftMotor = LargeMotor(OUTPUT_B);
rightMotor = LargeMotor(OUTPUT_C);
col=ColorSensor();
col.mode='COL-REFLECT'
btn = Button()
power=65
minpower=65
target=60
kp=0.5
ki=0.005
kd=0.25
direction=-1
minRef=41
maxRef=63
Black=0 
def init():
	global Black
	maxRef=0
	minRef=100
	i=0
	for i in range(0,1):
		read=col.value()
		if maxRef<read:
			maxRef=read
		if minRef>read:
			minRef=read
		sleep(1)
		i = i + 1
	if ((maxRef + minRef)/2) < 50:
		Black=1
	leftMotor.stop()
	rightMotor.stop()
def steering(course, power):
	if course >= 0:
		if course > 100:
			powerRight=0
			powerLeft=power
		else:
			powerLeft=power
			powerRight=power - ((power * course)/100)
	else:
		if course < -100:
			powerLeft=0
			powerRight=power
		else:
			powerRight=power
			powerLeft=power + ((power * course)/100)
	return (int(powerLeft), int(powerRight))

def run(power, target, kp, kd, ki, direction, minRef, maxRef):
	global minpower
	global Black
	itarget = target
	ipower = minpower 
	lastError = error = integral = 0
	while not btn.any():
		leftMotor.run_direct()
		rightMotor.run_direct()
		refRead=col.value()
		if refRead >= ((minRef + maxRef)/2) and Black == 0:
			leftMotor.duty_cycle_sp=minpower
			rightMotor.duty_cycle_sp=minpower
			continue
		else:
			if ipower >= power:
				ipower=power
			if itarget < 2:
				itarget=1
			Black = 1
			error = itarget - (100 * (refRead - minRef)/(maxRef - minRef))
			derivative = error - lastError
			lastError = error
			integral = float(0.5) * integral + error
			course = (kp * error + kd * derivative + ki * integral) * direction
			for(motor, pow) in zip((leftMotor, rightMotor), steering(course, ipower)):
				motor.duty_cycle_sp = pow
			sleep(0.01)
			ipower = ipower + 0.05
init()
run(power, target, kp, kd, ki, direction, minRef, maxRef)
leftMotor.stop()
rightMotor.stop()
