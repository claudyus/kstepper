#!/usr/bin/env python


import sys
from fcntl import ioctl
from string import atoi, find

motor_num = sys.argv[1]

fd = open("/dev/motor%d" % atoi(motor_num))


#option with 2 args
if find(sys.argv[2], "en") != -1:	#enable
	ioctl(fd, 19210, 1)
	print "enable"
	exit(1)

if find(sys.argv[2], "dis") != -1:	#disable
	ioctl(fd, 19210, 0)
	print "disable"
	exit(1)

if find(sys.argv[2], "on") != -1:	#pwm_on
	ioctl(fd, 19212)
	print "pwm_on"
	exit(1)

if find(sys.argv[2], "off") != -1:	#pwm_off
	ioctl(fd, 19213)
	print "pwm_off"
	exit(1)

if find(sys.argv[2], "ho") != -1:	#home
	ioctl(fd, 19215)
	print "home"
	exit(1)

if find(sys.argv[2], "sta") != -1:	#start
	ioctl(fd, 19217)
	print "start"
	exit(1)

#option with 3 args
if find(sys.argv[2], "dir") != -1:	#dir
	ioctl(fd, 19211, atoi(sys.argv[3]))
	print "direction"
	exit(1)

if find(sys.argv[2], "ste") != -1:	#steps set
	ioctl(fd, 19216, atoi(sys.argv[3]))
	print "step"
	exit(1)

if find(sys.argv[2], "set") != -1:	#pwm_set
	print "pwmset"
	ioctl(fd, 19214, atoi(sys.argv[3]))
	exit(1)

print "Command didn't match any of the possible choice."
