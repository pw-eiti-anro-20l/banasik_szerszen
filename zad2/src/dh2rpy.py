#!/usr/bin/env python

import sys
import rospy
import PyKDL
import math

def dh2rpy( inputFileDirectory, outputFileDirectory ):
	inFile = open(inputFileDirectory, 'r')
	data = inFile.readlines()
	inFile.close()

	words = []
	for line in data:
		words += line.split()

	imax = int(words[ words.index('imax:') + 1 ])
	alpha = []
	theta = []
	a = []
	d = []

	alphaIdx = words.index('alpha:') + 1
	thetaIdx = words.index('theta:') + 1
	aIdx = words.index('a:') + 1
	dIdx = words.index('d:') + 1

	for i in range(alphaIdx, alphaIdx + imax):
		alpha.append(float(words[i]))

	for i in range(thetaIdx, thetaIdx + imax):
		theta.append(float(words[i]))

	for i in range(aIdx, aIdx + imax):
		a.append(float(words[i]))

	for i in range(dIdx, dIdx + imax):
		d.append(float(words[i]))


	roll = []
	pitch = []
	yaw = []

	for i in range(0, imax):
		rot = PyKDL.Rotation.RotX(0)
		rot.DoRotX(alpha[i])
		rot.DoRotZ(theta[i])
		rpy = rot.GetRPY()
		roll.append(rpy[0])
		pitch.append(rpy[1])
		yaw.append(rpy[2])

	x = []
	y = []
	z = []

	for i in range(0, imax):
		x.append(a[i])
		y.append(d[i] * math.sin(alpha[i]))
		z.append(d[i] * math.cos(alpha[i]))


	outFile = open(outputFileDirectory, 'w')
	outFile.write("imax: %d\n" % imax)

	for i in range(0, imax):
		outFile.write("roll%d: %.2f\n" % ((i+1), roll[i]))
		outFile.write("pitch%d: %.2f\n" % ((i+1), pitch[i]))
		outFile.write("yaw%d: %.2f\n" % ((i+1), yaw[i]))
		outFile.write("x%d: %.2f\n" % ((i+1), x[i]))
		outFile.write("y%d: %.2f\n" % ((i+1), y[i]))
		outFile.write("z%d: %.2f\n" % ((i+1), z[i]))
		outFile.write("theta%d: %.2f\n" % ((i+1), theta[i]))

	outFile.close()


if __name__ == '__main__':

	if len(sys.argv) == 1:
		print 'ERROR: No input file provided!'

	else:
		inDir = sys.argv[1]
		outDir = 'rpy.yaml'
		if len(sys.argv) > 2:
			outDir = sys.argv[2]
		try:	
			dh2rpy( inDir, outDir )
		except rospy.ROSInterruptException:
			pass
