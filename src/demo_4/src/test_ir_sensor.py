import RPi.GPIO as GPIO
import time

def in_range(data, up, low):
	if data < up and data > low:
		return True
	else:
		return False

bound_1500 = [0.22, 0.17]
bound_600  = [0.32, 0.27]

GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.IN)

ts = time.time()
arr = []

while time.time() - ts <= 0.2:
	arr.append(GPIO.input(20)) 

data_len = len(arr)
data_0   = 0

for i in range(len(arr)):
	if arr[i] == 0:
		data_0 = data_0 + 1

ratio = data_0/float(data_len)
print "ratio = ", ratio

if in_range(ratio, bound_1500[0], bound_1500[1]):
	print "Door 1500 in front of me"
elif in_range(ratio, bound_600[0], bound_600[1]):
	print "Door 600 in front of me"
else:
	print "Neither 1500 nor 600 torward me"
