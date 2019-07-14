import rospy
import RPi.GPIO as GPIO
from Payment.srv import *
from time import time
from enum import Enum
#from time import sleep

class Paid(Enum):
	PAID_NONE = 0
	PAID_LESS = 2
	PAID_EXACT = 3
	PAID_MORE = 4

class CoinCounter(object):
	def __init__(self):
		self.coin_sum = 0
		self.last_call_time = 0
	def coin_count_callback(self, channel):
		self.coin_sum += 10
		self.last_call_time = time()
		print(self.coin_sum)
	def handle_payment(self, req):
		#while True:
		if self.coin_sum > int(req.price):
			return PaymentResponse(Paid.PAID_MORE)
		elif self.coin_sum == int(req.price):
			return PaymentResponse(Paid.PAID_EXACT)
		elif time() - coin_counter.last_call_time > 5:
			if self.coin_sum == 0:
				return PaymentResponse(Paid.PAID_NONE)
			else:
				return PaymentResponse(Paid.PAID_LESS)
		#sleep(0.01)

if __name__ == "__main__":
	GPIO.setmode(GPIO.BOARD)
	INPUT_PIN = 3
	GPIO.setup(INPUT_PIN, GPIO.IN)

	coin_counter = CoinCounter()
	
	GPIO.add_event_detect(INPUT_PIN, GPIO.FALLING, callback=coin_counter.coin_count_callback, bouncetime=100)
	
	rospy.init_node('payment_server')
	rospy.Service('payment', Payment, coin_counter.handle_payment)
	
	rospy.spin()