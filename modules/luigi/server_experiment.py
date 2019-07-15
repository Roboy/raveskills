import rospy
import RPi.GPIO as GPIO
from Payment.srv import *
from time import time
from enum import Enum
from time import sleep

class PaymentOptions(Enum):
	COIN = 0
	PAYPAL = 1

class Paid(Enum):
	NONE = 0
	LESS = 2
	EXACT = 3
	MORE = 4

class CoinCounter(object):
	def __init__(self):
		self.coin_sum = 0
		self.last_call_time = 0
		self.start_time = time()
	def reset_coin_sum(self):
		self.coin_sum = 0
	def coin_count_callback(self, channel, price):
		self.coin_sum += 10
		self.last_call_time = time()
		print(self.coin_sum)

		if self.coin_sum > price:
			self.reset_coin_sum()
			return PaymentResponse(Paid.MORE)
		elif self.coin_sum == price:
			self.reset_coin_sum()
			return PaymentResponse(Paid.EXACT)
		
		if self.last_call_time - self.start_time > 60:
			if self.coin_sum == 0:
				return PaymentResponse(Paid.NONE)
			else:
				self.reset_coin_sum()
				return PaymentResponse(Paid.LESS)

def set_coin_counter(price):
	GPIO.cleanup()

	GPIO.setmode(GPIO.BOARD)
	INPUT_PIN = 3
	GPIO.setup(INPUT_PIN, GPIO.IN)

	coin_counter = CoinCounter()
	
	coin_count_callback_lambda = lambda channel: coin_counter.coin_count_callback(channel, price)
	GPIO.add_event_detect(INPUT_PIN, GPIO.FALLING, callback=coin_count_callback_lambda, bouncetime=100)
	
	return coin_counter

def handle_payment(req):
	if int(req.payment_option) == PaymentOptions.COIN:
		set_coin_counter(int(req.price))
	elif int(req.payment_option) == PaymentOptions.PAYPAL:
		pass

if __name__ == "__main__":
	rospy.init_node('payment_server')
	# handle_payment_lambda = lambda req: handle_payment(req, coin_counter)
	rospy.Service('payment', Payment, handle_payment)
	
	rospy.spin()