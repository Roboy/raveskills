from time import sleep
from time import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
INPUT_PIN = 3
GPIO.setup(INPUT_PIN, GPIO.IN)

class CoinCounter(object):
	def __init__(self):
		self.coin_sum = 0
		self.last_call_time = 0
	def coin_count_callback(self, channel):
		self.coin_sum += 1
		self.last_call_time = time()
		print(self.coin_sum)

coin_counter = CoinCounter()

GPIO.add_event_detect(INPUT_PIN, GPIO.FALLING, callback=coin_counter.coin_count_callback, bouncetime=100)

try:
	while True:
		if time() - coin_counter.last_call_time > 5:
			if coin_counter.coin_sum != 0:
				print('You have', coin_counter.coin_sum * 10, 'cents as credit.')
			coin_counter.coin_sum = 0
		sleep(0.001)
except Exception as e:
	print(e)
finally:
	GPIO.cleanup()