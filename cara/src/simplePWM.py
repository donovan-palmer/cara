import RPi.GPIO as GPIO
import time

output_pins = {
	'JETSON_XAVIER': 18,
	'JETSON_NANO': 32,
	'JETSON_NX': 33,
}

output_pin = output_pins.get(GPIO.model, None)
if output_pin is None:
	raise Exception('PWM not supported on this board')

def main():
	# Pin setup
	# Board pin-numbering scheme
	GPIO.setmode(GPIO.BOARD)
	# set ppin as an output pin with optional initial state of HIGH
	GPIO.setup(output_pin, GPIO.OUT, initial = GPIO.HIGH)
	p = GPIO.PWM(output_pin, 50)
	val = 25
	incr = 5
	p.start(val)

	print('PWM running. Press CRTL+C to exit.')
	try:
		while True:
			time.sleep(0.25)
			if val >= 100:
				incr = -incr
			if val <= 0:
				incr = -incr
			val += incr
			p.ChangeDutyCycle(val)
	finally:
		p.stop()
		GPIO.cleanup()

if __name__ == '__main__':
	main()
