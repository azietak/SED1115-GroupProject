
def button(press: bool) -> bool :
	"""
	detects if button is being pressed or not (boolean value)
	then accordignly changes the button_state. there are two possible
	button states : up or down. a new button press toggles to other state
	"""
	return button_state 

def processing_button(button_state: bool) :
	"""
	determines the pen angle based on the button state.
	
	two specfic angles will be determined. one is when pen is up, the other
	for when pen is down. 
	"""
	return pen_angle

# then use translate function to make the pen servo move

# map potentiometer inputs to X and Y coordinates
def map_potentiometer(value: int, in_min: int, in_max: int, out_min: int, out_max: int) -> float:
	# in_min, in_max, out_min, out_max, will be given fixed values
	# in are the potentiometer readings
	# out defines the reachable area for the pen
	
	# linear mapping formula that scales a value from one range to another
	coordinate = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
	return coordinate

def def inverse_kinematics(X: float, Y: float) -> tuple[float, float]:
	"""
	use trigonometry to figure out what the angles of shoulder
	and elbow servo should be based on x,y coordinates
	"""
	return theta_1, theta_2

# converts angle (degrees) into duty cycle value (between 0 and 65535)
# that we want to give the different servos
def translate(angle: float) -> int:
	pulse_width = 500 + (2500 - 500) * (angle/180)
	duty_cycle = pulse_width / 20000 # 20000 microseconds = 20ms period
	duty_u16_value = int(duty_cycle*65535)

	duty_u16_value = max(1639, min(8192, duty_u16_value)) # clamps values within a safe range

	return duty_u16_value # Replace with your return value

def movement(duty_cycle) :
	# set PWM value to the right duty cycle so it makes servo move