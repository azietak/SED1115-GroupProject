from machine import Pin, PWM, ADC
import time

# Initialize button
button_pin = Pin(22, Pin.IN, Pin.PULL_DOWN)

# Initialize PWM servo ouputs
pen_servo = PWM(Pin(0))
elbow_servo = PWM(Pin(1))
shoulder_servo = PWM(Pin(2))

# set PWM frequency
pen_servo.freq(50)
elbow_servo.freq(50)
shoulder_servo.freq(50)

# Initialize ADC input knobs
left_knob = ADC(Pin(27))
right_knob = ADC(Pin(26))

# Yann
# fonction qui prend comme argument l'état du bouton et l'état du stylo
def button_checker(new_button_state,pen_state) :
    # mise à jour de l'état du bouton à chaque passage dans la boucle
    last_button_state = new_button_state
    # obenir état du bouton physique acctuel
    new_button_state = button_pin.value()
    # la détection de contour (tel que vu dans lab 3)
    if new_button_state == 1 and last_button_state == 0:
        # basculer l'état du stylo (si l'état était Vrai, alors ça devient Faux et inversment)
        pen_state = not pen_state
        print(f"L'état du stylo est {pen_state}")
        time.sleep(0.05) # délai anti-rebond (debounce delay)
    # retourner les états mis à jour
    return new_button_state, pen_state 

# Yann
# fonction pour déterminer l'angle en fonction de si le bouton a été pesé 
def pen_placement(pen_state: bool) :
    # Faux = stylo levé, Vrai = stylo baissé
    if pen_state == True : # stylo baissé
        pen_angle = 30
    elif pen_state == False : # stylo levé
        pen_angle = 0
    else :
        print("Problème! L'état du stylo n'est pas défini")
    return pen_angle

# Mbappe
# map potentiometer inputs to X and Y coordinates
def map_potentiometer(analogue_value: int, in_min: int, in_max: int, out_min: int, out_max: int) -> float:
	
	# linear mapping formula that scales a value from one range to another
	# in are the potentiometer readings
	# out defines the reachable area for the pen
	coordinate = (analogue_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
	# should in_min, in_max, out_min, out_max be given fixed values...?
	return coordinate

# Amelie
def inverse_kinematics(X: float, Y: float) -> tuple[float, float]:
	"""
	use trigonometry to figure out what the angles of shoulder
	and elbow servo should be based on x,y coordinates
	"""
	return theta_1, theta_2

# Done in Lab 6
# converts angle (degrees) into duty cycle value (between 0 and 65535)
# that we want to give the different servos
def translate(angle: float) -> int:
	pulse_width = 500 + (2500 - 500) * (angle/180)
	duty_cycle = pulse_width / 20000 # 20000 microseconds = 20ms period
	duty_u16_value = int(duty_cycle*65535)

	duty_u16_value = max(1639, min(8192, duty_u16_value)) # clamps values within a safe range

	return duty_u16_value # Replace with your return value

# Lili
def movement(duty_cycle, servo) :
    # set PWM value to the right duty cycle so it makes corresponding servo move
	servo.duty_u16(duty_cycle)

# Lili
def main():
	# initialize default "home" position for X, Y coordinates
	default_x = 0  # e.g. center of the workspace
	default_y = 0  # e.g. enter of the workspace
	
	# initialize necessary variables
	pen_state = False # initial state of the pen (F: up or T: down)
	new_button_state = button_pin.value()
	potentiometer_value_x = 0  # placeholder for X potentiometer input
	potentiometer_value_y = 0  # placeholder for Y potentiometer input

	# define the input and output ranges for mapping potentiometer values
	in_min, in_max = 0, 100      # input range for potentiometers
	out_min, out_max = 0, 100  # output range for coordinates
	
	while True:  # main loop
		# detect button press and toggle pen state
	        new_button_state, pen_state = button_checker(new_button_state, pen_state) # assign the returned values to variables
    		pen_angle = pen_placement(pen_state)
	
	        # read potentiometer inputs and map them to coordinates
	        x_coordinate = map_potentiometer(potentiometer_value_x, in_min, in_max, out_min, out_max)
	        y_coordinate = map_potentiometer(potentiometer_value_y, in_min, in_max, out_min, out_max)
	
	        # from X,Y coordinates, get associated arm and shoulder servo angles
	        theta_1, theta_2 = inverse_kinematics(x_coordinate, y_coordinate)
	
	        # determine the duty cycle values for the servos based on angles
	        duty_cycle_pen = translate(pen_angle)
	        duty_cycle_shoulder = translate(theta_1)
	        duty_cycle_elbow = translate(theta_2)
	
	        # move the servos to the desired positions
	        movement(duty_cycle_pen)      # move pen servo
	        movement(duty_cycle_shoulder) # move shoulder servo
        	movement(duty_cycle_elbow)    # move elbow servo

        	# add a delay or loop-breaking condition as needed (e.g., based on user input)
		
