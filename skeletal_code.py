from machine import Pin, PWM, ADC
import time
import math 

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
	pen_angle = None  # explicitly return None for invalid input
        print("Problème! L'état du stylo n'est pas défini")
    return pen_angle

# Mbappe
# map potentiometer inputs to X and Y coordinates
def map_potentiometer(analogue_value: int, knob_min: int, knob_max: int, paper_dimension_min: float, paper_dimension_max: float) -> float:
	# knob min/max are the potentiometer readings
	# paper dimension min/max defines the reachable area for the pen
	# linear mapping formula that scales a value from one range to another
	coordinate = (analogue_value - knob_min) * (paper_dimension_max - paper_dimension_min) / (knob_max - knob_min) + paper_dimension_min
	return coordinate

# Amelie
def inverse_kinematics(x: float, y: float) -> tuple[float,float]:
   """Calculates souldeer and elbow angles required to reach target x and y 
   
   Parameters:
   x (float) : target x coordinate
   y (float) : target y cooridinate
   
   Returns: 
   tuple: (shoulder angle, elbow angle)
   """
   
   #calculate the distance from the shoulder to the target spot
   distance = math.sqrt(x**2 + y**2)
   
   #check if it is out of reach
   if distance > (length_shoulder + length_elbow):
      print("Target is out of reach")
   
   #calculate the eblow angle beta
   cos_beta = (x**2 + y**2 - length_shoulder**2 - length_elbow**2)/(2 * length_shoulder * length_elbow)
   beta = math.acos(cos_beta)
   
   #calculate shoulder angle alpha
   sin_beta = math.sqrt(1-cos_beta**2) #sin of beta_deg
   x_coordinate = length_shoulder + length_elbow * cos_beta
   y_coordinate = length_shoulder * sin_beta
   alpha= math.atan2 (y,x) - math.atan2 (y_coordinate, x_coordinate)
   
   #convert to degrees
   alpha_deg = math.degrees(alpha)
   beta_deg = math.degrees(beta)
   
   return alpha_deg, beta_deg

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
	#lab 8 says length 155mm but zitai said 150mmm??
	length_shoulder = 150 #Shoulder to elbow segment (in mm) 
	length_elbow = 150 #Elbow to pen segment (in mm)
	
	# initialize default "home" position for X, Y coordinates
	default_x = 107.95  # e.g. center of the workspace
	default_y = 139.7  # e.g. enter of the workspace
	
	# initialize necessary variables
	pen_state = False # initial state of the pen (F: up or T: down)
	new_button_state = button_pin.value()
	potentiometer_value_x = 0  # placeholder for X potentiometer input
	potentiometer_value_y = 0  # placeholder for Y potentiometer input

	# define the input and output ranges for mapping potentiometer values
	in_min, in_max = 0, 100      # input range for potentiometers
	out_min, out_max = 0, 100  # output range for coordinates
	
	#amelie	
	#setting the home position
	try: 
	      #calculate the angles for shoulder and elbow servos to find home
	      alpha_home, beta_home = inverse_kinematics (default_x, default_y)
	      #convert the shoulder and elbow angle to PWM duty cycle and send to move it
	      movement(translate(alpha_home), shoulder_servo)
	      movement(translate(beta_home), eblow_servo)
	      
	      #Lifting the pen
	      movement(translate(0), pen_servo)
	      
	      print("Pen moved to home position")
	 except ValueError:
	      print("Issue moving to home position")
	
	while True:  # main loop
		# detect button press and toggle pen state
	        new_button_state, pen_state = button_checker(new_button_state, pen_state) # assign the returned values to variables
    		pen_angle = pen_placement(pen_state)
	
	        # read potentiometer inputs and map them to coordinates
	        x_value = map_potentiometer(left_knob.read_u16(), 0, 65535, 0, 279.4) #11in = 279.4mm
    		y_value = map_potentiometer(right_knob.read_u16(), 0, 65535, 0, 215.9) #8,5in = 215.9mm
	
	        # from X,Y coordinates, get associated arm and shoulder servo angles
	        alpha_deg, beta_deg = inverse_kinematics(x_value, y_value)
	
	        # determine the duty cycle values for the servos based on angles
	        duty_cycle_pen = translate(pen_angle)
	        duty_cycle_shoulder = translate(alpha_deg)
	        duty_cycle_elbow = translate(beta_deg)
	
	        # move the servos to the desired positions
	        movement(duty_cycle_pen)      # move pen servo
	        movement(duty_cycle_shoulder) # move shoulder servo
        	movement(duty_cycle_elbow)    # move elbow servo

        	# add a delay or loop-breaking condition as needed (e.g., based on user input)
		
main()
