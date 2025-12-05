'''
Takes user input from the command line and publishes a pwm_cli 
'''

# Plan: Figure out where to create actual list of pwms
# should likely exist as seperate functions in RobotCommand that edit a .pwm field

from cli_publisher import *

default_power = 70
current_command = "Stop"

# pwm constants
PWM_ZERO = 1500
EMERGENCY_BRAKES = [PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO]

def main():
	global current_command
	rclpy.init(args=None)

	cli = CLIPublisher()
	heartbeat = HeartbeatPublisher()
	rclpy.spin(heartbeat)

	reading_input = True

	while (reading_input):
		user_input = input("Input a command: ").lower()
		if (user_input == "end session"): break

		command = translate_command(user_input)

		# TODO: Also properly check for non-robot commands
		if command is None:
			print("{user_input} is not a valid command")
			continue
		elif command.name == "Stop":
			current_command = "Stop"
			cli.publish_pwm(command.pwm)
		elif command.confirm_command(command):
			# TODO: Check if command is timed, and if so schedule a stop set command.time seconds later
			cli.publish_pwm(command.pwm)

	# Stop robot before shutting down cli
	cli.publish_pwm(EMERGENCY_BRAKES)

	# Destroy the node explicitly
	heartbeat.destroy_node()
	rclpy.shutdown()


'''
Reads a string and returns that string as a command
@str is the string to read through
returns the command if valid, or null otherwise
'''
def translate_command(command):

	cmd = RobotCommand()
	
	if "power:" in command:
		power = find_num_in_string(command[command.index("power:")])
		if power <= 100:
			cmd.power = power
		else:
			return None
	elif "p:" in command:
		power = find_num_in_string(command[command.index("p:")])
		if power <= 100:
			cmd.power = power
		else:
			return None

	if "time:" in command:
		cmd.time = find_num_in_string(command[command.index("time:")])
	elif "t:" in command:
		cmd.time = find_num_in_string(command[command.index("t:")])

	if "set" and "power" in command:
		new_power = find_num_in_string()
		if new_power != "":
			global default_power
			default_power = new_power
		return "Set default power to {new_power}"

	if "stop" in command:
		cmd.name = "Stop"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd
	if "forwards" in command:
		cmd.name = "Move Forwards"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd
	if "backwards" in command:
		cmd.name = "Move Backwards"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd
	if "strafe" and "left" in command:
		cmd.name = "Strafe Left"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd
	if "strafe" and "right" in command:
		cmd.name = "Strafe Right"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd
	if "rise" in command:
		cmd.name = "Rise"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd
	if "sink" in command:
		cmd.name = "Sink"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd
	if "spin" and "clockwise" in command:
		cmd.name = "Spin Clockwise"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd
	if "spin" and "counter" and "clockwise" in command:
		cmd.name = "Spin Counterclockwise"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd
	if "pitch" and "forwards" in command:
		cmd.name = "Pitch Forwards"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd
	if "pitch" and "backwards" in command:
		cmd.name = "Pitch Backwards"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd
	if "roll" and "left" in command:
		cmd.name = "Roll Left"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd
	if "roll" and "right" in command:
		cmd.name = "Roll Right"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd

	# custom pwm syntax: [flt, frt, rlt, rrt, flb, frb, rlb, rrb]
	if "custom" and '[' and ']' in command:
		cmd.name = "Custom pwm"
		# TODO: Look for 8 ints given by user and store the result in an array
		cmd.pwm = EMERGENCY_BRAKES # Should be value read in from the user
		return cmd

	return None


'''
Looks through a string for the first number in it
string is the string to look through
Returns the number found, or an empty string
'''
def find_num_in_string(string):
	started = False
	is_decimal = False
	num = ""

	for char in string:
		if char.isdigit() or (char == '.' and not is_decimal):
			num += str(char)
			if char == '.':
				is_decimal = True
			if not started:
				started = True
		elif started:
			return num
	return ""
		
'''
Ask user for confirmation until valid response is given
prompt is the statement asking user to confirm command
Returns True if response is yes, or False if no
'''
def confirm(prompt):
	while True:
		response = input(prompt).lower()
		if "yes" in response:
			return True
		elif "no" in response:
			return False

class RobotCommand():
	def __init__(self, name = "Undeclared", power = default_power, time = -1, pwm = EMERGENCY_BRAKES):
		self.name = name
		self.power = power
		self.time = time
		self.pwm = pwm
		
	def command_dictionary(self):
		pwm_forwards = PWM_ZERO + 400 * (default_power / 100)
		pwm_reverse = PWM_ZERO - 400 * (default_power / 100)
		return { # TODO: Add all proper pwm sets
			"Stop" : [PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO],
			"Move Forwards" : [PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, pwm_forwards, pwm_reverse, pwm_forwards, pwm_reverse],
			"Move Backwards" : EMERGENCY_BRAKES,
			"Strafe Left" : EMERGENCY_BRAKES,
			"Strafe Right" : EMERGENCY_BRAKES,
			"Rise" : EMERGENCY_BRAKES,
			"Sink" : EMERGENCY_BRAKES,
			"Spin Clockwise" : EMERGENCY_BRAKES,
			"Spin Counterclockwise" : EMERGENCY_BRAKES,
			"Pitch Forwards" : EMERGENCY_BRAKES,
			"Pitch Backwards" : EMERGENCY_BRAKES,
			"Roll Left" : EMERGENCY_BRAKES,
			"Roll Right" : EMERGENCY_BRAKES,
		}

	def confirm_command(self):
		if self.time == -1:
			confirm("Are you sure you want to {self.name} at {self.power}% power until stopped?")
		else:
			confirm("Are you sure you want to {self.name} at {self.power}% power for {self.time} seconds?")



main()