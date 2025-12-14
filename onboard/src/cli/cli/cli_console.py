'''
Takes user input from the command line and publishes a pwm_cli 
'''

from .cli_publisher import *

default_power = 70
timer_running = False
current_command = None

# pwm constants
PWM_ZERO = 1500
EMERGENCY_BRAKES = [PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO]

# To test, check ROS documentation for checking what messages are sent

def main():
	global current_command
	current_command = RobotCommand("Stop", default_power, -1, EMERGENCY_BRAKES)

	rclpy.init(args=None)

	cli = CLIPublisher()
	heartbeat = HeartbeatPublisher()
	rclpy.spin(heartbeat)

	reading_input = True

	while (reading_input):
		user_input = input("Input a command: ").lower()

		if (user_input == "end session"):
			break

		command = translate_command(user_input)

		# if an invalid command is inputted, warn the user
		if command is None:
			print("{user_input} is not a valid command")
			continue

		# Valid non-robot commands should output their result
		elif isinstance(command, str):
			print(command)
		
		# A stop command should be processed immediately
		elif command.name == "Stop":
			cli.publish_pwm(command.pwm)
			current_command = "Stop"
		
		# A robot command should be processed after the user confirms it was intended
		elif command.confirm_command(command):
			# TODO (Zoe): Check if command is timed, and if so schedule a stop set command.time seconds later
			cli.publish_pwm(command.pwm)
			current_command = command.name
			if (command.time != -1):
				run_command_timer(command.time)

	# End of while loop
	reading_input = False

	# Stop robot before shutting down cli
	cli.publish_pwm(EMERGENCY_BRAKES)

	# Destroy the node explicitly
	heartbeat.destroy_node()
	rclpy.shutdown()

	print("Goodbye!")


'''
Reads a string and takes the action requested or outputs a robot command
@command is the string to process
returns a robot command or the result of the non-robot action taken
'''
def translate_command(command):

	# Non-Robot Commands
	if "set" and "power" in command:
		new_power = find_num_in_string()
		if new_power == "":
			global default_power
			default_power = new_power
		return "Set default power to {new_power}"

	if "current" and "command" in command:
		return get_current_command()


	cmd = RobotCommand()

	#Changing settings for a given robot command
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
	if cmd.time is None:
		return "Invalid time inputted"

	# Robot Commands
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
	if "yaw" and "counter" and "clockwise" in command:
		cmd.name = "Yaw Counterclockwise"
		cmd.pwm = cmd.command_dictionary()["{cmd.name}"]
		return cmd
	if "yaw" and "clockwise" in command:
		cmd.name = "Yaw Clockwise"
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

def get_current_command():
	global current_command
	if current_command.name == "Stop":
		return "There is no currently active command"
	elif current_command.time == -1:
		return "Current Command: {current_command.name} at {current_command.power}"
	else:
		return "Current Command: {current_command.name} at {current_command.power} for {current_command.time} seconds"



'''
IN PROGRESS
Begins a timer, after which 
'''
def run_command_timer(time):
	pass

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
	return None
		
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
		pwm_fwd = PWM_ZERO + 400 * (default_power / 100)
		pwm_rev = PWM_ZERO - 400 * (default_power / 100)
		# pwm order: [flt, frt, rlt, rrt, flb, frb, rlb, rrb]
		return {
			"Stop" : [PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO],
			"Move Forwards" : [PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, pwm_fwd, pwm_rev, pwm_fwd, pwm_rev],
			"Move Backwards" : [PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, pwm_rev, pwm_fwd, pwm_rev, pwm_fwd],
			"Strafe Left" : [PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, pwm_rev, pwm_rev, pwm_fwd, pwm_fwd],
			"Strafe Right" : [PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, pwm_fwd, pwm_fwd, pwm_rev, pwm_rev],
			"Rise" : [pwm_rev, pwm_fwd, pwm_rev, pwm_fwd, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO],
			"Sink" : [pwm_fwd, pwm_rev, pwm_fwd, pwm_rev, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO],
			"Yaw Counterclockwise" : [PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, pwm_rev, pwm_rev, pwm_rev, pwm_rev],
			"Yaw Clockwise" : [PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO, pwm_fwd, pwm_fwd, pwm_fwd, pwm_fwd],
			"Pitch Forwards" : [pwm_rev, pwm_rev, pwm_fwd, pwm_fwd, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO],
			"Pitch Backwards" : [pwm_fwd, pwm_fwd, pwm_rev, pwm_rev, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO],
			"Roll Left" : [pwm_rev, pwm_fwd, pwm_rev, pwm_fwd, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO],
			"Roll Right" : [pwm_fwd, pwm_rev, pwm_fwd, pwm_rev, PWM_ZERO, PWM_ZERO, PWM_ZERO, PWM_ZERO]
		}

	def confirm_command(self):
		if self.time == -1:
			confirm("Are you sure you want to {self.name} at {self.power}% power until stopped?")
		else:
			confirm("Are you sure you want to {self.name} at {self.power}% power for {self.time} seconds?")



main()