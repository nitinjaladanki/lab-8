import numpy as np
import math

class AUVController ():

	def __init__ (self, auv_state):

		self.__auv_state = auv_state
		self.__time_for_travel = None
		self.__heading_range = None
		self.__midpoint = None
		self.__results = None
		self.__gnext = None
		self.__rnext = None
		self.__gprev = None
		self.__rprev = None
		self.__time = 0
		return

	def decide (self, auv_state, gnext, rnext):

		self.__auv_state = auv_state
		self.__gnext = gnext
		self.__rnext = rnext

		if not((self.__gprev == self.__gnext) or (self.__rprev == self.__rnext)):

			self.__gprev = self.__gnext
			self.__rprev = self.__rnext

			self.__time = -1

		self.__time += 1

		if (self.__time == 0):

			self.__midpoint = (((self.__gnext[0] + self.__rnext[0]) / 2), ((self.__gnext[1] + self.__rnext[1]) / 2))
			self.__time_for_travel = int(np.floor((np.sqrt(np.power((self.__midpoint[0] - self.__auv_state['position'][0]), 2) + np.power((self.__midpoint[1] - self.__auv_state['position'][1]), 2))) / self.__auv_state['speed']))
			self.__heading_range = self.get_desired_heading_range(self.__auv_state['position'])
			self.__results = self.simulate()

		if ((self.__time == self.__results[1] + 1) or ((self.__results[3] == -1) and (self.__time == 0))):

			return "RUDDER AMIDSHIPS"

		elif ((self.__results[3] == 0) and (self.__time == 0)):

			return f"LEFT {self.__results[0]} DEGREES RUDDER"

		elif ((self.__results[3] == 1) and (self.__time == 0)):

			return f"RIGHT {self.__results[0]} DEGREES RUDDER"

		else:

			return None

	def simulate (self):

		possibilities = []
		auv_speed_in_knots = (self.__auv_state['speed'] * (3600/1852))
		will_pass = False
		distance_from_straight_away = 0
		distance_from_curve = 0
		battery_start = 10193.669806601898

		if (self.__heading_range[1] == self.__auv_state['heading']):

			return [0, 0, None, -1]

		elif (self.is_in_heading_range(self.__auv_state['heading'], self.__heading_range)):

			will_pass, distance = self.will_pass(self.__auv_state['position'], self.__auv_state['position'], self.__auv_state['heading'], 0)

			if (will_pass):

				return [0, 0, 0, -1]

		elif ((self.__heading_range[1] - self.__auv_state['heading']) < 0):

			direction = 0

		else:

			direction = 1

		for i in range(0, 31):

			will_pass = False
			distance_from_straight_away = 0
			distance_from_curve = 0
			last_position = self.__auv_state['position']
			last_heading = self.__auv_state['heading']

			for j in range(0, self.__time_for_travel):

				if (i == 0):

					delta_heading = 0

				else:

					delta_heading = 11.67 * (i / 35) * (auv_speed_in_knots / 10)

				if (i == 0):

					final_heading = last_heading
					avg_heading = last_heading


				else:

					if (direction == 0):

						final_heading = np.mod(last_heading - delta_heading + 360.0, 360.0)
						avg_heading = np.mod(last_heading - delta_heading / 2.0 + 360.0, 360.0)

					else:

						final_heading = np.mod(last_heading + delta_heading + 360.0, 360.0)
						avg_heading = np.mod(last_heading + delta_heading / 2.0 + 360.0, 360.0)

				dx = self.__auv_state['speed'] * np.sin(np.radians(avg_heading))
				dy = self.__auv_state['speed'] * np.cos(np.radians(avg_heading))
				x = last_position[0] + dx
				y = last_position[1] + dy
				new_auv_position = (x, y)

				distance_from_curve += np.sqrt(dx ** 2 + dy ** 2)

				if (self.gate_check(np.asarray(new_auv_position), np.asarray(last_position), np.asarray(self.__gnext), np.asarray(self.__rnext))):

					ossibilities.append([i, j, (distance_from_curve), (((distance_from_curve) + (0.35 * (2 * i))) / battery_start)]) #Angle, time to switch to rudder amidships, total distance, battery %

				last_position = new_auv_position
				last_heading = final_heading

				if (self.is_in_heading_range(final_heading, self.get_desired_heading_range(new_auv_position))):

					will_pass, distance_from_straight_away = self.will_pass(last_position, new_auv_position, final_heading, j)

					if (will_pass):

						possibilities.append([i, j, (distance_from_curve + distance_from_straight_away), (((distance_from_curve + distance_from_straight_away) + (0.35 * (2 * i))) / battery_start)]) #Angle, time to switch to rudder amidships, total distance, battery %

		min = [30, self.__time_for_travel, 100, direction] #Rudder, time, battery %, direction

		for i in range(len(possibilities) - 1, -1, -1):

			if ((possibilities[i][3] < min[2])): # and (i[0] <= min[0])

				min[0] = possibilities[i][0]

				if (possibilities[i][1] == 0):

					min[0] = 0

				else:

					min[1] = possibilities[i][1]

				min[2] = possibilities[i][3]

		#self.simulation(min)

		return min

	def will_pass (self, prev_position, position, heading, time):

		current_position = position
		previous_position = prev_position
		distance = 0
		last_true_true = []
		new_time_for_travel = int(np.floor((np.sqrt(np.power((self.__midpoint[0] - current_position[0]), 2) + np.power((self.__midpoint[1] - current_position[1]), 2))) / self.__auv_state['speed']))

		for i in range(0, new_time_for_travel + 3):

			if (current_position[1] > self.__rnext[1]):

				break

			dx = self.__auv_state['speed'] * np.sin(np.radians(heading))
			dy = self.__auv_state['speed'] * np.cos(np.radians(heading))

			if (self.gate_check(np.asarray(current_position), np.asarray(previous_position), np.asarray(self.__gnext), np.asarray(self.__rnext))):

				return True, distance

			distance += np.sqrt(dx ** 2 + dy ** 2)

			previous_position = current_position
			current_position = ((current_position[0] + dx), (current_position[1] + dy))

		return False, 0

	def simulation (self, min):

		print("Simulation:")

		auv_speed_in_knots = (self.__auv_state['speed'] * (3600/1852))

		if ((self.__heading_range[1] - self.__auv_state['heading']) < 0):

			direction = 0

		else:

			direction = 1

		last_position = self.__auv_state['position']
		last_heading = self.__auv_state['heading']
		rudder = min[0]

		for j in range(0, self.__time_for_travel):

			if (j == min[1]):

				rudder = 0

			if (rudder == 0):

				delta_heading = 0

			else:

				delta_heading = 11.67 * (rudder / 35) * (auv_speed_in_knots / 10)

			if (rudder == 0):

				final_heading = last_heading
				avg_heading = last_heading

			else:

				if (direction == 0):

					final_heading = np.mod(last_heading - delta_heading + 360.0, 360.0)
					avg_heading = np.mod(last_heading - delta_heading / 2.0 + 360.0, 360.0)

				else:

					final_heading = np.mod(last_heading + delta_heading + 360.0, 360.0)
					avg_heading = np.mod(last_heading + delta_heading / 2.0 + 360.0, 360.0)

			dx = self.__auv_state['speed'] * np.sin(np.radians(avg_heading))
			dy = self.__auv_state['speed'] * np.cos(np.radians(avg_heading))
			x = last_position[0] + dx
			y = last_position[1] + dy
			new_auv_position = (x, y)
			new_heading_range = self.get_desired_heading_range(new_auv_position)

			print(f"auv_heading is {final_heading}, target heading is [{new_heading_range[2]}, {new_heading_range[0]}]")

			last_position = new_auv_position
			last_heading = final_heading

		print()

	def get_distance (self, A, B):

		return np.sqrt(np.power((A[0] - B[0]), 2) + np.power((A[1] - B[1]), 2))

	def corridor_check(self, A, G, R):

		GR = np.array((R[0] - G[0], R[1] - G[1]))
		GA = np.array((A[0] - G[0], A[1] - G[1]))
		RA = np.array((A[0] - R[0], A[1] - R[1]))

		GRGA = np.dot(GR, GA)
		GRRA = np.dot(GR, RA)

		return (GRGA * GRRA < 0)

	def gate_check(self, B, A, old_G, old_R):

		midpoint = (((old_G[0] + old_R[0]) / 2), ((old_G[1] + old_R[1]) / 2))
		midpoint_of_midpoint_G = (((old_G[0] + midpoint[0]) / 2), ((old_G[1] + midpoint[1]) / 2))
		midpoint_of_midpoint_R = (((old_R[0] + midpoint[0]) / 2), ((old_R[1] + midpoint[1]) / 2))

		G = (((old_G[0] + midpoint_of_midpoint_G[0]) / 2), ((old_G[1] + midpoint_of_midpoint_G[1]) / 2))
		R = (((old_R[0] + midpoint_of_midpoint_R[0]) / 2), ((old_R[1] + midpoint_of_midpoint_R[1]) / 2))

		if not (self.corridor_check(A, G, R) and self.corridor_check(B, G, R) ):
			return False

		GR = np.array((R[0] - G[0], R[1] - G[1], 0))
		GA = np.array((A[0] - G[0], A[1] - G[1], 0))
		GB = np.array((B[0] - G[0], B[1] - G[1], 0))

		GRGA = np.cross(GR, GA)
		GRGB = np.cross(GR, GB)

		return (GRGA[2] * GRGB[2] < 0)

	def is_in_heading_range (self, heading, heading_range):

		if (heading_range[0] < heading_range[2]):

			if ((heading > heading_range[0]) and (heading < heading_range[2])):

				return True

			return False

		else:

			if ((heading < heading_range[0]) and (heading > heading_range[2])):

				return True

			return False

	def get_desired_heading (self):

		self.__heading_range = self.get_desired_heading_range(self.__auv_state['position'])
		return [self.__heading_range[2], self.__heading_range[0]]

	def get_desired_heading_range (self, position):

		red_heading = np.arctan2((self.__rnext[0] - position[0]), (self.__rnext[1] - position[1])) * (180 / np.pi)
		green_heading = np.arctan2((self.__gnext[0] - position[0]), (self.__gnext[1] - position[1])) * (180 / np.pi)
		midpoint_heading = np.arctan2((self.__midpoint[0] - position[0]), (self.__midpoint[1] - position[1])) * (180 / np.pi)

		return [red_heading, midpoint_heading, green_heading]

