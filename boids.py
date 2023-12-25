"""
Import Libraries
"""

import numpy as np
from numpy.random import rand
from numpy.linalg import norm

import math

from p5 import size, background, run, setup, draw, exit, save_frame
from p5 import rect, Vector, stroke, circle, fill, line, beginShape, vertex, no_fill, curveVertex
from p5 import pushMatrix, translate, rotate, endShape, popMatrix, CLOSE
from p5 import text_font, create_font, text, textAlign, CENTER
from p5 import millis

import matplotlib.pyplot as plt


"""
Changeable parameters

NOTE: These parameters are meant to help smoothly tune the changes in a drone's trajectory.
      Not having these parameters would result in the drone's having jittery movements that
      are not realistic. When one is driving and they need to make a right turn, the driver
      would start slowly apply pressure to the brakes and gradually turn the steering wheel
      to the right in order to make the turn as smooth as possible. Without the gradual
      turning and slow pressure, there is a high chance the car might turn over. 
"""

global border_margin                        # create a border margin to have the drone object change direction once it gets close to the border

global change_border_direction_factor       # a factor that helps change the drone's direction when it gets close to the border

global avoid_rock_radius                    # avoidance radius of the rock

global rock_avoidance_percentage_factor     # the percentage that can adjust the drone's velocity to prevent collision with a rock (curves the trajectory)

global drone_avoidance_percentage_factor    # the percentage that can adjust the drone's velocity to prevent collision with a drone (curves the trajectory)

global match_velocity_percentage_factor     # the percentage that can adjust the drone's velocity to match average velocity of nearby drones

global go_to_center_percentage_factor       # the percentage that can adjust the drone's velocity to go to the center of mass of nearby drones 


#border_margin = 50 + 100                   # (add 100 to shift graph)
border_margin = 75 + 100                    # (add 100 to shift graph)
#change_border_direction_factor = 4
change_border_direction_factor = 10

avoid_rock_radius = 0                       # NOTE: This would most likely shift the rock's position rather than create a circle of avoidance
rock_avoidance_percentage_factor = 0.25     # NOTE: The higher the percentage the sharper the change in trajectory (set it low for smoother curves)
#rock_avoidance_percentage_factor = 0.1

drone_avoidance_percentage_factor = 0.15    # NOTE: The higher the percentage the sharper the change in trajectory (set it low for smoother curves)
#drone_avoidance_percentage_factor = 0.1

match_velocity_percentage_factor = 0.1      # NOTE: Don't set this to a high percentage or the code will break

go_to_center_percentage_factor = 0.005      # NOTE: Don't set this to a percentage higher than 0.05 or the code will break


"""
Create the Boids class
"""

class Boids():
    """
    Function that initializes variables to an object
    """
    
    def __init__(self, x, y, width, height, flag, color):
        # border values
        self.width = width
        self.height = height


        # the object's position on graph
        self.position = Vector(x, y)


        # create the velocity range between 0 and 10
        #x_velocity_range = rand() * 10
        #y_velocity_range = rand() * 10

        # create the velocity range between -10 and 10
        x_velocity_range = (rand() * 20) - 10
        y_velocity_range = (rand() * 20) - 10

        # the object's velocity on graph
        self.velocity = Vector(x_velocity_range, y_velocity_range)


        # create the drone's acceleration range between 0 and 1
        x_acceleration_range = rand()
        y_acceleration_range = rand()

        # divide the acceleration range to 0 and 0.5
        x_acceleration_range = x_acceleration_range / 2
        y_acceleration_range = y_acceleration_range / 2

        # the object's acceleration on graph
        self.acceleration = Vector(x_acceleration_range, y_acceleration_range)


        # set object's speed limit
        self.max_speed = 10
        self.min_speed = 7

        # determine if object is a rock (yes = True, no = False)
        self.rock = False

        # set object's angle
        self.angle = 0

        # set object's viewing range
        self.sight_distance = 50

        # create a history of drone positions
        self.position_history = []

        # create a list of drone positions
        self.positions_list = []

        # random flag
        self.flag = flag

        # color
        self.color = color


    """
    Self-defined models:

    Function that plots drone and the direction its going on graph
    """

    def drone_position_and_direction(self):
        # set drone border color to black
        #stroke("black")
        stroke(self.color)

        # set drone fill color to black
        #fill("black")
        fill(self.color)


        """
        NOTE: Do not change any of the code needed to plot the arrow.
        """

        # create the direction arrow for the drone object

        # push the current transformation matrix onto the matrix stack
        pushMatrix()

        # update the angle of the drone's velocity
        self.angle = math.atan2(self.velocity.y, self.velocity.x)

        # the amount to displace objects (the arrow) within the window
        translate(self.position.x, self.position.y)

        # rotates the arrow based on the angle
        rotate(self.angle)

        # begin making the shape
        beginShape()

        # the arrow structure
        vertex(16, 0)
        vertex(-8, 8)
        vertex(-8, -8)

        # end making the shape
        endShape(CLOSE)
        
        # pop the current transformation matrix onto the matrix stack
        popMatrix()


    """
    Function that updates the drone's position, velocity, acceleration, and trail
    """

    def update_drone_parameters_and_trail(self):
        # store the drone's position history
        self.position_history.append([self.position.x, self.position.y])

        # store the drone's positions
        self.positions_list.append([self.position.x, self.position.y])


        """
        NOTE: Do not change any of the code needed to plot the tail.
        """

        # draw the drone tail

        # set the trail to orange
        stroke("orange")

        # don't fill the shape
        no_fill()

        # begin making the shape
        beginShape()

        # add the first point to ensure smooth curves
        curveVertex(self.position_history[0][0], self.position_history[0][1])

        # go through every point
        for point in self.position_history:
            # plot the point
            vertex(point[0], point[1])

        # add the last point to ensure smooth curves
        curveVertex(self.position_history[-1][0], self.position_history[-1][1])

        # close the shape
        endShape(CLOSE)
        

        # remove last 1 positions from the history
        """
        NOTE: ONLY REMOVE 1 position from the history. Removing more at a time
              will cause the erasure to look laggy. Removing 1 will make it
              look the smoothest.
        """

        # check if positions history has 10 rows
        if (len(self.position_history) >= 10):
            # remove the first 1 rows from the history
            self.position_history = self.position_history[1:]


        # update the drone's position
        self.position = self.position + self.velocity

        # commenting this out as adding a random value to the velocity does not make sense
        # update the drone's velocity
        #self.velocity = self.velocity + self.acceleration


        """
        NOTE: These if statements will change the drone's trajectory and speed at
              random. If you want the drones to not have random movement and
              trajectory and want them to basically bounce around then comment out
              these statements.

        NOTE 2: This if statement does not do the behavior described above anymore.
                The reasoning behind it had to do with how the acceleration was
                coded in the early versions of this project (the range was from
                -0.5 to 0.5). Now that the range is from 0 to 0.5, that random
                change in direction does not happen anymore. Only a slight change
                in direction such as a slight turn to the right happens.
        """

        """
        Commenting this out as adding a random value to the velocity does not make sense.
        """
        
        """
        # check if the drone's acceleration is set to 0
        if (self.acceleration == Vector(0, 0)):
            # create the drone's acceleration range between 0 and 1
            x_acceleration_range = rand()
            y_acceleration_range = rand()

            # divide the acceleration range to 0 and 0.5
            x_acceleration_range = x_acceleration_range / 2
            y_acceleration_range = y_acceleration_range / 2

            # the object's acceleration on graph
            self.acceleration = Vector(x_acceleration_range, y_acceleration_range)
        """
        


    """
    Function that checks the drone's speed and adjusts it accordingly
    """

    def drone_speed_check(self):
        # get the current drone's velocity
        current_drone_velocity = self.velocity

        # calculate the drone's velocity magnitude
        drone_x_velocity = self.velocity.x
        drone_y_velocity = self.velocity.y
        velocity_magnitude = self.drone_velocity_magnitude(drone_x_velocity, drone_y_velocity)

        # check if the drone's velocity is over the speed limit
        if(self.max_speed < velocity_magnitude):
            # slow down the drone's velocity

            # divide the current velocity by the calculated velocity
            update_velocity = current_drone_velocity / velocity_magnitude

            # multiply the updated velocity by the max speed
            update_velocity = update_velocity * self.max_speed

            # update the drone's velocity
            self.velocity = update_velocity

            # set the drone's acceleration to 0
            self.acceleration = Vector(0, 0)

        # check if the drone's velocity is under the speed limit
        if (self.min_speed > velocity_magnitude):
            # speed up the drone's velocity

            # divide the current velocity by the calculated velocity
            update_velocity = current_drone_velocity / velocity_magnitude

            # multiply the updated velocity by the min speed
            update_velocity = update_velocity * self.min_speed

            # update the drone's velocity
            self.velocity = update_velocity

            # set the drone's acceleration to 0
            self.acceleration = Vector(0, 0)

    """
    Function that calculates the drone's velocity magnitude
    """

    def drone_velocity_magnitude(self, x_velocity, y_velocity):
        # square the velocities
        x_velocity_squared = x_velocity * x_velocity
        y_velocity_squared = y_velocity * y_velocity

        # add the velocities together
        add_velocity = x_velocity_squared + y_velocity_squared

        # square root the velocities
        velocity_magnitude = math.sqrt(add_velocity)

        # return the velocity magnitude
        return velocity_magnitude


    """
    Function that checks if drone object is at the border and avoid it
    """

    def border_avoidance(self):
        # get the current drone's position
        current_drone_x_position = self.position.x
        current_drone_y_position = self.position.y

        # get the current drone's velocity
        current_drone_x_velocity = self.velocity.x
        current_drone_y_velocity = self.velocity.y

        # check if the drone is at the left of the screen
        if (current_drone_x_position < border_margin):
            # reverse the velocity in the x direction (make the velocity positive)
            current_drone_x_velocity = current_drone_x_velocity + change_border_direction_factor

        # check if the drone is at the right of the screen
        # (add 100 to shift graph)
        if (current_drone_x_position > (self.width + 100 - border_margin)):
            # reverse the velocity in the x direction (make the velocity negative)
            current_drone_x_velocity = current_drone_x_velocity - change_border_direction_factor

        # check if the drone is at the top of the screen
        if (current_drone_y_position < border_margin):
            # reverse the velocity in the y direction (make the velocity positive)
            current_drone_y_velocity = current_drone_y_velocity + change_border_direction_factor

        # check if the drone is at the bottom of the screen
        # (add 100 to shift graph)
        if (current_drone_y_position > (self.height + 100 - border_margin)):
            # reverse the velocity in the y direction (make the velocity negative)
            current_drone_y_velocity = current_drone_y_velocity - change_border_direction_factor

        # update the drone's velocity
        border_avoid_velocity = Vector(current_drone_x_velocity, current_drone_y_velocity)
        #self.velocity = border_avoid_velocity

        # return the border avoidance velocity
        return border_avoid_velocity

    
    """
    Function that creates rocks
    """

    def create_rock(self):
        # determines the border color
        stroke("red")

        # determines the fill color
        fill("red")

        # creates a circle in a defined position and radius
        circle((self.position.x, self.position.y), 20)

        # set obstacle flag to true
        self.obstacle = True


    """
    Function that checks if drone is near a rock and avoid it
    """

    def rock_avoidance(self, list):
        # get the current drone's position
        current_drone_position = self.position

        # get the current drone's velocity
        current_drone_velocity = self.velocity

        # the direction needed to avoid colliding with a rock
        rock_avoid_direction = Vector(0, 0)

        # go through each rock
        for rock in list:
            # create an area of avoidance for each rock
            x_position_rock_avoidance = rock[0] + avoid_rock_radius
            y_position_rock_avoidance = rock[1] + avoid_rock_radius 
            position_rock_avoidance = Vector(x_position_rock_avoidance, y_position_rock_avoidance)

            # calculate the distance magnitude between the drone and rock
            distance_magnitude = math.dist(current_drone_position, position_rock_avoidance)

            # check if the distance between the drone and rock is less than the allowed distance
            # (add 10 to increase the sight distance)
            """
            NOTE: We need to check if the drone is close to a rock unlike shown in the pseudocode
                  here (http://www.kfish.org/boids/pseudocode.html). The reason being is that
                  there is no point in changing the drone's trajectory when the rock is far from
                  the drone at the other side of the map.
            """
            if (distance_magnitude < (self.sight_distance + 10)):
                # calulate the distance between the drone and rock
                distance_between_drone_and_rock = current_drone_position - position_rock_avoidance

                # add the distance to the total
                rock_avoid_direction = rock_avoid_direction + distance_between_drone_and_rock

        # multiply the avoid direction by the rock avoid percentage
        rock_avoid_velocity = rock_avoidance_percentage_factor * rock_avoid_direction

        # add the velocity to the current drone's velocity
        current_drone_velocity = current_drone_velocity + rock_avoid_velocity

        # update the current drone's velocity
        #self.velocity = current_drone_velocity

        # return the rock avoid velocity
        return rock_avoid_velocity


    """
    Boids Algorithm

    Rule 1: Separation
    Function that checks if drone is near other drones and avoids them
    """

    def rule_one_separation(self, drones):
        # get the current drone's position
        current_drone_position = self.position

        # get the current drone's velocity
        current_drone_velocity = self.velocity

        # the direction needed to avoid colliding with a drone
        drone_avoid_direction = Vector(0, 0)

        # store the current drone
        current_drone = self

        # the drone avoid velocity
        drone_avoid_velocity = Vector(0, 0)

        # go through each existing drone
        for drone in drones:
            # store the other drone
            other_drone = drone

            # check if the current drone is not itself
            if((current_drone != other_drone) == True):
                # get the other drone's position
                other_drone_position = drone.position

                # calculate the distance magnitude between the current drone and another drone
                distance_magnitude = math.dist(current_drone_position, other_drone_position)

                # check if the distance between the two drones is less than the drone's sight distance
                # (add 20 to increase sight distance)
                """
                NOTE: We need to check if the drone is close to another drone unlike shown in the
                      pseudocode here (http://www.kfish.org/boids/pseudocode.html). The reason being
                      is that there is no point in changing the drone's trajectory when the drone is
                      far from the other drone at the other side of the map.
                """
                if (distance_magnitude < (self.sight_distance)):
                    # calculate the distance between the two drones
                    distance_between_drones = current_drone_position - other_drone_position

                    # add the distances to the total
                    """
                    NOTE: Subtracting the total by the distance would break this rule's behavior if
                          we follow the pseudocode here (http://www.kfish.org/boids/pseudocode.html).
                          Adding the total will get the results we are looking for.
                    """
                    drone_avoid_direction = drone_avoid_direction + distance_between_drones

        # multiply the avoid direction by the drone avoid percentage
        drone_avoid_velocity = drone_avoidance_percentage_factor * drone_avoid_direction

        # add the velocity to the current drone's velocity
        current_drone_velocity = current_drone_velocity + drone_avoid_velocity

        # update the current drone's velocity
        #self.velocity = current_drone_velocity

        # return the drone avoid velocity
        return drone_avoid_velocity


    """
    Rule 2: Alignment
    Function that makes current drone match the average velocity of nearby drones
    """

    def rule_two_alignment(self, drones):
        # get the current drone's position
        current_drone_position = self.position

        # get the current drone's velocity
        current_drone_velocity = self.velocity

        # the number of drones nearby current drone
        number_of_nearby_drones = 0

        # the total drone velocity of nearby drones
        total_drone_velocity = Vector(0, 0)

        # store the current drone
        current_drone = self

        # the drone alignment velocity
        drone_alignment_velocity = Vector(0, 0)

        # go through each existing drone
        for drone in drones:
            # store the other drone
            other_drone = drone

            # check if the current drone is not itself
            if((current_drone != other_drone) == True):
                # get the other drone's position
                other_drone_position = drone.position

                # calculate the distance magnitude between the current drone and another drone
                distance_magnitude = math.dist(current_drone_position, other_drone_position)

                # check if the distance between the two drones is less than the drone's sight distance
                # (add 50 to increase sight distance)
                """
                NOTE: We need to check if the drone is close to another drone unlike shown in the
                      pseudocode here (http://www.kfish.org/boids/pseudocode.html). The reason being
                      is that there is no point in changing the drone's trajectory when the drone is
                      far from the other drone at the other side of the map.
                """
                if (distance_magnitude < (self.sight_distance + 30)):
                    # increase the number of nearby drones
                    number_of_nearby_drones = number_of_nearby_drones + 1

                    # get the other drone's velocity
                    other_drone_velocity = drone.velocity

                    # add the velocity to the total
                    total_drone_velocity = total_drone_velocity + other_drone_velocity

        # check if the number of nearby drones is not 0
        """
        NOTE: We check if there are any nearby drones to avoid dividing by 0.
        """
        if (number_of_nearby_drones != 0):
            # divide the total velocity by the number of nearby drones to get the average velocity
            average_drone_velocity = total_drone_velocity / number_of_nearby_drones

            # subtract the current drone's velocity by the average velocity
            subtract_drone_velocity = average_drone_velocity - current_drone_velocity

            # multiply the average velocity by the match velocity percentage
            drone_alignment_velocity = match_velocity_percentage_factor * subtract_drone_velocity

            # update the current drone's velocity
            current_drone_velocity = current_drone_velocity + drone_alignment_velocity

        # store current drone's new velocity
        #self.velocity = current_drone_velocity

        # return the drone alignment velocity
        return drone_alignment_velocity


    """
    Rule 3: Cohesion
    Function that finds the center of mass from the nearby drones and makes the current drone fly towards it
    """

    def rule_three_cohesion(self, drones):
        # get the current drone's position
        current_drone_position = self.position

        # get the current drone's velocity
        current_drone_velocity = self.velocity

        # the number of drones nearby current drone
        number_of_nearby_drones = 0

        # the total drone position of nearby drones
        total_drone_position = Vector(0, 0)

        # store the current drone
        current_drone = self

        # the drone cohesion velocity
        drone_cohesion_velocity = Vector(0, 0)

        # go through each existing drone
        for drone in drones:
            # store the other drone
            other_drone = drone

            # check if the current drone is not itself
            if((current_drone != other_drone) == True):
                # get the other drone's position
                other_drone_position = drone.position

                # calculate the distance magnitude between the current drone and another drone
                distance_magnitude = math.dist(current_drone_position, other_drone_position)

                # check if the distance between the two drones is less than the drone's sight distance
                # (add 50 to increase sight distance)
                """
                NOTE: We need to check if the drone is close to another drone unlike shown in the
                      pseudocode here (http://www.kfish.org/boids/pseudocode.html). The reason being
                      is that there is no point in changing the drone's trajectory when the drone is
                      far from the other drone at the other side of the map.
                """
                if (distance_magnitude < (self.sight_distance + 30)):
                    # increase the number of nearby drones
                    number_of_nearby_drones = number_of_nearby_drones + 1

                    # add the position to the total
                    total_drone_position = total_drone_position + other_drone_position

        # check if the number of nearby drones is not 0
        """
        NOTE: We check if there are any nearby drones to avoid dividing by 0.
        """
        if (number_of_nearby_drones != 0):
            # divide the total position by the number of nearby drones to get the center of mass
            center_of_mass = total_drone_position / number_of_nearby_drones

            # subtract the center of mass by the current drone's position to get the distance
            distance_between_mass_and_drone = center_of_mass - current_drone_position

            # multiply the distance by the go to center percentage
            drone_cohesion_velocity = go_to_center_percentage_factor * distance_between_mass_and_drone

            # update the current drone's velocity
            current_drone_velocity = current_drone_velocity + drone_cohesion_velocity

        # store current drone's new velocity
        #self.velocity = current_drone_velocity

        # return the drone cohesion velocity
        return drone_cohesion_velocity