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

from boids import Boids


"""
Create window size
"""

window_height = 1200
#window_width = 1300     # for adding a legend
window_width = 1200


"""
Create area size
"""

area_height = 1000
area_width = 1000


"""
Create time array
"""

time = []
#time.append(0)


"""
Create time step
"""

global time_step
time_step = 0


"""
Create flags
"""

flag_0 = False
flag_1 = False
flag_2 = False
flag_3 = False
flag_4 = False
flag_5 = False
flag_6 = False


"""
Create drones and scatter them throughout the graph
"""

# the amount of drones in the system
number_of_drones = 10

# list of drones
drones = []

# list of drone colors
colors = ["firebrick", "purple", "green", "dodgerblue", "gold", "black", "blue", "magenta", "greenyellow", "turquoise"]

# go through each drone
for i in range(number_of_drones):
    # calculate a random x position (600, 600 is the center of the graph)
    # (add 100 for shifting the grid)
    x_position = rand() * 1000 + 100

    # calculate a random y position
    # (add 100 for shifting the grid)
    y_position = rand() * 1000 + 100

    # create drone object
    # (add 100 for shifting the grid)
    if (i == 0):
        drone = Boids(x_position, y_position, area_width + 100, area_height + 100, True, colors[i])

    else:
        drone = Boids(x_position, y_position, area_width + 100, area_height + 100, False, colors[i])

    # store the drone object in an array
    drones.append(drone)


"""
Create rocks and scatter them throughout the graph
"""

# the amount of rocks in the system
number_of_rocks = 5

# list of rocks
rocks = []
rocks_positions = []

# go through each rock
for i in range(number_of_rocks):
    # calculate a random x position (600, 600 is the center of the graph)
    # (add 100 for shifting the grid)
    x_position = rand() * 1000 + 100

    # calculate a random y position
    # (add 100 for shifting the grid)
    y_position = rand() * 1000 + 100

    # create rock object
    # (add 100 for shifting the grid)
    rock = Boids(x_position, y_position, area_width + 100, area_height + 100, False, "red")

    # store the rock object in an array
    rocks.append(rock)

    # store the rock's position
    rocks_positions.append([x_position, y_position])


"""
NOTE: These are to create a set number of drones and rocks at random spots in the graph.
      The only things that are allowed to be changed are the set numbers and giving each
      object a defined starting point. Everything else must NOT be touched.
"""


"""
Function that sets up window
"""

def setup():
    # create the window size
    size(window_width, window_height)

    # font to use inside window
    text_font(create_font("Roboto-Black.ttf", size = 20))




"""
NOTE: The setup function should NEVER be touched unless you need to physically alter
      the window in some shape or form.
"""


"""
Function that draws and animates the legend the window
"""

def draw_legend():
    # create legend

    # x and y positions
    x_legend = 1150
    y_legend = 150

    # determines the border color
    stroke("black")

    # determines the fill color
    fill("black")

    # create the legend text
    textAlign(CENTER)
    legend_string = "Legend"
    text(legend_string, x_legend + 50, y_legend)

    # iterate the positions
    y_legend = y_legend + 50

    # create legend box
    line(1130, 130, 1285, 130)  # top
    line(1130, 720, 1285, 720)  # bottom
    line(1130, 130, 1130, 720)  # left
    line(1285, 130, 1285, 720)  # right

    # loop through the number of drones
    for i in range(number_of_drones):
        # determines the border color
        stroke(colors[i])

        # determines the fill color
        fill(colors[i])

        # create a circle in a defined position and radius
        circle((x_legend, y_legend), 20)

        # determines the border color
        stroke("black")

        # determines the fill color
        fill("black")

        # create a text next to the circle
        textAlign(CENTER)
        rover_string = "Rover #" + str(i + 1)
        text(rover_string, x_legend + 70, y_legend - 13)

        # iterate the positions
        y_legend = y_legend + 50

    # determines the border color
    stroke("red")

    # determines the fill color
    fill("red")

    # create a circle in a defined position and radius
    circle((x_legend, y_legend), 20)

    # determines the border color
    stroke("black")

    # determines the fill color
    fill("black")

    # create a text next to the circle
    textAlign(CENTER)
    rock_string = "Obstacle"
    text(rock_string, x_legend + 70, y_legend - 13)


"""
Function that draws and animates the graph inside the window
"""

def draw_graph(time_step):
    # set the line color
    stroke(175)

    # fill color
    fill(0)

    # put simulation time onto graph
    textAlign(CENTER)
    print_time = '%.3f' % time_step
    time_string = "Time: " + str(print_time) + " seconds"
    text(time_string, 600, 60)

    # put simulation title onto graph
    textAlign(CENTER)
    title_string = "Simulation of the Boids Algorithm"
    text(title_string, 600, 15)

    # create the graph

    # spacing in between each line and label
    line_spacing = 100
    label_spacing = 10

    # numbering and labeling of the vertical lines
    x_label = 0
    x = 100

    # loop until we reach max width (add 100 to shift area in window)
    while (x <= area_width + 100):

        # create a line
        # (y coordinate should be 100 so that graph is at center)
        # (add 100 to area height to shift graph)
        line(x, 100, x, area_height + 100)

        # add and center text
        textAlign(CENTER)
        text(str(x_label), x, area_height + 100)

        # check if x_label reached 50
        if (x_label == 50):
            # add and center axis label
            textAlign(CENTER)
            text(str("X (Meters)"), x, area_height + 150)

        # increment values
        x_label = x_label + label_spacing
        x = x + line_spacing

    # numbering and labeling of the horizontal lines
    y_label = 100
    #y_label = 0
    y = 100

    # loop until we reach max height (add 100 to shift area in window)
    while (y <= area_height + 100):
        
        # create a line
        # (x coordinate should be 100 so that graph is at center)
        # (add 100 to area width to shift graph)
        line(100, y, area_width + 100, y)

        # add and center text
        textAlign(CENTER)
        text(str(y_label), 80, y - 10)
        #text(str(y_label), 70, y - 10)

        # check if the y_label reached 50
        if (y_label == 50):
            # add and center axis label
            textAlign(CENTER)
            text(str("Y"), 50, y + 10)
            textAlign(CENTER)
            text(str("(Meters)"), 50, y + 30)

        # increment values
        y_label = y_label - label_spacing
        #y_label = y_label + label_spacing
        y = y + line_spacing


"""
Function that draws and animates things inside the window
"""

def draw():
    # make the time step a global variable
    global time_step

    # print time
    #print(time_step)

    # global flags
    global flag_0
    global flag_1
    global flag_2
    global flag_3
    global flag_4
    global flag_5
    global flag_6

    # check if simulation is at 0 seconds and take screenshot of simulation
    if ((time_step >= 0) and (flag_0 == False)):
        # set flag to True
        flag_0 = True

        # screenshot the simulation
        save_frame("Screenshots/Simulation at 0 seconds.jpg")

    # check if simulation is at 5 seconds and take screenshot of simulation
    if ((time_step >= 5) and (flag_1 == False)):
        # set flag to True
        flag_1 = True

        # screenshot the simulation
        save_frame("Screenshots/Simulation at 5 seconds.jpg")

    # check if simulation is at 10 seconds and take screenshot of simulation
    if ((time_step >= 10) and (flag_2 == False)):
        # set flag to True
        flag_2 = True

        # screenshot the simulation
        save_frame("Screenshots/Simulation at 10 seconds.jpg")

    # check if simulation is at 15 seconds and take screenshot of simulation
    if ((time_step >= 15) and (flag_3 == False)):
        # set flag to True
        flag_3 = True

        # screenshot the simulation
        save_frame("Screenshots/Simulation at 15 seconds.jpg")

    # check if simulation is at 20 seconds and take screenshot of simulation
    if ((time_step >= 20) and (flag_4 == False)):
        # set flag to True
        flag_4 = True

        # screenshot the simulation
        save_frame("Screenshots/Simulation at 20 seconds.jpg")

    # check if simulation is at 25 seconds and take screenshot of simulation
    if ((time_step >= 25) and (flag_5 == False)):
        # set flag to True
        flag_5 = True

        # screenshot the simulation
        save_frame("Screenshots/Simulation at 25 seconds.jpg")

    # check if simulation is at 30 seconds and take screenshot of simulation
    if ((time_step >= 30) and (flag_6 == False)):
        # set flag to True
        flag_6 = True

        # screenshot the simulation
        save_frame("Screenshots/Simulation at 30 seconds.jpg")

    # check if the current time reaches the end of the simulation
    if (time_step >= 30):
        # print time
        #print(time_step)

        # exit the simulation
        exit()

    # set the background color to white
    background(255, 255, 255)

    # draw the graph and legend
    draw_graph(time_step)
    #draw_legend()  # NOTE: adding a legend causes the simulation to lag a lot

    """
    NOTE: The code before this note should NEVER be changed/touched.
          Those are meant to create the window and the area for the
          drones to drive around.
    """


    # go through each rock
    for rock in rocks:
        # show rock on graph
        rock.create_rock()

    # go through each drone
    for drone in drones:
        # self defined models
        
        # show the drone's position and direction
        drone.drone_position_and_direction()

        # update the drone's velocity if velocity is over/under the max/min speed
        drone.drone_speed_check()

        # update the drone's parameters and trail
        drone.update_drone_parameters_and_trail()


        # Boids algorithm

        # Rule 1: Separation
        # check if the drone is near another drone and avoid it
        rule_1_velocity = drone.rule_one_separation(drones)
        #drone.rule_one_separation(drones)
        drone.velocity = drone.velocity + rule_1_velocity

        # Rule 2: Alignment
        # check if the drone is near other drones and match their average velocity
        rule_2_velocity = drone.rule_two_alignment(drones)
        #drone.rule_two_alignment(drones)
        drone.velocity = drone.velocity + rule_2_velocity

        # Rule 3: Cohesion
        # check if the drone is near other drones and go to their center of mass
        rule_3_velocity = drone.rule_three_cohesion(drones)
        #drone.rule_three_cohesion(drones)
        drone.velocity = drone.velocity + rule_3_velocity


        # self defined models

        # check if the drone is near the border and avoid it
        border_avoid_velocity = drone.border_avoidance()
        #drone.border_avoidance()
        drone.velocity = drone.velocity + border_avoid_velocity

        # check if the drone is near a rock and avoid it
        rock_avoid_velocity = drone.rock_avoidance(rocks_positions)
        #drone.rock_avoidance(rocks_positions)
        drone.velocity = drone.velocity + rock_avoid_velocity


        """
        NOTE: The velocity MUST be updated whenever a NEW velocity is calculated. This
              will give the drone a much smoother trajectory and make sure the defined
              behaviors are shown in real-time. Adding them together at the end as
              shown in the pseudocode here (http://www.kfish.org/boids/pseudocode.html)
              will cause some issues where the behaviors will lag behind. Additionally,
              the position MUST NOT be updated at the end as well. It is already
              updated in the update_drone_parameters_and_trail() function, so updating
              it twice will drastically and incorrectly increase the drone's distance.
        """

    # find the current time in seconds and add it to the time list
    time.append(time_step)
    time_step = millis() / 1000
    #time_step = second()
        

"""
Run the program
"""

run()


"""
Create plots for the drones' x and y velocities
"""

# create a figure
#plt.figure()
fig = plt.figure(num = 1, figsize = (20, 15), dpi = 80, facecolor = 'w', edgecolor = 'k')

# incrementor
i = 1

# go through all the drones
for drone in drones:
    #drone.plot_drones_equilibrium(time)
    # grab the drone's list of positions
    drone_positions = drone.positions_list

    # create empty lists
    magnitudes = []
    x_positions = []
    y_positions = []
    #magnitudes.append(0)

    # go through the drone's positions
    for position in drone_positions:
        # get the x and y positions
        # subtract it by 100 due to the graph shift
        # divide it by 10 to get them in meters
        # subtract y position from 100 to get true y position
        x_position = (position[0] - 100) / 10
        y_position = 100 - ((position[1] - 100) / 10)

        #magnitude = math.sqrt((x_position * x_position) + (y_position * y_position))
        #magnitudes.append(magnitude)

        # add the x position to the list
        x_positions.append(x_position)

    if (i <= 5):
        # plot the drone's x position over time
        plt.plot(time, x_positions, label = "Rover #" + str(i), color = drone.color)

    # increment the incrementor
    i = i + 1

# make the plot
plt.xlabel("Time (seconds)")
plt.ylabel("X Position (meters)")
plt.legend(bbox_to_anchor = (1.0, 1.0), loc = "upper left")
plt.title("Rovers' X Velocity (Meters / Second)")
plt.grid()

# save the plot
plt.savefig("Plots/Rovers 1 to 5 X Position.jpg")

# show the plot
plt.show()



# create a figure
#plt.figure()
fig = plt.figure(num = 1, figsize = (20, 15), dpi = 80, facecolor = 'w', edgecolor = 'k')

# incrementor
i = 1

# go through all the drones
for drone in drones:
    #drone.plot_drones_equilibrium(time)
    # grab the drone's list of positions
    drone_positions = drone.positions_list

    # create empty lists
    magnitudes = []
    x_positions = []
    y_positions = []
    #magnitudes.append(0)

    # go through the drone's positions
    for position in drone_positions:
        # get the x and y positions
        # subtract it by 100 due to the graph shift
        # divide it by 10 to get them in meters
        # subtract y position from 100 to get true y position
        x_position = (position[0] - 100) / 10
        y_position = 100 - ((position[1] - 100) / 10)

        #magnitude = math.sqrt((x_position * x_position) + (y_position * y_position))
        #magnitudes.append(magnitude)

        # add the x position to the list
        x_positions.append(x_position)

    if (i >= 6):
        # plot the drone's x position over time
        plt.plot(time, x_positions, label = "Rover #" + str(i), color = drone.color)

    # increment the incrementor
    i = i + 1

# make the plot
plt.xlabel("Time (seconds)")
plt.ylabel("X Position (meters)")
plt.legend(bbox_to_anchor = (1.0, 1.0), loc = "upper left")
plt.title("Rovers' X Velocity (Meters / Second)")
plt.grid()

# save the plot
plt.savefig("Plots/Rovers 6 to 10 X Position.jpg")

# show the plot
plt.show()



# create a figure
fig = plt.figure(num = 1, figsize = (20, 15), dpi = 80, facecolor = 'w', edgecolor = 'k')

# incrementor
i = 1

# go through all the drones
for drone in drones:
    #drone.plot_drones_equilibrium(time)
    # grab the drone's list of positions
    drone_positions = drone.positions_list

    # create empty lists
    magnitudes = []
    x_positions = []
    y_positions = []
    #magnitudes.append(0)

    # go through the drone's positions
    for position in drone_positions:
        # get the x and y positions
        # subtract it by 100 due to the graph shift
        # divide it by 10 to get them in meters
        # subtract y position from 100 to get true y position
        x_position = (position[0] - 100) / 10
        y_position = 100 - ((position[1] - 100) / 10)

        #magnitude = math.sqrt((x_position * x_position) + (y_position * y_position))
        #magnitudes.append(magnitude)

        # add the y position to the list
        y_positions.append(y_position)

    if (i <= 5):
        # plot the drone's y position over time
        plt.plot(time, y_positions, label = "Rover #" + str(i), color = drone.color)

    # increment the incrementor
    i = i + 1

# make the plot
plt.xlabel("Time (seconds)")
plt.ylabel("Y Position (meters)")
plt.legend(bbox_to_anchor = (1.0, 1.0), loc = "upper left")
plt.title("Rovers' Y Velocity (Meters / Second)")
plt.grid()

# save the plot
plt.savefig("Plots/Rovers 1 to 5 Y Position.jpg")

# show the plot
plt.show()


# create a figure
fig = plt.figure(num = 1, figsize = (20, 15), dpi = 80, facecolor = 'w', edgecolor = 'k')

# incrementor
i = 1

# go through all the drones
for drone in drones:
    #drone.plot_drones_equilibrium(time)
    # grab the drone's list of positions
    drone_positions = drone.positions_list

    # create empty lists
    magnitudes = []
    x_positions = []
    y_positions = []
    #magnitudes.append(0)

    # go through the drone's positions
    for position in drone_positions:
        # get the x and y positions
        # subtract it by 100 due to the graph shift
        # divide it by 10 to get them in meters
        # subtract y position from 100 to get true y position
        x_position = (position[0] - 100) / 10
        y_position = 100 - ((position[1] - 100) / 10)

        #magnitude = math.sqrt((x_position * x_position) + (y_position * y_position))
        #magnitudes.append(magnitude)

        # add the y position to the list
        y_positions.append(y_position)

    if (i >= 6):
        # plot the drone's y position over time
        plt.plot(time, y_positions, label = "Rover #" + str(i), color = drone.color)

    # increment the incrementor
    i = i + 1

# make the plot
plt.xlabel("Time (seconds)")
plt.ylabel("Y Position (meters)")
plt.legend(bbox_to_anchor = (1.0, 1.0), loc = "upper left")
plt.title("Rovers' Y Velocity (Meters / Second)")
plt.grid()

# save the plot
plt.savefig("Plots/Rovers 6 to 10 Y Position.jpg")

# show the plot
plt.show()




"""
# create a figure
fig = plt.figure(num = 1, figsize = (20, 15), dpi = 80, facecolor = 'w', edgecolor = 'k')

# incrementor
i = 1

# go through all the drones
for drone in drones:
    #drone.plot_drones_equilibrium(time)
    # grab the drone's list of positions
    drone_positions = drone.positions_list

    # create empty lists
    magnitudes = []
    x_positions = []
    y_positions = []
    #magnitudes.append(0)

    # go through the drone's positions
    for position in drone_positions:
        # get the x and y positions
        # subtract it by 100 due to the graph shift
        # divide it by 10 to get them in meters
        x_position = (position[0] - 100) / 10
        y_position = (position[1] - 100) / 10

        #magnitude = math.sqrt((x_position * x_position) + (y_position * y_position))
        #magnitudes.append(magnitude)

        # add the y position to the list
        x_positions.append(x_position)
        y_positions.append(y_position)

    if (i <= 5):
        # plot the drone's x and y position over time
        plt.plot(x_positions, y_positions, label = "Rover #" + str(i), color = drone.color)

    # increment the incrementor
    i = i + 1

# make the plot
plt.xlabel("X Position (meters)")
plt.ylabel("Y Position (meters)")
plt.legend(bbox_to_anchor = (1.0, 1.0), loc = "upper left")
plt.title("Rovers' X Position vs Y Position")
plt.grid()

# save the plot
#plt.savefig("Plots/Rovers 6 to 10 Y Position.jpg")

# show the plot
plt.show()



# create a figure
fig = plt.figure(num = 1, figsize = (20, 15), dpi = 80, facecolor = 'w', edgecolor = 'k')

# incrementor
i = 1

# go through all the drones
for drone in drones:
    #drone.plot_drones_equilibrium(time)
    # grab the drone's list of positions
    drone_positions = drone.positions_list

    # create empty lists
    magnitudes = []
    x_positions = []
    y_positions = []
    #magnitudes.append(0)

    # go through the drone's positions
    for position in drone_positions:
        # get the x and y positions
        # subtract it by 100 due to the graph shift
        # divide it by 10 to get them in meters
        x_position = (position[0] - 100) / 10
        y_position = (position[1] - 100) / 10

        #magnitude = math.sqrt((x_position * x_position) + (y_position * y_position))
        #magnitudes.append(magnitude)

        # add the y position to the list
        x_positions.append(x_position)
        y_positions.append(y_position)

    if (i >= 6):
        # plot the drone's x and y position
        plt.plot(x_positions, y_positions, label = "Rover #" + str(i), color = drone.color)

    # increment the incrementor
    i = i + 1

# make the plot
plt.xlabel("X Position (meters)")
plt.ylabel("Y Position (meters)")
plt.legend(bbox_to_anchor = (1.0, 1.0), loc = "upper left")
plt.title("Rovers' X Position vs Y Position")
plt.grid()

# save the plot
#plt.savefig("Plots/Rovers 6 to 10 Y Position.jpg")

# show the plot
plt.show()
"""