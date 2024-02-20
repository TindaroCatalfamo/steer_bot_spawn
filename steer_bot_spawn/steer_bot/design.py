#!/usr/bin/env python

import rospy
import math
import numpy as np
import random
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from scipy import interpolate

# Constants
SKIP_LEFT_POINTS = [] #[24, 25]
SKIP_RIGHT_POINTS = [] #[11, 13, 39, 40, 41]
DISTANCE = 1.8  # Updated distance

# Function to calculate the distance between two points
def calculate_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

# Function to read an SDF file
def read_file_sdf(filename):
    with open(filename, 'r') as file:
        return file.read()

# Read the model content
MODEL_CONTENT_CONE_LEFT = read_file_sdf('../steer_bot/cone_yellow/model.sdf')
MODEL_CONTENT_CONE_RIGHT = read_file_sdf('../steer_bot/cone_blue/model.sdf')
MODEL_CONTENT_ROAD = read_file_sdf('../steer_bot/road/road.sdf')
MODEL_CONTENT_TREE = read_file_sdf('../steer_bot/tree/tree.sdf')

# Generate the primary trajectory
def generate_trajectory():
    cx = np.arange(0, 80, 1)
#    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
#    tck = interpolate.splrep(cx, cy)
    cy = [math.sin(ix / 8.33) * ix / 4.0 for ix in cx]
    # Interpolazione spline
    tck = interpolate.splrep(cx, cy, s=0)

    return cx, cy, tck

# Function to spawn a model
def spawn(name, x, y, z, model_content):
    spawn_model = rospy.ServiceProxy('steer_bot/gazebo/spawn_sdf_model', SpawnModel)
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z
    response = spawn_model(name, model_content, "", initial_pose, "world")
    rospy.loginfo(response.status_message)

# Main function
def node():
    rospy.init_node('spawn_model')
    rospy.wait_for_service('steer_bot/gazebo/spawn_sdf_model')

    cx, cy, tck = generate_trajectory()

    # Generate points for cones
    left_points = []
    right_points = []
    for i in range(len(cx)):
        x0, y0 = cx[i], cy[i]
        dydx = interpolate.splev(x0, tck, der=1)
        angle = np.arctan(abs(dydx))

        if dydx >= 0:
            x1, y1 = x0 - DISTANCE * np.sin(angle), y0 + DISTANCE * np.cos(angle)
            x2, y2 = x0 + DISTANCE * np.sin(angle), y0 - DISTANCE * np.cos(angle)
        else:
            x1, y1 = x0 + DISTANCE * np.sin(angle), y0 + DISTANCE * np.cos(angle)
            x2, y2 = x0 - DISTANCE * np.sin(angle), y0 - DISTANCE * np.cos(angle)

        left_points.append((x1, y1))
        right_points.append((x2, y2))

    try:
        # Generate road
        for i in range(len(cx) - 1):
            start_point = (cx[i], cy[i])
            end_point = (cx[i + 1], cy[i + 1])
            dist = calculate_distance(start_point, end_point)

            num_segments = int(math.ceil(dist / 0.6))
            segment_length = dist / num_segments

            x_diff = end_point[0] - start_point[0]
            y_diff = end_point[1] - start_point[1]

            for j in range(num_segments):
                x = start_point[0] + (j * x_diff / num_segments)
                y = start_point[1] + (j * y_diff / num_segments)
                modified_road_sdf = MODEL_CONTENT_ROAD.replace('<size>3 4 .1</size>', f'<size>3 {segment_length} .1</size>')
                spawn(f'roadSegment_{i}_{j}', x, y, 0, modified_road_sdf)

        # Generate cones
        for i, (x, y) in enumerate(left_points):
            if i not in SKIP_LEFT_POINTS:
                spawn(f'pointL_{i}', x, y, 0.3, MODEL_CONTENT_CONE_LEFT)

        for i, (x, y) in enumerate(right_points):
            if i not in SKIP_RIGHT_POINTS:
                spawn(f'pointR_{i}', x, y, 0.3, MODEL_CONTENT_CONE_RIGHT)

        # Generate trees
        for i in range(80):
            x = random.uniform(min(cx), max(cx))
            y = random.uniform(min(cy), max(cy))

            # Make sure trees are at least 3 units away from cones defining road edges
            if all(calculate_distance((x, y), point) > 3 for point in left_points + right_points):
                spawn(f'tree_{i}', x, y, 0, MODEL_CONTENT_TREE)

    except rospy.ServiceException as exc:
        rospy.logerr("Error during service invocation: %s", str(exc))

if __name__ == '__main__':
    node()
