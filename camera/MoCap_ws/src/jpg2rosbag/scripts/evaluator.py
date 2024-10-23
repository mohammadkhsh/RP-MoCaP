from custom_msg_f32.msg import Float32MultiArrayPlusStamp
import os


import rosbag
import json
import numpy as np
from sklearn.metrics import mean_squared_error
from math import sqrt


joints_info = {
        "Body": 0,
        "Chest": 1,
        "Head": 2,
        "HeadEnd": 3,
        "Hips": 4,
        "LeftArm": 5,
        "LeftFinger": 6,
        "LeftFingerEnd": 7,
        "LeftFoot": 8,
        "LeftForearm": 9,
        "LeftHand": 10,
        "LeftHeel": 11,
        "LeftLeg": 12,
        "LeftShoulder": 13,
        "LeftThigh": 14,
        "LeftToe": 15,
        "LeftToeEnd": 16,
        "Neck": 17,
        "RightArm": 18,
        "RightFinger": 19,
        "RightFingerEnd": 20,
        "RightFoot": 21,
        "RightForearm": 22,
        "RightHand": 23,
        "RightHeel": 24,
        "RightLeg": 25,
        "RightShoulder": 26,
        "RightThigh": 27,
        "RightToe": 28,
        "RightToeEnd": 29,
        "SpineLow": 30,
        "SpineMid": 31
    }



def load_json_file(filename):
    try:
        with open(filename, 'r') as f:
            json_data = json.load(f)
        return json_data
    except json.JSONDecodeError as e:
        print(f"JSON decoding error: {e}. Failed to load JSON file: {json_file_path}.")
        return None
    except Exception as e:
        print(f"Failed to load JSON file: {json_file_path}. Error: {e}")
        return None




# Load the bag file
date_version = "_23-5-2023_2"
current_dir = os.path.dirname(os.path.abspath(__file__))
output_bag = "OUTPUT" + date_version + "/full_output.bag"
output_bag = os.path.join(current_dir, output_bag)


# Load the JSON file
json_file_path = 'output_684850487699178880.json'
json_file_path = os.path.join(current_dir, json_file_path)


json_data = load_json_file(json_file_path)


# Extract the labeled points from the JSON file
labeled_points = {}
for shape in json_data['shapes']:
    label = shape['label']
    if label not in labeled_points:
        labeled_points[label] = []
    points = shape['points']
    labeled_points[label].extend(points)


# Initialize lists to store the errors for each joint
mse_list = []
rmse_list = []
pck_list = []

# Function to calculate Mean Squared Error (MSE)
def calculate_mse(labeled_points, generated_points):
    return mean_squared_error(labeled_points, generated_points)

# Function to calculate Root Mean Squared Error (RMSE)
def calculate_rmse(labeled_points, generated_points):
    mse = mean_squared_error(labeled_points, generated_points)
    return sqrt(mse)

# Function to calculate Percentage of Correct Keypoints (PCK)
def calculate_pck(labeled_points, generated_points, threshold):
    correct_count = 0
    for i in range(len(labeled_points)):
        dist = np.linalg.norm(np.array(labeled_points[i]) - np.array(generated_points[i]))
        if dist <= threshold:
            correct_count += 1
    return correct_count / len(labeled_points)

# Process the messages in the bag file

generated_points = {joint_label: [] for joint_label in labeled_points.keys()}

bag = rosbag.Bag(output_bag, 'r')
for topic, msg, t in bag.read_messages(topics=['/projected_points']):
    # Extract the timestamp from the message
    timestamp = str(msg.header.stamp)

    # Generate the JSON filename based on the timestamp
    json_filename = 'output_' + timestamp[1:] + '.json'
    json_filename = os.path.join(current_dir, json_filename)


    # Load the corresponding JSON file
    try:
        json_data = load_json_file(json_filename)
        if json_data is None:
            continue
    except NameError:
        print(f"Failed to process JSON file for timestamp {timestamp[1:]}. Skipping...")
        continue



    # Extract the labeled points from the JSON file
    labeled_points = {}
    for shape in json_data['shapes']:
        label = shape['label']
        if label not in labeled_points:
            labeled_points[label] = []
        points = shape['points']
        labeled_points[label].extend(points)
    print(labeled_points)

    # Extract the generated points from the message
    generated_points = {}
    for joint_label, idx in joints_info.items():
        if joint_label in labeled_points:
            # Extract the generated points from msg.data using the joint index
            u = msg.data[idx * 2]  # X-coordinate
            v = msg.data[idx * 2 + 1]  # Y-coordinate
            generated_points[joint_label] = [[u, v]]


    print(generated_points)
    # Calculate the errors for each joint
    mse_joint = {}
    rmse_joint = {}
    pck_joint = {}

    for joint_label in generated_points.keys():
        labeled = labeled_points[joint_label]
        generated = generated_points[joint_label]

        mse = calculate_mse(labeled, generated)
        rmse = calculate_rmse(labeled, generated)
        pck = calculate_pck(labeled, generated, threshold=10)

        mse_joint[joint_label] = mse
        rmse_joint[joint_label] = rmse
        pck_joint[joint_label] = pck

    mse_list.append(mse_joint)
    rmse_list.append(rmse_joint)
    pck_list.append(pck_joint)

# Calculate the average errors across all messages
mse_avg = {}
rmse_avg = {}
pck_avg = {}

for joint_label in labeled_points.keys():
    mse_joint_values = [mse[joint_label] for mse in mse_list]
    rmse_joint_values = [rmse[joint_label] for rmse in rmse_list]
    pck_joint_values = [pck[joint_label] for pck in pck_list]

    mse_avg[joint_label] = np.mean(mse_joint_values)
    rmse_avg[joint_label] = np.mean(rmse_joint_values)
    pck_avg[joint_label] = np.mean(pck_joint_values)

# Function to calculate Mean Squared Error (MSE) for each joint
def calculate_joint_mse(labeled_points, generated_points):
    mse_joint = {}
    for joint_label in labeled_points.keys():
        labeled = labeled_points[joint_label]
        generated = generated_points[joint_label]
        mse = calculate_mse(labeled, generated)
        mse_joint[joint_label] = mse
    return mse_joint

# Function to calculate Root Mean Squared Error (RMSE) for each joint
def calculate_joint_rmse(labeled_points, generated_points):
    rmse_joint = {}
    for joint_label in labeled_points.keys():
        labeled = labeled_points[joint_label]
        generated = generated_points[joint_label]
        rmse = calculate_rmse(labeled, generated)
        rmse_joint[joint_label] = rmse
    return rmse_joint

# Function to calculate Percentage of Correct Keypoints (PCK) for each joint
def calculate_joint_pck(labeled_points, generated_points, threshold):
    pck_joint = {}
    for joint_label in labeled_points.keys():
        labeled = labeled_points[joint_label]
        generated = generated_points[joint_label]
        pck = calculate_pck(labeled, generated, threshold)
        pck_joint[joint_label] = pck
    return pck_joint

# Print the average errors
for joint_label in labeled_points.keys():
    print('Joint:', joint_label)
    print('MSE:', mse_avg[joint_label])
    print('RMSE:', rmse_avg[joint_label])
    print('PCK:', pck_avg[joint_label])
    print('---')

# Calculate the joint errors
joint_mse = calculate_joint_mse(labeled_points, generated_points)
joint_rmse = calculate_joint_rmse(labeled_points, generated_points)
joint_pck = calculate_joint_pck(labeled_points, generated_points, threshold=20)

# Print the joint errors
for joint_label in labeled_points.keys():
    print('Joint:', joint_label)
    print('MSE:', joint_mse[joint_label])
    print('RMSE:', joint_rmse[joint_label])
    print('PCK:', joint_pck[joint_label])
    print('---')

# Close the bag file
bag.close()
