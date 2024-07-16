#!/usr/bin/env python
import rospy

import os
import csv
import time
import glob
import pandas as pd

from std_msgs.msg import Header
from object_detection_msgs.msg import ObjectDetectionInfoArray

timestr = ''

def callback(msg):
    infos = []
    for info in msg.info:
        info = {
            'class_id': info.class_id,
            'id': info.id,
            'x': info.position.x, # in camera frame
            'y': info.position.y, # in camera frame
            'z': info.position.z # in camera frame
        }
        infos.append(info)
    
    print(infos)
    
    # Check if the CSV file exists
    dirname = os.path.dirname(__file__)
    #timestr = time.strftime("%Y%m%d-%H%M%S")
    file_name = 'artifacts-' + timestr + '.csv' # Add the already set timestamp
    #file_name = 'artifacts.csv'
    file_path = os.path.join(dirname, './../data/' + file_name) # Relative path
    file_exists = os.path.isfile(file_path)
    
    # Save the detected artifacts to a CSV file
    if file_exists:
        with open(file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            for info in infos:
                writer.writerow([info['class_id'], info['id'], info['x'], info['y'], info['z']])

def shutdown_hook():
    print("Shutting down node and performing cleanup.")

    # Take the latest saved CSV file
    dirname = os.path.dirname(__file__)
    file_path = os.path.join(dirname, './../data/')
    list_of_files = glob.glob(os.path.join(file_path, "*.csv"))
    if list_of_files:
        latest_file = max(list_of_files, key=os.path.getctime)
        print(f"Processing the last saved CSV file: {latest_file}")

        # Final filtering and clustering

        # Load the CSV file into a pandas DataFrame
        df = pd.read_csv(latest_file)
        
        # Calculate the average of 'x', 'y', and 'z' for each 'class_id' and 'id' combination
        avg_df = df.groupby(['class_id', 'id']).agg({'x': 'mean', 'y': 'mean', 'z': 'mean'}).reset_index()
        
        # Save the filtered data to a new CSV file
        file_name = 'artifacts-' + timestr + '-clustered.csv' # Add the already set timestamp + being clustered flag
        avg_file_path = os.path.join(file_path, file_name)
        avg_df.to_csv(avg_file_path, index=False)
        
        print(f"Filtered data saved to: {avg_file_path}")

def object_detection_saver():
    rospy.init_node('object_detection_saver', anonymous=True)

    global timestr
    
    # Check if the CSV file exists
    dirname = os.path.dirname(__file__)
    timestr = time.strftime("%Y%m%d-%H%M%S")
    #file_name = 'artifacts.csv'
    file_name = 'artifacts-' + timestr + '.csv' # Add timestamp for the beginning of the run
    file_path = os.path.join(dirname, './../data/' + file_name) # Relative path
    file_exists = os.path.isfile(file_path)

    # Write the header only if the file doesn't exist
    if not file_exists:
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['class_id', 'id', 'x', 'y', 'z'])

    # Register the shutdown hook
    rospy.on_shutdown(shutdown_hook)

    # Subscribe to topics
    #rospy.Subscriber('/object_detector/detection_info', ObjectDetectionInfoArray, callback) # For independent testing
    rospy.Subscriber('/object_inspector/unique_artifacts', ObjectDetectionInfoArray, callback)

    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    object_detection_saver()
