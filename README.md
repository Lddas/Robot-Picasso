### Robot Drawing Project

## Overview

This project was completed as part of an academic course at TECNICO LISBON. The objective was to design and implement a system that allows a robot to autonomously draw any given image. The project involved image processing, path planning, and control of the robot’s motion to reproduce the input drawing with high precision.

Features

	•	Image Processing: Converts an input image into a binary format that identifies the relevant lines and shapes to be drawn by the robot.
	•	Path Planning: Computes the optimal path for the robot to follow, ensuring the drawing is as accurate as possible.
	•	Robot Control: Directs the robot to follow the path based on the processed image data.
	•	Downscaling and Skeletonization: The system scales down the image and extracts the skeleton to minimize the number of points the robot needs to process while keeping the shape intact.

Project Structure

	•	main.py: Main script that runs the image processing, path planning, and robot control functions.
	•	code_leo.py: Contains helper functions related to image processing and binary mask generation.
	•	test_draw_1.png, test_draw_2.png: Sample images used to test the robot’s drawing capability.
	•	curvedline.png, circle.png: Examples of more complex shapes for the robot to draw.

Installation

To run this project, you will need:

	•	Python 3.7+
	•	OpenCV
	•	Numpy
	•	Scipy
	•	Serial (if connecting to a physical robot)

 How to Run

	1.	Prepare an image for the robot to draw (e.g., a PNG file).
	2.	Set the image path in main.py.
	3.	Run the script
 The system will process the image, compute the path, and send commands to the robot for drawing.

Key Components

	•	Binary Image Mask: Converts the image into a binary format where only the relevant pixels (lines and shapes) are retained.
	•	Path Detection: Identifies the shortest path that the robot needs to follow in order to draw the entire image.
	•	Kinematic Control: Ensures that the robot’s motion follows the computed path with minimal deviation.

Project Details

	•	Downscaling: To reduce the processing load, the image is resized to a manageable size while preserving the important details.
	•	Skeletonization: The image is reduced to its basic structure, creating a skeleton that the robot can follow more easily.
	•	Real-Time Drawing: The robot follows the computed path in real-time, adjusting its position as needed to accurately trace the image.

Future Improvements

	•	Enhance path optimization for more complex drawings.
	•	Introduce real-time obstacle avoidance to allow the robot to correct its course if necessary.
	•	Integrate feedback from sensors to improve the accuracy of the drawing.
