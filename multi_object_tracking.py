# USAGE
# python multi_object_tracking.py --video videos/soccer_01.mp4 --tracker csrt

# import the necessary packages
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import socketio
import csv
import os
import math

calibration_points = []

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", type=str,
	help="path to input video file")
ap.add_argument("-t", "--tracker", type=str, default="csrt",
	help="OpenCV object tracker type")
ap.add_argument("-d", "--distance", type=int, default=0,
	help="OpenCV object distance shadow")
args = vars(ap.parse_args())
sio = socketio.Client()
@sio.on('connect')
def on_connect():
	print('Connected')
def contains(topLeft,topRight,bottomRight,bottomLeft,p):
    return p["x"] >= topLeft["x"] and p["x"] <= topRight["x"] and p["y"] >= topLeft["y"] and p["y"] <= bottomRight["y"] 
sio.connect('https://dronie.vincentriva.fr')

# initialize a dictionary that maps strings to their corresponding
# OpenCV object tracker implementations
OPENCV_OBJECT_TRACKERS = {
	"csrt": cv2.TrackerCSRT_create,
	"kcf": cv2.TrackerKCF_create,
	"boosting": cv2.TrackerBoosting_create,
	"mil": cv2.TrackerMIL_create,
	"tld": cv2.TrackerTLD_create,
	"medianflow": cv2.TrackerMedianFlow_create,
	"mosse": cv2.TrackerMOSSE_create
}

# initialize OpenCV's special multi-object tracker
trackers = cv2.MultiTracker_create()
middleBottom = None

if os.path.isfile('calibration.csv'):
	with open('calibration.csv') as f:
		reader = csv.reader(f)
		for row in reader:
			calibration_points.append({'x': int(row[0]), 'y': int(row[1])})
			sio.emit('DRONE:CALIBRATION', {'x': int(row[0]), 'y': int(row[1])})
		if args["distance"] > 0:
			middleBottom = {
				'x': (calibration_points[2]["x"] + calibration_points[3]["x"]) / 2,
				'y': (calibration_points[2]["y"] + calibration_points[3]["y"]) / 2,
			}
			print("Middle: ")
			print(middleBottom['x'])
			print(middleBottom['y'])

# if a video path was not supplied, grab the reference to the web cam
if not args.get("video", False):
	print("[INFO] starting video stream...")
	vs = VideoStream(src=0).start()
	time.sleep(1.0)

# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])

# loop over frames from the video stream
while True:
	# grab the current frame, then handle if we are using a
	# VideoStream or VideoCapture object
	frame = vs.read()
	frame = frame[1] if args.get("video", False) else frame

	# check to see if we have reached the end of the stream
	if frame is None:
		break

	# resize the frame (so we can process it faster)
	frame = imutils.resize(frame, width=1200)	

	# grab the updated bounding box coordinates (if any) for each
	# object that is being tracked
	(success, boxes) = trackers.update(frame)

	# loop over the bounding boxes and draw then on the frame
	for box in boxes:
		(x, y, w, h) = [int(v) for v in box]
		cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
		if middleBottom is not None:
			angle = math.atan2(middleBottom["y"] - y, middleBottom["x"] - x)
			sio.emit('DRONE:DETECT', {'x': x - math.cos(angle) * args["distance"], 'y': y})
		else:
			sio.emit('DRONE:DETECT', {'x': x, 'y': y})

		# if(len(calibration_points) == 4 and contains(calibration_points[0],calibration_points[1],calibration_points[2],calibration_points[3],{'x': x, 'y': y}) is False) :
			# sio.emit('DRONE:STOP')

	key = cv2.waitKey(1) & 0xFF

	if key == ord("a"):
		print('Sending event')
		sio.emit('DRONE:CALIBRATION', {'x': x, 'y': y})
		calibration_points.append({'x': x, 'y': y})
		if len(calibration_points) >= 4:
			if args["distance"] > 0:
				middleBottom = {
					'x': (calibration_points[2]["x"] + calibration_points[3]["x"]) / 2,
					'y': (calibration_points[2]["y"] + calibration_points[3]["y"]) / 2,
				}
				print("Middle: ")
				print(middleBottom['x'])
				print(middleBottom['y'])
			with open('calibration.csv', 'w') as f:
				writer = csv.writer(f)
				for point in calibration_points:
					writer.writerow([point["x"], point["y"]])

	for point in calibration_points:
		cv2.rectangle(frame, (point["x"], point["y"]), (point["x"] + 5, point["y"] + 5), (0, 0, 255), cv2.FILLED)

	# show the output frame
	cv2.imshow("Frame", frame)


	# if the 's' key is selected, we are going to "select" a bounding
	# box to track
	if key == ord("s"):
		# select the bounding box of the object we want to track (make
		# sure you press ENTER or SPACE after selecting the ROI)
		box = cv2.selectROI("Frame", frame, fromCenter=False,
			showCrosshair=True)

		# create a new object tracker for the bounding box and add it
		# to our multi-object tracker
		tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
		trackers.add(tracker, frame, box)

	# if the `q` key was pressed, break from the loop
	elif key == ord("q"):
		break

# if we are using a webcam, release the pointer
if not args.get("video", False):
	vs.stop()

# otherwise, release the file pointer
else:
	vs.release()

# close all windows
cv2.destroyAllWindows()