import cv2
import numpy as np

#Python Imports
import multiprocessing

cores_available = multiprocessing.cpu_count()

def image_capture_background(imgcap_connection):
	global cap, latest_image

def startCamera():
	global cap, parent_conn, imgcap_conn, is_backgroundCap
	is_backgroundCap=False
	cap = cv2.VideoCapture(2)

	if not cap.isOpened():
		print("Cannot open camera")
		exit(0)

	if(cores_available > 3):
		print("BG process")
	print("Camera is opened")

def get_frame():
    global cap, is_backgroundCap, parent_conn, img_counter
    img_counter =1
    if(is_backgroundCap):
        if(parent_conn == None):
            return None
        parent_conn.send(img_counter)
        img_counter = img_counter + 1
        img = parent_conn.recv()


def cap_end():
	global cap
	print("Releasing camera")
	cap.release()

if __name__ == "__main__":
	startCamera()
	i=1
	while True:
		img = get_frame()
		print("Got image " +str(i))
		if i==200:
			break
		i+=1
	cap_end()