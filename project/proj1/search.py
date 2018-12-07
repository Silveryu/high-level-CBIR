#python3 search.py --path=car.jpg

import cv2 as cv
import argparse
import sys
import numpy as np
import os.path
import glob
import pickle


# Initialize the parameters
confThreshold = 0.5  #Confidence threshold
nmsThreshold = 0.4   #Non-maximum suppression threshold
inpWidth = 416       #Width of network's input image
inpHeight = 416      #Height of network's input image




parser = argparse.ArgumentParser(description='Object Detection using YOLO in OPENCV')
parser.add_argument('--image', help='Path to images file.')
parser.add_argument('--video', help='Path to videos file.')
parser.add_argument('--path', help='Path to videos file.')
args = parser.parse_args()


# Load names of classes
classesFile = "coco.names";
classes = None
with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')

print(classes)
# Give the configuration and weight files for the model and load the network using them.
modelConfiguration = "yolov3.cfg";
modelWeights = "yolov3.weights";

net = cv.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)


# Get the names of the output layers
def getOutputsNames(net):
    # Get the names of all the layers in the network
    layersNames = net.getLayerNames()
    # Get the names of the output layers, i.e. the layers with unconnected outputs
    return [layersNames[i[0] - 1] for i in net.getUnconnectedOutLayers()]



def deserializeObj(name):
     with open('obj/' + name + '.pkl', 'rb') as f:
        return pickle.load(f)
index = deserializeObj("index")

def searchIndex(countsClass):
    result = []
    for key,val in countsClass.items():
        if key in index:
            result.append((index[key],val))
    print(result)

# Remove the bounding boxes with low confidence using non-maxima suppression
def postprocess(file, outs):
    # Scan through all the bounding boxes output from the network and keep only the
    # ones with high confidence scores. Assign the box's class label as the class with the highest score.
    classIds = []
    countsClass = {}
    confidences = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > confThreshold:
                classIds.append(classId)
                countsClass[classId] = countsClass.get(classId, 0) + 1
                #confidences.append(float(confidence))
                #boxes.append([left, top, width, height])
    #index File
    print(countsClass)
    searchIndex(countsClass)


file=args.path
cap = cv.VideoCapture(file)
while True:
    # get frame from the video
    hasFrame, frame = cap.read()
    # Stop the program if reached end of video
    if not hasFrame:
        # Release device
        cap.release()
        break
    # Create a 4D blob from a frame.
    blob = cv.dnn.blobFromImage(frame, 1/255, (inpWidth, inpHeight), [0,0,0], 1, crop=False)

    # Sets the input to the network
    net.setInput(blob)

    # Runs the forward pass to get output of the output layers
    outs = net.forward(getOutputsNames(net))

    # Remove the bounding boxes with low confidence
    postprocess(file, outs)
