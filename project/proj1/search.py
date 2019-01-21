#python3 search.py --path=car.jpg
import cv2 as cv
import argparse
import sys
import numpy as np
import os.path
import glob
from utils import salience_score, draw_pred, get_output_names, deserialize_obj
import math

# Initialize the parameters
confThreshold = 0.5  # Confidence threshold
nmsThreshold = 0.4   # Non-maximum suppression threshold
inpWidth = 416       # Width of network's input image
inpHeight = 416      # Height of network's input image
classesFile = "coco.names"
with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')
# Give the configuration and weight files for the model and load the network using them.
modelConfiguration = "yolov3.cfg"
modelWeights = "yolov3.weights"


def search_index(img_info, imgs_info, index):

    # union of all matches
    result = []
    for obj, _ in img_info.items():
        if obj in index:
            result += [img for img in index[obj]]

    print("img_info")
    for img in result:
        print((img, dist(img_info, imgs_info[img])))

    return sorted([(img, dist(img_info, imgs_info[img])) for img in result], key=lambda res_tup: res_tup[1])


def dist(img_info_query, img_info_doc):
    cost = 0
    for obj in set(img_info_query.keys()) | set(img_info_doc.keys()):
        y, y_hat, y_imp, y_hat_imp = 0, 0, 0, 0
        if obj in img_info_query:
            y, y_imp = img_info_query.get(obj)
        if obj in img_info_doc:
            y_hat, y_hat_imp = img_info_doc.get(obj)
        print(j(y, y_imp, y_hat, y_imp))
        print()
        cost += j(y, y_imp, y_hat, y_imp)
    return cost


# y is original vector
def j(y, y_salience, y_hat, y_hat_salience):
    print(y, y_salience, y_hat, y_hat_salience)
    if y == 0:
        if y_hat == 0:
            return 0
        else:
            return math.log(1 + y_hat)*math.e**y_salience
    else:
        if y_hat == 0:
            return (1 + math.log10(y)) * math.e ** y_salience
        else:
            return abs(math.log10(y) - math.log10(y_hat))*abs(1 + math.e**y_salience - math.e**y_hat_salience)


# Remove the bounding boxes with low confidence using non-maxima suppression
def postprocess(image, outs, debug=False):
    # Scan through all the bounding boxes output from the network and keep only the
    # ones with high confidence scores. Assign the box's class label as the class with the highest score.
    image_height = image.shape[0]
    image_width = image.shape[1]

    class_ids = []
    confidences = []
    boxes = []
    img_info = {}

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > confThreshold:
                center_x = int(detection[0] * image_width)
                center_y = int(detection[1] * image_height)
                width = int(detection[2] * image_width)
                height = int(detection[3] * image_height)
                left = int(center_x - width / 2)
                top = int(center_y - height / 2)
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([left, top, width, height])

    saliency = cv.saliency.StaticSaliencySpectralResidual_create()
    _, saliency_map = saliency.computeSaliency(image)

    indices = cv.dnn.NMSBoxes(boxes, confidences, confThreshold, nmsThreshold)

    for i in indices:
        i = i[0]
        class_id = class_ids[i]
        box = boxes[i]

        # (count, salience_score)
        class_tup = img_info.get(class_id, (0, 0))

        img_info[class_id] = (class_tup[0] + 1, class_tup[1] + salience_score(saliency_map, box))

        if debug:
            left = box[0]
            top = box[1]
            width = box[2]
            height = box[3]
            draw_pred(image, classes, class_ids[i], confidences[i], left, top, left + width, top + height)

    if debug:
        cv.imshow("img", image)
        cv.imshow("saliency_map", saliency_map)
        cv.waitKey(0)
        print(img_info)

    return img_info


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Object Detection using YOLO in OPENCV')
    parser.add_argument('--image', help='Path to images file.')
    parser.add_argument('--path', help='Path to videos file.')
    args = parser.parse_args()

    file = args.path
    image = cv.imread(file)

    index = deserialize_obj("index")
    imgs_info = deserialize_obj("imgs_info")

    net = cv.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
    net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)

    # Create a 4D blob from a frame.
    blob = cv.dnn.blobFromImage(image, 1/255, (inpWidth, inpHeight), [0, 0, 0], 1, crop=False)
    # Sets the input to the network
    net.setInput(blob)
    # Runs the forward pass to get output of the output layers
    outs = net.forward(get_output_names(net))
    # Remove the bounding boxes with low confidence
    img_info = postprocess(image, outs)

    print(search_index(img_info, imgs_info, index))
