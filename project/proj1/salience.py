# import the necessary packages
import argparse
import cv2

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
                help="path to input image")

args = vars(ap.parse_args())

# load the input image
image = cv2.imread(args["image"])
print(image)
# initialize OpenCV's static fine grained saliency detector and
# compute the saliency map

# FINE GRAINED not used, seems to perform worse
# saliency = cv2.saliency.StaticSaliencyFineGrained_create()
# (success, saliencyMap) = saliency.computeSaliency(image)

saliency = cv2.saliency.StaticSaliencySpectralResidual_create()
(success, saliency_map) = saliency.computeSaliency(image)
saliencyMap = (saliency_map * 255).astype("uint8")
left = 0
top = 0
width = 1000
height = 1000
roi = saliency_map[left:left+width, top:top+height]

# thresholding was deemed a bad idea (by me), might be good if time beacomes a problem3
# threshMap = cv2.threshold(saliency_map.astype("uint8"), 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

for img_name in ["image", "output", "thresh"]:
    cv2.namedWindow(img_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(img_name, 600, 600)


# show the images
cv2.imshow("image", roi)
cv2.imshow("output", saliencyMap.astype("uint8"))
cv2.waitKey(0)