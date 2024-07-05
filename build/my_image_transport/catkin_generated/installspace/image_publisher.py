#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import math
import pyzbar.pyzbar as pyzbar

# Variable
camID = '/dev/video0'    # camera ID, or pass string as filename to the camID

# Real world measured Distance and width of QR code (in cm)
KNOWN_DISTANCE = 76.5  # cm (30.1 inches * 2.54)
KNOWN_WIDTH = 12.7  # cm (5.0 inches * 2.54)

# Define the fonts
fonts = cv.FONT_HERSHEY_COMPLEX
Pos = (50, 50)
# Colors (BGR)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
MAGENTA = (255, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
RED = (0, 0, 255)
CYAN = (255, 255, 0)
GOLD = (0, 255, 215)
YELLOW = (0, 255, 255)
ORANGE = (0, 165, 230)

def showText(image, text, position, color, animateOnValue=None):
    fonts = cv.FONT_HERSHEY_COMPLEX
    x, y = position
    y = y - 20
    lineThicknes = 30
    offset = int(lineThicknes / 2)
    center = int(offset / 2)
    new_pos = (x, y + center)
    lineLength = 229
    newThickness = int(lineThicknes * 0.7)

    cv.line(image, (x, y), (x + lineLength, y), ORANGE, lineThicknes)
    cv.line(image, (x, y), (x + lineLength, y), GREEN, newThickness)
    if animateOnValue is not None:
        cv.line(image, (x, y), (x + animateOnValue, y), WHITE, newThickness)
    cv.putText(image, text, new_pos, fonts, 0.6, BLACK, 2)

def euclideanDistance(x, y, x1, y1):
    return math.sqrt((x1 - x) ** 2 + (y1 - y) ** 2)

def focalLengthFinder(knownDistance, knownWidth, widthInImage):
    return (widthInImage * knownDistance) / knownWidth

def distanceFinder(focalLength, knownWidth, widthInImage):
    return (knownWidth * focalLength) / widthInImage

def detectQRcode(image):
    if image is None:
        return None, None, None
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    qr_codes = pyzbar.decode(gray)
    for qr_code in qr_codes:
        points = qr_code.polygon
        if len(points) > 4:
            hull = cv.convexHull(np.array([point for point in points], dtype=np.float32))
            hull = list(map(tuple, np.squeeze(hull)))
        else:
            hull = points

        n = len(hull)
        for j in range(n):
            cv.line(image, hull[j], hull[(j + 1) % n], ORANGE, 2)

        pts = np.array(hull, dtype="float32")

        # Determine the width and height of the QR code
        (tl, tr, br, bl) = pts
        widthA = euclideanDistance(tl[0], tl[1], tr[0], tr[1])
        widthB = euclideanDistance(bl[0], bl[1], br[0], br[1])
        heightA = euclideanDistance(tl[0], tl[1], bl[0], bl[1])
        heightB = euclideanDistance(tr[0], tr[1], br[0], br[1])

        maxWidth = max(int(widthA), int(widthB))
        maxHeight = max(int(heightA), int(heightB))

        # Compute the perspective transform matrix and then apply it
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype="float32")

        M = cv.getPerspectiveTransform(pts, dst)
        warped = cv.warpPerspective(gray, M, (maxWidth, maxHeight))

        qr_size = max(maxWidth, maxHeight)
        qr_type = qr_code.data.decode("utf-8")
        return qr_size, hull[3], qr_type
    return None, None, None

def main():
    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher('camera/image', Image, queue_size=10)
    bridge = CvBridge()

    camera = cv.VideoCapture(camID)
    if not camera.isOpened():
        rospy.logerr("Error: Could not open camera.")
        return

    referenceImage = cv.imread("TYPE1.png")
    Rwidth, _, _ = detectQRcode(referenceImage)

    if Rwidth:
        focalLength = focalLengthFinder(KNOWN_DISTANCE, KNOWN_WIDTH, Rwidth)
        rospy.loginfo(f"Focal length: {focalLength}")
    else:
        rospy.logwarn("Could not detect QR code in reference image.")
        focalLength = 1

    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and camera.isOpened():
        ret, frame = camera.read()
        frame = cv.resize(frame, (720, 480))
        if not ret:
            rospy.logwarn("Failed to grab frame.")
            break

        codeSize, Pos, qr_type = detectQRcode(frame)
        if codeSize:
            Distance = distanceFinder(focalLength, KNOWN_WIDTH, codeSize) * 142.8
            if qr_type == "TYPE1":
                showText(frame, f"Distance: {round(Distance, 2)} cm (TYPE1)", Pos, GREEN, int(Distance * 4.5))
            elif qr_type == "TYPE2":
                showText(frame, f"Distance: {round(Distance, 2)} cm (TYPE2)", Pos, BLUE, int(Distance * 4.5))
            elif qr_type == "TYPE3":
                showText(frame, f"Distance: {round(Distance, 2)} cm (TYPE3)", Pos, RED, int(Distance * 4.5))
            else:
                showText(frame, f"Distance: {round(Distance, 2)} cm (UNKNOWN)", Pos, YELLOW, int(Distance * 4.5))

        pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        cv.imshow("frame", frame)
        # key = cv.waitKey(1)
        # if key == ord('q'):
        #     break

        rate.sleep()

    camera.release()
    cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
