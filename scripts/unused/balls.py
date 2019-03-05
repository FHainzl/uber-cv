import cv2
import numpy as np


def find_balls(img):
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT,
                               dp=1, minDist=10,
                               param1=50, param2=30,
                               minRadius=30, maxRadius=60)

    return circles


def draw_circles_on_img(circles, img):
    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")

        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            cv2.circle(img, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255),
                          -1)

        # show the output image
        cv2.imshow("Circles", img)
        cv2.waitKey(0)


def main():
    img = cv2.imread("test.jpg")
    blurred = cv2.GaussianBlur(img, (11, 11), 0)
    grey = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    circles = find_balls(grey)
    print circles.shape
    draw_circles_on_img(circles, img)


if __name__ == '__main__':
    main()
