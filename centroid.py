#!/usr/bin/env python3
import numpy as np
import cv2

class Centroid:
    # static variables for describing centroid state
    # TODO: add confidence (weakly/strongly target)
    S_TARGET = 2
    S_CANDIDATE = 1
    S_STAR = -1
    S_UNKNOWN = 0

    ERROR = 0.1

    def __init__(self, cx, cy, state=S_UNKNOWN):
        self.cX = cx
        self.cY = cy
        # TODO: change the pixel data to angles based on camera intrinsics
        # self.cRA, self.cDE
        self.state = state

    def equals(self, other):
        # TODO: make it about angles
        return abs(self.cX-other.cX) <= self.ERROR and \
                abs(self.cY-other.cY) <= self.ERROR


def find_centroids(img):
    # image is a numpy array
    # find contours in the binary image
    noise_level = 7
    minSNR = 3
    ret, thresh = cv2.threshold(img, noise_level*minSNR, 255, 0)

    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,
                    cv2.CHAIN_APPROX_SIMPLE)
    centers = []
    for c in contours:
        # calculate moments for each contour
        M = cv2.moments(c)

        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        centers.append(Centroid(cX, cY))

    return centers

def compare_centroids(frame1, frame2):
    candidates = []
    for c2 in frame2:
        c2.state += 1 # assume no equality
        for c1 in frame1:
            if c1.equals(c2):
                if c2.state > Centroid.S_STAR + 1: # the one we added before
                    c2.state -= 2 # drop 1 state overall
                else:
                    c2.state = Centroid.S_STAR # equivalent of dropping by 1

        if c2.state >= Centroid.S_CANDIDATE:
            candidates.append(c2)
    return candidates


def plot_centroids(img, centroids):

    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    for c in centroids:
        if c.state == Centroid.S_STAR:
            cv2.circle(img, (c.cX, c.cY), 10, (0, 0, 200), 1)
            cv2.putText(img, "star", (c.cX - 25, c.cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 200), 2)
        elif c.state == Centroid.S_UNKNOWN:
            cv2.circle(img, (c.cX, c.cY), 10, (250, 250, 250), 1)
            cv2.putText(img, "unknown", (c.cX - 25, c.cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (250, 250, 250), 2)
        elif c.state == Centroid.S_CANDIDATE:
            cv2.circle(img, (c.cX, c.cY), 10, (10, 200, 200), 1)
            cv2.putText(img, "candidate", (c.cX - 25, c.cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 200, 200), 2)
        elif c.state == Centroid.S_TARGET:
            cv2.circle(img, (c.cX, c.cY), 10, (10, 255, 50), 1)
            cv2.putText(img, "target", (c.cX - 25, c.cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 255, 50), 2)

    # display the image
    cv2.imshow("Image", img)
    cv2.waitKey(0)

if __name__=='__main__':
    import image_read
    imgs, dct = image_read.read_dir('./test')
    centroids = list(map(find_centroids,imgs))
    # map(lambda img, cntr: plot_centoids(img, cntr), imgs, centroids)
    plot_centroids(imgs[0], centroids[0])
