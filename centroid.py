#!/usr/bin/env python3
import numpy as np
import quaternion
from scipy.spatial import distance
from datetime import datetime
from frame import Frame
import utils
from interval import interval, inf, imath
import math
import cv2

class Camera:
    def __init__(self, _height=1944, _width=2592, _vfov=math.radians(10.69)):
        self.height = _height
        self.width = _width
        self.vfov = _vfov
        self.set_ifov()

    def set_ifov(self):
        self.ifov = float(self.vfov)/float(self.height)

# Camera = namedtuple('Camera', ['height','width','vfov','ifov'])
cam = Camera(1944, 2592, math.radians(10.69))
# cam.ifov = get_ifov(cam.vfov, cam.height)

class Centroid:
    # static variables for describing centroid state
    # TODO: add confidence (weakly/strongly target)
    S_TARGET = 2
    S_CANDIDATE = 1
    S_STAR = -1
    S_UNKNOWN = 0

    ERROR = 0.000001

    def __init__(self, cx, cy, att=(0,0,0), state=S_UNKNOWN, frame_id=0):
    # def __init__(self, cx, cy, quat, att=(0,0,0), state=S_UNKNOWN, frame_id=0):
        # pixel data (should really choose a better name)
        self.cX = cx
        self.cY = cy
        # TODO: make it less naive and solve angle wrap
        # self.cRA, self.cDE
        # relative to the center of the frame
        # assumes same angular scale (only works for small fov)
        self.cRA = -(cx-cam.width/2)*cam.ifov + att[0]
        self.cDE = -(cy-cam.height/2)*cam.ifov + att[1]
        # position on the unit sphere relative
        self.pos = utils.matrix_rotation(att,
                utils.pix2cart_unit(cx, cy, cam.ifov, cam.width, cam.height))
        # conj = quat.conjugate()
        # self.pos = quat * \
            # utils.pix2cart_unit(cx, cy, cam.ifov, cam.width, cam.height) * \
            # conj
        self.state = state
        self.frame_id = frame_id

    def equals(self, other):
        # return abs(self.cRA-other.cRA) <= self.ERROR and \
            # abs(self.cDE-other.cDE) <= self.ERROR
        return distance.euclidean(self.pos, other.pos) <= self.ERROR

    # TODO: angle-wrapping and ROLL polygon
    def is_in_interval(self, ra_int, de_int):
        return (self.cRA in ra_int) and (self.cDE in de_int)

    def is_in_frame(self, frame):
        return self.is_in_interval(frame.ra_int, frame.de_int)

    def get_pix(self):
        return (self.cX, self.cY)

def find_centroids(img_frame, camera=cam):
    # image is a numpy array
    # camera is a named tuple (see above)
    # find contours in the binary image
    noise_level = 7
    minSNR = 3
    ret, thresh = cv2.threshold(img_frame.px, noise_level*minSNR, 255, 0)

    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,
                    cv2.CHAIN_APPROX_SIMPLE)
    centers = []
    for c in contours:
        # calculate moments for each contour
        M = cv2.moments(c)

        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        centers.append(Centroid(cX, cY,
                        # img_frame.attitude_quaternion,
                        att=(img_frame.attitude_ra,
                            img_frame.attitude_de,
                            img_frame.attitude_roll),
                        frame_id=img_frame.id))
        # centers.append(Centroid(cX, cY,
                        # att=img_frame.attitude_quaternion,
                        # frame_id=img_frame.id))

    return centers

def compare_centroids(frame1, frame2):
    # Union of frames (TODO: Implement kd tree maybe)
    union = list(frame1)
    for new in frame2:
        found = False
        # c2.state += 1 # assume no match
        for old in union:
            if old.equals(new):
                if old.state > Centroid.S_STAR: # the one we added before
                    old.state -= 1 # drop 1 state overall
                found = True
                break
                    # c1.state = Centroid.S_STAR # equivalent of dropping by 1
        if found == False:
            # add candidate to union
            # Shouldn't go higher than this state
            # if c2.state > Centroid.S_TARGET:
                # c2.state = Centroid.S_TARGET
            union.append(new)

        # Return if candidate
        # if c2.state >= Centroid.S_CANDIDATE:
            # candidates.append(c2)
        # Or return all
    return union

def check_for_candidates(unknowns, frame1, frame2):
    # check if found potential candidates are in intersection of frames
    # TODO: make this check frame properly it only works for 0 roll now
    frame_width = cam.width*cam.ifov
    frame_height = cam.width*cam.ifov

    ra_int_1 = interval[frame1.attitude_ra-frame_width/2,
                frame1.attitude_ra+frame_width/2]
    de_int_1 = interval[frame1.attitude_de-frame_height/2,
                frame1.attitude_de+frame_height/2]
    ra_int_2 = interval[frame2.attitude_ra-frame_width/2,
                frame2.attitude_ra+frame_width/2]
    de_int_2 = interval[frame2.attitude_de-frame_height/2,
                frame2.attitude_de+frame_height/2]

    ra_intrsc = ra_int_1&ra_int_2
    de_intrsc = de_int_1&de_int_2

    for c in unknowns:
        if c.state < Centroid.S_CANDIDATE \
            and c.state >= Centroid.S_UNKNOWN \
            and c.is_in_interval(ra_intrsc, de_intrsc):
            c.state += 1

    return unknowns


def find_candidates(centroids):
    candidates = []
    for c in centroids:
        if c.state == Centroid.S_CANDIDATE:
            candidates.append(c)
    return candidates

def find_stars(centroids):
    stars = []
    for c in centroids:
        if c.state == Centroid.S_STAR:
            stars.append(c)
    return stars

def find_targets(centroids):
    targets = []
    for c in centroids:
        if c.state == Centroid.S_TARGET:
            targets.append(c)
    return targets

def find_unknowns(centroids):
    unknowns = []
    for c in centroids:
        if c.state == Centroid.S_UNKNOWN:
            unknowns.append(c)
    return unknowns

def plot_centroids(input_img, centroids, write=False, frame_no=None,
        frame_att=None):

    img = cv2.cvtColor(input_img, cv2.COLOR_GRAY2BGR)

    for c in centroids:
        if c.state == Centroid.S_STAR:
            cv2.circle(img, (c.cX, c.cY), 10, (0, 0, 200), 1)
            cv2.putText(img, "star", (c.cX + 25, c.cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 200), 2)
        elif c.state == Centroid.S_UNKNOWN:
            cv2.circle(img, (c.cX, c.cY), 10, (250, 250, 250), 1)
            cv2.putText(img, "unknown", (c.cX + 25, c.cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (250, 250, 250), 2)
        elif c.state == Centroid.S_CANDIDATE:
            cv2.circle(img, (c.cX, c.cY), 10, (10, 200, 200), 1)
            cv2.putText(img, "candidate", (c.cX + 25, c.cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 200, 200), 2)
        elif c.state == Centroid.S_TARGET:
            cv2.circle(img, (c.cX, c.cY), 10, (10, 255, 50), 1)
            cv2.putText(img, "target", (c.cX + 25, c.cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 255, 50), 2)

    if frame_no != None:
        cv2.putText(img, "Frame #"+str(frame_no),(25, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    if frame_att != None:
        frame_att = (math.degrees(frame_att[0]), math.degrees(frame_att[1]))
        cv2.putText(img, f"Satellite RA (deg): {frame_att[0]:.2f}", (25, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, f"Satellite DE (deg): {frame_att[1]:.2f}", (25, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    # display the image
    cv2.imshow("Image", img)
    cv2.waitKey(0)
    if write:
        cv2.imwrite(str(datetime.now())+".png", img)

if __name__=='__main__':
    import image_read
    # imgs, dct = image_read.read_dir('./test')
    # centroids = list(map(find_centroids,imgs))
    # map(lambda img, cntr: plot_centoids(img, cntr), imgs, centroids)
    # plot_centroids(imgs[0], centroids[0])
    print(cam.ifov)
