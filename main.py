#!/usr/bin/env python3

from collections import namedtuple
import numpy as np
from scipy.spatial import distance as dist
import centroid
import image_read as imr
from frame import Frame
import math
import cv2

Camera = namedtuple('Camera', ['height','width','vfov'])
cam = Camera(1944, 2592, math.radians(10.69))

def find_target(frames, window=2):
    # imgs: batch of frames
    target_found = False

    n_frames = len(frames)
    print("Frames: ", n_frames)
    # centroiding
    # v1 simple
    centroids = []
    centroids = list(map(centroid.find_centroids, frames))

    # star id => attitude
    # v1 assume known
    # TODO: apply transformations to vector angles and rotations

    # shift angles, crop
    # compare bearing angles
    # v1 assume no error
    candidates = []             # each element a list of candidates per frame

    # window for initialization
    # acc = []
    # empty_frame = Frame(np.zeros(imgs[0].shape, dtype=np.uint8))
    # spr = empty_frame
    acc = centroids[0]
    spr = frames[0]
    spr.plot_frame(acc, write=False)
    # centroid.plot_centroids(spr.px, acc, write=False)
    for idx in range(1, n_frames): # TODO: implement variable window

        acc = centroid.compare_centroids(acc, centroids[idx])
        acc = centroid.check_for_candidates(acc, frames[idx-1], frames[idx])
        frames[idx].plot_frame(acc, write=False)

        # spr = Frame.superimpose(spr, frames[idx])
        print("No of centroids: ", len(acc))
        # centroid.plot_centroids(spr.px, acc, write=False)
    # TODO: THINK about this better
    # This is only temporary
    # if there is one candidate, that's the target
    # if len(candidates[n_frames-2]) == 1:
        # candidates[n_frames-2][0].state = centroid.Centroid.S_TARGET
        # target = candidates[n_frames-2][0]
    # else:
        # target = None
    # food for thought:
    # for t in candidates[n_frames-2]:
        # target = None
        # if t.state == centroid.Centroid.S_TARGET:
            # target_found = True
            # target = t
            # print("found target")
    # centroid.plot_centroids(imgs[idx+1], centroids[idx+1])

    return acc, target_found

def label_candidates(centroids, n_frames):
    from sklearn.cluster import DBSCAN

    # TODO: put centroids in 3D or 4D (quaternions) to need no angle-wrapping
    # TODO: change centroid.Centroid equals to reflect that
    # TODO: allow roll rotations
    # TODO: test if point is in rectangle (maybe rotated) rather than just the
    # intervals
    clustering = DBSCAN(eps=0.5, min_samples=2, leaf_size=n_frames)

def track_target(tgt_hist, img):
    # at this point more like plotting than tracking
    print("tracking target")
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    for i in range(0, len(tgt_hist)-1):
        cv2.line(img, tgt_hist[i].get_pix(), tgt_hist[i+1].get_pix(), (0,i*50,255), 3)

    i = 0
    for c in tgt_hist:
        i += 1
        if c.state == centroid.Centroid.S_STAR:
            cv2.circle(img, (c.cX, c.cY), 10, (0, 0, 200), 1)
            cv2.putText(img, str(i), (c.cX - 25, c.cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 200), 2)
        elif c.state == centroid.Centroid.S_UNKNOWN:
            cv2.circle(img, (c.cX, c.cY), 10, (250, 250, 250), 1)
            cv2.putText(img, str(i), (c.cX - 25, c.cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (250, 250, 250), 2)
        elif c.state == centroid.Centroid.S_CANDIDATE:
            cv2.circle(img, (c.cX, c.cY), 10, (10, 200, 200), 1)
            cv2.putText(img, str(i), (c.cX - 25, c.cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 200, 200), 2)
        elif c.state == centroid.Centroid.S_TARGET:
            cv2.circle(img, (c.cX, c.cY), 10, (10, 255, 50), 1)
            cv2.putText(img, str(i), (c.cX - 25, c.cY - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 255, 50), 2)

    # display the image
    cv2.imshow("Image", img)
    cv2.waitKey(0)
    cv2.imwrite("line.png", img)

def main():

    target_found = False

    # read frames -- determine frame window
    # v1 no noise
    img_dir = './test2'
    imgs, dct_lst = imr.read_dir(img_dir)
    # for now sort imgs based on dictionary


    tgt_history = []
    # while not target_found:
    # for i in range(0, len(imgs)-2):
        # target, target_found = find_target(imgs[i:i+3])
        # tgt_history.append(target)
    frames = list(map(lambda x, y: Frame(x, y), imgs, dct_lst))
    acc, _  = find_target(frames)

    print("stars: ", len(centroid.find_stars(acc)),
            ", candidates: ", len(centroid.find_candidates(acc)))

    for cand in centroid.find_candidates(acc):

        attitude = (dct_lst[cand.frame_id]["attitude_ra"],
                    dct_lst[cand.frame_id]["attitude_de"])

        print("frame: ", cand.frame_id,
                "\tsra: ", attitude[0],
                "\t\tsde: ", attitude[1],
                "\n\t\ttra: ", math.degrees(cand.cRA),
                "\ttde: ", math.degrees(cand.cDE),
                "\n\t\ttra':", (dct_lst[cand.frame_id]["target_ra"]),
                "\t\t\ttde':", (dct_lst[cand.frame_id]["target_de"]))
    # track_target(tgt_history, imgs[i])


if __name__=='__main__':
    main()


