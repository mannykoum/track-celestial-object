#!/usr/bin/env python3

import numpy as np
from scipy.spatial import distance as dist
import centroid
import image_read as imr

def find_target():

    target_found = False

    # read frames -- determine frame window
    # v1 no noise
    img_dir = './test'
    imgs, dct_lst = imr.read_dir(img_dir)
    n_frames = len(imgs)
    print(n_frames)
    # centroiding
    # v1 simple
    centroids = []
    centroids = list(map(centroid.find_centroids, imgs))

    # star id => attitude
    # v1 assume known
    # TODO: apply transformations to vector angles and rotations

    # shift angles, crop
    # compare bearing angles
    # v1 assume no error
    candidates = []             # each element a list of candidates per frame
    # window for initialization
    for idx in range(0, n_frames-1):
        candidates.append(centroid.compare_centroids(centroids[idx],
                            centroids[n_frames-1]))
        # print(candidates[idx])
        # centroid.plot_centroids(imgs[n_frames-1], centroids[n_frames-1])

    for t in candidates[n_frames-2]:
        target = None
        if t.state == centroid.Centroid.S_TARGET:
            target_found = True
            target = t
            print("found target")
    return target, target_found

def track_target():
    print("tracking target")

def main():

    target_found = False
    # while not target_found:
    target, target_found = find_target()

    track_target()


if __name__=='__main__':
    main()


