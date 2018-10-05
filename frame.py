#!/usr/bin/env python3
import image_read as imr
import centroid
import utils
from interval import interval
import numpy as np
import cv2
import math

class Frame:
    def __init__(self, arr, dct): # TODO: add optical characteristics and target data
        self.px = arr
        self.height, self.width = arr.shape
        self.attitude_ra = math.radians(dct["attitude_ra"])
        self.attitude_de = math.radians(dct["attitude_de"])
        self.attitude_roll = math.radians(dct["attitude_roll"])
        self.attitude_quaternion = utils.euler2quaternion(
                np.array([
                    self.attitude_ra,
                    self.attitude_de,
                    self.attitude_roll
                ])
                )
        self.target_ra = math.radians(dct["target_ra"])
        self.target_de = math.radians(dct["target_de"])
        self.id = dct["idx"]
        # self.centroids = []
        # self.centroids = centroid.find_centroids(self)
        self.ra_int = interval[self.attitude_ra - centroid.cam.ifov*self.height/2,
                            self.attitude_ra + centroid.cam.ifov*self.height/2]
        self.de_int = interval[self.attitude_de - centroid.cam.ifov*self.width/2,
                            self.attitude_de + centroid.cam.ifov*self.width/2]

    # because of above lines it only works for 0 roll now
    def plot_frame(self, centroids, write=False):
        centroids_to_plot = [c for c in centroids if c.is_in_frame(self)]
        centroid.plot_centroids(self.px, centroids_to_plot, frame_no=self.id,
                frame_att=(self.attitude_ra, self.attitude_de), write=write)

    @staticmethod
    def superimpose(frame1, frame2, att_change=0):
        if att_change==0:
            # be careful cause it will broadcast less-dimensional arrays
            pixels = np.maximum(frame1.px,frame2.px)
            frame1.px = pixels # (TODO: change this to account for attitude)
            return frame1
        else:
            pixels = np.empty(frame1.px.shape) # placeholder for now (TODO)
        return pixels


def main():

    img_dir = './test'
    imgs, dct_lst = imr.read_dir(img_dir)

    print(imgs[0].shape)
    final = Frame.superimpose(Frame(imgs[0]), Frame(imgs[1]))
    cv2.imshow("image", final)
    cv2.waitKey(0)

if __name__=='__main__':
    main()
