#!/usr/bin/env python3
import image_read as imr
import numpy as np
import cv2
import math

class Frame:
    def __init__(self, arr, dct): # TODO: add optical characteristics and target data
        self.px = arr
        self.height, self.width = arr.shape
        self.attitude_ra = math.radians(dct["attitude_ra"])
        self.attitude_de = math.radians(dct["attitude_de"])
        self.target_ra = math.radians(dct["target_ra"])
        self.target_de = math.radians(dct["target_de"])
        self.id = dct["idx"]
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
