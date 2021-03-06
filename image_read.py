#!/usr/bin/env python3

import os
import cv2
import numpy as np

def parse_fname(filename, idx = 0):
    # get rid of the extension
    filename = '.'.join(filename.split('.')[:-1])
    filename = filename.split('_')
    return {"attitude_ra": float(filename[2]),
            "attitude_de": float(filename[4]),
            "attitude_roll": float(0.0),
            "target_ra": float(filename[7]),
            "target_de": float(filename[9]),
            "idx": idx}

def read_dir(dir):
    images = []
    dct_lst = []
    idx = 0

    # TODO: read frames on a moving window
    # read all image files and make a dictionary associated with each
    for file_name in os.listdir(dir):
        if file_name.split(".")[-1].lower() in {"jpeg", "jpg", "png"}:
            images.append(cv2.imread(os.path.join(dir, file_name),
                cv2.IMREAD_GRAYSCALE));
            tmp_dct = parse_fname(file_name, idx)
            dct_lst.append(tmp_dct)
            idx += 1
    return (images, dct_lst)

if __name__=='__main__':
    image_read('./')
