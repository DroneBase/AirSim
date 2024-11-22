#!/usr/bin/env python3

from rosbags.highlevel import AnyReader
from rosbags.image import message_to_cvimage
from pathlib import Path
import cv2
import os
import pyexiv2
import sys

if len(sys.argv) < 2 or not sys.argv[1]:
    print('You must provide a bag name.')
    sys.exit(1)

BAG_NAME=sys.argv[1]
BAG_PATH=os.path.join('/', 'bags', BAG_NAME)

IMAGES_DIR=os.path.join('/', 'images', BAG_NAME)

os.makedirs(IMAGES_DIR, exist_ok=True)

with AnyReader([Path(BAG_PATH)]) as reader:
    depth_planar_count = 0
    depth_perspective_count = 0
    optical_count = 0
    segmentation_count = 0

    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/airsim_node/Copter/front_center_fixed_jpeg/Scene':
            print(f'image {optical_count} metadata:')
            msg = reader.deserialize(rawdata, connection.msgtype)
            img = pyexiv2.ImageData(msg.data.tobytes())
            print(img.read_exif(), img.read_xmp())

            image_path = os.path.join(IMAGES_DIR, f'optical_{optical_count}.jpg')

            print(f'saving {image_path}')

            file = open(image_path, 'wb')
            file.write(msg.data.tobytes())
            file.close()

            optical_count += 1

        elif connection.topic == '/airsim_node/Copter/front_center_fixed/Segmentation':
            msg = reader.deserialize(rawdata, connection.msgtype)
            image_path = os.path.join(IMAGES_DIR, f'segmentation_{segmentation_count}.png')
            print(f'saving {image_path}')
            image = message_to_cvimage(msg, 'bgr8')
            cv2.imwrite(image_path, image)

            segmentation_count += 1

        elif connection.topic == '/airsim_node/Copter/front_center_fixed/DepthPlanar':
            msg = reader.deserialize(rawdata, connection.msgtype)
            image_path = os.path.join(IMAGES_DIR, f'depth_planar_{depth_planar_count}.png')
            print(f'saving {image_path}')
            image = message_to_cvimage(msg, '32FC1')
            cv2.imwrite(image_path, image)

            depth_planar_count += 1

        elif connection.topic == '/airsim_node/Copter/front_center_fixed/DepthPerspective':
            msg = reader.deserialize(rawdata, connection.msgtype)
            image_path = os.path.join(IMAGES_DIR, f'depth_perspective_{depth_perspective_count}.png')
            print(f'saving {image_path}')
            image = message_to_cvimage(msg, '32FC1')
            cv2.imwrite(image_path, image)

            depth_perspective_count += 1
