#!/usr/bin/env python3

from rosbags.highlevel import AnyReader
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
    count=0
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/airsim_node/Copter/front_center_gimbaled_jpeg/Scene':
            print(f'image {count} metadata:')
            msg = reader.deserialize(rawdata, connection.msgtype)
            img = pyexiv2.ImageData(msg.data.tobytes())
            print(img.read_exif(), img.read_xmp())

            IMAGE_PATH=os.path.join(IMAGES_DIR, f'{count}.jpg')

            print(f'saving {IMAGE_PATH}')

            file = open(IMAGE_PATH, 'wb')
            file.write(msg.data.tobytes())
            file.close()

            count += 1

