#!/usr/bin/env python

from __future__ import print_function
import os, sys
import zmq
import numpy as np
import cv2
import time
import struct

sys.path.append(os.path.join(os.path.dirname(__file__), "../"))
video_path = "./%d.avi" % round(time.time())
height = 300
width = 608

def parse_buffer(s):
    t = struct.unpack("<L", s[:4])[0]
    w = t >> 16
    h = t & 0xFFFF
    t1 = struct.unpack("<L", s[4:8])[0]
    t2 = struct.unpack("<L", s[8:12])[0]
    return h, w, s[12:], t1 << 32 + t2

if __name__ == "__main__":
    context = zmq.Context()
    poller = zmq.Poller()
    sock = context.socket(zmq.SUB)
    sock.connect("tcp://192.168.8.188:9999")
    sock.setsockopt(zmq.SUBSCRIBE, "")

    height, width, _, __ = parse_buffer(sock.recv())
    print(height, width)
    cv2.namedWindow("image", 0)
    out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc('X', 'V', 'I', 'D'), 20, (width, height))
    while True:
        _, __, s, tt = parse_buffer(sock.recv())
        print(len(s))
        d = np.frombuffer(s, dtype=np.uint8)
        d = np.reshape(d, (height, width, 3))
        
        out.write(d)
        cv2.imshow("image", d)
        cv2.waitKey(1)
    out.release()
