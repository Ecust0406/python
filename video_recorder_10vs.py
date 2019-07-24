#!/usr/bin/env python
import rospy
import yaml
import json 
import sys
import cv2
import numpy as np
from mpa_msgs.msg import PerceptionPrediction 
from sensor_msgs.msg import Image
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError
import tool.drawer as drawer
import time

remap_pub = rospy.Publisher('/sensor/camera/image', Image, queue_size = 12)
tag_list = ["LF100", "F50", "RF100", "LB100", "B50", "RB100", "sv_front", "sv_left", "sv_right"]

def prediction_callback(msg, meta):
    
  def get_frame_dict(msg, meta):
    frame_dict = {}
    for tag in tag_list:
      if (not (tag in meta["image_queue"])):
        continue

      while (len(meta["image_queue"][tag]) > 1) :
        meta["image_queue"][tag].pop(0)

      if (len(meta["image_queue"][tag]) > 0 ):
        frame_dict[tag] = meta["image_queue"][tag][0]["image"] 
        meta["image_queue"][tag].pop(0)
    return frame_dict

  def layout_image(layout, image_dict):
    shape = None
    for key, value in image_dict.items():
      if (key == 'bv'):
         continue
      if (shape == None):
         shape = value.shape
      else:
         assert(shape == value.shape)
    assert(shape != None)

    single_image_scale_shape = (640, 360)

    row_images = []
    for row in layout:
      col_images = []
      for image_tag in row:
        if (image_tag in image_dict):
          img = cv2.resize(image_dict[image_tag], single_image_scale_shape) 
          col_images.append(img)
        else:
          col_images.append(np.zeros((single_image_scale_shape[1], single_image_scale_shape[0], 3)))
      row_images.append(np.concatenate(col_images, axis=1)) 
    bvimg = image_dict['bv']
    img2d = np.concatenate(row_images, axis=0)
    scale = img2d.shape[0] / float(bvimg.shape[0]) 
    bvimg = cv2.resize(bvimg, (int(bvimg.shape[1]*scale), int(bvimg.shape[0]*scale)))
    return np.concatenate([img2d, bvimg], axis=1)
    #return img2d

  def save_image(cvimg, meta):
    size = (cvimg.shape[1], cvimg.shape[0])
    if (meta["writer"] == None):
      meta["size"] = size
      meta["writer"] = cv2.VideoWriter(meta["filename"], meta["fourcc"], meta["fps"], meta["size"])
    assert(meta["size"] == size)
    meta["writer"].write(cvimg)
  
  def publish_image(cvimg, meta):
    msg = meta['bridge'].cv2_to_imgmsg(cvimg, encoding="bgr8")
    msg.header.frame_id = '10vs_result'
    meta['debug_pub'].publish(msg)

  start_time = time.time()
  frame_dict = get_frame_dict(msg, meta) 

  if (len(frame_dict) == 0):
    #rospy.logerr("no images received to match meta.relavant_frames.")
    return

  if (len(frame_dict) != len(tag_list)):
    return

  drawer.draw_predcition(frame_dict, msg) 
  cvimg = layout_image(meta["layout"], frame_dict) 
    
  if (meta["output_debug_video"]):
    save_image(cvimg, meta)

  if meta['output_debug_topic']:
    publish_image(cvimg, meta)

  rospy.loginfo("prediction callback process time %.10f" % (time.time() - start_time)) 
  

def image_callback(ros_data, meta):
  if (not (ros_data.header.frame_id in meta["image_queue"])):
    meta["image_queue"][ros_data.header.frame_id] = []
  cvimg = meta["bridge"].imgmsg_to_cv2(ros_data, desired_encoding="passthrough")
  while (len(meta["image_queue"][ros_data.header.frame_id]) >= 100):
    meta["image_queue"][ros_data.header.frame_id].pop(0)
  meta["image_queue"][ros_data.header.frame_id].append(
          {"stamp": ros_data.header.stamp.secs * 1000000 + ros_data.header.stamp.nsecs / 1000, "image": cvimg})

def sv_front_callback(ros_data):
  ros_data.header.frame_id = "sv_front"
  remap_pub.publish(ros_data)

def sv_rear_callback(ros_data):
  ros_data.header.frame_id = "sv_rear"
  remap_pub.publish(ros_data)

def sv_left_callback(ros_data):
  ros_data.header.frame_id = "sv_left"
  remap_pub.publish(ros_data)

def sv_right_callback(ros_data):
  ros_data.header.frame_id = "sv_right"
  remap_pub.publish(ros_data)

def listener():
  rospy.init_node('json_recorder', anonymous=True)
  sub_prediction_topic = rospy.get_param("~sub_prediction_topic", "/mpa/compute/prediction")
  sub_image_topic = rospy.get_param("~sub_image_topic", "/camera/images")
  layout_image_tag = rospy.get_param("~layout_image_tag", None)
  fps = rospy.get_param("~fps", 20)
  video_file = rospy.get_param("~video_file", "map_ros.avi")
  output_debug_video = rospy.get_param("~output_debug_video", False)
  debug_topic = rospy.get_param("~debug_topic", "/compute/vision/6cam")
  output_debug_topic = rospy.get_param("~output_debug_topic", False)
  debug_pub = None
  if output_debug_topic:
      debug_pub = rospy.Publisher(debug_topic, Image, queue_size = 2)

  layout = json.loads(layout_image_tag) if layout_image_tag != None else None

  
  meta = {
    "fps": fps,
    "filename": video_file,
    "output_debug_video": output_debug_video,
    "output_debug_topic": output_debug_topic,
    "debug_pub": debug_pub,
    "fourcc": cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),
    "size": None,
    "writer": None,
    "bridge": CvBridge(),
    "layout": layout,
    "image_queue": {}
  }


  rospy.loginfo("Start recording topic '%s'", sub_prediction_topic) 
  rospy.Subscriber(sub_prediction_topic, PerceptionPrediction, prediction_callback, meta, queue_size = 2)
  rospy.Subscriber(sub_image_topic, Image, image_callback, meta, queue_size = 12)
  rospy.Subscriber("/mvp/drv/sv_front_raw_img", Image, sv_front_callback, queue_size = 12)
  #rospy.Subscriber("/mvp/drv/sv_behind_raw_img", Image, sv_rear_callback, queue_size = 12)
  rospy.Subscriber("/mvp/drv/sv_right_raw_img", Image, sv_right_callback, queue_size = 12)
  rospy.Subscriber("/mvp/drv/sv_left_raw_img", Image, sv_left_callback, queue_size = 12)

  rospy.spin()
  if output_debug_video:
    rospy.loginfo("Save video file to %s", video_file) 


if __name__ == '__main__':
  listener()
