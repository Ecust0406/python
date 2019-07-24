import cv2
import numpy as np
from mpa_msgs.msg import PerceptionPrediction 
from mpa_msgs.msg import CarPropertyEnum
from mpa_msgs.msg import HumanPropertyEnum

COLOR_CAR_BOX = (0, 255, 0)
COLOR_MAP = {'ped': (106, 106, 255),
             'ofo': (255, 0, 0),
             'car_3d': (255, 255, 0),
             'car_rear': (106, 106, 255),
             'ped_waist': (0, 255, 0),
             'rider_waist': (250, 131, 255),
             'rider_ofo_match': (106, 255, 255)}

def draw_car(image, car):
  properties = map(lambda x:x.value, car.properties)
  if (CarPropertyEnum.CAR_PROPERTY_BOUNDING_BOX_2D not in properties):
      return
  left = int(car.bounding_box_2d.left)
  top = int(car.bounding_box_2d.top)
  cv2.rectangle(image, (int(car.bounding_box_2d.left), int(car.bounding_box_2d.top)), 
          (int(car.bounding_box_2d.right), int(car.bounding_box_2d.bottom)), 
          COLOR_MAP['car_3d'], 2, 2)

  if (CarPropertyEnum.CAR_PROPERTY_TRACK_INFO in properties):
    cv2.putText(image, str(car.track_info.track_id), (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
            COLOR_MAP['car_3d'], 2)

  COLORS = [(255, 255, 255), (0, 0, 255), (255, 0, 0), (0, 255, 0)]
  if (CarPropertyEnum.CAR_PROPERTY_WHEELS_2D in properties):
    for j in range(4):
      color = COLORS[j]
      if car.wheels_2d[j].visibility < 0.5:
        color = (100, 100, 100)
      point = (int(car.wheels_2d[j].position.x), int(car.wheels_2d[j].position.y))
      cv2.circle(image, point, 5, color, -1)

def draw_human(image, human):
  properties = map(lambda x:x.value, human.properties)
  if (HumanPropertyEnum.HUMAN_PROPERTY_BOUNDING_BOX_2D not in properties):
      return
  left = int(human.bounding_box_2d.left)
  top = int(human.bounding_box_2d.top)
  right = int(human.bounding_box_2d.right)
  bottom = int(human.bounding_box_2d.bottom)
  if (HumanPropertyEnum.HUMAN_PROPERTY_KEYPOINTS_2D in properties):
    cv2.rectangle(image, (left, top), (right, bottom), 
            COLOR_MAP['ped'], 2, 2)
    line_color = (0, 255, 255)
    pos = int(human.keypoints_2d[2].y)
    cv2.line(image, (left, pos), (right, pos), line_color, 2, cv2.LINE_AA)
  else:
    cv2.rectangle(image, (left, top), (right, bottom), 
            COLOR_MAP['ofo'], 2, 2)

  if (HumanPropertyEnum.HUMAN_PROPERTY_TRACK_INFO in properties):
    cv2.putText(image, str(human.track_info.track_id), (left, top), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
            COLOR_MAP['ped'], 2)

def draw_car_predcitions(image_dict, car_predictions):
  for car in car_predictions.cars:
    if (car.tag in image_dict):
      draw_car(image_dict[car.tag], car)

def draw_human_predcitions(image_dict, human_predictions):
  for human in human_predictions.humans:
    if (human.tag in image_dict):
      draw_human(image_dict[human.tag], human)

def bvCoord_to_bvImg(pt_bv, bot_center_bv, bot_center_img, scale):
  y = int((pt_bv[0] - bot_center_bv[0]) * scale)
  x = int((pt_bv[1] - bot_center_bv[1]) * scale)
  return (bot_center_img[0] - x, bot_center_img[1] - y)

def draw_bv_canvas(bvImg, depth_bot, half_lateral_range, bg_color):
  bvImg[:,:] = bg_color
  scale = bvImg.shape[1] / float(half_lateral_range * 2)
  range_longitude = int(bvImg.shape[0] / scale)
  bot_center_bv = (depth_bot, 0)
  bot_center_img = (bvImg.shape[1] / 2, bvImg.shape[0] - 1)

  # add distance ruler
  RANGE = 10
  min_val = depth_bot / RANGE
  max_val = (range_longitude + depth_bot) / RANGE
  for i in range(min_val, max_val + 1):
    dist = RANGE * i
    pt2 = bvCoord_to_bvImg((dist, -half_lateral_range), bot_center_bv,
        bot_center_img, scale)
    pt1 = (pt2[0] - 30, pt2[1])
    cv2.line(bvImg, pt1, pt2, (255, 255, 255), 1)
    pt1 = (pt1[0] - 5, pt1[1] - 10)
    cv2.putText(bvImg, str(dist) + "m", pt1, cv2.FONT_HERSHEY_SIMPLEX, 0.4,
        (255, 255, 255), 1)

  # add center line
  cv2.line(bvImg, (bvImg.shape[1] / 2, 0), (bvImg.shape[1] / 2, bvImg.shape[0] - 1),
      (255, 255, 255), 1)

  # add bottom line at 0
  left = bvCoord_to_bvImg((0, -half_lateral_range), bot_center_bv,
      bot_center_img, scale)
  right = bvCoord_to_bvImg((0, half_lateral_range), bot_center_bv,
      bot_center_img, scale)
  cv2.line(bvImg, left, right, (255, 255, 255), 1)

def draw_car_bv(bvImg, car3d, depth_bot, half_lateral_range, color, draw_speed):
  scale = bvImg.shape[1] / float(half_lateral_range * 2)
  bot_center_bv = (depth_bot, 0)
  bot_center_img = (bvImg.shape[1] / 2, bvImg.shape[0] - 1)

  p0 = (car3d.bounding_box_bv[0].x, car3d.bounding_box_bv[0].y)
  p1 = (car3d.bounding_box_bv[1].x, car3d.bounding_box_bv[1].y)
  p2 = (car3d.bounding_box_bv[2].x, car3d.bounding_box_bv[2].y)
  p3 = (car3d.bounding_box_bv[3].x, car3d.bounding_box_bv[3].y)
  pcenter = (int((p0[0] + p3[0]) * 0.5), int((p0[1] + p3[1]) * 0.5))

  polygon0 = bvCoord_to_bvImg(p0, bot_center_bv, bot_center_img, scale)
  polygon1 = bvCoord_to_bvImg(p1, bot_center_bv, bot_center_img, scale)
  polygon2 = bvCoord_to_bvImg(p3, bot_center_bv, bot_center_img, scale)
  polygon3 = bvCoord_to_bvImg(p2, bot_center_bv, bot_center_img, scale)
  LINE_WIDTH = 1
  cv2.line(bvImg, polygon0, polygon1, color, LINE_WIDTH)
  cv2.line(bvImg, polygon1, polygon2, color, LINE_WIDTH)
  cv2.line(bvImg, polygon2, polygon3, color, LINE_WIDTH)
  cv2.line(bvImg, polygon3, polygon0, color, LINE_WIDTH)

  cv2.circle(bvImg, polygon1, 3, (0, 0, 255), -1)
  cv2.circle(bvImg, polygon2, 3, (0, 255, 0), -1)
  cv2.circle(bvImg, polygon3, 3, (255, 0, 0), -1)
  cv2.circle(bvImg, polygon0, 3, (255, 255, 255), -1)

  # draw speed vector
  if (draw_speed):
    vx = car3d.velocity_bv.x
    vy = car3d.velocity_bv.y
    end1 = bvCoord_to_bvImg(pcenter, bot_center_bv, bot_center_img, scale)
    tmp = (pcenter[0] + vx, pcenter[1] + vy)
    end2 = bvCoord_to_bvImg(tmp, bot_center_bv, bot_center_img, scale)
    cv2.line(bvImg, end1, end2, color, 2)

  cv2.putText(bvImg, str(car3d.track_info.track_id),
      (polygon1[0], polygon1[1] + 10),
      cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, LINE_WIDTH)

def draw_human_bv(bvImg, human, depth_bot, half_lateral_range, color, draw_speed):
  scale = bvImg.shape[1] / float(half_lateral_range * 2)
  bot_center_bv = (depth_bot, 0)
  bot_center_img = (bvImg.shape[1] / 2, bvImg.shape[0] - 1)

  pt = (human.location_bv.x, human.location_bv.y)
  center = bvCoord_to_bvImg(pt, bot_center_bv, bot_center_img, scale)
  cv2.circle(bvImg, center, 3, color, -1);
  #print "python human %s location"%(human.track_info.track_id), pt

  # draw speed vector
  if (draw_speed):
    vx = human.velocity_bv.x
    vy = human.velocity_bv.y
    tmp = (pt[0] + vx, pt[1] + vy)
    end2 = bvCoord_to_bvImg(tmp, bot_center_bv, bot_center_img, scale)
    cv2.line(bvImg, center, end2, color, 2)

  cv2.putText(bvImg, str(human.track_info.track_id), center,
      cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

def draw_3d_birdview(image_dict, prediction):
    bvimg = np.zeros((720, 225, 3), dtype=np.uint8)
    depth_bot = -100
    lateral_half_range = 40
    bg_color = (100, 100, 100)
    color_car = COLOR_MAP['car_3d']
    draw_bv_canvas(bvimg, depth_bot, lateral_half_range, bg_color)
    # draw car 3d
    for car in prediction.car_predictions.cars:
      #if not car.tag == '6cam':
      #    continue
      properties = map(lambda x:x.value, car.properties)
      if (CarPropertyEnum.CAR_PROPERTY_IS_FAILED_3D not in properties):
          continue
      if (car.is_failed_3d):
          continue
      draw_car_bv(bvimg, car, depth_bot, lateral_half_range, color_car, True) 
    # draw human 3d
    for human in prediction.human_predictions.humans:
      properties = map(lambda x:x.value, human.properties)
      if (HumanPropertyEnum.HUMAN_PROPERTY_IS_FAILED_3D not in properties):
          continue
      if (human.is_failed_3d):
          continue
      color = COLOR_MAP['ped']
      if (HumanPropertyEnum.HUMAN_PROPERTY_KEYPOINTS_2D not in properties):
          color = COLOR_MAP['ofo']
      draw_human_bv(bvimg, human, depth_bot, lateral_half_range, color, True) 

    image_dict['bv'] = bvimg

def draw_predcition(image_dict, prediction):
  #draw_car_predcitions(image_dict, prediction.car_predictions)
  #draw_human_predcitions(image_dict, prediction.human_predictions)
  draw_3d_birdview(image_dict, prediction)

