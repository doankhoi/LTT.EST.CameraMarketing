#pragma once

#define MAX_TRACKER_NUM  100
#define MAX_TEMPLATE_SIZE 10
#define EXPERT_THRESH 5
#define BODYSIZE_TO_DETECTION_RATIO 0.64
#define TRACKING_TO_BODYSIZE_RATIO 0.5
#define FRAME_RATE 25
#define TIME_WINDOW_SIZE 11
#define HOG_DETECT_FRAME_RATIO  1.0
#define TRACKING_TO_DETECTION_RATIO BODYSIZE_TO_DETECTION_RATIO*TRACKING_TO_BODYSIZE_RATIO

//Tham so khung hinh
#define FRAME_WIDTH 400 //768
#define FRAME_HEIGHT 380 //700
#define MIN_TIME_IN 10
#define MAX_DISTANCE 30
#define DATABASE_PATH  "D:/IpCam/Database/ETSCameraClientCache.db3"
//#define URL "D:/IpCam/Video/CLIP_20150122-141303.mp4"
#define URL "D:/IpCam/Video/CLIP_20150115-140156.mp4"
//#define URL "D:/IpCam/Video/CLIP_20150116-140203.mp4"