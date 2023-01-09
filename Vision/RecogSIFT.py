#!/usr/bin/env python3.6
import cv2 
from matplotlib import pyplot as plt
import numpy as np
from sklearn.cluster import MeanShift, estimate_bandwidth
import pyrealsense2 as rs
MIN_MATCH_COUNT = 3
from scipy.spatial.transform import Rotation as R
# Configure depth and color streams

class GetObjectLocation:
    def getLocation(self):
        # Get device product line for setting a supporting resolution
        pipeline = rs.pipeline()
        config = rs.config()
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        min_matches = 10
        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        cfg = pipeline.start(config)
        profile = cfg.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
        intr = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
        try:
            while True:
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                cv2.imwrite("Copy.jpg", color_image )
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                  
                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape
                if depth_colormap_dim != color_colormap_dim:
                    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                source = cv2.imread('1.jpg', cv2.COLOR_BGR2GRAY)
                source = cv2.cvtColor(source, cv2.COLOR_BGR2GRAY)
                scale_percent = 5
                width = int(source.shape[1] * scale_percent / 100)
                height = int(source.shape[0] * scale_percent / 100)
                dim = (width, height)
                detect = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                source = cv2.resize(source, dim, interpolation = cv2.INTER_AREA)
                orb = cv2.ORB_create()
                features1, des1 = orb.detectAndCompute(source, None)
                features2, des2 = orb.detectAndCompute(detect, None)
            
                bf = cv2.BFMatcher(cv2.NORM_HAMMING)
                
                matches = bf.match(des1, des2)
                matches = sorted(matches, key = lambda x:x.distance) 
                matches = matches[:10]
                good = []    
                good_without_lists = []    
                '''matches = [match for match in matches if len(match) == 2] 
                for m, n in matches:
                    if m.distance < 0.875 * n.distance:
                        good.append(m)
                '''
                kp2 = np.array([features2[mat.trainIdx].pt for mat in matches])
                x = kp2[:,0]
                y = kp2[:,1]
                x_center = np.mean(x)
                y_center = np.mean(y)
                delta_x = np.abs(np.max(x)-x_center)
                delta_y = np.abs(np.max(y)-y_center)
                if len(matches) >= 10:
                    h, w = source.shape[:2]
                    pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
                    detect = cv2.rectangle(detect, np.int32((x_center-delta_x,y_center-delta_y)),np.int32((x_center+delta_x,y_center+delta_y)), (255, 0, 0), 2)
                    depth = depth_frame.get_distance(int(x_center),int(y_center))
                    result = rs.rs2_deproject_pixel_to_point(intr, [int(x_center), int(y_center)], depth)
                    r = R.from_quat([-0.0136107,-0.0151583,-0.00612233,0.999774])
                    x = -0.0009842
                    y = -0.0863986
                    z = 0.069957099
                    rot_mat = np.array(r.as_matrix())
                    hom_matrix = np.array([[rot_mat[0,0],rot_mat[0,1],rot_mat[0,2],x],
                                       [rot_mat[1,0],rot_mat[1,1],rot_mat[1,2],y],
                                        [rot_mat[2,0],rot_mat[2,1],rot_mat[2,2],z],[0,0,0,1]]) 
                    #hom_matrix = np.array([[0.99,0.0650,-0.1213,0.51],[-0.0596,0.997,-0.04780,-2.143],[-0.124,-0.0401,-0.99145,0.33782],[0,0,0,1]])
                    robot_position = hom_matrix @ np.array([result[0],result[1],result[2],1]).T
                    img_text = "Delta from EE x: "+str(round(robot_position[0],2))+" y: "+str(round(robot_position[1],2))+" z: "+str(round(robot_position[2],2))
                    detect = cv2.putText(detect, img_text, np.int32((x_center-delta_x,y_center-delta_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2, cv2.LINE_AA)               
                
                img3 = cv2.drawMatches(source,features1,detect,features2,matches,None, flags=2) 
                cv2.imshow('Matches', img3)
                #r = R.from_euler('xyz',[0.0504,-0.3324,-0.3643], degrees=False)
              
                print("Position from the Robot: ",robot_position)
                #print(np.int32((x_center-delta_x,y_center-delta_y)))
                #print(np.int32((x_center+delta_x,y_center+delta_y)))
                
                
                print("World Coordinates :", result)
                cv2.waitKey(1)
        finally:
            pipeline.stop()


if __name__=="__main__":
    g = GetObjectLocation()
    g.getLocation()


  