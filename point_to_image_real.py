# -*- coding: utf-8 -*-
 
#  数据来源: calib_cam_to_cam.txt 
#  下载链接: http://www.cvlibs.net/datasets/kitti/raw_data.php?type=road  >  2011_10_03_drive_0047  >  [calibration]
# R_rect_00: 9.999454e-01 7.259129e-03 -7.519551e-03 -7.292213e-03 9.999638e-01 -4.381729e-03 7.487471e-03 4.436324e-03 9.999621e-01
## P_rect_00: 7.188560e+02 0.000000e+00 6.071928e+02 0.000000e+00 0.000000e+00 7.188560e+02 1.852157e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
# ...
## R_rect_02: 9.999191e-01 1.228161e-02 -3.316013e-03 -1.228209e-02 9.999246e-01 -1.245511e-04 3.314233e-03 1.652686e-04 9.999945e-01
# P_rect_02: 7.188560e+02 0.000000e+00 6.071928e+02 4.538225e+01 0.000000e+00 7.188560e+02 1.852157e+02 -1.130887e-01 0.000000e+00 0.000000e+00 1.000000e+00 3.779761e-03
 
#  数据来源: calib_velo_to_cam.txt
#  下载链接: http://www.cvlibs.net/datasets/kitti/raw_data.php?type=road  >  2011_10_03_drive_0047  >  [calibration]
#   calib_time: 15-Mar-2012 11:45:23
#   R: 7.967514e-03 -9.999679e-01 -8.462264e-04 -2.771053e-03 8.241710e-04 -9.999958e-01 9.999644e-01 7.969825e-03 -2.764397e-03
#   T: -1.377769e-02 -5.542117e-02 -2.918589e-01
 
 
# # png bin来源
# data_odometry_color/dataset/sequences/00/image_2
# data_odometry_velodyne/dataset/sequences/00/velodyne
 
import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
#import utils
from PIL import Image
import math
from cv2 import cv2 as cv

def a2mat(att):
    att=att*math.pi/180
    sp = math.sin(att[0])
    cp = math.cos(att[0])
    sr = math.sin(att[1])
    cr = math.cos(att[1])
    sy = math.sin(att[2])
    cy = math.cos(att[2])
    m = np.array([cy*cr - sy*sp*sr, -sy*cp, cy*sr + sy*sp*cr,
	            sy*cr + cy*sp*sr, cy*cp, sy*sr - cy*sp*cr,
	            -cp*sr, sp, cp*cr]).reshape((3,3))
    return m

# R_rect_02 = np.array( [ 9.999191e-01, 1.228161e-02 -3,.316013e-03,
#                         -1.228209e-02, 9.999246e-01, -1.245511e-04, 
#                         3.314233e-03, 1.652686e-04, 9.999945e-01]).reshape((3,3))
 
 
#velo激光雷达 到 相机00(此处已知条件重点注意) 的变换矩阵
#Tr_velo_to_cam = np.array( [    7.967514e-03, -9.999679e-01, -8.462264e-04, -1.377769e-02,
#                                -2.771053e-03, 8.241710e-04, -9.999958e-01, -5.542117e-02,
#                                9.999644e-01, 7.969825e-03, -2.764397e-03, -2.918589e-01]).reshape((3,4))    

#-----------------------------------相机02内参矩阵-----------------------------------
#P_rect_02 = np.array( [ 7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, 4.538225000000e+01, 
#                        0.000000000000e+00,7.188560000000e+02, 1.852157000000e+02, -1.130887000000e-01,
#                        0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 3.779761000000e-03]).reshape((3,4))

#1027
# P_rect_02 = np.array( [ 1155.37981619255,0,514.574972587357,0,
#                        0, 1157.65374098386, 398.083765998826,0,
#                        0, 0, 1,0]).reshape((3,4))
# Tr_velo_to_cam = np.array( [ -0.0697220865854439,-0.997560317901041,0.00336993168534795,0.145795112218959,
#                               -0.0254936552628620,-0.00159525324931659,-0.999672934422386,-0.418790648854031,
#                                  0.997240827463846,-0.0697851608814216,-0.0253203464498418,-0.265396030545724]).reshape((3,4))    

R_rect_00 = np.array( [ 1,0,0,
                        0,1,0,
                        0,0,1]).reshape((3,3))

P_rect_02 = np.array( [ 1188.2751919716982,0,507.46110050511254,0,
             0,1186.585848167888,400.54197615210126,0,
             0,0,1,0]).reshape((3,4))


att_L_C=np.array([0,  -90,   90])

#att_L_C=np.array([3.8158,  -91.743,   90.5977])

Tr_velo_to_cam=a2mat(att_L_C)

Tr_velo_to_cam=np.insert(Tr_velo_to_cam,3,values=[0.136,-0.38,-0.465],axis=1)

# Tr_velo_to_cam = np.array( [ 
# 0.0487984663507069,	-0.998321006365647,	0.0311998041946708,	0.136,
# -0.0272060903139672,	-0.0325539801274806,	-0.999099400298282,	-0.38,
# 0.998437827492068,	0.0479057083880477,	-0.0287490007931511,	-0.465]).reshape((3,4))    
 
#-----------------------------------数据文件位置---------------------------------------
#1027
#velo_files = "C:\\Users\\feiya\\Desktop\\1603766696733788928.bin"
#rgbimg = "C:\\Users\\feiya\\Desktop\\1603766696750000128.png"
# velo_files = "I:\\2020_10_27_0003\\laser\\data\\1603766544536114944.bin"
# rgbimg = "I:\\2020_10_27_0003\\img0\\data\\1603766544349999872.png"
resultImg = "I:\\Data\\KITTI\\2011_09_30\\2011_09_30_drive_0016_sync\\result_merge1.png"

velo_files = "I:\\2021_12_22\\laser\\data\\1640188382891764992.bin"
rgbimg = "I:\\2021_12_22\\image\\image\\12222021075303-34488.png"
 
data = {}
data['P_rect_20'] = P_rect_02
# Compute the velodyne to rectified camera coordinate transforms
data['T_cam0_velo'] = Tr_velo_to_cam
data['T_cam0_velo'] = np.vstack([data['T_cam0_velo'], [0, 0, 0, 1]])
 
# pattern1:
R_rect_00 = np.insert(R_rect_00,3,values=[0,0,0],axis=0)
R_rect_00 = np.insert(R_rect_00,3,values=[0,0,0,1],axis=1)
data['T_cam2_velo'] = R_rect_00.dot(data['T_cam0_velo']) #雷达 到 相机02的变换矩阵
print(data['T_cam2_velo'])
 
#pointCloud = utils.load_velo_scan(velo_files)   #读取lidar原始数据
pointCloud = np.fromfile(velo_files, dtype=np.float32).reshape((-1,4))
points = pointCloud[:, 0:3]                                           # 获取 lidar xyz (front, left, up)
points_homo = np.insert(points,3,1,axis=1).T    # 齐次化,并转置(一列表示一个点(x,y,z,1), 多少列就有多少个点)
points_homo = np.delete(points_homo,np.where(points_homo[0,:]<0),axis=1) #以列为基准, 删除深度x=0的点
 
proj_lidar = data['P_rect_20'].dot( data['T_cam2_velo'] ).dot(points_homo)  #相机坐标系3D点=相机02内参*激光雷达到相机02的变换矩阵*激光雷达3D点
cam = np.delete(proj_lidar,np.where(proj_lidar[2,:]<0),axis=1)  #以列为基准, 删除投影图像点中深度z<0(在投影图像后方)的点 #3xN
cam[:2,:] /= cam[2,:]   # 等价写法 cam[:2] /= cam[2] # 前两行元素分布除以第三行元素(归一化到相机坐标系z=1平面)(x=x/z, y =y/z)
 
 
# -----------------------------------将激光投影点绘制到图像平面:绘制原图------------------------------------
srcImg = mpimg.imread(rgbimg)
IMG_H,IMG_W,_ = srcImg.shape

img_size = (IMG_W, IMG_H)
camera_matrix = np.array( [ 1188.2751919716982,0,507.46110050511254,
             0,1186.585848167888,400.54197615210126,
             0,0,1]).reshape((3,3))

distortion_coefficients = np.array( [-0.07213967321763622, 0.38997778448155435, 0.0007896523262099673, -0.0018071194058766741]).reshape([4, 1])

R = np.eye(3)

mapx, mapy = cv.fisheye.initUndistortRectifyMap( camera_matrix, distortion_coefficients, R, camera_matrix, img_size, cv.CV_32FC1)

png = cv.remap(srcImg, mapx, mapy, cv.INTER_LINEAR,  cv.BORDER_CONSTANT)

#png = cv.cvtColor(png0,cv.COLOR_BGR2GRAY)

plt.figure(figsize=((IMG_W)/72.0,(IMG_H)/72.0),dpi=72.0, tight_layout=True)
# restrict canvas in range
plt.axis([0,IMG_W,IMG_H,0])
# plt.axis('off') 
plt.imshow(png) #在画布上画出原图
 
# filter point out of canvas
u,v,z = cam
u_out = np.logical_or(u<0, u>IMG_W)
v_out = np.logical_or(v<0, v>IMG_H)
outlier = np.logical_or(u_out, v_out)
cam = np.delete(cam,np.where(outlier),axis=1)
# generate color map from depth
u,v,z = cam

z=z/2
# 将激光投影点绘制到图像平面:绘制激光深度散点图
plt.scatter([u],[v],c=[z],cmap='gist_ncar',alpha=0.8,s=200,marker='.')
plt.title("projection")
plt.savefig(resultImg,bbox_inches='tight')
plt.show()  #在画布上画出散点雷达深度投影
 
 
#-----------------------------------单独保存深度图像成灰度图像---------------------------------------------------
image_array = np.zeros((IMG_H, IMG_W), dtype=np.int16)
for i in range(cam.shape[1]):
    x = int(round(u[i]))
    y = int(round(v[i]))
 
    # x = math.ceil(u[i]) #向上取整
    # y = math.ceil(v[i])
 
    depth =  int(z[i]*256)
    if 0<x<image_array.shape[1] and 0<y<image_array.shape[0]:
        image_array[y,x] = depth
 
image_pil = Image.fromarray(image_array, 'I;16')   
image_pil.save("result_16.png")
 
 
 
print("done")