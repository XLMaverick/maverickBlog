---
# 常用定义
title: "vins系统的构建"        # 标题
date: 2019-08-08               # 创建时间
lastmod: 2019-08-08            # 最后修改时间
draft: false                   # 是否是草稿？
tags: ["SLAM" ,"VIO"]          # 标签
categories: ["VIO"]           # 分类
author: "Yang"                 # 作者

# 用户自定义
# 你可以选择 关闭(false) 或者 打开(true) 以下选项
comment: true    # 打开评论
toc: true        # 打开文章目录
# 你同样可以自定义文章的版权规则
contentCopyright: '<a rel="license noopener" href="https://creativecommons.org/licenses/by-nc-nd/4.0/" target="_blank">CC BY-NC-ND 4.0</a>'
reward: false	 # 关闭打赏
mathjax: false   # 关闭 mathjax
---

vins系统的构建
------------------------
将第二讲中仿真数据集（视觉特征，`imu`数据）接入自己的`VINS`代码，并运行出轨迹结果。

- 仿真数据集无噪声
- 仿真数据集有噪声（不同噪声设定时，需要配置`vins`中的`imu noise`大小）

<div align=left>
<img src="../images/vins系统的构建/image1.png" >
</div>

### 答案

相比较与`EUROC`数据集，我们需要改动三个地方：

- 运行的配置文件`config`
- 运行的接口文件`run_simlatute_data.run`
- `System.cpp`中的处理函数`PubImageData`函数。

1、config文件的修改：
仿真数据产生的是每一帧所有的特征点在相机归一化平面下的坐标，所以相机的内参这一块没有影响，`config`文件中的相机的内参数无需更改；  
相机和`IMU`的外参需要自己计算，通过查看仿真数据，我们可以读取相机到世界坐标系的变换矩阵、以及`IMU`到世界坐标系的变换矩阵，则我们很容易求出相机到`IMU`坐标系的变换矩阵，然后修改`config`中的外参即可。对于有噪声的`imu`数据，需要设置`config`文件中的噪声大小。相应的代码和结果如下：  

```
    Eigen::Quaterniond Qwc(0.474386,0.524365,-0.474386,-0.524365);
    Eigen::Isometry3d Twc = Eigen::Isometry3d::Identity();
    Twc.rotate(Qwc.normalized().toRotationMatrix());
    Twc.pretranslate(Eigen::Vector3d(20.5,5.03681,5.03384));

    Eigen::Quaterniond Qwi(0.99875,0.0499792,0,0);
    Eigen::Isometry3d Twi = Eigen::Isometry3d::Identity();
    Twi.rotate(Qwi.normalized().toRotationMatrix());
    Twi.pretranslate(Eigen::Vector3d(20,5,5));

    Eigen::Isometry3d Tic = Eigen::Isometry3d::Identity();
    Tic = Twi.inverse()*Twc;
    cout<<"Tic \n"<<Tic.matrix()<<endl;
```
<div align=left>
<img src="../images/vins系统的构建/result4.png" >
</div>

2、运行的接口文件，主要是读取`IMU`数据和特征点的数据，是`C++`对于文件的基本操作，其中特征点文件中没有时间戳信息，这里使用`cam_pose_tum.txt`对应的时间戳，详细代码如下；  
3、`System.cpp`中的处理函数`PubImageData`函数的修改，主要是删除`trackerData`变量，将特征点归一化平面的坐标直接赋值给`feature_points`变量，详细的代码如下。

最终结果如下：
没有噪声的情况,结果如下：

<div align=left>
<img src="../images/vins系统的构建/result1.png" width = 550 height = 350 >
</div>

有噪声的情况下，若没有更改config文件中的噪声参数，结果如下：

<div align=left>
<img src="../images/vins系统的构建/result3.png" width = 550 height = 350>
</div>

更改噪声参数后，`acc_n: 0.8  gyr_n: 0.4`结果如下：

<div align=left>
<img src="../images/vins系统的构建/result5.png" width = 550 height= 350>
</div>

更改噪声参数后，`acc_n: 1  gyr_n: 1`结果如下：

<div align=left>
<img src="../images/vins系统的构建/result2.png" width = 550 height= 350>
</div>

使用`evo`工具对上述的轨迹与真值进行比对，其中有噪声情况下，未修改配置文件的时候，轨迹明显的与真值差别较大，对于这种情况我们不在进行赘述，具体的情况如下：
使用带噪声的`imu`数据，配置文件参数`acc_n: 0.8  gyr_n: 0.4`时，生成轨迹和真值轨迹的比对如下：

<div align=left>
<img src="../images/vins系统的构建/result8.png" width = 650 height= 350>
</div>

<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/vins系统的构建/result7.png"  width="320"  height = "250">
</div>

<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/vins系统的构建/result9.png" width="320" height = "250">
</div>

<div style="float:none;clear:both;">
</div>

<!-- <div align=left>
<img src="图片/result7.png" width = 220 height= 250>
<img src="图片/result9.png" width = 220 height= 250>
</div> -->


使用带噪声的`imu`数据，配置文件参数`acc_n: 1  gyr_n: 1`时，生成轨迹和真值轨迹的比对如下：
<div align=left>
<img src="../images/vins系统的构建/result10.png" width = 650 height= 350>
</div>


<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/vins系统的构建/result6.png"  width="355"  height = "250">
</div>

<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/vins系统的构建/result11.png" width="320" height = "250">
</div>

<div style="float:none;clear:both;">
</div>

<!-- <div align=left>
<img src="图片/result6.png" width = 255 height= 250>
<img src="图片/result11.png" width = 220 height= 250>
</div> -->

详细代码如下：
```
// System::PubImageData函数修改
void System::PubImageData(double dStampSec, vector<cv::Point2f> &feacturePoint)
{
    if (!init_feature){
        cout << "1 PubImageData skip the first detected feature, which doesn't contain optical flow speed" << endl;
        init_feature = 1;
        return;
    }
    if (first_image_flag){
        cout << "2 PubImageData first_image_flag" << endl;
        first_image_flag = false;
        first_image_time = dStampSec;
        last_image_time = dStampSec;
        return;
    }

    if (dStampSec - last_image_time > 1.0 || dStampSec < last_image_time){
        cerr << "3 PubImageData image discontinue! reset the feature tracker!" << endl;
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        return;
    }
    last_image_time = dStampSec;
    if (round(1.0 * pub_count / (dStampSec - first_image_time)) <= FREQ){
        PUB_THIS_FRAME = true;
        if (abs(1.0 * pub_count / (dStampSec - first_image_time) - FREQ) < 0.01 * FREQ){
            first_image_time = dStampSec;
            pub_count = 0;
        }
    }
    else{
        PUB_THIS_FRAME = false;
    }
    for (unsigned int i = 0;; i++){
        bool completed = false;
        completed |= trackerData[0].updateID(i);

        if (!completed)
            break;
    }
    if (PUB_THIS_FRAME){
        pub_count++;
        shared_ptr<IMG_MSG> feature_points(new IMG_MSG());
        feature_points->header = dStampSec;
        vector<set<int>> hash_ids(NUM_OF_CAM);

        for(unsigned i = 0; i<feacturePoint.size();i++){
            auto &un_pts = feacturePoint[i];
            feature_points->points.push_back(Vector3d(un_pts.x, un_pts.y, 1));
            feature_points->id_of_point.push_back(i);
            feature_points->u_of_point.push_back(un_pts.x);
            feature_points->v_of_point.push_back(un_pts.y);
            feature_points->velocity_x_of_point.push_back(un_pts.x);
            feature_points->velocity_y_of_point.push_back(un_pts.y);
        }
        if (!init_pub){
            cout << "4 PubImage init_pub skip the first image!" << endl;
            init_pub = 1;
        }
        else{
            m_buf.lock();
            feature_buf.push(feature_points);
            // cout << "5 PubImage t : " << fixed << feature_points->header<< " feature_buf size: " << feature_buf.size() << endl;
            m_buf.unlock();
            con.notify_one();
        }
    } 
}
```

```
//run_simulate_data.cpp文件
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/vbtang/worksapce/vio/vio_data_simulation/bin/";
string sConfig_path = "../config/";

std::shared_ptr<System> pSystem;

void PubImuData()
{
	string sImu_data_file = sData_path + "imu_pose_noise.txt";
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open()){
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;
	Eigen::Quaterniond Qwb;
	Vector3d position;
	
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampNSec >>Qwb.w()>>Qwb.x()>>Qwb.y()>>Qwb.z()>>position.x()>>position.y()>>position.z() >>vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec, vGyr, vAcc);
		usleep(5000*nDelayTimes);
	}
	fsImu.close();
}

void PubImageData()
{
	string sImage_file = sData_path + "cam_pose_tum.txt";
	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;
	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open()){
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	int n = 0;
	// cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		vector<cv::Point2f> feacturePoint;
		std::istringstream ssImageData(sImage_line);
		ssImageData >> dStampNSec ;
		// cout << "Image t : " << fixed << dStampNSec <<endl;

		std::stringstream filename1;
        filename1<<"keyframe/all_points_"<<n<<".txt";
		n = n+1;
		string featurePath = sData_path + filename1.str();
		// cout<<"featurePath: "<<featurePath<<endl;

		ifstream fsFeature;
		fsFeature.open(featurePath.c_str());
		if (!fsFeature.is_open()){
			cerr << "Failed to open image file! " << featurePath << endl;
			return;
		}
		std::string sFeature_line;
		while(std::getline(fsFeature,sFeature_line) && !sFeature_line.empty()){
			cv::Point2f featureTemp;
			Vector4d temp;
			std::istringstream ssFeatureData(sFeature_line);
			double u = 0.0;
			double v = 0.0;
			ssFeatureData>>temp.x()>>temp.y()>>temp.z()>>temp.w()>>u>>v;
			featureTemp.x = u;featureTemp.y = v;
			feacturePoint.push_back(featureTemp);
		}
		// cout<<"feacturePoint.size(): "<<feacturePoint.size()<<endl;
		pSystem->PubImageData(dStampNSec, feacturePoint);
		usleep(50000*nDelayTimes);
	}
	fsImage.close();
}

int main(int argc, char **argv)
{
	/* if(argc != 3){
		cerr << "./run_euroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config \n" 
			<< "For example: ./run_euroc /home/stevencui/dataset/EuRoC/MH-05/mav0/ ../config/"<< endl;
		return -1;
	}
	sData_path = argv[1];
	sConfig_path = argv[2]; */

	pSystem.reset(new System(sConfig_path));

	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);
	std::thread thd_PubImuData(PubImuData);
	std::thread thd_PubImageData(PubImageData);
	std::thread thd_Draw(&System::Draw, pSystem);
	
	thd_PubImuData.join();
	thd_PubImageData.join();
	thd_BackEnd.join();
	thd_Draw.join();

	cout << "main end... see you ..." << endl;
	system("pause");
	return 0;
}
```

```
//simulate_data_config.yaml文件
%YAML:1.0

#common parameters
imu_topic: "/imu0"
image_topic: "/cam0/image_raw"
output_path: "/home/vbtang/worksapce/"

#camera calibration 
model_type: PINHOLE
camera_name: camera
image_width: 752
image_height: 480
distortion_parameters:
   k1: -2.917e-01
   k2: 8.228e-02
   p1: 5.333e-05
   p2: -1.578e-04
projection_parameters:
   fx: 4.616e+02
   fy: 4.603e+02
   cx: 3.630e+02
   cy: 2.481e+02

# Extrinsic parameter between IMU and Camera.
estimate_extrinsic: 0   # 0  Have an accurate extrinsic parameters. We will trust the following imu^R_cam, imu^T_cam, don't change it.
                        # 1  Have an initial guess about extrinsic parameters. We will optimize around your initial guess.
                        # 2  Don't know anything about extrinsic parameters. You don't need to give R,T. We will try to calibrate it. Do some rotation movement at beginning.                        
#If you choose 0 or 1, you should write down the following matrix.
#Rotation from camera frame to imu frame, imu^R_cam
extrinsicRotation: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [0,0, -1,
           -1, 0, 0, 
           0, 1, 0]
#Translation from camera frame to imu frame, imu^T_cam
extrinsicTranslation: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [0.5,0.0400045, 0.0299961]

#feature traker paprameters
max_cnt: 150            # max feature number in feature tracking
min_dist: 30            # min distance between two features 
freq: 10                # frequence (Hz) of publish tracking result. At least 10Hz for good estimation. If set 0, the frequence will be same as raw image 
F_threshold: 1.0        # ransac threshold (pixel)
show_track: 1           # publish tracking image as topic
equalize: 1             # if image is too dark or light, trun on equalize to find enough features
fisheye: 0              # if using fisheye, trun on it. A circle mask will be loaded to remove edge noisy points

#optimization parameters
max_solver_time: 0.04  # max solver itration time (ms), to guarantee real time
max_num_iterations: 8   # max solver itrations, to guarantee real time
keyframe_parallax: 10.0 # keyframe selection threshold (pixel)

#imu parameters       The more accurate parameters you provide, the better performance
acc_n: 1          # accelerometer measurement noise standard deviation. #0.2   0.04
gyr_n: 1         # gyroscope measurement noise standard deviation.     #0.05  0.004
acc_w: 0.00004         # accelerometer bias random work noise standard deviation.  #0.02
gyr_w: 2.0e-6       # gyroscope bias random work noise standard deviation.     #4.0e-5
g_norm: 9.81007     # gravity magnitude

#loop closure parameters
loop_closure: 1                    # start loop closure
load_previous_pose_graph: 0        # load and reuse previous pose graph; load from 'pose_graph_save_path'
fast_relocalization: 0             # useful in real-time and large project
pose_graph_save_path: "/home/vbtang/worksapce/" # save and load path

#unsynchronization parameters
estimate_td: 0                      # online estimate time offset between camera and imu
td: 0.0                             # initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock)

#rolling shutter parameters
rolling_shutter: 0                  # 0: global shutter camera, 1: rolling shutter camera
rolling_shutter_tr: 0               # unit: s. rolling shutter read out time per frame (from data sheet). 

#visualization parameters
save_image: 0                   # save image in pose graph for visualization prupose; you can close this function by setting 0 
visualize_imu_forward: 0        # output imu forward propogation to achieve low latency and high frequence results
visualize_camera_size: 0.4      # size of camera marker in RVIZ

```
