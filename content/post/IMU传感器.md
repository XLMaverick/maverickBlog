---
# 常用定义
title: "IMU传感器"        # 标题
date: 2019-06-16               # 创建时间
lastmod: 2019-06-16            # 最后修改时间
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

习题一：IMU仿真数据
--------------------------------
1、设置IMU仿真代码中的不同参数，生成Allan方差的标定曲线。其中Allan方差工具：

- https://github.com/gaowenliang/imu_utils
- https://github.com/rpng/kalibr_allan

2、将IMU仿真代码中的欧拉积分替换为中值积分。

### 答案

由于平时使用`ROS`较多，本次作业均已`ros`版本的`IMU`仿真数据，以及第一个`Allan`的方差工具。本次工程中共有三个`package`，`vio_data_simulation`、`code_utils`、`imu_utils`。第一个用于`IMU`仿真数据的生成，最后一个用于生成`Allen`方差，第二个是最后一个的依赖包。题外，本次作业，代码量很少，只是工程环境的配置以及参数的修改。

代码运行：
```
mkdir -p imu_ws/src
将三个package放入src文件中
在imu_ws中运行 catkin_make
```
#### 参数修改

IMU仿真参数的修改，本次实验主要进行了三组，一组使用默认参数，一组使用默认参数的十倍，一组使用默认参数的十分之一。具体结果如下：

1、第一组实验 使用默认参数
<div style="text-align: center">
<img src="../images/IMU传感器/test1.png"/>
</div>

实验结果为：

<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/IMU传感器/test1-acc.png"  width="320"  height = "150">
</div>

<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/IMU传感器/test1-gro.png" width="320" height = "150">
</div>

<div style="float:none;clear:both;">

</div>


Allan曲线为：
<div style="text-align: center">
<img src="../images/IMU传感器/test1.bmp"/>
</div>

2、第二组实验 使用默认参数的十倍
<div style="text-align: center">
<img src="../images/IMU传感器/test2.png"/>
</div>

实验结果为：

<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/IMU传感器/test2-acc.png"  width="320"  height = "150">
</div>

<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/IMU传感器/test2-gro.png" width="320" height = "150">
</div>

<div style="float:none;clear:both;">

</div>

Allan曲线为：
<div style="text-align: center">
<img src="../images/IMU传感器/test2.bmp"/>
</div>

3、第三组实验 使用默认参数的十分之一
<div style="text-align: center">
<img src="../images/IMU传感器/test3.png"/>
</div>

实验结果为：

<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/IMU传感器/test3-acc.png"  width="320"  height = "150">
</div>

<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/IMU传感器/test3-gro.png" width="320" height = "150">
</div>

<div style="float:none;clear:both;">

</div>

Allan曲线为：
<div style="text-align: center">
<img src="../images/IMU传感器/test3.bmp"/>
</div>

#### 中值积分
中值积分主要修改了imu.cpp文件，修改如下：
```
void IMU::testImu(std::string src, std::string dist)
{
    std::vector<MotionData>imudata;
    LoadPose(src,imudata);

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;
    Eigen::Vector3d Pwb = init_twb_;              // position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);            // quaterniond:  from imu measurements
    Eigen::Quaterniond Qwb_last(init_Rwb_); 
    Eigen::Vector3d Vw = init_velocity_;          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
    MotionData lastimupose = imudata[0];
    for (int i = 1; i < imudata.size(); ++i) {

        MotionData imupose = imudata[i];

        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
        // Eigen::Vector3d dtheta_half =  imupose.imu_gyro * dt /2.0;
       Eigen::Vector3d dtheta_half =  0.5*(imupose.imu_gyro * dt /2.0+lastimupose.imu_gyro*dt/2);
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();
/* 
        // imu 动力学模型 欧拉积分
        Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
        Qwb = Qwb * dq;
        Vw = Vw + acc_w * dt;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
*/
        //中值积分
        Qwb = Qwb_last * dq;
        Eigen::Vector3d acc_w = 0.5*(Qwb * (imupose.imu_acc) + gw + Qwb_last*(lastimupose.imu_acc)+gw); 
        Vw = Vw + acc_w * dt;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
        Qwb_last = Qwb;
        lastimupose = imupose;

        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points<<imupose.timestamp<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<std::endl;
    }
    std::cout<<"test　end"<<std::endl;
}
```
最后结果对比图如下所示，第一个为欧拉积分，第二个为中值积分：
<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/IMU传感器/result.png"  width="320" >
</div>

<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/IMU传感器/zhongzhi.png" width="320">
</div>



