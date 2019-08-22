---
# 常用定义
title: "VIO概述与基本知识"        # 标题
date: 2019-06-09               # 创建时间
lastmod: 2019-06-09            # 最后修改时间
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

习题一：VIO 文献阅读
--------------------------------
阅读`VIO`相关综述文献，回答以下问题：

- 视觉与`IMU`进行融合之后有何优势？
- 有哪些常见的视觉+`IMU`融合方案？有没有工业界应用的例子？
- 在学术界，`VIO`研究有哪些新进展？有没有将学习方法用到`VIO`中的例子？
你也可以对自己感兴趣的方向进行文献调研,阐述你的观点。

### 答案
1、视觉与`IMU`进行融合之后的优势：

- IMU适合计算短时间、快速的运动，视觉适合计算长时间、慢速的运动。融合之后，可以利用视觉定位信息来估计`IMU`的零偏，减少`IMU`由零偏导致的发散和累计误差。反之，`IMU`可以为视觉提供快速运动时的定位；
- 单目视觉`SLAM`算法存在一些本身框架无法克服的缺陷，首先是尺度的问题；同时视觉`SLAM`一般采取第一帧作为世界坐标系，视觉方法估计的位姿不能和重力方向对齐。通过引入 IMU 信息可以很好地解决上述问题。
- 融合`IMU`和视觉信息的`VINS`算法可以很大程度地提高单目`SLAM`算法性能，是一种低成本高性能的导航方案。

2、视觉和`IMU`的融合问题可以分成基于滤波和基于优化的两大类，同时按照是否把图像特征信息加入状态向量进行分类，可以分为松耦合和紧耦合两大类；  
松耦合将视觉传感器和`IMU`作为两个单独的模块，两个模块均可以计算得到位姿信息，然后一般通过`EKF`进行融合。  
紧耦合则是指将视觉和`IMU`得到的中间数据通过一个优化滤波器进行处理，紧耦合需要把图像特征加入到特征向量中，最终得到位姿信息的过程。由于这个原因，系统状态向量最终的维度也会非常高，同时计算量也很大。  
总体来看，松耦合算法相对简单，且扩展性很强，可以加入多种传感器信息进行融合，但融合得到的位姿估计精度相对较差；紧耦合算法实现起来相对复杂，可扩展性差，但融合得到的位姿估计精度较高。

- 基于滤波的松耦合：可参考的算法有ethz的Stephen Weiss的ssf和msf
- 基于滤波的紧耦合：MSCKF，ROVIO
- 基于优化的松耦合：< Inertial Aided Dense & Semi-Dense Methods for Robust Direct Visual Odometry >
- 基于优化的紧耦合：okvis，VINS

其他：  
`VIO`目前实现比较好的是VINS、okvis和MSCKF。前两个目前开源，最后一个目前没有开源，同时，MSCKF也是Google Tango上使用的方法。同时，`VIO`在工业上的应用还有苹果的`ARKit`和谷歌的`ARCore`。

3、目前传统方法：

- RGBD+IMU，基于VINS的方法  
RGBD-Inertial Trajectory Estimation and Mapping for Ground Robots
- 低功耗版的VINS  
Visual Inertial Odometry At the Edge: A Hardware-Software Co-design Approach for Ultra-low Latency and Power
- 在VIO中根据三角剖分加入结构化特征  
Incremental Visual-Inertial 3D Mesh Generation with Structural Regularities

基于学习的方法：  

- 学会了在没有惯性测量单元（IMU）内在参数或IMU和摄像机之间的外部校准的情况下执行视觉惯性里程计（VIO）  
Unsupervised Deep Visual-Inertial Odometry with Online Error Correction for RGB-D Imagery.

- 论文集中在如何学习多传感器融合策略上。提出了一种针对单目VIO的端到端的多传感器选择融合策略。  
Selective Sensor Fusion for Neural Visual-Inertial Odometry.
 
- 这篇文章针对传统的VIO需要进行标定的问题，提出一种网络，可以不需要标定  
Visual-Inertial Odometry for Unmanned Aerial Vehicle using Deep Learning


习题二：四元数和李代数更新
--------------------------------
课件提到了可以使用四元数或旋转矩阵存储旋转变量。当我们用计算出来的$ ω $对某旋转更新时，有两种不同方式：

$$
\begin{array}{l}{\mathbf{R} \leftarrow \mathbf{R} \exp \left(\boldsymbol{\omega}^{\wedge}\right)} \end{array}
$$

$$
\begin{array} {l}{\mathbf{q} \leftarrow \mathbf{q} ⊗ \left[1, \frac{1}{2} \boldsymbol{\omega}\right]^{\mathrm{T}}} \end{array}
$$

请编程验证对于小量 $ ω = [0.01, 0.02, 0.03]^T $，两种方法得到的结果非常接近，实践当中可视为等同。因此，在后文提到旋转时，我们并不刻意区分旋转本身是$ q $还是$ R $，也不区分其更新方式为上式的哪一种。

### 答案

程序计算旋转矩阵的更新有两种方式，第一种是使用了`Sophus`库，第二种是只使用`Eigen`库。完整的工程见附件；
最终结果为：

<div style="text-align: center">
<img src="../images/VIO概述与基本知识/result.png"/>
</div>

从中可以看到，两种方法得到的结果非常接近。同时使用`Sophus`和未使用的结果一样。
相关代码：
```
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d ( 0,0,1 ) );     //沿 Z 轴旋转 45 度
    rotation_matrix = rotation_vector.toRotationMatrix();
    // cout<<"rotation matrix =\n"<<rotation_matrix<<endl;

    Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );
    // cout<<"quaternion = \n"<<q.coeffs().transpose() <<endl;   

    Eigen::Quaterniond q_add(1,0.005,0.01,0.015);
    q = q*q_add;
    cout<<"quaternion update = \n"<<q.coeffs().transpose() <<endl;

    Eigen::Vector3d update(0.01,0.02,0.03);
    Sophus::SO3 SO3_R(rotation_matrix);
    Sophus::SO3 SO3_update = SO3_R*Sophus::SO3::exp(update);

    cout<<"rotation matrix update = \n"<<SO3_update.matrix()<<endl;
    cout<<"R from q  \n"<<q.normalized().toRotationMatrix()<<endl;
    cout<<"q frome R  \n"<<Eigen::Quaterniond(SO3_update.matrix()).coeffs().transpose()<<endl;

    //不使用李代数库进行旋转矩阵的更新
    double theat = update.norm();
    Eigen::Vector3d update_norm = update.normalized();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d update_m = Eigen::Matrix3d::Zero();
    update_m(0,1) = -update_norm[2];
    update_m(0,2) = update_norm[1];
    update_m(1,0) = update_norm[2];
    update_m(1,2) = -update_norm[0];
    update_m(2,0) = -update_norm[1];

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    
    R = rotation_matrix*(cos(theat)*I+(1-cos(theat))*update_norm*update_norm.transpose()+sin(theat)*update_m);

    cout<< "R only use eigen  \n"<<R<<endl;
```


习题三：其他导数
--------------------------------
使用右乘 $ so(3) $,推导以下导数:

<p>
$$
\begin{array}{c}{\frac{\mathrm{d}\left(\mathbf{R}^{-1} \mathbf{p}\right)}{\mathrm{d} \mathbf{R}}} 
\\ {\frac{\mathrm{d} \ln \left(\mathbf{R}_{1} \mathbf{R}_{2}^{-1}\right)}{\mathrm{d} \mathbf{R}_{2}}}\end{array}
$$
</p>

### 答案

第一个式子证明：（右扰动）

<p>
$$
\begin{aligned} \frac{\partial\left(\mathbf{R}\mathbf{p}\right)}{\partial \varphi} &=\lim _{\varphi \rightarrow 0} \frac{(\mathbf{R} \exp \left(\boldsymbol{\varphi}^{\wedge}\right))^{-1} \mathbf{p}-\mathbf{R^{-1}} \mathbf{p}}{\varphi} 
\\ &=\lim _{\varphi \rightarrow 0} \frac{(\exp \left(\boldsymbol{\varphi}^{\wedge}\right))^{-1} \mathbf{R}^{-1} \mathbf{p}-\mathbf{R^{-1}} \mathbf{p}}{\varphi} 
\\ &=\lim _{\varphi \rightarrow 0} \frac{\left(\mathbf{I}+\varphi^{\wedge}\right)^{-1} \mathbf{R}^{-1} \mathbf{p}-\mathbf{R}^{-1} \mathbf{p}}{\varphi} 
\\ &=\lim _{\varphi \rightarrow 0} \frac{\left(\mathbf{I}+\varphi^{\wedge}\right)^{-T} \mathbf{R}^{-1} \mathbf{p}-\mathbf{R}^{-1} \mathbf{p}}{\varphi} 
\\ &=\lim _{\varphi \rightarrow 0} \frac{\left(\mathbf{I}-\varphi^{\wedge}\right) \mathbf{R}^{-1} \mathbf{p}-\mathbf{R}^{-1} \mathbf{p}}{\varphi} 
\\ &=\lim _{\varphi \rightarrow 0} \frac{-\varphi^{\wedge} \mathbf{R}^{-1} \mathbf{p}}{\varphi} = \lim _{\varphi \rightarrow 0} \frac{(\mathbf{R}^{-1} \mathbf{p})^{\wedge} \varphi}{\varphi} = (\mathbf{R}^{-1} \mathbf{p})^{\wedge} \end{aligned}
$$
</p>

第二个式子证明：

<p>
$$
\begin{aligned} \frac{\mathrm{d} \ln \left(\mathbf{R}_{1} \mathbf{R}_{2}^{-1}\right)}{\mathrm{d} \mathbf{R}_{2}} &= \lim _{\phi \rightarrow 0} \frac{\ln \left(\mathbf{R}_{1} (\mathbf{R}_{2} \exp \left(\boldsymbol{\phi}^{\wedge}\right))^{-1}\right)-\ln \left(\mathbf{R}_{1} \mathbf{R}_{2}^{-1}\right)}{\phi} 
\\ &= \lim _{\phi \rightarrow 0} \frac{\ln \left(\mathbf{R}_{1} (\exp \left(\boldsymbol{\phi}^{\wedge}\right))^{-1} \mathbf{R}_{2}^{-1}\right)-\ln \left(\mathbf{R}_{1} \mathbf{R}_{2}^{-1}\right)}{\phi} 
\\ &= \lim _{\phi \rightarrow 0} \frac{\ln \left(\mathbf{R}_{1} (\exp \left(\boldsymbol{(-\phi)}^{\wedge}\right)) \mathbf{R}_{2}^{-1}\right)-\ln \left(\mathbf{R}_{1} \mathbf{R}_{2}^{-1}\right)}{\phi} 
\\ &= \lim _{\phi \rightarrow 0} \frac{\ln \left(\mathbf{R}_{1} \mathbf{R}_{2}^{-1} \exp \left(\boldsymbol{(-\mathbf{R}_{2}\phi)}^{\wedge}\right)\right)-\ln \left(\mathbf{R}_{1} \mathbf{R}_{2}^{-1}\right)}{\phi} 
\\ &=\lim _{\phi \rightarrow 0} \frac{\ln \left(\mathbf{R}_{1} \mathbf{R}_{2}^{-1}\right)+\mathbf{J}_{r}^{-1}\ln \left(\mathbf{R}_{1} \mathbf{R}_{2}^{-1}\right) \boldsymbol{(-(\mathbf{R}_{2} \phi))}-\ln \left(\mathbf{R}_{1} \mathbf{R}_{2}^{-1}\right)}{\phi}  
\\ &=-\mathrm{J}_{r}^{-1}\left(\ln \left(\mathbf{R}_{1} \mathbf{R}_{2}^{-1}\right)\right)\mathbf{R}_{2} \end{aligned}
$$
</p>

- 其中第三行到第四行使用了`SO(3)`的伴随性质
- 第四行到第五行是`BCH`近似；
- 上述推到中，旋转矩阵的逆为旋转矩阵的转置，这一点未具体说明。