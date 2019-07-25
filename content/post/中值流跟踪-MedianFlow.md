---
# 常用定义
title: "Median Flow tracker 中值流跟踪"           # 标题
date: 2019-07-15    # 创建时间
lastmod: 2019-07-15 # 最后修改时间
draft: false                       # 是否是草稿？
tags: [“cv”]  # 标签
categories: ["cv"]              # 分类
author: "Yang"                  # 作者

# 用户自定义
# 你可以选择 关闭(false) 或者 打开(true) 以下选项
comment: true   # 打开评论
toc: true       # 打开文章目录
# 你同样可以自定义文章的版权规则
contentCopyright: '<a rel="license noopener" href="https://creativecommons.org/licenses/by-nc-nd/4.0/" target="_blank">CC BY-NC-ND 4.0</a>'
reward: false	 # 关闭打赏
mathjax: false    # 关闭 mathjax
---

Median Flow tracker 中值流跟踪
---------------------------------------
中值流跟踪算法，是属于`TLD`跟踪算法中的`Tracking`部分。它是基于`LK`光流跟踪算法的基础上，使用`FB`（正向、反向评估点追踪质量的一种方法）以及NCC交叉验证进行评估反馈。`TLD`的跟踪模块在这个基础上增加了失败检测。中值流跟踪方法利用目标框`(Bounding box)`来表示被跟踪的目标，并在连续的相邻视频帧之间估计目标的运动。
当然，相应的算法已经在`opencv`的`contrib`进行了集成，[contrib\tracking\src](https://github.com/opencv/opencv_contrib/tree/master/modules/tracking/src)中，还有其他的相关的跟踪算法，可以选择使用，同时，还有多目标的跟踪函数接口。这里主要记录和理解其中的一个，其余的可以上述的连接中查看。
具体的流程如下：
1. 先在选中的`box`中均匀采样100个特征点（网格均匀撒点），存于`points`中，函数为`TLD::bbPoints`:
```
    //将bb切成10*10的网格，将网格交点存在points
    void TLD::bbPoints(vector<cv::Point2f>& points,const BoundingBox& bb)
    {
        int max_pts=10;
        int margin_h=0;
        int margin_v=0;
        int stepx = ceil((bb.width-2*margin_h)/max_pts);//取整
        int stepy = ceil((bb.height-2*margin_v)/max_pts);
        for (int y=bb.y+margin_v;y<bb.y+bb.height-margin_v;y+=stepy)
        {
            for (int x=bb.x+margin_h;x<bb.x+bb.width-margin_h;x+=stepx)
            {
                points.push_back(Point2f(x,y));//最多有11*11=121个点
            }
        }
    }
```
2. 利用[金字塔LK光流法](https://www.xlmaverick.me/post/%E8%A7%86%E8%A7%89%E9%87%8C%E7%A8%8B%E8%AE%A12-lk/)跟踪这些特征点，并预测当前帧的特征点。然后计算`FB error`和匹配相似度`sim`，然后筛选出`FB_error[i] <= median(FB_error)和sim_error[i] > median(sim_error)`的特征点（舍弃跟踪结果不好的特征点），剩下的是不到50%的特征点。上述的三部均在函数`tracker.trackf2f(img1, img2, points, points2)`实现。
```
    //points1->points2，由于调用了filterPts，所以只有通过筛选的point对还保留在points1，points2
    bool LKTracker::trackf2f(const Mat& img1, const Mat& img2,vector<Point2f> &points1, vector<cv::Point2f> &points2)
    {
        //先利用金字塔LK光流法跟踪预测前向轨迹
        calcOpticalFlowPyrLK( img1,img2, points1, points2, status, similarity, window_size, level, term_criteria, lambda, 0);
        //再往回跟踪，产生后向轨迹
        calcOpticalFlowPyrLK( img2,img1, points2, pointsFB, FB_status,FB_error, window_size, level, term_criteria, lambda, 0);
        //然后计算 FB-error：前向与 后向 轨迹的误差
        for( int i= 0; i<points1.size(); ++i )
        {
            FB_error[i] = norm(pointsFB[i]-points1[i]);
        }  
        //再从前一帧和当前帧图像中（以每个特征点为中心）使用亚象素精度提取10x10象素矩形（使用函数getRectSubPix得到），匹配前一帧和当前帧中提取的10x10象素矩形，得到匹配后的映射图像（调用matchTemplate），得到每一个点的NCC相关系数（也就是相似度大小）。
        normCrossCorrelation(img1, img2, points1, points2);
        //然后筛选出 FB_error[i] <= median(FB_error) 和 sim_error[i] > median(sim_error)的特征点（舍弃跟踪结果不好的特征点），剩下的是不到50%的特征点
        return filterPts(points1, points2);
    }
```
3. 利用剩下的这不到一半的跟踪点输入来预测`bounding box`在当前帧的位置和大小，其中`points`和`points2`是前面筛选完之后的点对，现在要依据`points`，`points2`来估计`bb1`的位移和尺度变化，这两个信息都有了，自然可以决定`lastbox`在当前帧的位置`tbb`。具体的实现函数如下：
```
    bbPredict(points, points2, lastbox, tbb);
```
4. 跟踪失败检测：如果`FB error`的中值大于10个像素（经验值），或者预测到的当前`box`的位置移出图像，则认为跟踪错误，此时不返回`bounding box`：
```
    if (tracker.getFB()>10 || tbb.x>img2.cols ||  tbb.y>img2.rows || tbb.br().x < 1 || tbb.br().y <1)
```
5. 归一化`img2(bb)`对应的`patch`的`size`（放缩至`patch_size = 15*15`），存入`pattern`：
```
    getPattern(img2(bb),pattern,mean,stdev);
```
6. 计算图像片`pattern`到在线模型`M`的保守相似度：
```
    classifier.NNConf(pattern,isin,dummy,tconf);
```
7. 如果保守相似度大于阈值，则评估本次跟踪有效，否则跟踪无效：
```
    if (tconf>classifier.thr_nn_valid) tvalid =true;
```
后面5,6,7对应的详细的代码为：
```
    Mat pattern;
    Scalar mean, stdev;
    BoundingBox bb;
    bb.x = max(tbb.x,0);
    bb.y = max(tbb.y,0);
    bb.width = min(min(img2.cols-tbb.x,tbb.width),min(tbb.width,tbb.br().x));
    // bb.height = min(min(img2.rows-tbb.y,tbb.height),min(tbb.height,tbb.br().y));
    getPattern(img2(bb),pattern,mean,stdev);
    vector<int> isin;
    float dummy;
    classifier.NNConf(pattern,isin,dummy,tconf); //1.tconf是用Conservative Similarity
    tvalid = lastvalid; 
    if (tconf>classifier.thr_nn_valid){//thr_nn_valid
    tvalid =true;//2.判定轨迹是否有效，从而决定是否要增加正样本，标志位tvalid
```
