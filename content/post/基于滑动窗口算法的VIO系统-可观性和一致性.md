---
# 常用定义
title: "基于滑动窗口的VIO系统-可观性和一致性"        # 标题
date: 2019-07-14               # 创建时间
lastmod: 2019-07-14            # 最后修改时间
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

习题一：信息矩阵的绘制
--------------------------------
某时刻，SLAM系统中相机和路标点的观测关系如下图所示，其中 $\xi$ 表示相机的位姿，$L$ 表示观测到的路标点。当路标点 $L$ 表示在世界坐标系下时，第 $k$ 个路标被第 $i$ 时刻的相机观测到，重投影的误差为 $\mathbf{r}\left(\xi_ {i}, L_ {k}\right)$。

<div align=center>
<img src="../images/基于滑动窗口算法的VIO系统-可观性和一致性/picture1.png"  >
</div>

1. 请绘制上述系统的信息矩阵$\Lambda$。
2. 请绘制相机 $\xi_{1}$ 被 $marg$ 以后的信息矩阵 $\mathbf{\Lambda}^{\prime}$。

### 答案：

1. 上述系统的信息矩阵$\Lambda$如下图所示：
<div align=center>
<img src="../images/基于滑动窗口算法的VIO系统-可观性和一致性/result2.png",width = "300" height = "200" >
</div>

2. 相机 $\xi_{1}$ 被 $marg$ 以后的信息矩阵 $\mathbf{\Lambda}^{\prime}$如下图所示：

<div align=center>
<img src="../images/基于滑动窗口算法的VIO系统-可观性和一致性//result3.png",width = "300" height = "200" >
</div>


习题二：单目BA信息矩阵
--------------------------------
请补充作业代码中单目`Bundle Adjustment`信息矩阵的计算，并输出正确的结果。正确的结果为：奇异值最后$7$维接近$0$，表明零空间的维度为$7$。

### 答案

增加H矩阵的右下角部分和右上的计算两部分，完整的代码如下：

```
    H.block(i*6,i*6,6,6) += jacobian_Ti.transpose() * jacobian_Ti;
    H.block(j*3 + 6*poseNums,j*3 + 6*poseNums,3,3) +=jacobian_Pj.transpose() * jacobian_Pj;
    H.block(i*6,j*3 + 6*poseNums, 6,3) += jacobian_Ti.transpose() * jacobian_Pj;
    H.block(j*3 + 6*poseNums,i*6 , 3,6) += jacobian_Pj.transpose() * jacobian_Ti;
```
最终奇异值分解的结果如下，我们可以看到，奇异值最后$7$维接近$0$，表明零空间的维度为$7$。

<div align=center>
<img src="../images/基于滑动窗口算法的VIO系统-可观性和一致性/result1.png" >
</div>
