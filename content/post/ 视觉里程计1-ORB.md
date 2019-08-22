---
# 常用定义
title: "视觉里程计1-ORB"           # 标题
date: 2019-05-06                 # 创建时间
lastmod: 2019-05-08              # 最后修改时间
draft: false                     # 是否是草稿？
tags: ["SLAM","十四讲"]           # 标签
categories: ["SLAM十四讲"]            # 分类
author: "Yang"                   # 作者

# 用户自定义
# 你可以选择 关闭(false) 或者 打开(true) 以下选项
comment: true   # 打开评论
toc: true       # 打开文章目录
# 你同样可以自定义文章的版权规则
contentCopyright: '<a rel="license noopener" href="https://creativecommons.org/licenses/by-nc-nd/4.0/" target="_blank">CC BY-NC-ND 4.0</a>'
reward: false	 # 关闭打赏
mathjax: false    # 关闭 mathjax
---

视觉里程计1
============================
习题一：ORB特征点
---------------------------
ORB(Oriented FAST and BRIEF)特征是SLAM中一种很常用的特征,由于其二进制特性,使得它可以非常快速地提取与计算。下面,你将按照本题的指导,自行书写ORB的提取、描述子的计算以及匹配的代码。代码框架参照computeORB.cpp文件，图像见1.png文件和2.png。
### 1.1 ORB提取 
ORB即Oriented FAST简称。它实际上是FAST特征再加上一个旋转量。本习题将使用OpenCV自带的FAST提取算法,但是你要完成旋转部分的计算。旋转的计算过程描述如下：  
在一个小图像块中，先计算质心。质心是指以图像块灰度值作为权重的中心。

1.在一个小的的图像块B中，定义图像块的距为：

<p>
$$
m_{pq} = \sum_{x,y\in B}x^py^qI(x,y),p,q=\begin{Bmatrix} 0,&1 \end{Bmatrix}
$$
</p>

2.通过矩可以找到图像块的质心：

<p>
$$
C=(\frac{m_{10}}{m_{00}},\frac{m_{01}}{m_{00}})
$$
</p>

3.连接图像块的几何中心$O$和质心$C$，可以得到一个方向向量$\overrightarrow{OC}$，于是特征点的方向可以定义为：

<p>
$$
\theta = arctan(\frac{m_{01}}{m_{10}})
$$
</p>

实际上只需计算$m_ {01}$和$m_{10}$即可。习题中取图像块大小为$16\times16$，即对于任意点$(u,v)$，图像块从$(u−8,v−8)$取到$(u+7,v+7)$即可。请在习题的computeAngle中，为所有特征点计算这个旋转角。
#### 提示
1. 由于要取图像$16\times16$块，所以位于边缘处的点(比如$u<8$的)对应的图像块可能会出界,此时需要判断该点是否在边缘处,并跳过这些点。
2. 由于矩的定义方式,在画图特征点之后，角度看起来总是指向图像中更亮的地方。
3. `std::atan`和`std::atan2`会返回弧度制的旋转角，但OpenCV中使用角度制，如使用`std::atan `类函数,请转换一下。
4. Opencv中row、col和Point中的x、y是相反的，这个需要注意，即Mat是行列的顺序，point是列行的顺序。
```
    row == heigh == Point.y //行
    col == width == Point.x //列
    Mat::at(Point(x, y)) == Mat::at(y,x)
```
5. 计算质心的公式中的`x，y`是$-8,7$，不是图像中关键点的坐标，这一点需要注意，下面计算描述子的时候也是一样的。

### 1.2 ORB描述 
ORB描述即带旋转的BRIEF描述。所谓BRIEF描述是指一个$0-1$组成的字符串(可以取256位或128位)，每一个`bit`表示一次像素间的比较。算法流程如下:  

1.给定图像$I$和关键点 $(u,v)$ ，以及该点的转角$θ$。以256位描述为例，那么最终描述子

<p>
$$
d = [d_1,d_2, \cdots , d_{256} ]
$$
</p>

2.对任意$i=1,\cdots,256,d_i$的计算如下。取$(u,v)$附近任意两个点$p,q$，并按照$θ$进行旋转:

<p>
$$
\begin{bmatrix} u_p^{'}  \\ v_p^{'} \\ \end{bmatrix} = 
\begin{bmatrix} cosθ & −sinθ \\ sinθ & cosθ \\ \end{bmatrix} 
\begin{bmatrix} u_p  \\ v_p \\ \end{bmatrix}
$$
</p>

其中$u_p,v_p$为$p$的坐标，对$q$亦然。记旋转后的$p,q$为$p^′,q^′$，那么比较$I(p^′)$和$I(q^′)$，若前者大，记$d_i=0$，反之记$d_i=1$。  

这样我们就得到了ORB的描述。我们在程序中用256个`bool`变量表达这个描述。请你完`compute-ORBDesc` 函数,实现此处计算。注意，通常我们会固定$p,q$的取法(称为`ORB`的`pattern`)，否则每次都重新随机选取，会使得描述不稳定。我们在全局变量`ORB_pattern`中定义了$p,q$的取法，格式为$u_p,v_p,u_q,v-q$。请你根据给定的`pattern`完成`ORB`描述的计算。

#### 提示
1. $p,q$同样要做边界检查，否则会跑出图像外。如果跑出图像外，就设这个描述子为空。
2. 调用`cos`和`sin`时同样请注意弧度和角度的转换。  

### 1.3 暴力匹配
在提取描述之后，我们需要根据描述子进行匹配。暴力匹配是一种简单粗暴的匹配方法，在特征点不多时很有用。下面你将根据习题指导,书写暴力匹配算法。  
所谓暴力匹配思路很简单。给定两组描述子$P=[p_1, \cdots, p_M]$和$Q=[q_1, \cdots,q_N]$。那么,对$P$中任意一个点,找到$Q$中对应最小距离点，即算一次匹配。但是这样做会对每个特征点都找到一个匹配,所以我们通常还会限制一个距离阈值$d_max$，即认作匹配的特征点距离不应该大于$d_max$。下面请你根据上述描述,实现函数`bfMatch`，返回给定特征点的匹配情况。实践中取$d_max = 50$。
最后,请结合实验,回答下面几个问题:

1. 为什么说 ORB 是一种二进制特征？
2. 为什么在匹配时使用 50 作为阈值，取更大或更小值会怎么样？
3. 暴力匹配在你的机器上表现如何?你能想到什么减少计算量的匹配方法吗？

#### 提示

1. 你需要按位计算两个描述子之间的汉明距离。汉明距离的定义是两个描述子中不相同的个数。
2. OpenCV的`DMatch`结构，`queryIdx`为第一图的特征ID，`trainIdx`为第二个图的特征 ID。
3. 在这个函数中用到了`sort`函数对二维数组的排序操作，`sort`函数可以制定维度对多维数组进行排序，未指定则默认为第一维。

### 答案

最前面先回答实验的问题：

1. ORB 的描述子是用许多对像素点坐标(此处坐标需要经过旋转校正，就是第二问用到的公式)所在位置的灰度值的大小对比来组成,对比结果为0和1，假设有256对像素点，那么ORB的特征点描述子便有256个0或1组成,因此称ORB是一种二进制特征;
2. ORB 匹配时是计算两个描述子之前的汉明距离,即计算两个二进制值在每一位上不相等的个数,当然我们希望这个不相等的个数越少越好,当个数为 0,则说明两个描述子的特征完全一致,随着个数越多,说明两个特征不一致的程度也就越高,因此若将阈值取小,则匹配点对的数量越少,但是误匹配几率越小,反之,若将阈值调高,则匹配出的点对数量增加,但误匹配几率也增加。
3. 匹配117对点。耗时为3.964秒。若在 CMakeLists 中增加-O2 编译优化,则耗时可减小至0.19秒。若改为-O3 编译优化,耗时可继续减小至0.186221秒。  
另外一种办法是减少计算量,可以使用 FLANN 快速近似最近邻算法。    
这个具体是编译优化的问题，在`Visual Studio`中我们可以生成`debug`版和`release`版的程序,使用`CMake`我们也可以达到上述效果。debug 版的项目生成的可执行文件需要有调试信息并且不需要进行优化,而 release 版的不需要调试信息但需要优化。这些特性在 gcc/g++ 中是通过编译时的参数来决定的,如果将优化程度调到最高需要设置参数`-O3`，最低是`-O0`，即不做优化；添加调试信息的参数是`-g -ggdb`，如果不添加这个参数，调试信息就不会被包含在生成的二进制文件中。


最终结果如下：
<div style="text-align:center">
<img src="../images/视觉历程计-ORB/feat1.png">
</div>

匹配结果如下：
<div style="text-align:center">
<img src="../images/视觉历程计-ORB/matches.png">
</div>

完整程序链接：https://github.com/XLMaverick/Visual-Localization-Percessing/tree/master/vslam_fourteen_lectures/computeORB   

1.ORB增加旋转的最终结果如下：再强调一遍注意看第一问的提示部分。

```
    void computeAngle(const cv::Mat &image, vector<cv::KeyPoint> &keypoints) 
    {
        int half_patch_size = 8;
        for(auto& kp:keypoints)
        {
            int u = kp.pt.x;
            int v = kp.pt.y;
            if(u-8<0 || u+8>=image.cols || v-8<0 || v+8 >= image.rows)
            {
                cout<<"Keypoint is out of range"<<kp.pt<<endl;
                continue;
            }
            int m01 = 0;
            int m10 = 0;
            for(int j= -8;j<8;j++)
            {
                for(int i=-8;i<8;i++)
                {
                    m01 += j*image.at<uchar>(v+j,u+i);
                    m10 += i*image.at<uchar>(v+j,u+i); 
                }
            }
            kp.angle = (float)atan(m01/m10)*180/pi;
        }
        return;
    }
```

2.编写ORB描述子的代码如下：
```
    void computeORBDesc(const cv::Mat &image, vector<cv::KeyPoint> &keypoints, vector<DescType> &desc) 
    {
        for (auto &kp: keypoints) 
        {
            DescType d(256, false);
            for (int i = 0; i < 256; i++) 
            {
                int up = ORB_pattern[i*4];
                int vp = ORB_pattern[i*4+1];
                int uq = ORB_pattern[i*4+2];
                int vq = ORB_pattern[i*4+3];

                int up1 = up*cos(kp.angle/180*pi)-vp*sin(kp.angle/180*pi) + kp.pt.x;
                int vp1 = up*sin(kp.angle/180*pi)+vp*cos(kp.angle/180*pi) + kp.pt.y;

                int uq1 = uq*cos(kp.angle/180*pi)-vq*sin(kp.angle/180*pi) + kp.pt.x;
                int vq1 = uq*sin(kp.angle/180*pi)+vq*cos(kp.angle/180*pi) + kp.pt.y;

                if(up1<0 || up1>=image.cols || vp1<0 || vp1>=image.rows 
                || uq1<0 || uq1>=image.cols || vq1<0 || vq1>=image.rows)
                {
                    d.clear();
                    break;
                }
                else
                {
                    d[i] =( image.at<uchar>(vp1,up1)>image.at<uchar>(vq1,uq1) )?0:1;
                }
            }
            desc.push_back(d);  
        }

        int bad = 0;
        for (auto &d: desc) {
            if (d.empty()) bad++;
        }
        cout << "bad/total: " << bad << "/" << desc.size() << endl;
        return;
    }
```
3.暴力匹配的代码如下：
```
    void bfMatch(const vector<DescType> &desc1, const vector<DescType> &desc2, vector<cv::DMatch> &matches) {
        int d_max = 50;
        int d1_num = -1;
        int time_stt = clock();
        for(auto &d1:desc1)
        {
            d1_num++;
            if(d1.empty())
            {
                continue;
            }
            //之所以使用二维数组，是因为不仅需要记录距离来排序，同时还要记录下标。
            vector<vector<int>> d1_match(0,vector<int>(2));
            int d2_num = -1;
        
            for(auto &d2:desc2)
            {
                d2_num++;
                if(d2.empty())
                {
                    continue;
                }
                int HammingDis = 0;//注意是d2每一次需要为0.
                vector<int> d2_hamming(2);
                for(int i = 0;i<256;i++)
                {
                    HammingDis += (d1[i] == d2[i])?0:1;
                }
                d2_hamming = {HammingDis, d2_num};
                d1_match.push_back(d2_hamming);
            }
            sort(d1_match.begin(), d1_match.end());
            if(d1_match[0][0]<d_max)
            {
                cv::DMatch m;
                m.queryIdx = d1_num;
                m.trainIdx = d1_match[0][1];
                m.distance = d1_match[0][0];
                matches.push_back(m);
            }
        }
        int time_end = clock();
        cout<<"time is "<<(time_end-time_stt)/(double)CLOCKS_PER_SEC<<"s"<<endl;
        for (auto &m: matches) {
            cout << m.queryIdx << ", " << m.trainIdx << ", " << m.distance << endl;
        }
        return;
    }

```

