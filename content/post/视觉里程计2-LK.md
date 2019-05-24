---
# 常用定义
title: "视觉里程计2-LK"           # 标题
date: 2019-05-16    # 创建时间
lastmod: 2019-05-20 # 最后修改时间
draft: false                       # 是否是草稿？
tags: ["slam","十四讲"]  # 标签
categories: ["slam"]              # 分类
author: "Yang"                  # 作者

# 用户自定义
# 你可以选择 关闭(false) 或者 打开(true) 以下选项
comment: true   # 打开评论
toc: true       # 打开文章目录
# 你同样可以自定义文章的版权规则
contentCopyright: '<a rel="license noopener" href="https://creativecommons.org/licenses/by-nc-nd/4.0/" target="_blank">CC BY-NC-ND 4.0</a>'
reward: false	 # 关闭打赏
mathjax: false    # 关闭mathjax
---

视觉里程计2
============================
习题一：LK光流法
---------------------------

### 1.1 光流文献综述
我们课上演示了`Lucas-Kanade`稀疏光流，用`OpenCV`函数实现了光流法追踪特征点。实际上，光流法有很长时间的研究历史，直到现在人们还在尝试用`Deep learning`等方法对光流进行改进。本题将指导你完成基于`Gauss-Newton`的金字塔光流法。首先，请阅读文献，回答下列问题。
问题:

1. 按此文的分类,光流法可分为哪几类?
2. 在`compositional`中，为什么有时候需要做原始图像的`wrap`？该`wrap`有何物理意义?
3. `forward`和`inverse`有何差别?

#### 答案：

1.围绕Image Alignment，文章总共介绍了四种方法，分别是`FAIA（Forward Additional Image Alignment）`，`FCIA（Forward Composition Image Alignment）`，`ICIA(Inverse Compositional Image Alignment`)和`IAIA（Inverse Additional Image Alignment）`。其中LK对应上述四种中的`FAIA`，`ICIA`使用于直接法SVO中块匹配方法。  

| 增量方式\更新方式 | forward   |  inverse |
| -------        | -----:    | :----:   |
| additive       | FAIA      | IAIA     |
| compositional    | FCIA      | ICIA   |

2.首先，我们区分一下`compositional`和`additive`两种方法，其中：如果迭代的结果是在原始的值（6个运动参数）上增加一个微小量，那么这种方法成为`additive`；如果在放射矩阵上乘以一个矩阵（增量运动参数形成的增量放射矩阵），这种方法称为`compositional`。这两种方法理论上是等效的，计算量也差不多。  
针对`compositional`的两种计算方式`FCIA`，`ICIA`，都需要在当前位姿估计之前引入增量式`warp`以建立半群约束要求。  
FCIA：`warp`集合包含`identity warp`，`warp`集合包含在`compositional`操作上是闭的（`semi-group`），其中包含`Homograph，3D rotation`等。  
ICIA：`semi-group`，另外要求增量`warp`可逆，其中包括`Homograph，3D rotation`等，但是不包括`piece wise affine`。
`warp`的物理意义：对图像做微小的平移或者仿射变换。  
3.前向（`forward`）和后向（`inverse`）的对比：  
前向方法对于输入图像进行参数化（包括放射变换以及放射增量）。后向方法则是同时参数化输入图像和模板图像，其中输入图像参数化仿射变换，模板图像参数化放射增量。因此后向方法的计算量显著降低。由于图像灰度值和运动参数非线性化，整个优化过程为非线性的。  
参数化过程主要计算：图像的梯度，位置对运动参数导数，运动参数增量。前向方法中`Hessian`是运动参数的函数，提高效率的主要思想是交换模板图像和输入图像的角色；后向方法中，迭代中的`Hessian`是固定的。  
前向方法和后向方法在目标函数上不太一样，一个是把运动向量$p$都是跟着$I$（被匹配的图像），但是前向方法中的迭代的微小的量$Δp$使用I计算，后向方法中的$Δp$使用$T$计算，这样计算量变小。
#### 题记
1. 两个前向方法的计算复杂度相似，后向方法几乎相等。后向方法的速度远远比前向方法要快；
2. 前向`additive`可以用于任何变形`warp`；
3. 反向`compositional`只能用于`wraps that form groups`；
4. 反向`additive`可以用于`simple 2D linear warps such as translations and affine warps`。
5. 如果不考虑效率的话，可以使用两种前向方法。前向`compositional`的方法中`Jacobian`是常量，因此具有一定的优势；
6. 如果考虑效率的话，那么后向`compositional`方法是首选，推导简单，很容易确定；
7. `Jacobian`矩阵和残差计算的方式有关，由于`compositional`计算误差的方式会使得雅克比矩阵为常数，通常采用`compositional`；
8. 最后的最后，推荐一篇博客，对LK光流的解释以及对相应公式的推导均写的较为详细 https://www.twblogs.net/a/5b8190582b71772165ad3c9b  
https://blog.csdn.net/wendox/article/details/52505971  

### 1.2 `forward-addtive Gauss-Newton`光流的实现
接下来我们来实现最简单的光流，即上文所说的`forward-addtive`。我们先考虑单层图像的`LK`光流，然后再推广至金字塔图像。按照教材的习惯，我们把光流法建模成一个非线性优化问题，再使用`Gauss-Newton`法迭代求解。设有图像`1.png`，`2.png`，我们在`1.png`中提取了`GFTT`角点，然后希望在`2.png`中追踪这些关键点。设两个图分别为$I_1$，$I_2$，第一张图中提取的点集为$P = {p_i}$，其中$p_i = [x_i,y_i]^T$为像素坐标值。考虑第i个点，我们希望计算$∆x_i, ∆y_i$，满足:

<p>
$$
\min _{\Delta x_{i}, \Delta y_{i}} \sum_{W}\left\|I_{1}\left(x_{i}, y_{i}\right)-I_{2}\left(x_{i}+\Delta x_{i}, y_{i}+\Delta y_{i}\right)\right\|_{2}^{2}
$$
</p>

即最小化二者灰度差的平方，其中$ \sum_{W}$表示我们在某个窗口（`Window`）中求和（而不是单个像素，因为问题有两个未知量,单个像素只有一个约束,是欠定的）。实践中，取此`window`为$8×8$大小的小块,即从$x_i − 4$取到$x_i + 3$，`y`坐标亦然。显然,这是一个`forward-addtive`的光流，而上述最小二乘问题可以用`Gauss-Newton`迭代求解。请回答下列问题，并根据你的回答，实现`codeoptical_flow.cpp`文件中的`OpticalFlowSingleLevel`函数。

1. 从最小二乘角度来看，每个像素的误差怎么定义？
2. 误差相对于自变量的导数如何定义？

下面是有关实现过程中的一些提示:

1. 同上一次作业，你仍然需要去除那些提在图像边界附近的点，不然你的图像块可能越过边界。
2. 该函数称为单层的光流，下面我们要基于这个函数来实现多层的光流。在主函数中，我们对两张图像分别测试单层光流、多层光流，并与`OpenCV`结果进行对比。作为验证，正向单层光流结果应该如图1所示，它结果不是很好，但大部分还是对的。
3. 在光流中，关键点的坐标值通常是浮点数，但图像数据都是以整数作为下标的。之前我们直接取了浮点数的整数部分，即把小数部分归零。但是在光流中，通常的优化值都在几个像素内变化，所以我们还用浮点数的像素插值。函数`GetPixelValue`为你提供了一个双线性插值方法(这也是常用的图像插值法之一)，你可以用它来获得浮点的像素值。

#### 答案：
1. 从最小二乘的角度，每个像素的误差为：

$$
g(p) = I(W(x;p+∆p)) - T(x)
$$

2. 误差相对于自变量的导数为：

$$
\frac{\partial g}{\partial p}=\nabla \mathrm{I} \frac{\partial W}{\partial p}=\left[\frac{\partial I}{\partial x}, \frac{\partial I}{\partial y}\right]
$$

最终结果为：(右边为opencv结果)

<div style="text-align:center">
<img src="../images/视觉里程计-LK/single_f.png" width='350'> <img src="../images/视觉里程计-LK/opencv_op.png"  width="350">
</div>

#### 题记
1. 对于LK光流的总结和公式推导:  
https://blog.csdn.net/sgfmby1994/article/details/68489944。

### 1.3 反向法
在你实现了上述算法之后，就会发现，在迭代开始时，`Gauss-Newton`的计算依赖于$I_2$在$(x_i,y_i)$处的梯度信息。然而，角点提取算法仅保证了$I_1(x_i,y_i)$处是角点(可以认为角度点存在明显梯度)，但对于$I_2$，我们并没有办法假设$I_2$在$x_i,y_i$处亦有梯度，从而`Gauss-Newton`并不一定成立。反向的光流法(inverse)则做了一个巧妙的技巧，即用$I_1(x_i,y_i)$处的梯度，替换掉原本要计算的$I_2 (x_i + ∆x_i,y_i+∆y_i)$的梯度。这样做的好处有:

- $I_1(x_i,y_i)$是角点，梯度总有意义；
- $I_1(x_i,y_i)$处的梯度不随迭代改变，所以只需计算一次，就可以在后续的迭代中一直使用，节省了大量计算时间。

我们为`OpticalFlowSingleLevel`函数添加一个`bool inverse`参数，指定要使用正常的算法还是反向的算法。请你根据上述说明，完成反向的`LK`光流法。

#### 答案：
1. 正向和后向的区别在于关键点梯度的计算上面。正向是计算第二张片的梯度，后向是计算第一张图片的梯度。

最终结果如下（左为正向，右为后向）：
<div style="text-align:center">
<img src="../images/视觉里程计-LK/single_f.png" width="350"> <img src="../images/视觉里程计-LK/single_i.png"  width="350">
</div>

最终结果如下（左为后向，右为opencv）：

<div style="text-align:center">
<img src="../images/视觉里程计-LK/single_i.png" width="350"> <img src="../images/视觉里程计-LK/opencv_op.png"  width="350">
</div>

### 2.4 推广至金字塔
通过实验，可以看出光流法通常只能估计几个像素内的误差。如果初始估计不够好，或者图像运动太大，光流法就无法得到有效的估计(不像特征点匹配那样)。但是，使用图像金字塔，可以让光流对图像运动不那么敏感。下面请你使用缩放倍率为2，共四层的图像金字塔，实现`coarse-to-fine`的`LK`光流。函数在`OpticalFlowMultiLevel`中。  

实现完成后，给出你的光流截图(正向、反向、金字塔正向、金字塔反向)，可以和`OpenCV`作比较。然后回答下列问题:

1. 所谓`coarse-to-fine`是指怎样的过程？
2. 光流法中的金字塔用途和特征点法中的金字塔有何差别？
提示:你可以使用上面写的单层光流来帮助你实现多层光流。

#### 答案：

1. 所谓的`coarse-to-fine`是指：先跟踪金字塔的最顶层，然后用被跟踪帧在最顶层的跟踪结果，作为次顶层的跟踪初始值，再次进行跟踪，依次至第0层（原始图像）。相当于从低分辨率（对运动不敏感）的图像开始跟踪，向高精度图像进行逐层迭代，从而得到一个更为准确的关键点和光流的过程。
2. 光流法中的金字塔是逐层迭代寻找最佳的关键点位置和光流方向（像素梯度），目的是解决光流在运动过程中难以检测的问题；同时图像中的金字塔也是为了排除图像运动过快导致跟踪不上的问题，因此可以跟踪金字塔上层下采样后的图像，然后层层细化。
特征点法中的金字塔是通过逐层检测特征点来增加尺度描述，目的是解决特征点的尺度不变性问题。特征点中的金字塔是为了排除焦点距离的影响，即从远处看是一个特征点，但是近看时却不是，当近看时可以用金字塔的上面几层来跟踪，实现尺度的不变性。

最终结果如下：（左为正向，右为金字塔正向）
<div style="text-align:center">
<img src="../images/视觉里程计-LK/single_f.png" width="350"> <img src="../images/视觉里程计-LK/multi_f.png"  width="350">
</div>

左为后向，右为金字塔后向：
<div style="text-align:center">
<img src="../images/视觉里程计-LK/single_i.png" width="350"> <img src="../images/视觉里程计-LK/multi_i.png"  width="350">
</div>

### 2.5 讨论
现在你已经自己实现了光流，看到了基于金字塔的`LK`光流能够与`OpenCV`达到相似的效果(甚至更好)。根据光流的结果，你可以和上讲一样，计算对极几何来估计相机运动。下面针对本次实验结果，谈谈你对下面问题的看法：

- 我们优化两个图像块的灰度之差真的合理吗？哪些时候不够合理？你有解决办法吗？
- 图像块大小是否有明显差异？取$16 \times 16 $和$8 \times 8$的图像块会让结果发生变化吗？
- 金字塔层数对结果有怎样的影响?缩放倍率呢？

#### 答案：

1. 光流法有三个基本的假设：a：灰度不变假设；b：小运动假设；c：局部一致性假设。
其中，灰度不变假设，当相机存在自动曝光或者当物体有强光或者阴影时，灰度不变的建设是不成立的。通常的解决办法是，对相机进行光度模型标定，将图像校正到一致的状态。
本题中金字塔的方法是对运动较大时的一种解决方法。
2. 当采用了金字塔的方法时，窗口固定，将图像生成金字塔过程中，在每一层金字塔上都用同一个大小的窗口来进行光流计算，这样很好的去解决了图像块的问题，这样一来图像块大小并不会带来明显差异。  
当没有采用金字塔方法时。当窗口较大时，光流计算更鲁棒，当窗口较小时，光流计算更正确。原因在于，当图像中每一个部分的运动都不一致的时候，如果开的窗口过大，很容易违背窗口(邻域)内的所有点光流一致的基本假设，这可能与实际不一致，所以窗口小，包含的像素少，更精确些。  
在本题中，当图像块从$8×8$调到$16×16$，多层金字塔效果不明显，单程`IAIA`效果略有改善。
3. 金字塔层数一般越多效果越好，但是一般图像大于4~5层之后都变得太小，特征点像素太过紧密容易出现错误追踪。放大倍率同样的道理，放大倍率小，金字塔的层数可以增加，迭代层数增多，效果自然要好。
对于本题中，将金字塔数从4层改为6层，结果基本无差别；将4层的缩放倍率从0.5改为0.25，结果基本无差别。

完整程序链接：https://github.com/XLMaverick/Visual-Localization-Percessing/tree/master/vslam_fourteen_lectures/optical_flow 

本题相关完整代码如下：

    inline float GetPixelValue(const cv::Mat &img, float x, float y) 
    {
        uchar *data = &img.data[int(y) * img.step + int(x)];
        float xx = x - floor(x);
        float yy = y - floor(y);
        return float(
                (1 - xx) * (1 - yy) * data[0] +
                xx * (1 - yy) * data[1] +
                (1 - xx) * yy * data[img.step] +
                xx * yy * data[img.step + 1]
        );
    }

    int main(int argc, char **argv) {

        // images, note they are CV_8UC1, not CV_8UC3
        Mat img1 = imread(file_1, 0);
        Mat img2 = imread(file_2, 0);

        // key points, using GFTT here.
        vector<KeyPoint> kp1;
        Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
        detector->detect(img1, kp1);

        // now lets track these key points in the second image
        // first use single level LK in the validation picture
        vector<KeyPoint> kp2_single;
        vector<bool> success_single;
        OpticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single);

        // then test multi-level LK
        vector<KeyPoint> kp2_multi;
        vector<bool> success_multi;
        OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi, success_multi);

        // use opencv's flow for validation
        vector<Point2f> pt1, pt2;
        for (auto &kp: kp1) pt1.push_back(kp.pt);
        vector<uchar> status;
        vector<float> error;
        cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error, cv::Size(8, 8));

        // plot the differences of those functions
        Mat img2_single;
        cv::cvtColor(img2, img2_single, CV_GRAY2BGR);
        for (int i = 0; i < kp2_single.size(); i++) {
            if (success_single[i]) {
                cv::circle(img2_single, kp2_single[i].pt, 2, cv::Scalar(0, 250, 0), 2);
                cv::line(img2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 250, 0));
            }
        }

        Mat img2_multi;
        cv::cvtColor(img2, img2_multi, CV_GRAY2BGR);
        for (int i = 0; i < kp2_multi.size(); i++) {
            if (success_multi[i]) {
                cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
                cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 250, 0));
            }
        }

        Mat img2_CV;
        cv::cvtColor(img2, img2_CV, CV_GRAY2BGR);
        for (int i = 0; i < pt2.size(); i++) {
            if (status[i]) {
                cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
                cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
            }
        }

        cv::imshow("tracked single level", img2_single);
        cv::imwrite("../single_i.png", img2_single);
        cv::imshow("tracked multi level", img2_multi);
        cv::imwrite("../multi_i.png", img2_multi);
        cv::imshow("tracked by opencv", img2_CV);
        cv::imwrite("../opencv_op.png", img2_CV);
        cv::waitKey(0);

        return 0;
    }

    void OpticalFlowSingleLevel(
            const Mat &img1,
            const Mat &img2,
            const vector<KeyPoint> &kp1,
            vector<KeyPoint> &kp2,
            vector<bool> &success,
            bool inverse
    ) {

        // parameters
        int half_patch_size = 4;
        int iterations = 10;
        bool have_initial = !kp2.empty();

        for (size_t i = 0; i < kp1.size(); i++) 
        {
            auto kp = kp1[i];
            double dx = 0, dy = 0; // dx,dy need to be estimated
            if (have_initial) 
            {
                dx = kp2[i].pt.x - kp.pt.x;
                dy = kp2[i].pt.y - kp.pt.y;
            }

            double cost = 0, lastCost = 0;
            bool succ = true; // indicate if this point succeeded

            // Gauss-Newton iterations
            for (int iter = 0; iter < iterations; iter++) 
            {
                Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
                Eigen::Vector2d b = Eigen::Vector2d::Zero();
                cost = 0;

                if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                    kp.pt.y + dy <= half_patch_size || kp.pt.y + dy >= img1.rows - half_patch_size) 
                {   // go outside
                    succ = false;
                    break;
                }

                // compute cost and jacobian
                for (int x = -half_patch_size; x < half_patch_size; x++)
                    for (int y = -half_patch_size; y < half_patch_size; y++) 
                    {
                        double error = 0;
                        float u1 = float(kp.pt.x + x), v1 = float(kp.pt.y+y);
                        float u2 = float(u1+dx), v2 = float(v1+dy);
                        Eigen::Vector2d J;  // Jacobian
                        if (inverse == false) 
                        {
                            J.x() = double(GetPixelValue(img2,u2+1,v2) - GetPixelValue(img2,u2-1,v2))/2;
                            J.y() = double(GetPixelValue(img2,u2,v2+1) - GetPixelValue(img2,u2,v2-1))/2;
                            error = double(GetPixelValue(img2,u2,v2) - GetPixelValue(img1,u1,v1));
                            // Forward Jacobian
                        } else 
                        {
                            // Inverse Jacobian
                            // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                            J.x() = double(GetPixelValue(img1,u1+1,v1) - GetPixelValue(img1,u1-1,v1))/2;
                            J.y() = double(GetPixelValue(img1,u1,v1+1) - GetPixelValue(img1,u1,v1-1))/2;
                            error = double(GetPixelValue(img2,u2,v2) - GetPixelValue(img1,u1,v1));
                        }

                        H += J*J.transpose();
                        b += -J*error;
                        cost += error*error;
                    }

                // compute update
                Eigen::Vector2d update;
                update = H.ldlt().solve(b);

                if (isnan(update[0])) 
                {
                    // sometimes occurred when we have a black or white patch and H is irreversible
                    cout << "update is nan" << endl;
                    succ = false;
                    break;
                }
                if (iter > 0 && cost > lastCost) 
                {
                    cout << "cost increased: " << cost << ", " << lastCost << endl;
                    break;
                }

                // update dx, dy
                dx += update[0];
                dy += update[1];
                lastCost = cost;
                succ = true;
            }
            success.push_back(succ);
            // set kp2
            if (have_initial) 
            {
                kp2[i].pt = kp.pt + Point2f(dx, dy);
            } else 
            {
                KeyPoint tracked = kp;
                tracked.pt += cv::Point2f(dx, dy);
                kp2.push_back(tracked);
            }
        }
    }

    void OpticalFlowMultiLevel(
            const Mat &img1,
            const Mat &img2,
            const vector<KeyPoint> &kp1,
            vector<KeyPoint> &kp2,
            vector<bool> &success,
            bool inverse) {

        // parameters
        int pyramids = 4;
        double pyramid_scale = 0.5;
        double scales[] = {1.0, 0.5, 0.25, 0.125};

        // create pyramids
        vector<Mat> pyr1, pyr2; // image pyramids
        // TODO START YOUR CODE HERE (~8 lines)
        for (int i = 0; i < pyramids; i++) 
        {
            Mat img1_temp, img2_temp;
            resize(img1, img1_temp,Size(img1.cols*scales[i], img1.rows*scales[i]));
            resize(img2, img2_temp,Size(img2.cols*scales[i], img2.rows*scales[i]));
            pyr1.push_back(img1_temp);
            pyr2.push_back(img2_temp);
            cout<<"Pyramid"<<i<<"im1 size: "<<img1_temp.cols<<" " <<img1_temp.rows<<endl;
        }
        // coarse-to-fine LK tracking in pyramids
        vector<KeyPoint> vkp2_now;
        vector<KeyPoint> vkp2_last;
        vector<bool> vsucc;
        for(int i = pyramids-1;i>=0;i--)
        {
            vector<KeyPoint> vkp1;
            for(int j = 0; j<kp1.size();j++)
            {
                KeyPoint kp1_temp = kp1[j];
                kp1_temp.pt *= scales[i];
                vkp1.push_back(kp1_temp);
                if(i<pyramids-1)
                {
                    KeyPoint kp2_temp = vkp2_last[j];
                    kp2_temp.pt /= pyramid_scale;
                    vkp2_now.push_back(kp2_temp);
                }
            }
            vsucc.clear();
            OpticalFlowSingleLevel(pyr1[i], pyr2[i], vkp1, vkp2_now, vsucc, inverse);
            vkp2_last.clear();
            vkp2_last.swap(vkp2_now);
            cout<<"pyramid: "<<i<<"vkp2_last size: "<<vkp2_last.size()<<"vkp2_noe size "<<vkp2_now.size()<<endl;
        }
        kp2 = vkp2_last;
        success = vsucc;
        // TODO END YOUR CODE HERE
        // don't forget to set the results into kp2
    }
