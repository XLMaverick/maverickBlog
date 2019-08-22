---
# 常用定义
title: "基于优化的IMU预积分与视觉信息融合"        # 标题
date: 2019-06-23               # 创建时间
lastmod: 2019-06-23            # 最后修改时间
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

习题一：LM优化算法
--------------------------------
样例代码给出了使用`LM`算法来估计曲线 $y = exp(ax 2 + bx + c)$ 参数 $a, b, c$ 的完整过程。

1. 请绘制样例代码中`LM`阻尼因子`μ`随着迭代变化的曲线图;
2. 将曲线函数改成 $y = ax^2 + bx + c$，请修改样例代码中残差计算，雅克比计算等函数，完成曲线参数估计。
3. 如果有实现其他阻尼因子更新策略可加分(选做)

### 答案：
1、编译运行程序后，设置迭代次数为30次，在第11次结果收敛，最终结果和阻尼因子的变化曲线图如下：

<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/基于优化的IMU预积分与视觉信息融合/result5.png"  width="300"  height = "150">
</div>

<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/基于优化的IMU预积分与视觉信息融合/result6.png" width="350" height = "350">
</div>

<div style="float:none;clear:both;">
</div>

2、这一问之中，更改曲线函数之后，需要更改三处地方，构造观测值；计算残差；残差对变量的雅克比；针对这个曲线，修改此三处较为简单，不再赘述。  
但是修改完之后，会发现优化的结果较差。针对这种情况有下面三种解决的办法：

- 增加数据量，例如数据点数N可以增加至1000；
- 减小设置的噪声的方差，`w_sigma`可以设置为0.1；
- 修改待估计的参数`a、b、c`优化的初始值。

选择上述中的任意一种方法，可以获得较优的结果，最终结果如图所示：

<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/基于优化的IMU预积分与视觉信息融合/result3.png"  width="300"  height = "150">
</div>
<div style="float:none;clear:both;">
</div>

3、阻尼因子的更新策略的实现是在`problem.cc`文件中`Problem::IsGoodStepInLM()`函数中，重写该函数，实现了Marquardt策略，具体的实现代码如下，但是有一个问题需要注意，就是一个良好的初始值对最优化问题非常重要，（更换策略后，如果仍使用0为初始值，则会出现问题），更换阻尼因子的更新策略后，我们选用的带估计参数的初始值为`2,-1,5`，最终结果如下图：
最终结果如下：
<div style="float:left;border:solid 1px 000;margin:2px;">
<img src="../images/基于优化的IMU预积分与视觉信息融合/result4.png"  width="300"  height = "150">
</div>
<div style="float:none;clear:both;">
</div>

代码如下：

```
bool Problem::IsGoodStepInLM() {
    double scale = 0;
    scale = delta_x_.transpose() * (currentLambda_ * delta_x_ + b_);
    scale += 1e-3;    // make sure it's non-zero :)

    // recompute residuals after update state
    // 统计所有的残差
    double tempChi = 0.0;
    for (auto edge: edges_) {
        edge.second->ComputeResidual();
        tempChi += edge.second->Chi2();
    }
    double rho = (currentChi_ - tempChi) / scale;
    if (rho > 0 && isfinite(tempChi))   
    {
        if(rho>0.75)
        {
             currentLambda_ = currentLambda_*0.33;
        }
        else if(rho<0.25)
        {
             currentLambda_ =currentLambda_* 2;
        }
        currentLambda_ =currentLambda_*1; 
        currentChi_ = tempChi;
        return true;
    }
    else
    {
        currentLambda_ *= 2;
        return false;
    }
}
```

习题二：公式推导
--------------------------------
根据课程知识，完成`F，G`中如下两项的推导过程：

<p>
$$
\begin{aligned} \mathbf{f}_{15} &=\frac{\partial \boldsymbol{\alpha}_{b_{i} b_{k+1}}}{\partial \delta \mathbf{b}_{k}^{g}}=-\frac{1}{4}\left(\mathbf{R}_{b_{i} b_{k+1}}\left[\left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right)\right] \times \delta t^{2}\right)(-\delta t) \\ \mathbf{g}_{12} &=\frac{\partial \boldsymbol{\alpha}_{b_{i} b_{k+1}}}{\partial \mathbf{n}_{k}^{g}}=-\frac{1}{4}\left(\mathbf{R}_{b_{i} b_{k+1}}\left[\left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right)\right] \times \delta t^{2}\right)\left(\frac{1}{2} \delta t\right) \end{aligned}
$$
</p>

### 答案

公式$f_{15}$的证明：

<p>
$$
\mathbf{a}=\frac{1}{2}\left(\mathbf{q}_{b_{i} b_{k}}\left(\mathbf{a}^{b_{k}}-\mathbf{b}_{k}^{a}\right)+\mathbf{q}_{b_{i} b_{k+1}}\left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right)\right)
$$
</p>

<p>
$$
\mathbf{q}_{b_{i} b_{k+1}}=\mathbf{q}_{b_{i} b_{k}} \otimes\left[\begin{array}{c}{1} \\ {\frac{1}{2} \omega \delta t}\end{array}\right]
$$
</p>
根据上面两个公式，带入可得：

<p>
$$
\begin{aligned} \boldsymbol{\alpha}_{b_{i} b_{k+1}} &= \boldsymbol{\alpha}_{b_{i} b_{k}}+\boldsymbol{\beta}_{b_{i} b_{k}} \delta t+\frac{1}{2} \mathbf{a} \delta t^{2}
\\ &= \boldsymbol{\alpha}_{b_{i} b_{k}}+\boldsymbol{\beta}_{b_{i} b_{k}} \delta t +  \frac{1}{4}\left(\mathbf{q}_{b_{i} b_{k}}\left(\mathbf{a}^{b_{k}}-\mathbf{b}_{k}^{a}\right)+\mathbf{q}_{b_{i} b_{k}} \otimes\left[\begin{array}{c}{1} \\ {\frac{1}{2} \omega \delta t}\end{array}\right]\left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right)\right) \delta t^2\end{aligned}
$$
</p>

<p>
$$
\begin{aligned} \mathbf{f}_{15}=\frac{\partial \boldsymbol{\alpha}_{b_{i} b_{k+1}}}{\partial \delta \mathbf{b}_{k}^{g}} &= \frac{\partial \frac{1}{4} \mathbf{q}_{b_{i} b_{k}} \otimes\left[\begin{array}{c}{1} \\ {\frac{1}{2} \boldsymbol{\omega} \delta t}\end{array}\right] \otimes\left[\begin{array}{c}{1} \\ {-\frac{1}{2} \delta \mathbf{b}_{k}^{g} \delta t}\end{array}\right]\left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right) \delta t^2}{\partial \delta \mathbf{b}_{k}^{g}}
\\ &= \frac{1}{4} \frac{\partial \mathbf{R}_{b_{i} b_{k+1}} \exp \left(\left[-\delta \mathbf{b}_{k}^{g} \delta t\right]_{ \times}\right)\left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right) \delta t^2}{\partial \delta \mathbf{b}_{k}^{g}}
\\ &= \frac{1}{4} \frac{\partial \mathbf{R}_{b_{i} b_{k+1}}\left(\mathbf{I}+\left[-\delta \mathbf{b}_{k}^{g} \delta t\right]_{ \times}\right)\left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right) \delta t^2}{\partial \delta \mathbf{b}_{k}^{g}}
\\ &= \frac{1}{4} \frac{\partial-\mathbf{R}_{b_{i} b_{k+1}}\left(\left[\left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right) \delta t^2\right]_{ \times}\right)\left(-\delta \mathbf{b}_{k}^{g} \delta t\right)}{\partial \delta \mathbf{b}_{k}^{g}}
\\ &= -\frac{1}{4}\left(\mathbf{R}_{b_{i} b_{k+1}}\left[\left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right)\right]_{ \times} \delta t^2\right)(-\delta t)\end{aligned}
$$
</p>

公式$g_{12}$的证明：

<p>
$$
\begin{aligned} \omega &=\frac{1}{2}\left(\left(\boldsymbol{\omega}^{b_{k}}+\mathbf{n}_{k}^{g}-\mathbf{b}_{k}^{g}\right)+\left(\boldsymbol{\omega}^{b_{k+1}}+\mathbf{n}_{k+1}^{g}-\mathbf{b}_{k}^{g}\right)\right)
\\ &= \frac{1}{2}\left(\boldsymbol{\omega}^{b_{k}}+\boldsymbol{\omega}^{b_{k+1}}+n_k^g + n_{k+1}^{g}\right)-\mathbf{b}_{k}^{g} \end{aligned}
$$
</p>

<p>
$$
\begin{aligned} \boldsymbol{\alpha}_{b_{i} b_{k+1}} &=\boldsymbol{\alpha}_{b_{i} b_{k}}+\boldsymbol{\beta}_{b_{i} b_{k}} \delta t+\frac{1}{2} \mathbf{a} \delta t^{2}
\\ &= \boldsymbol{\alpha}_{b_{i} b_{k}}+\boldsymbol{\beta}_{b_{i} b_{k}} \delta t +  \frac{1}{4}\left(\mathbf{q}_{b_{i} b_{k}}\left(\mathbf{a}^{b_{k}}-\mathbf{b}_{k}^{a}\right)+\mathbf{q}_{b_{i} b_{k}} \otimes\left[\begin{array}{c}{1} \\ {\frac{1}{2} \omega \delta t}\end{array}\right]\left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right)\right) \delta t^2\end{aligned}
$$
</p>

<p>
$$
\begin{aligned} \mathbf{g}_{12}=\frac{\partial \boldsymbol{\alpha}_{b_{i} b_{k+1}}}{\partial \mathbf{n}_{k}^{g}} &= \frac{\partial \frac{1}{4} \mathbf{q}_{b_{i} b_{k}} \otimes\left[\begin{array}{c}{1} \\ {\frac{1}{2} \boldsymbol{\omega} \delta t}\end{array}\right] \otimes\left[\begin{array}{c}{1} \\ {-\frac{1}{2} \delta \mathbf{b}_{k}^{g} \delta t}\end{array}\right] \otimes\left[\begin{array}{r}{1} \\ {\frac{1}{4} \delta \mathbf{n}_{k}^{g} \delta t}\end{array}\right]  \otimes\left[\begin{array}{rr}{1} \\ {\frac{1}{4} \delta \mathbf{n}_{k+1}^{g}}{\delta t}\end{array}\right]  \left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right) \delta t^2}{\partial \mathbf{n}_{k}^{g}}
\\ &= \frac{1}{4} \frac{\partial \mathbf{R}_{b_{i} b_{k+1}} \exp \left(\left[-\delta \mathbf{b}_{k}^{g} \delta t\right]_{ \times}\right) \exp \left(\left[ \frac{1}{2}\delta \mathbf{n}_{k}^{g} \delta t\right]_{ \times}\right)  \exp \left(\left[\frac{1}{2} \delta \mathbf{n}_{k+1}^{g} \delta t\right]_{ \times}\right)  \left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right) \delta t^2}{\partial  \mathbf{n}_{k}^{g}}
\\ &= \frac{1}{4} \frac{\partial \mathbf{R}_{b_{i} b_{k+1}}\left(\mathbf{I}+\left[-\delta \mathbf{b}_{k}^{g} \delta t\right]_{ \times} + \left[\frac{1}{2}\delta \mathbf{n}_{k}^{g} \delta t\right]_{ \times} +\left[\frac{1}{2} \delta \mathbf{n}_{k+1}^{g} \delta t\right]_ { \times} \right) \left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right) \delta t^2}{\partial  \mathbf{n}_{k}^{g}}
\\ &= \frac{1}{4} \frac{\partial-\mathbf{R}_{b_{i} b_{k+1}}\left(\left[\left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right) \delta t^2\right]_{ \times}\right)  \left[\left(\frac{1}{2}\delta \mathbf{n}_{k}^{g} \delta t\right)  + \left(-\delta \mathbf{b}_{k}^{g} \delta t\right) +  \left(\frac{1}{2}\delta \mathbf{n}_{k+1}^{g} \delta t\right) \right] }{\partial \mathbf{n}_{k}^{g}}
\\ &= -\frac{1}{4}\left(\mathbf{R}_{b_{i} b_{k+1}}\left[\left(\mathbf{a}^{b_{k+1}}-\mathbf{b}_{k}^{a}\right)\right]_{ \times} \delta t^2\right)(\frac{1}{2}\delta t)\end{aligned}
$$
</p>

习题三：证明
--------------------------------
LM算法对高斯牛顿法进行了改进，求解过程中引入了阻尼因子：

<p>
$$
\left(\mathbf{J}^{\top} \mathbf{J}+\mu \mathbf{I}\right) \Delta \mathbf{x}_{\operatorname{lm}}=-\mathbf{J}^{\top} \mathbf{f} \quad \text { with } \mu \geq 0
$$
</p>

设$J^TJ$矩阵为$H$矩阵，$J^Tf$矩阵为$F^{'T}$矩阵。
同时，$H$矩阵的特征值${\lambda_{j}}$和对应的特征向量为${v_j}$。对$H$矩阵做特征值分解后有：

<p>
$$
\mathbf{H}=\mathbf{V} \mathbf{\Lambda} \mathbf{V}^{\top}
$$
</p>

同时，考虑矩阵$H+uI$，我们可以得知：

<p>
$$
(H+uI)v_j = Hv_j+uIv_j = \lambda_{j} v_j + uv_j = (\lambda_{j}+u)v_{j}
$$
</p>

则，对矩阵$H+uI$进行分解可得：

<p>
$$
H+uI = \mathbf{V} (\mathbf{\Lambda}+u) \mathbf{V}^{\top}
$$
</p>

将上述公式带入：

<p>
$$
\begin{aligned}
(H+uI) \Delta \mathbf{x}_{\operatorname{lm}} &= -F^{'T} \\
\mathbf{V} (\mathbf{\Lambda}+uI) \mathbf{V}^{\top} \Delta  \mathbf{x}_{\operatorname{lm}} &= -F^{'T} \\
\Delta \mathbf{x}_{\operatorname{lm}} &= -\left[ \mathbf{V}(\mathbf{\Lambda}+uI)\mathbf{V}^{\top} \right] ^{-1} F^{'T}  \\
\Delta \mathbf{x}_{\operatorname{lm}} &= - \mathbf{V}(\mathbf{\Lambda}+uI)^{-1}\mathbf{V}^{\top}F^{'T}
\end{aligned}
$$
</p>

其中，将矩阵$\mathbf{V}$、$(\mathbf{\Lambda}+uI)^{-1}$、$\mathbf{V}^{\top}$展开，我们可以得到：

<p>
$$
\begin{aligned}
\mathbf{V}(\mathbf{\Lambda}+uI)^{-1}\mathbf{V}^{\top}F^{'T} &= \left[ \begin{matrix}  
\mathbf{v}_1 &
\mathbf{v}_2 &
\cdots &
\mathbf{v}_n
\end{matrix}  \right] *
\left[ \begin{matrix}  
\frac{1}{\lambda_{1}+u}  & 0 & \cdots & 0\\
0 & \frac{1}{\lambda_{2}+u}  & \cdots & 0\\
\vdots  & \vdots & \cdots & \vdots\\
0 & 0 & \cdots & \frac{1}{\lambda_{n}+u}
\end{matrix}  \right]*
\left[ \begin{matrix}  
\mathbf{v}_1^{T} \\
\mathbf{v}_2^{T} \\
\vdots \\
\mathbf{v}_n^{T}
\end{matrix}  \right] *F^{'T}\\
&= \left[ \begin{matrix}  
\mathbf{v}_1 &&
\mathbf{v}_2 &&
\cdots &&
\mathbf{v}_n
\end{matrix}  \right] *
\left[ \begin{matrix}  
\frac{1}{\lambda_{1}+u}  & 0 & \cdots & 0\\
0 & \frac{1}{\lambda_{2}+u}  & \cdots & 0\\
\vdots  & \vdots & \cdots & \vdots\\
0 & 0 & \cdots & \frac{1}{\lambda_{n}+u}
\end{matrix}  \right]*
\left[ \begin{matrix}  
\mathbf{v}_1^{T}F^{'T}\\
\mathbf{v}_2^{T}F^{'T} \\
\vdots \\
\mathbf{v}_n^{T}F^{'T}
\end{matrix}  \right]  \\
&= \left[ \begin{matrix}  
\frac{1}{\lambda_{1}+u}\mathbf{v}_1 &&
\frac{1}{\lambda_{2}+u}\mathbf{v}_2 &&
\cdots &&
\frac{1}{\lambda_{n}+u}\mathbf{v}_n
\end{matrix}  \right] * 
\left[ \begin{matrix}  
\mathbf{v}_1^{T}F^{'T}\\
\mathbf{v}_2^{T}F^{'T} \\
\vdots \\
\mathbf{v}_n^{T}F^{'T}
\end{matrix}  \right]  \\
&= \sum_{j=1}^{n} \frac{\mathbf{v}_{j}^{\top} \mathbf{F}^{\prime \top}}{\lambda_{j}+\mu} \mathbf{v}_{j}
\end{aligned}
$$
</p>

最终可得：

<p>
$$
\Delta \mathbf{x}_{\mathrm{lm}}=-\sum_{j=1}^{n} \frac{\mathbf{v}_{j}^{\top} \mathbf{F}^{\prime \top}}{\lambda_{j}+\mu} \mathbf{v}_{j}
$$
</p>
