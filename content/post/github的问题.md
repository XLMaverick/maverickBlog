---
title: "github的问题"
date: 2018-09-13
lastmod: 2018-09-13
draft: false
tags: ['git']
categories: ['others']
author: "Yang"
comment: true
toc: true
autoCollapseToc: true
contentCopyright: '<a href="https://github.com/gohugoio/hugoBasicExample" rel="noopener" target="_blank">See origin</a>'
reward: false
mathjax: false
---

### 一、git提交github的命令顺序

1. `git add  ` 要提交文件夹的名称，将文件添加到跟踪列表，`git add .`提交所有文件；
2. 输入`git commit -m "提交信息" `，将文件提交到本地仓库，提交信息必须填写；
3. 输入`git remote add origin github代码仓库的url地址`，将代码仓库与`github`关联；
4. 输入`git pull origin master`，将远程代码仓库拉回到本地，第一次提交可以不加，但以后提交时要加，否则代码可能会杂糅在一起;
5. 输入`git push  -u origin master`，将代码提交到`github`，输入回车后需要输入你的`github`名称和密码。

### 二、git 提交github后文件夹为灰颜色问题  
因为使用`git clone`之后，文件夹里面会包含原来相关的信息，所以重新`push`之后会显示灰颜色。解决方案：  
\* 删除`clone`的代码里面的`.git`和`.gitignore`文件，重新`push`； 
\* 若上述方法无效时，是因为已经有了缓存，需要先将缓存删除`git rm -r --cached some-directory`。

### 三、git提交空文件夹  
在空目录下创建`.gitkeep`文件。

### 四、push 每次需要输入用户名和密码的问题  
每次都需要输入用户名和密码是因为你采用的是`https`方式提交代码， 如果采用的是`ssh`方式只需要在版本库中添加用户的`sha`的`key`就可以实现提交时无需输入用户名和密码。  
可以在更改配置`HTTPS`地址为`SSH`地址，当然也可以通过设置`git`的`cache`，可以让它记住密码，之后自己设置一个`cache`有效时间 这样也一定程度保证了一些安全性，具体代码如下
```  
git config --global credential.helper cache  
git config --global credential.helper 'cache --timeout=3600' 
```

### 五、更新远程仓库代码到本地

1. 查看远程分支
使用如下命令可以查看远程仓库（我这里有一个origin仓库）
```
$ git remote -v
origin  git@github.com:username/Animations.git (fetch)
origin  git@github.com:username/Animations.git (push)
```
2. 从远程获取最新版本到本地
使用如下命令可以在本地新建一个`temp`分支，并将远程`origin`仓库的`master`分支代码下载到本地`temp`分支
```
$ git fetch origin master:temp
```

3. 比较本地仓库与下载的`temp`分支
使用如下命令来比较本地代码与刚刚从远程下载下来的代码的区别，这里可以配置一个图形化比较工具来可视化。
```
$ git diff temp
```

4. 合并`temp`分支到本地的`master`分支
对比区别之后，如果觉得没有问题，可以使用如下命令进行代码合并：
```
$ git merge temp
```

5. 删除`temp`分支
如果`temp`分支不想要保留，可以使用如下命令删除该分支：
```
$ git branch -d temp
```
如果该分支的代码之前没有`merge`到本地，那么删除该分支会报错，可以使用一下命令强制删除该分支。
```
Git branch -D temp
```