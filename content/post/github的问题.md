---
title: "github的问题"
date: 2018-09-13T20:58:50+00:00
lastmod: 2018-09-13T20:58:50+00:00
draft: false
tags: [ ]
categories: [ ]
author: "Yang"

# You can also close(false) or open(true) something for this content.
# P.S. comment can only be closed
# comment: false
# toc: false
autoCollapseToc: true
# You can also define another contentCopyright. e.g. contentCopyright: "This is another copyright."
contentCopyright: '<a href="https://github.com/gohugoio/hugoBasicExample" rel="noopener" target="_blank">See origin</a>'
# reward: false
# mathjax: false
---


- git 提交github后文件夹为灰颜色问题  
因为使用git clone 之后，文件夹里面会包含原来相关的信息，所以重新push之后会显示灰颜色。解决方案：  
\* 删除clone的代码里面的.git和.gitignore文件，重新push。  
\* 若上述方法无效时，是因为已经有了缓存，需要先将缓存删除  
git rm -r --cached some-directory

- git提交空文件夹  
在空目录下创建.gitkeep文件。

- push 每次需要输入用户名和密码的问题  
每次都需要输入用户名和密码是因为你采用的是 https 方式提交代码， 如果采用的是 ssh 方式只需要在版本库中添加用户的 sha 的key就可以实现提交时无需输入用户名和密码。  
可以在更改配置HTTPS地址为SSH地址，当然也可以通过设置git的cache，可以让它记住密码，之后自己设置一个cache有效时间 这样也一定程度保证了一些安全性，具体代码如下  
git config --global credential.helper cache  
git config --global credential.helper 'cache --timeout=3600' 