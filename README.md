不稳定的道路检测代码  
  
如果你需要文档，需要再等一等。  
  
###道路检测的大致思路如下: 
1. 模糊、IPM、canny，必要时用高帽、方向卷积  
2. 概率hough变换获取直线  
3. 根据车道线特征，从hough直线中检测生成车道线   
4. 跟踪车道线，如果车道线消失，则重新检测
  
大致思路如上  

###依赖
1. opencv
2. linux、osx

###如何使用
1. 进入src 中，根据你的操作系统情况修改 makefile，然后make 一下
2. ./main ../sample/wandaor.mov  

以上操作，你能看到一个初步的车道检测效果   
![效果图](http://i.imgur.com/KF01ENH.png)  
另，之前的检测[视频](https://www.youtube.com/watch?v=tSoJ5f0X7Y0)


###现状

因为现在人手严重不足，所以作者需要把精力花在作者认为更重要的事情上。   
作者在上海，如果有什么问题，你可以发email给我 akBpb2N0bC5jbg== (base64)，或者加入我们的qq群302975729    
另: h.youxiamotors.com   
后面会慢慢公开更多的东西。  

十分感谢
