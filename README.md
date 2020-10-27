# xchu_odom
基于NDT算法的激光里程计，参考autoware、ndt_omp

kitti测试结果，全长3700m，路口基本对齐，偏移量较小。

![image-20201027024048000](README/image-20201027024048000.png)

已知问题：

- imu和轮速计未调试好
- gpu版本ndt未调试，目前支持autoware_ndt和ndt_omp