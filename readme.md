# IMU预积分
包含手写积分与使用gtsam预积分，二者效果几乎一样。

使用外部odom更新的速度对积分影响很大。

## RUN

```
roslaunch XIO xio.launch
```
# noted
1. 似乎是由于imu回调函数计算速度太慢，导致无法满足实时性。设置imu的subscriber的queue_size=1后可以较好更随。已解决，是发布path过于密集导致，需要降低频率，10Hz。不需要设置queue_size=1，计算用时$10^{-5}$，发布占用0.004s左右。

## TODO
- [ ] 使用lio-sam输出的低频odom积分效果不佳。直接放入lio-sam代码中积分，效果与`/odometry/imu_incremental`的结果一致，判断是衔接问题，待解决。
- [ ] 添加一个估计速度较为准确的slam算法，配合imu积分。lio-sam mapping/odom话题不包含速度，是使用前一激光odom至当前的激光odom使用Opt积分优化得到的，在给imu积分。学习《视觉SLAM进阶：从零开始手写VIO》的方法。