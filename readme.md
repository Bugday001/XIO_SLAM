# IMU预积分
包含手写积分与使用gtsam预积分，二者效果几乎一样。

使用外部odom更新的速度对积分影响很大。

## RUN

```
roslaunch XIO xio.launch
```

## TODO
- [ ] 使用lio-sam输出的低频odom积分效果不佳。
- [ ] 添加一个估计速度较为准确的slam算法，配合imu积分