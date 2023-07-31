# IMU预积分
为dlo添加自己的预积分部分，后加以紧耦合。

## finished
- [X] 使用优化器耦合imu和lidar的odom
    - [X] 添加到g2o的第一条边，补全函数
    - [X] 学习lio-sam单独开辟了用于优化的imu预积分。
## TO DO
- [ ] 解决imu的odom抖动问题，imu预积分效果不好。