# SLAMWARE ROS SDK

[SLAMWARE ROS SDK 官方文档](https://developer.slamtec.com/docs/slamware/ros-sdk/2.6.0_rtm/)

### 启动节点

若移动机器人处于AP模式，连接机器人WIFI，启动节点

```shell
roslaunch slamware_ros_sdk slamware_ros_sdk_server_node.launch ip_address:=192.168.11.1
```

通过`rviz`查看

```shell
roslaunch slamware_ros_sdk view_slamware_ros_sdk_server_node.launch
```



# Cartographer ROS

[Cartographer ROS 官方文档](https://google-cartographer-ros.readthedocs.io/en/latest/index.html)

## Running Cartographer ROS on your own bag

### Validate your bag

