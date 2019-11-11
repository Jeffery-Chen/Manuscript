# Sick LMS511-20100

## [Windows下连接激光](https://blog.csdn.net/u013453604/article/details/50725833)

### Ethernet方式

- 电源线：24V直流电源
- 以太网线：与电脑网线端口直连，设置电脑IP与激光雷达IP（默认为192.168.12.140）同网段（此次安装选用192.168.12.10），手动设定子网掩码为255.255.255.0

1. 启动[SOPAS ET](https://www.sick.com/de/en/downloads/eula?code=swp367244)客户端软件后为激光雷达上电，初次运行会自动搜索到该设备
2. 双击右侧栏搜索到的设备即可添加到左侧项目栏，单击设备底部出现的**安装设备驱动程序**提示，选择**从设备上传**，等待十分钟左右即可自行安装完毕
3. 点击**离线**即可切换为**在线**模式
4. 双击在线的设备即可查看激光扫描的可视化结果

**帮助**——**SOPAS ET手册**可找到该软件的中文操作手册，进一步说明如何连接LMS系列设备



## [源码驱动安装](https://blog.csdn.net/zhuoyueljl/article/details/75244563)

```shell
$ mkdir -p ~/erobot_ws/src
$ cd  ~/erobot_ws/src
$ catkin_create_pkg laser_node std_msgs rospy roscpp
$ rm -rf laser_node/*
$ cd laser_node
$ git clone https://github.com/NatanBiesmans/lms5xx
```

> - 将源文件**src**中的**lms5xx_node.cpp**激光雷达IP（ip_add）改为自己的IP（192.168.12.140）

```shell
$ cd  ~/erobot_ws
$ catkin_make
$ source ~/erobot_ws/devel/setup.bash
```

### 测试

开启终端，执行

```shell
$ roscore
```

开启新终端，执行

```shell
$ rosrun laser_node laser_node
```

再开启新终端，执行

```shell
$ rosrun rviz rviz
```

`rviz`可视化界面下选择`Add`–> `by topic` –> `LaserScan`

此时会出现**status error**，再将`FixedFrame`修改为`laser`即可

> Sick官方[github](https://github.com/SICKAG/sick_scan)可下载此雷达的**.urdf**文件

