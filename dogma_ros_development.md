# dogma_ros开发

[利用Python来编写ROS程序](https://www.ncnynl.com/archives/201611/1055.html)

## 学习笔记

### 自定义package.xml

`catkin_create_pkg`自动生成`beginner_tutorials`包

- 描述标签

```xml
<description>The beginner_tutorials package</description>
```

- 维护者标签

```xml
<!-- Example:  -->
<!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
<maintainer email="jeff@todo.todo">jeff</maintainer>
```

- 许可标签

```xml
<!-- One license tag required, multiple allowed, one license per tag -->
<!-- Commonly used license strings: -->
<!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
<license>BSD</license>
```

ROS核心组件的剩余部分已经使用了BSD协议，故这里选用BSD

- 依赖项标签

包括`build_depend` `buildtool_depend` `run_depend` `test_depend` 详情参考`Catkin Dependencies`相关文档

当编译和运行时需要用到指定依赖包，则还需要将每一个依赖包添加到`run_depend`标签中

- **最后完成的package.xml**

```xml
<buildtool_depend>catkin</buildtool_depend>
<!-- for build tool packages-->
<build_depend>roscpp</build_depend>	
<build_depend>rospy</build_depend>					
<build_depend>std_msgs</build_depend>
<!-- for packages you need at compile time-->
<build_export_depend>roscpp</build_export_depend>	
<build_export_depend>rospy</build_export_depend>
<build_export_depend>std_msgs</build_export_depend>
<!-- for packages you need in order to build against this package-->
<exec_depend>roscpp</exec_depend>					
<exec_depend>rospy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<!-- for packages you need at runtime-->
```

> `<depend>roscpp</depend>`==`<build_depend>roscpp</build_depend>` `<exec_depend>roscpp</build_depend>`

### 话题发布与订阅

#### 文件权限

```shell
$ ls -l /home/jeff/dogma_ws/src/beginner_tutorials/package.xml	'查看权限'
-rwxrwxrwx 1 jeff jeff 2908 Nov 16 16:08 /home/jeff/dogma_ws/src/beginner_tutorials/package.xml
$ chmod o w xxx.xxx		'为other授予read权限'
$ chmod go-rw xxx.xxx	'删除文件中group和other的read与write权限'
```

其中从左到右依次为：`-`表示类型，中间的`rw-`表示所有者user[u]，之后的`rw-`代表组群group[g]，最后的`r--`代表其他人other[o]，全部人all[a]（`r[4]`表示文件可读read，`w[2]`表示文件可写write，`x[1]`表示文件可被执行，`-[0]`表示相应的权限并未被授予）

|    指令    | 数字代号 |                        含义                        |
| :--------: | :------: | :------------------------------------------------: |
| -rw------- |   600    |                只有所有者有读写权限                |
| -rw-r--r-- |   644    |      只有所有者有读写权限，组群和其他人只能读      |
| -rwx------ |   700    |              只有所有者有读写执行权限              |
| -rwxr-xr-x |   755    | 只有所有者有读写执行权限，群组和其他人只能读、执行 |
| -rwx--x--x |   711    |  只有所有者才有读写执行权限，群组和其他人只能执行  |
| -rw-rw-rw- |   666    |                 每个人都有读写权限                 |
| -rwxrwxrwx |   777    |              每个人都有读写、执行权限              |

#### 定义消息类型

- 新建`msg`文件与`Num.msg`文件
- 定义`Num`消息类型
- 在`package.xml`中增加依赖

```
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
```

- 在`CMakeLists.txt`中增加依赖

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
 message_generation
)
```

- 在`CMakeLists.txt`中增加消息文件

```
add_message_files(
  FILES
  Num.msg
)
```

- 在`CMakeLists.txt`中增加消息生成包

```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

- 在`CMakeLists.txt`中增加编译依赖

```
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
```

#### 发布器

[rospyAPI接口](docs.ros.org/api/rospy/html/)

[ros/ros_tutorials/rospy_tutorials/001_talker_listener/talker.py](ros/ros_tutorials/rospy_tutorials/001_talker_listener/talker.py)

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String                                 
# 导入python标准字符处理库
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)     
    # Classes--发布主题为chatter，类型为std_msgs.msg.String，队列条目数为10
    rospy.init_node("talker", anonymous=True)                   
    # Functions--初始化ros节点talker，若anonymous=True则该节点无法remap
    rate = rospy.Rate(10)                                       
    # Classes--设置发布频率[Hz]，reset=False
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()         
        # Functions--返回当前浮点数时间[s]
        rospy.loginfo(hello_str)                                
        # Functions--屏幕输出日志信号，写入到rosout节点，可通过rqt_console查看
        pub.publish(hello_str)                                   
        # Method--发布消息到话题
        rate.sleep()                                            
        # Method--睡眠一定时间，若参数为负数，则立即返回
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:                         
        # 中断异常操作，ROSException下的子类
        pass
```

#### 订阅器

[ros/ros_tutorials/rospy_tutorials/001_talker_listener/listener.py](ros/ros_tutorials/rospy_tutorials/001_talker_listener/listener.py)

```python 
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    # 获取本地节点的全解析名称，例如/listener_1958_1574062415297[str]
def listener():
    rospy.init_node("listener", anonymous=True)
    # 初始化节点listener，anonymous=True可避免多个同名节点冲突，以保证同时运行多个listener.py，但也因此无法remap
    rospy.Subscriber("chatter", String, callback)
    # 订阅chatter主题，当有内容更新时调用callback，传入主题内容
    rospy.spin()
    # 保持节点运行，直到节点关闭，且不会影响订阅的回调函数，回调函数有自己的线程  
if __name__ == '__main__':
    listener()
```

#### 启动文件

在`beginner_tutorials`下新建`launch`文件夹用于存放启动文件`talker_and_listener.launch`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <node name="talker" pkg="beginner_tutorials" type="talker.py"/>
  <node name="listener" pkg="beginner_tutorials" type="listener.py"/>
</launch>
```

> `pkg`小心误键入空格

