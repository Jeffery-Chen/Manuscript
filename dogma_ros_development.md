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

#### rospy & roslib

对于`python`开发者而言，`roslib`与`rospy`提供的功能相近，如需使用`roslib`只需调用一下命令：

```python
import roslib; roslib.load_manifest('YOUR_PACKAGE_NAME')
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
    # 如果以固定速率发送信息，可以使用较小的队列大小；在突发多个消息的情况下应保证队列足够大
    # 例如发送频率在10Hz左右，设置1~3的队列大小是很合适的
    # queue_size=1 适用于传感器数据，即有了新信息就不发送旧信息
    # queue_size>10 适用于用户界面信息或想要记录所有已发布的值
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

### 启动文件

在`beginner_tutorials`下新建`launch`文件夹用于存放启动文件`talker_and_listener.launch`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <node name="talker" pkg="beginner_tutorials" type="talker.py"/>
  <node name="listener" pkg="beginner_tutorials" type="listener.py"/>
</launch>
```

> `pkg`小心误键入空格

### 使用参数

#### 操作参数

- 获取参数`rospy.get_param(param_name)`

```python
# 获取全局参数
rospy.get_param('/global_param_name')
# 获取目前命名空间的参数
rospy.get_param('param_name')
# 获取私有命名空间参数
rospy.get_param('~private_param_name')
# 获取参数，如果没，使用默认值
rospy.get_param('foo', 'default_value')
```

- 设置参数` rospy.set_param(param_name, param_value) `

```python
rospy.set_param('some_numbers', [1., 2., 3., 4.])
rospy.set_param('truth', True)
rospy.set_param('~private_bar', 1+2)
```

- 删除参数` rospy.delete_param('param_name') `

```python
rospy.delete_param('to_delete')
```

- 判断参数是否存在` rospy.has_param('param_name') `

```python
if rospy.has_param('to_delete'):
    rospy.delete_param('to_delete')
```

#### 解释参数名

- 获取参数的实际名称（被映射成不同名）`rospy.resolve_name(name)`

```python
value = rospy.get_param('~foo')
rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~foo'), value)
```

#### 搜索参数

- 当不知道命名空间时，可由私有命名空间开始向上到全局命名空间` rospy.search_param(param_name) `

```python
full_param_name = rospy.search_param('foo')
param_value = rospy.get_param(full_param_name)
```

### ROS日志

#### 写入日志API

```python
rospy.logdebug(msg, *args)
rospy.logwarn(msg, *args)
rospy.loginfo(msg, *args)
rospy.logerr(msg, *args)
rospy.logfatal(msg, *args)
```

当你看到一个消息输出在`stdout`不在`/rosout`，很可能是初始化未完成，或者忘记调用`rospy.init_node` 

节点日志文件路径一般为：

`ROS_ROOT/log`或者`~/.ros/log`

```
DEBUG = 1
ERROR = 8
FATAL = 16
INFO = 2
WARN = 4
```

如果要显示错误，首先需要监听`rosout`节点，

```shell
$ rostopic echo /rosout
```

### 脚本安装与模块导入

`Makefile`规定了工程源文件中的编译顺序（自动化编译，对于`ROS`工作空间即自动生成`message`和`service`代码）

如果需要导入自定义模块，最通用的方法是定义一个安装程序，将该模块移动到`PYTHONPATH`，具体方法如下：

- 在包根目录下（如`/my_pkg`）创建`__init__.py`
- 在包根目录下（如`/my_pkg`）创建`setup.py`

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

# setup.py是供catkin使用的，手动调用该文件可能会破坏你的ROS安装

from distutils.core import setup
# 不推荐setuptools，它会在src中生成文件egg-info
from catkin_pkg.python_setup import generate_distutils_setup
#使用generate_distutils_setup函数读取package.xml文件中的值，并作一些转换

setup_args = generate_distutils_setup(
    packages = ['tutorial_package'],
    package_dir = {'': 'src'},
    # 罗列并不在package.xml文件中的信息：引入的python包的路径为src/tutorial_package/
)

setup(**setup_args)
```

- 为了让`catkin`找到并使用`setup.py`，需要去掉`CMakeList.txt`中的注释

```
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
 catkin_python_setup()
```

- 为保证将包安装于正确路径，需要再修改`CMakeList.txt`

```
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   bin/hello	# 运行的主函数hello.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
```

- 回到工作空间目录下，编译包，之后便可使用`rosrun`执行自定义脚本与模块

