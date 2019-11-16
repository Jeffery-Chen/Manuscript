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

