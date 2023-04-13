[文档](http://wiki.ros.org/rospy?distro=melodic)

只要能导入 `rospy`，python 脚本就能与 ROS 计算图进行交互，无需 [[Catkin 编译系统]]。

---

# 节点

## 初始化节点

```python
rospy.init_node('my_node_name')
```

- anonymous=True

	匿名参数主要用于运行多个实例且不关心名称的节点。它在节点名称的末尾添加一个随机数，使其独一无二。如果在 ROS 图上检测到两个同名节点，则旧节点将关闭。

- log_level=rospy.INFO

	设置将日志消息发布到 rosout 的最小日志级别。

- disable_signals=True

	关闭 rospy 的信号处理，使程序可以捕获 SEGINT（KeyboardInterrupt，即 Ctrl-C）。默认情况下 SEGINT 会停止 ros 节点。使用了类似 `while input()` 的语句时，这样做才能使用 Ctrl-C 退出。

只有创建了节点才能使用 [[#日志]]。

## 节点状态

```python
rospy.get_name()               # 返回节点名称
rospy.is_shutdown()            # 返回节点是否关闭
rospy.spin()                   # 当前节点关闭前阻止python退出
rospy.on_shutdown(callback)    # 注册关闭节点时的回调函数
rospy.signal_shutdown(reason)  # 手动关闭节点，reason为字符串
```

---

# 消息

catkin_make 从 msg 文件生成 Python 类，详见[[计算图模型#构建自定义消息]]

-   package/msg/Foo.msg → package.msg.Foo

## 创建消息

- **无参数**：实例化空消息并填充要初始化的字段

	```python
	msg = sensor_msgs.msg.Imu()
	msg.header.stamp = rospy.Time.now()
	```

- **顺序参数**：按顺序为所有字段提供值

	```python
	msg = std_msgs.msg.ColorRGBA(255.0, 255.0, 255.0, 128.0)
	```

- **关键字参数**：只初始化提供值的字段，其余的接收默认值

	```python
	msg = std_msgs.msg.ColorRGBA(b=255)
	```

## 发布消息

```python
pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)
```

- **显式风格**：创建并发布

	```python
	pub.publish(std_msgs.msg.String("hello world"))
	```

- **带有顺序参数的隐式风格**：按顺序为所有字段提供值

	```python
	pub.publish(255.0, 255.0, 255.0, 128.0)
	```

- **带有关键字参数的隐式风格**：只初始化提供值的字段，其余的接收默认值

	```python
	pub.publish(b=255)
	```

## 接收消息（订阅主题）

```python
def callback(msg):
    rospy.loginfo("I heard %s",msg.data)
    
def listener():
    rospy.init_node('node_name')
    rospy.Subscriber("chatter", std_msgs.msg.String, callback)
    rospy.spin()
```

---

# 服务

catkin_make 从 srv 文件生成 Python 源代码，并创建三个类：服务定义、请求消息和响应消息。这些类的名称直接来自 srv 文件名：

-   package/srv/Foo.srv → package.srv.Foo
-   package/srv/Foo.srv → package.srv.FooRequest
-   package/srv/Foo.srv → package.srv.FooResponse

## 调用服务

调用服务不需要创建节点。

```python
rospy.wait_for_service('add_two_ints')
add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
try:
  resp1 = add_two_ints(x, y)
except rospy.ServiceException as exc:
  print("Service did not process request: " + str(exc))
```

服务的三种调用风格与[[rospy#发布消息|发布消息]]相同。

## 提供服务

通过创建一个带有回调函数的 rospy.Service 实例来提供服务，以便在收到新请求时调用。每个传入请求都在自己的线程中处理，因此服务**必须是线程安全的**。

```python
def add_two_ints(req):
      return rospy_tutorials.srv.AddTwoIntsResponse(req.a + req.b)
def add_two_ints_server():
      rospy.init_node('add_two_ints_server')
      s = rospy.Service('add_two_ints', rospy_tutorials.srv.AddTwoInts, add_two_ints)
      rospy.spin()
```

回调函数允许下列返回值：

-   None（失败）
-   ServiceResponse
-   元组，列表或字典（用于自动创建响应对象，其中的项只能是单个值而不能嵌套）
-   字段的值（仅限单参数响应）

# 参数服务器

注意：参数服务器方法不是线程安全的，因此如果您从多个线程使用它们，则必须适当锁定。

## 获取参数

	rospy.get_param(param_name)

从参数服务器获取值。如果参数未设置，可以选择传递默认值。名称相对于节点的命名空间进行解析。如果使用 get_param()获取命名空间，则返回字典，其键等于该命名空间中的参数值。如果未设置参数，则会引发 KeyError。

```python
global_name = rospy.get_param("/global_name")
relative_name = rospy.get_param("relative_name")
private_param = rospy.get_param('~private_name')
default_param = rospy.get_param('default_param', 'default_value')

# 获取一组参数（字典）
gains = rospy.get_param('gains')
p, i, d = gains['p'], gains['i'], gains['d']
```

## 设置参数

	rospy.set_param(param_name, param_value)

可以将参数设置为存储字符串、整数、浮点数、布尔值、列表和字典。字典必须有字符串键，因为它们被认为是命名空间。

```python
# 使用 rospy 和原始 python 对象
rospy.set_param('a_string', 'baz')
rospy.set_param('~private_int', 2)
rospy.set_param('list_of_floats', [1., 2., 3., 4.])
rospy.set_param('bool_True', True)
rospy.set_param('gains', {'p': 1, 'i': 2, 'd': 3})

# 使用 rosparam 和 yaml 字符串
rosparam.set_param('a_string', 'baz')
rosparam.set_param('~private_int', '2')
rosparam.set_param('list_of_floats', "[1., 2., 3., 4.]")
rosparam.set_param('bool_True', "true")
rosparam.set_param('gains', "{'p': 1, 'i': 2, 'd': 3}")

rospy.get_param('gains/p') #应该返回1
```

## 参数存在性

	rospy.has_param(param_name)

如果设置了参数，则返回 True，否则返回 False。

## 删除参数

	rospy.delete_param(param_name)

从参数服务器中删除参数。必须设置参数（如果没有设置，则会引发 KeyError）。名称相对于节点的命名空间进行解析。

```python
try:
    rospy.delete_param('to_delete')
except KeyError:
    print("value not set")
```

## 获取参数名称列表

	rospy.get_param_names()

返回参数服务器上所有参数名称的列表。

## 搜索参数键

	rospy.search_param(param_name)

查找最接近的参数名称，从私有命名空间开始，向上搜索到全局命名空间。如果找不到匹配项，则返回 None。

```python
param_name = rospy.search_param('global_example')
v = rospy.get_param(param_name)
```

如果此代码出现在节点/foo/bar 中，rospy.search_param 将尝试按顺序寻找参数

1.  /foo/bar/global_example
2.  /foo/global_example
3.  /global_example

# 日志

```python
rospy.log{debug|info|warn|err|fatal}(msg, *args, **kwargs)
```

如果 msg 是格式化字符串，可以单独传递字符串的参数，如

```python
rospy.logerr("%s returned the invalid value %s", other_name, other_value)
```

|          | Debug | Info | Warn | Error | Fatal |
| -------- |:-----:|:----:|:----:|:-----:|:-----:|
| stdout   |       |  X   |      |       |       |
| stderr   |       |      |  X   |   X   |   X   |
| log file |   X   |  X   |  X   |   X   |   X   |
| /rosout  |       |  X   |  X   |   X   |   X   |

/rosout 的输出级别在 [[#初始化节点]]时设置，标准输出是否会打印到屏幕上取决于 launch 文件中 output 参数的值。

定时记录日志，需要一直被运行：

```python
while True:
    rospy.loginfo_throttle(60, "This message will print every 60 seconds")
    rospy.loginfo_once("This message will print only once")
```

# 创建包

运行于 catkin 工作空间根目录下：

```shell
PKG_NAME="name"

mkdir ${PKG_NAME}
mkdir ${PKG_NAME}/src
mkdir ${PKG_NAME}/nodes
mkdir ${PKG_NAME}/launch
mkdir ${PKG_NAME}/msg
mkdir ${PKG_NAME}/srv

cd ${PKG_NAME}

echo "<?xml version=\"1.0\"?>
<package format=\"2\">
  <name>${PKG_NAME}</name>
  <version>1.0.0</version>
  <description>TODO</description>
  <maintainer email=\"todo@todo.todo\">TODO</maintainer>
  <license>TODO</license>

  <depend>rospy</depend>
  <buildtool_depend>catkin</buildtool_depend>
</package>" > package.xml
echo "cmake_minimum_required(VERSION 3.0.2)
project(${PKG_NAME})

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  mymsg.msg
)

add_service_files(
  FILES
  mysrv.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
)

include_directories(
  \${catkin_INCLUDE_DIRS}
)" > CMakeLists.txt
echo "from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[\"${PKG_NAME}\"],
    package_dir={\"\": \"src\"},
)

setup(**setup_args)" > setup.py

cd src

echo "import ${PKG_NAME}
" > utils.py

cd ../nodes

echo "#!/usr/bin/env python

import rospy

from ${PKG_NAME}.msg import mymsg
from ${PKG_NAME}.srv import mysrv, mysrvResponse
import utils

if __name__ == \"__main__\":
    rospy.init_node(\"${PKG_NAME}_server\")
    rospy.spin()" > node.py

chmod +x ./node.py

cd ../launch

echo "<launch>
  <node pkg=\"${PKG_NAME}\" type=\"node.py\" name=\"${PKG_NAME}_server\" output=\"screen\"/>
</launch>" > node.launch

cd ../msg

echo "int64 a" > mymsg.msg

cd ../srv

echo "int64 a
---
int64 b" > mysrv.srv

echo "手动向.vscode/settings.json的python.analysis.extraPaths中添加\"./src/${PKG_NAME}/src"
```

# 技巧

## 异常字符串解析

服务中抛出异常时，调用者捕获的异常字符串可能包含 16 进制内容，如

	service [/xxx] responded with an error: b'service cannot process request: \xe6\x9c\xaa\xe6\xa3\x80...

此时可以这样解码出 utf-8 字符：

```python
str(exp).encode('utf-8').decode('unicode_escape').encode('latin1').decode('utf-8')
```

## 异常 traceback 格式

未捕获异常的 traceback 由 `/opt/ros/melodic/lib/python2.7/dist-packages/rospy/impl/tcpros_service.py` 中的函数 `error_handler` 记录为日志：

```python
def error_handler(self, e, exc_type, exc_value, tb):
logerr("Error processing request: %s\n%s" % (e, traceback.format_exception(exc_type, exc_value, tb)))
```

此时产生的日志中 traceback 为列表，难以阅读：

```
[ERROR] [1680007708.746949]: Error processing request:['Traceback (most recent call last):\n', '  File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/impl/tcpros_service.py", line 633,in _handle_request\n    response = convert_return_to_response(self.handler(request), self.response_class)\n', ...
```

如下修改即可恢复格式：

```python
def error_handler(self, e, exc_type, exc_value, tb):
logerr("Error processing request: %s\n%s" % (e, traceback.format_exc()))
```

## 工作目录

由 [[命令行工具#rosrun|rosrun]] 启动的 python 脚本，工作路径为 rosrun 的执行路径；由 [[命令行工具#roslaunch|roslaunch]] 启动则为 `~/.ros`。