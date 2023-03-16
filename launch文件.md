launch文件遵循[[XML#语法|xml语法]]，文件内容包围在一对\<launch>标签内。加粗的属性是必须的。

## \<node>

指定一个要启动的ROS节点，不保证启动顺序。

```xml
<node name="listener1" pkg="rospy_tutorials" type="listener.py" args="--test" respawn="true" />
```

### 属性

- **pkg**：表示节点所在的功能包名称，如pkg = “tf_lidar_task”
- **type**：表示节点的可执行文件名称，如type = “lidar_broadcaster”
- **name**：表示该节点在ROS系统中运行时的节点名，覆盖ros::init赋予的名称
- output="log|screen"：控制是否将日志信息打印在终端界面上，默认为log
- respawn="true|false"：表示在检测到节点停止时是否会自动重启，默认为false
- respawn_delay="30"：如果respawn为真，在检测到节点故障后重启节点前等待的秒数，默认为0
- required="true|false"：表示该节点是否必须要启动；如为真，当该节点终止时launch文件中的其他节点也被终止
- ns="foo"：为节点内的相对名称添加命名空间前缀，避免命名冲突
- args="arg1 arg2 arg3"：表示节点需要的输入参数

## \<include>

将另一个roslaunch XML文件导入当前文件。

```xml
<include file="$(find pkg-name)/path/filename.launch" ns="foo"/>
```

## \<param>

定义在参数服务器上设置的参数，还可以指定文本文件、二进制文件或命令属性来设置参数的值。可以放在\<node>标签内，此时参数被视为私有参数。

```xml
<param name="publish_frequency" type="double" value="10.0" />
```

### 属性

- **name**="namespace/name"：参数名称，可以包含命名空间
- value="value"：定义参数的值。如果省略则必须指定binfile、textfile或命令。
- type="str|int|double|bool|yaml"指定参数的类型。如果没有指定类型将尝试自动确定：
	- 带有'.'的数字是浮点，否则是整数；
	- "true"和"false"是布尔值（不区分大小写）。
	- 所有其他值都是字符串
- textfile="\$(find pkg-name)/path/file.txt"文件的内容将被读取并存储为字符串。该文件必须可在本地访问，但强烈建议使用与包相关的$(find)/file.txt语法来指定位置。
- binfile="\$(find pkg-name)/path/file"文件的内容将被读取并存储为base64编码的XML-RPC二进制对象。该文件必须可在本地访问，但强烈建议使用与包相关的$(find)/file.txt语法来指定位置。   
- command="\$(find pkg-name)/exe '\$(find pkg-name)/arg.txt'"命令的输出将被读取并存储为字符串。强烈建议使用与package-relative $(find)/file.txt语法来指定文件参数。由于XML转义要求，还应该使用单引号引用文件参数。

## \<rosparam>

使用yaml文件向参数服务器加载或删除参数，相当于[[命令行工具#rosparam|rosparam]]命令

```xml
<rosparam command="load" file="$(find rosparam)/example.yaml" />
<rosparam command="delete" param="my/param" />
```

### 属性

- command="load|dump|delete"（默认load）
- file="$(find pkg-name)/path/foo.yaml"（加载或转储命令） rosparam文件的名称。
- param="param-name"参数名称。
- ns="namespace"将参数限制到指定的命名空间。

## \<arg>

定义launch文件内部的参数

 ```xml
<!-- file.launch -->
<include file="included.launch">
  <!-- included.launch使用到的所有参数必须被设置 -->
  <arg name="hoge" value="fuga"/>
</include>
```

```xml
<!-- included.launch -->
<launch>
  <!-- 声明要被传入的参数 -->
  <arg name="hoge" default="1"/> 
  <!-- 读取参数的值 -->
  <param name="param" value="$(arg hoge)"/>
</launch>
```

## \<remap>

重新映射ROS计算图资源的名称，影响其后的行

```xml
<remap from="/turtlebot/cmd_vel"to"/cmd_vel"/>
```

