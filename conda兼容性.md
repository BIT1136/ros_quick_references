要让conda与ros一同使用，只需在python脚本第一行加上环境的python解释器路径[^1]，如

	#!/home/yangzhuo/mambaforge/envs/2dgrasp/bin/python

并在此环境中安装`rospkg`。此时脚本可以通过rosrun使用，能导入conda环境中的包，但导入的rospy来自`/opt/ros/melodic/lib/python2.7/dist-packages/`。

[^1]:这一写法被称为[Shebang](https://zh.wikipedia.org/wiki/Shebang)，此时可以直接以`./script.py`的形式运行python脚本。