Catkin 是 CMake 的扩展，提高了自动化程度。

# Catkin 软件包

一个包要想称为 catkin 软件包，必须符合以下要求：

- 这个包必须有一个符合 catkin 规范的 package.xml 文件，提供有关该软件包的元信息
- 这个包必须有一个 catkin 版本的 CMakeLists.txt 文件，如果它是个 Catkin 元包的话，则需要有一个 CMakeList.txt 文件的相关样板
- 每个包必须有自己的目录，这意味着在同一个目录下不能有嵌套的多个软件包存在

# 创建软件包

```
catkin_create_pkg [--meta] [-V PKG_VERSION] [-D DESCRIPTION]
                        [-l LICENSE] [-a AUTHOR]
                        name [dependencies ...]

  name                  软件包名
  dependencies          依赖

  --meta                创建元包文件
  -V PKG_VERSION        包版本
  -D DESCRIPTION        描述
  -l LICENSE            许可证名, (e.g. BSD, MIT, GPLv3...)
  -m MAINTAINER         一个维护者名，可以使用多次
```

	catkin_create_pkg PACKAGE roscpp rospy std_msgs

# Catkin 工作空间

在其中运行 `catkin_make`

```
.
├── build //构建空间，存放中间文件
│   ├── catkin
│   ├── catkin_generated
│   ├── catkin_make.cache
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   ├── package1
│   ├── Makefile
│   └── ...
├── devel //开发空间，存放开发中可执行文件
│   ├── include
│   ├── lib
│   ├── setup.bash //编译后source来更新环境
│   └── ...
├── install //安装空间，最终可执行文件
└── src //源代码空间
    ├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
    └── package1
        ├── CMakeLists.txt
        ├── package.xml
        ├── .git
        ├── src
        ├── include
        ├── scripts //python脚本
        ├── launch //启动文件
        ├── msg //定义消息
        └── ...
```

# catkin_make

递归地查找 src/中的包进行构建

```
catkin_make [args]

  --directory DIRECTORY 工作空间的路径 (默认为 '.')
  --source SOURCE       src的路径 (默认为'workspace_base/src')
  --build BUILD         build的路径 (默认为'workspace_base/build')
  --force-cmake         强制cmake，即使已经cmake过
  --no-color            禁止彩色输出(只对catkin_make和CMake生效)
  --pkg PKG [PKG ...]   只对某个PKG进行make
  --only-pkg-with-deps  ONLY_PKG_WITH_DEPS [ONLY_PKG_WITH_DEPS ...]
                        将指定的package列入白名单CATKIN_WHITELIST_PACKAGES，
                        只编译白名单里的package。该环境变量存在于CMakeCache.txt。
  --cmake-args [CMAKE_ARGS ...]
                        传给CMake的参数
  --make-args [MAKE_ARGS  ...]
                        传给Make的参数
  --override-build-tool-check
                        用来覆盖由于不同编译工具产生的错误
```

## Python 3 兼容性

使用 catkin_make 编译时要使用 python 3 进行消息和服务的代码生成，只需要运行

	catkin_make -DPYTHON_EXECUTABLE:FILEPATH=/root/mambaforge/envs/name/bin/python

在这里指定的 python 解释器需要能访问 rospkg 和 empy 包，此时生成的 python 类文件位于 `devel/lib/python3/dist-packages/package_name`

要使 python3兼容 tf，参考[这里](https://zhuanlan.zhihu.com/p/578530492)

# package.xml（格式 2）

至少要有以下的最小标签嵌套在\<package>标签中才能使软件包清单完成。

- \<name> - 软件包的名称
- \<version> - 软件包的版本号（需要 3 个点分隔的整数）
- \<description> - 软件包内容的描述
- \<maintainer> - 维护软件包的人的姓名
- \<license> - 软件许可证

软件包可以有六种类型的依赖项：

- **构建依赖项**指定构建此软件包所需的软件包。当构建时需要这些软件包中的任何文件时，就会出现这种情况。这可以在编译时包含来自这些软件包的标头，与来自这些软件包的库链接，或在构建时需要任何其他资源（特别是当这些软件包在 CMake 中是_find_package()-ed 时）。在交叉编译场景中，构建依赖项适用于目标架构。
- **构建导出依赖项**指定构建此软件包的库所需的软件包。当您在此软件包中的公共标头中传递它们的标头时，就会出现这种情况（特别是当这些软件包在 CMake 的_catkin_package()_中声明为(CATKIN_)DEPENDS 时）。
- **运行依赖项**指定在此软件包中运行代码所需的软件包。当您依赖此软件包中的共享库时（特别是当这些软件包在 CMake 中的_catkin_package()中声明为(CATKIN_)DEPENDS 时）。
- **测试依赖项**仅为单元测试指定_额外的_依赖项。它们绝不应复制任何已提到的构建或运行依赖项。
- **构建工具依赖项**指定此软件包需要自行构建的构建系统工具。通常，唯一需要的构建工具是 catkin。在交叉编译场景中，构建工具的依赖性适用于执行编译的架构。
- **文档工具依赖项**指定此软件包生成文档所需的文档工具。

这六种类型的依赖项使用以下相应的标签指定：

- \<depend>指定依赖项是构建、导出和运行依赖项。这是最常用的依赖标签。
- <build_depend>
- <build_export_depend>
- <exec_depend>
- <test_depend>
- <buildtool_depend>
- <doc_depend>

所有软件包至少会有 catkin 作为构建工具依赖。

[格式参考](http://wiki.ros.org/catkin/package.xml)

# package.xml（格式 1）

如果\<package\>标签没有格式属性，则它是一个格式 1 包。其最小标签与格式 2 相同。

软件包可以有四种类型的依赖项：

- **构建工具依赖项**指定此软件包需要构建自己的构建系统工具。通常，唯一需要的构建工具是 catkin。在交叉编译场景中，构建工具依赖项适用于执行编译的架构。
- **构建依赖项**指定构建此软件包所需的软件包。当在构建时需要这些软件包中的任何文件时，情况就是这种情况。这可以包括在编译时来自这些软件包的标头，链接到这些软件包中的库，或在构建时需要任何其他资源（特别是当这些软件包在 CMake 中_查找_package()_-ed 时）。在交叉编译场景中，构建依赖项是针对目标架构的。
- **运行依赖项**指定在此软件包中运行代码所需的软件包，或针对此软件包构建库。当您依赖共享库或在此软件包中的公共标头中传递地包含其标头时，情况就是这种情况（特别是当这些软件包在 CMake 的 catkin_package()中声明为(CATKIN_)DEPENDS 时）。
- **测试依赖项**仅为单元测试指定_额外的_依赖项。它们永远不应该复制任何已经提到的构建或运行依赖项。

这四种类型的依赖项使用以下各自的标签指定：

- <buildtool_depend>
- <build_depend>
- <run_depend>
- <test_depend>

所有软件包至少会有 catkin 作为构建工具依赖。

# CMakeLists.txt

在 CMake 中 [[CMakeLists.txt]] 的基础上添加了一些宏。

## 查找编译依赖

除 catkin 依赖之外，项目依赖的其他软件包都会自动成为 catkin 的组件（components）。因此可以将这些依赖包指定为 catkin 的组件从而简化语法：

	find_package(catkin REQUIRED COMPONENTS rospy roscpp sensor_msgs)

## 生成消息和服务

见[[计算图模型#构建自定义消息]]和[[计算图模型#构建自定义服务]]。

这些宏必须在 `catkin_package()` 之前调用。

http://wiki.ros.org/catkin/CMakeLists.txt
https://blog.csdn.net/jinking01/article/details/102891918

# python 节点

https://blog.csdn.net/qq_38288618/article/details/103398428
http://docs.ros.org/en/jade/api/catkin/html/user_guide/setup_dot_py.html

#待整理 


在包的根目录下创建 setup.py，指定包名和搜索模块的路径：

```python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["package_name"],
    package_dir={"": "src"},
)

setup(**setup_args)
```

这会在 `catkin_make` 后将相应的导入路径设置存储到 `devel/lib/python2.7/dist-packages/name/__init__.py`，在 `import package_name` 或其中的模块后生效。

因此，未使用的 `import package_name` 不一定可以安全地删除。