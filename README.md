国轩源码（调度通信、磁条驱动、磁条tf）

启动磁条驱动磁条tf程序
roslaunch magnetic_driver_tf magnetic_driver_tf.launch
启动与调度通信程序
roslaunch dispatch_robot_comm dispatch_robot_comm.launch

### Glog-CMake Support

#### 1.Get the source code and change to it. e.g. cloning with git:

```
git clone https://github.com/google/glog
sudo apt-get install autoconf automake libtool
cd glog
```

#### 2.Run CMake to configure the build tree.

```
cmake 版本在3.0以上
cmake -H. -Bbuild -G "Unix Makefiles"
```

note: To get the list of available generators (e.g. Visual Studio), use `-G ""`

#### 3.Afterwards, generated files can be used to compile the project.

```
cmake --build build
```

------



|    日期     |  作者  |                修改记录                |
| :-------: | :--: | :--------------------------------: |
| 2019-2-14 |  潘   | 1.socket通信函数write重写；2.增加谷歌日志系统glog |
|           |      |                                    |
|           |      |                                    |

