# Swarm Robot Formation
&emsp;&emsp;本项目需要使用到[move_base](http://wiki.ros.org/move_base)导航包，请预先安装
## 下面本项目的使用教程
#### 一、Arduino的代码
1、下载turtlebot的Arduino代码：</br>
&emsp;&emsp;https://github.com/SwarmRoboticsSUSTechOPAL/turtlebot3_arduino_sustc</br>

2、根据如下[教程](http://emanual.robotis.com/docs/en/platform/turtlebot3/opencr1_0_software_setup/#usb-port-settings)配置环境:</br>
&emsp;&emsp;http://emanual.robotis.com/docs/en/platform/turtlebot3/opencr1_0_software_setup/#usb-port-settings</br>

3、在代码中搜索“SUSTC”，修改相应行的代码，如：</br>
```
joint_states.header.frame_id = " robot1_base_footprint "; // "base_footprint" SUSTC 
```
&emsp;&emsp;如果是2号机器人就将"robot1_base_footprint"改成"robot2_base_footprint"</br>

4、用Arduino将代码烧录到opencr板子上</br>

#### 二、ROS的代码
1、下载turtlebot的ROS代码：</br>
&emsp;&emsp;https://github.com/SwarmRoboticsSUSTechOPAL/swarm_robot_formation</br>
&emsp;&emsp;https://github.com/SwarmRoboticsSUSTechOPAL/tuttlebot3_sustc</br>

2、如果想使用QTCreator进行ROS开发，可以按照如下方式配置QTCreator开发环境：</br>
&emsp;&emsp;https://ros-industrial.github.io/ros_qtc_plugin/_source/How-to-Install-Users.html</br>

3、在工程中搜索"sustc"，对出现的"robot*"进行修改就行了，如：</br>
&emsp;&emsp;如果是2号机器人就将改成"robot2"</br>

4、将代码放到树莓派上进行编译，直接用catkin_make可能会出现资源不够导致卡住的问题，可以用catkin_make –j2进行编译，多试几次就能编译过了</br>

#### 三、运行
1、配置环境export TURTLEBOT3_MODEL=burger</br>

2、启动turtlebot硬件(in robot)：</br>
&emsp;&emsp;roslaunch turtlebot3_bringup turtlebot3_robot.launch</br>

3、启动navigation(in robot):</br>
&emsp;&emsp;export TURTLEBOT3_MODEL=burger</br>
&emsp;&emsp;roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml</br>
&emsp;&emsp;本项目使用的地图位于[maps](https://github.com/SwarmRoboticsSUSTechOPAL/turtlebot_maps)中，每个地图包含.pgm和.yaml两个文件，将两个文件同时拷到home目录下并修改名称为"map"即可使用</br>
Note：如果想使用自己环境下的地方，请使用如下slam包自行构建：</br>
&emsp;&emsp;https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_slam</br>

4、启动formation</br>
&emsp;&emsp;roslaunch formation formation.launch</br>

5、启动rviz:</br>
&emsp;&emsp;export TURTLEBOT3_MODEL=burger</br>
&emsp;&emsp;rosrun rviz rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_nav.rviz</br>


#### 四、下面所列出的是对应于不同编号机器需要修改的文件（在工程中搜索SUSTC即可找到）
1、编队启动文件：</br>
&emsp;&emsp;formation formation.launch

2、turtlebot硬件相关启动文件：</br>
&emsp;&emsp;turtlebot3 turtlebot3_robot.launch（turtlebot3_core.launch、turtlebot3_lidar.launch）

3、turtlebot描述文件：</br>
&emsp;&emsp;turtlebot3 turtlebot3_burger.urdf.xacro（turtlebot3_burger.gazebo.xacro）

4、导航启动及配置相关文件：</br>
&emsp;&emsp;turtlebot3 amcl.launch.xml</br>
&emsp;&emsp;turtlebot3 turtlebot3_navigation.launch(turtlebot3_remote.launch(description.launch.xml
(turtlebot3_burger.urdf.xacro)))</br>
&emsp;&emsp;turtlebot3 global_costmap_params.yaml</br>
&emsp;&emsp;turtlebot3 local_costmap_params.yaml</br>

5、编队导航的rviz配置文件（不需要修改文件，可以在rivz界面上配置）：</br>
&emsp;&emsp;turtlebot3 turtlebot3_nav.rviz

6、下面的一些启动文件是gazebo仿真相关的（如果想在仿真环境下配合导航、slam包、编队包使用，需要修改一下机器人的编号）：</br>
&emsp;&emsp;turtlebot3_simulations turtlebot3_world.launch</br>
&emsp;&emsp;turtlebot3_simulations turtlebot3_gazebo_rviz.launch（turtlebot3_gazebo_model.rviz、description.launch.xml）启动rviz</br>

&emsp;&emsp;turtlebot3_simulations turtlebot3_model.launch（model.rviz）</br>
