# swarm_robot_formation
一、	Arduino的代码
1、下载turtlebot的Arduino代码：
https://github.com/SwarmRoboticsSUSTechOPAL/turtlebot_arduino
2、根据如下教程配置环境：
http://emanual.robotis.com/docs/en/platform/turtlebot3/opencr1_0_software_setup/#usb-port-settings
3、在代码中搜索“SUSTC”，修改相应行的代码，如：
joint_states.header.frame_id = " robot1_base_footprint "; // "base_footprint" SUSTC
如果是2号机器人就将改成”robot2_base_footprint”
4、用Arduino将代码烧录到opencr板子上

二、	ROS的代码
1、	下载turtlebot的ROS代码：
https://github.com/SwarmRoboticsSUSTechOPAL/multi_turtlebot3_formation
2、	可以按照如下方式配置QTCreator开发环境：
https://ros-industrial.github.io/ros_qtc_plugin/_source/How-to-Install-Users.html
3、	在工程中搜索robot2，对出现的robot2进行修改就行了，如：
如果是3号机器人就将改成”robot3”
4、	将代码放到树莓派上进行编译，直接用catkin_make可能会出现资源不够导致卡住的问题，可以用catkin_make –j2进行编译，多试几次就能编译过了

三、	启动
1、	配置环境export TURTLEBOT3_MODEL=burger

2、	启动turtlebot硬件：
roslaunch turtlebot3_bringup turtlebot3_robot.launch
	
3、	启动navigation:
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
地图位于maps文件下，每个地图包含.pgm和.yaml两个文件，将两个文件同时拷到home目录下
Note：如果想使用自己环境下的地方，请使用如下包自行构建：
https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_slam

4、	启动formation
roslaunch formation formation.launch

5、	启动rviz:
export TURTLEBOT3_MODEL=burger
rosrun rviz rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_nav.rviz


四、	下面就是对应于不同编号的机器需要修改的对于文件（在工程中搜索SUSTC即可找到）
编队启动文件：
formation formation.launch

turtlebot硬件相关启动文件：
turtlebot3 turtlebot3_robot.launch（turtlebot3_core.launch、turtlebot3_lidar.launch）

turtlebot描述文件：
turtlebot3 turtlebot3_burger.urdf.xacro（turtlebot3_burger.gazebo.xacro）

导航启动及配置相关文件：
turtlebot3 amcl.launch.xml
turtlebot3 turtlebot3_navigation.launch(turtlebot3_remote.launch(description.launch.xml
(turtlebot3_burger.urdf.xacro)))
turtlebot3 global_costmap_params.yaml
turtlebot3 local_costmap_params.yaml

编队导航的rviz配置文件（不需要修改文件，可以在rivz界面上配置）：
turtlebot3 turtlebot3_nav.rviz



下面的一些启动文件是gazebo仿真相关的（如果想配合导航、slam包、编队包使用，需要修改一下机器人的编号）：
turtlebot3_simulations turtlebot3_world.launch
turtlebot3_simulations turtlebot3_gazebo_rviz.launch（turtlebot3_gazebo_model.rviz、description.launch.xml）启动rviz

turtlebot3_simulations turtlebot3_model.launch（model.rviz）
