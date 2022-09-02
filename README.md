# PX4-wind_est
Developing: Wind estimation without airspeed sensors.
1.
需要在Firmware/boards/px4/sitl/default.cmake里面 EXAMPLE下添加 wind_est。
在文件中 wind_est_main中修改_c_torque和_c_force两个参数。
启动仿真时需要在起飞前手动打开 例如:wind_est start
（自启动需要rcS文件配置，后续上传）
2.
也可以将代码的主体部分放在姿态控制和位置控制中，位置在/src/modules/mc_att_coontrol和/mc_pos_control替换掉本来应该publish的力矩和推力消息进行发布。
但是调参有点麻烦，后续上传替换版本。
3.
wind_est_main主文件。
wind_est_params.c文件中和姿态解算的基本一样，目的是为了拿到初始姿态。
windy.world文件需要放在/Tools/sitl_gazebo/worlds下面。
4.
目前源码结构是px4风格。只考虑了iris一种机型。
iris的基本属性可以从 https://github.com/PX4/PX4-SITL_gazebo/blob/main/models/iris/iris.sdf.jinja 中读取。
后续计划发布gazebo_plugin版本和ros控制的离板控制（或许会比较方便获取电机转速等消息）。
