本文件用于记录ardupilot学习中的操作过程，计划可以作为后续的教程

由于先使用旋翼机来测试航点功能以及图传（旋翼机没那么容易炸），因此本文档关于硬件连接和飞行的部分也会有旋翼机相关

# 1.环境搭建


克隆库
    
    https://github.com/ArduPilot/ardupilot.git

打开文件夹，安装一些依赖
    
    Tools/environment_install/install-prereqs-ubuntu.sh -y

第一次安装遇到了一些不知道严不严重的报错，选择了暂时无视。后来由于每次arm throttle都基本不成功，于是索性重装一次，第二次没有出现报错，并且可以正常运行。所以重装解决99%的问题，遇事不决，建议重装。

执行

     .~/.profile


# 2.SITL基本指令运行


首次使用，打开ardupilot中你需要机型的文档，在终端运行

    sim_vehicle.py -w ##（-w用于擦除可能已有的错误参数，并配置默认参数）
        
可以使用sim_vehicle.py -h来获取帮助信息

完成配置后，运行

    sim_vehicle.py --console --map
    
其中--console用于打开mavprovy的参数界面，--map用于打开mavprovy的地图界面。Mavprovy似乎也是GCS（Ground Control Station）的一种，Mavprovy的文档在ardupilot网站中可以看到

Mavproxy的运行可以参考官方网站sitl中的视频，很原始的图形界面，但是反而很清晰，这真的是泰酷辣。指令参考文档

    https://ardupilot.org/mavproxy/docs/getting_started/cheatsheet.html
 
在此记录一些比较基本的指令过程

    mode guided  #设置为导引模式
    arm throttle  #解锁飞机（如果是旋翼机，解锁后要及时起飞，否则15s左右飞机会自动disarm）
    wp load ../Tools/autotest/Generic_Missions/CMAC-circuit.txt #载入航点文件（注意一下，ap文档里的斜杠是反的）
    mode auto #设置模式为自动，飞机会自动执行航点任务
    mode manual #切换为手动，飞机会退出航点任务

关于不同飞行模式的解释，参考：

    https://doc.cuav.net/tutorial/plane/flight-modes.html

mavproxy还有其他的设置，也很有意思，比如加入遥控器连接，挖个坑，很想试一试。

要观察三维姿态图像的话，可以运行flightgear。在Tools/autotest文件夹中打开终端，运行

    bash fg_plane_view.sh #固定翼
    bash fg_quad_view.sh #旋翼机

当然，也可以打开其他GCS与其进行连接，如果是在同一台电脑上跑sitl一般可以自动连接，连接不上就要捣鼓一下端口了，一般也是默认的14550udp端口


# 3.使用代码进行任务接入

Mavproxy是以命令行为端口的控制手段，没有上层api了

使用py代码控制的一个选择是使用dronekit，但是dronekit已经很多年未维护更新了，其mavlink协议已经落后，似乎已经不能正常使用了。

dronekit参考资料：

    https://ardupilot.org/dev/docs/droneapi-tutorial.html

ps：听说了dronekit已经很久没维护，但没想到因为mavlink任务协议的改变导致其cmd类不能和ardupilot正常通信了，尝试了使用simple_goto的方法代替跑航点，在模拟器中可行，但是有可能可靠度不足。

（另外Mavproxy也提供了二次开发的接口，编写一个自己的指令也是一种可能的任务规划方式）

考察后认为pymavlink是比较好的选择，官方文档参考：

    https://mavlink.io/en/mavgen_python/
    https://github.com/mavlink/MAVSDK-Python

pymavlink更多内容参考README

# 4.硬件连接

## （1）数传

首次连接要对地面端和天空端的数传进行配对，参考教程，参数设置正确则数传会自动配对连接
     
    https://doc.cuav.net/tutorial/copter/optional-hardware/radio/3dr-radio/3dr-debug.html

注意事项：

    一个数传只能支持一种飞控固件，不匹配的话在加载固件时会出现提示done但是参数不能调整的情况。目前队里标识为V5的数传支持ardupilot，标识为3DR的数传支持PX4，可以暂时区分（如果后续买了别的数传，建议贴标签）
    
    另外注意天线头都是915或者都是433，混搭是连不上的

然后按照3dr使用教程即可远程连接飞控
    
## （2）接收机

对我们使用的v5+飞控和Q9来说，用一种奇怪的线连接飞控的sbus（六口的）和R9DS接收机的横向接口。

理论上在遥控器校准页面上会出现绿色条，如果没有，可以参考这个

    http://www.nufeichuiyun.com/?p=28

遇事不决，重启重装

## （3）电机

多旋翼：按照雷讯文档，M1-M4连接四个电调即可

固定翼：M1副翼，M2升降，M3油门，M4偏航，其余通道可以自定义

## （4）GPS

V5+的GPS是八口的，同时还包括safety switch，如果出现了“Hardware safety switch”的preArm报错，可以在参数表中将BRD_SAFETYENABLE设为无效

其他飞控可能会不同，比如pixhawk是6+2接法，自行查找文档即可

# 5.飞行前配置

要正常arm，必要的配置有机架选择（仅多旋翼）、遥控器校准、指南针校准、加速计校准等。可以参考preArm报错来进行相应校准，可以参考

     https://doc.cuav.net/flight-controller/v5-autopilot/zh-hans/pre-arm.html

## ps

第一次配置建议到有正常gps信号（比如主楼前）的地方进行，刚开始时就算force arm了电机还是不反应；但解决所有prearm报错正常arm一次后，就可以通过force arm启动电机了，算是比较玄学的问题，因此不建议在没有gps的环境下浪费时间

## pps:

如果遇到电调短促鸣叫的情况，表明油门量需要校准。但需要注意的是，在有飞控的前提下，应该先给飞控上电并解锁后，再把油门打到最大并上电！

# 6.芜湖起飞

## 关于航点

多旋翼以自己起飞的位置作为home点；固定翼会以第一次连接到gps的位置作为home点（大概），建议自己设一个home点

## 关于地图

必须使用卫星地图！！！普通地图存在偏移

## 关于飞行模式

### 多旋翼

建议设置stabilize（增稳），loiter（悬停）和auto（自动）三个模式就差不多了。

需要稳定操作应使用loiter模式，stabilize模式实际并不是增稳，仅能保证未打杆时姿态水平，不能维持位置和高度

操作时要先用stabilize模式arm解锁，然后再切换别的模式。如果设置了航点，则在解锁后再切换到auto模式并稍微推油，飞机会自动执行航点飞行

### 固定翼

多旋翼默认使用旋钮作为模型选择开关，主要使用MANUAL,STABELIZE,FBWA,AUTO,RTL

MANUAL和直接连接收机是一样的；STABELIZE会使飞机保持平飞；FBWA是辅助飞行的优质选择，但是不能自动维持高度（FBWB可以定高）
