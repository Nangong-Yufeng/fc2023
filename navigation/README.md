
# 1.环境搭建


克隆库
    
    https://github.com/ArduPilot/ardupilot.git

打开文件夹，安装一些依赖
    
    Tools/environment_install/install-prereqs-ubuntu.sh -y

第一次安装遇到了一些不知道严不严重的报错，选择了暂时无视。后来由于每次arm throttle都基本不成功，于是所幸重装一次，第二次没有出现报错，并且可以正常运行。所以重装解决99%的问题，遇事不决，建议重装。

执行

     ~/.profile

然后重启


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
    arm throttle  #解锁飞机（如果是旋翼机，解锁后要即使起飞，否则15s左右飞机会自动disarm）
    wp load ../Tools/autotest/Generic_Missions/CMAC-circuit.txt #载入航点文件（注意一下，ap文档里的斜杠是反的）
    mode auto #设置模式为自动，飞机会自动执行航点任务
    mode manual #切换为手动，飞机会退出航点任务

mavproxy还有其他的设置，也很有意思，比如加入遥控器连接，挖个坑，很想试一试。

要观察三维姿态图像的话，可以运行flightgear。在Tools/autotest文件夹中打开终端，运行

    bash fg_plane_view.sh #固定翼
    bash fg_quad_view.sh #旋翼机

当然，也可以打开其他GCS与其进行连接，如果是在同一台电脑上跑sitl一般可以自动连接，连接不上就要捣鼓一下端口了，一般也是默认的14550udp端口


# 3.使用代码进行任务接入

# 4.硬件连接

（1）数传

首次连接要对地面端和天空端的数传进行配对，参考教程，参数设置正确则数传会自动配对连接
     
    https://doc.cuav.net/tutorial/copter/optional-hardware/radio/3dr-radio/3dr-debug.html

注意事项：

    ardupilot和3dr数传不适配，要用v5数传
    注意天线头都是915或者都是433，混搭是连不上的

然后按照3dr使用教程即可远程连接飞控
    
（2）遥控器
