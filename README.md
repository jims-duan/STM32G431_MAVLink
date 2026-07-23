# STM32G431CBT6
飞控，ROS通信桥梁

GPIO说明：
    LED：                              PC13
    USMART调试串口(LPUART1)：           PA2(Tx)    PA3(Rx)
    SBUS转发串口(USART3)：              PB10(Tx)    PB11(Rx)
    GPS模拟串口(USART1):                PA9(Tx)     PA10(Rx)
    USB虚拟串口：                       PA11(DM)    PA12(DP)
    bootloader：                       PA8，不在APP中，置低复位或上电会触发bootloader代码

程序烧录说明：
    基于状态机的软件架构，系统启动LED以500ms闪烁，出现呼吸灯效果说明正在执行bootloader程序，需烧录固件(在Jetson /home/nano/ROS/python目录下运行 python3 send.py G431_MAVLink.bin 或在/home/ggg/CubeMX/STM32G431_MAVLink/build/Release/c_arrays下运行python3 send.py G431_MAVLink.bin)，按照提示完成固件烧录(可在执行APP固件时进行烧录)，烧录后系统自动启动(未自动进入APP可断电重新进入)，若启动失败，检查固件大小和编译的FLASH大小是否一样。
    若无法进入bootloader，可将PA8引脚置低电平，重新上电即可强制进入。
    
    bin固件制作，在/home/ggg/CubeMX/STM32G431_MAVLink/build/Release目录下运行python3 main.py --full可生成bin文件到/home/ggg/CubeMX/STM32G431_MAVLink/build/Release/c_arrays中，注意文件大小和编译大小。

关于代码中的调试信息debug_info在Jetson的/home/nano/ROS/python下执行python3 mavlink_info.py即可看到(前提是打开mavros)。

led没有任何反应，可插拔USB，再重启mavros
