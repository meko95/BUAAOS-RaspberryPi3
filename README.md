@[TOC]

## BUAA OS Raspberry Pi 3 - boot a small linux operating system

*笔者MacOS系统*

## 参考资料整理

- 一些驱动程序(Bare Metal)的资源: 

  https://github.com/bztsrc/raspi3-tutorial 

- 一些现有的可以提供参考的实现: 

  https://github.com/tonnylyz/Raspberry-Pi-3-JOS 
  https://github.com/Yradex/RaspberryPi3_OS 
	https://github.com/tonnylyz/RPI-EVO

## 实验报告

### Lab0 - 实验环境搭建
- host操作系统：Ubuntu Linux，Win10, both using Parallels Desktop
- 安装gcc交叉编译器
  - 查看路径：which gcc
  - 查看版本：cd /usr/bin, gcc -v
  - 试用：gcc hello.c -o hello, ./hello
- 硬件仿真器
	- 用到再说吧
- 安装minicom
	- sudo apt install minicom

### Lab1 - boot
#### 文件列表
##### 






