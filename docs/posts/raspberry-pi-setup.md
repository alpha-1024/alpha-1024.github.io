---
title: 树莓派配置教程
description: 从串口登录开始，完成树莓派的基础配置与 GPIO 检查。
date: 2025-06-21
tags: [树莓派, Linux, GPIO]
---

# 树莓派配置教程

树莓派使用 SD 卡作为系统盘。刷入镜像后，把 SD 卡插入开发板即可启动，不需要像很多 RK 系列开发板一样先在宿主机交叉编译并下载完整镜像。

| 配置项 | 建议值 |
| --- | --- |
| 串口波特率 | `115200` |
| 数据格式 | `8N1` |
| 登录方式 | 串口 / SSH |
| 系统盘 | microSD |

## 设置串口连接

初次配置时，串口是最稳定的登录方式。树莓派的蓝牙和串口通常复用同一个硬件模块，因此需要关闭蓝牙占用。

编辑 `/boot/config.txt`，加入：

```ini
dtoverlay=pi3-miniuart-bt
```

然后检查 `/boot/cmdline.txt`，确保串口控制台参数存在，例如：

```text
console=serial0,115200
```

保存后重启，使用 USB 转串口工具连接 GND、TX、RX，终端参数设置为 `115200 8N1`。

## 检查 GPIO

登录后可以使用 `gpio readall` 查看引脚映射：

```sh
gpio readall
```

物理引脚、BCM 编号和 WiringPi 编号并不相同，接线前请以当前型号的官方引脚图为准。不同版本的树莓派在 GPIO 复用和设备树配置上也可能存在差异。

## 后续配置建议

完成串口登录后，建议立即修改默认密码、启用 SSH 密钥登录，并执行一次系统更新：

```sh
sudo apt update && sudo apt full-upgrade -y
```
