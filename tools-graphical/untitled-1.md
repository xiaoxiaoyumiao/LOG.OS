# X11

## Introduction

![](<../.gitbook/assets/image (5) (2).png>)

（source: [https://en.wikipedia.org/wiki/X\_Window\_System\_protocols\_and\_architecture](https://en.wikipedia.org/wiki/X\_Window\_System\_protocols\_and\_architecture)）

X (X Window System) 是一个基于 C/S 模型的位图显示系统，其核心为 X server，它与多个 X client 通信，并通过输入设备（鼠标、键盘等）与输出设备（屏幕等）实现与用户的图形交互。

Xorg X Server 是 Linux 系统中最流行的一种 X server 实现。Windows 系统则有 Xming、VcXsrv 等。

X server 与 X client 通过名为 X11 的协议进行通信。一般而言，软件可以调用实现了 X11 客户端协议的库与 X server 通信。常见的 X11 库如 Xlib、xcb。（ref: \[4]）

## GUI for Remote Server

TODO

在 Windows 上可以通过 PuTTY 建立开启 X11 Forwarding 的 ssh 连接，从而使远程服务器上的 X client 可以通过 ssh 通信连接到本地 Windows 中的 X server。在运行 X client 前确保本地的 X server 在正常运作（例如 Xming, VcXsrv）等，启动时需要选择 no access control。

## GUI for WSL

TODO

## xfvb

可以在诸如 remote server 这样的 headless 环境下为需要渲染图形界面才能运行的程序提供一个虚拟的显示界面（或者说一个 buffer）。

## Reference

\[1] [https://developer.ibm.com/tutorials/l-lpic1-106-1/](https://developer.ibm.com/tutorials/l-lpic1-106-1/)

\[2] [https://askubuntu.com/questions/213678/how-to-install-x11-xorg](https://askubuntu.com/questions/213678/how-to-install-x11-xorg)

\[3] [https://en.wikipedia.org/wiki/X\_Window\_System\_protocols\_and\_architecture](https://en.wikipedia.org/wiki/X\_Window\_System\_protocols\_and\_architecture)

\[4] [https://magcius.github.io/xplain/article/x-basics.html](https://magcius.github.io/xplain/article/x-basics.html)

\[5] [https://superuser.com/questions/1217280/why-is-x11-forwarding-so-inefficient](https://superuser.com/questions/1217280/why-is-x11-forwarding-so-inefficient)

\[6] [https://techcommunity.microsoft.com/t5/windows-dev-appconsult/running-wsl-gui-apps-on-windows-10/ba-p/1493242](https://techcommunity.microsoft.com/t5/windows-dev-appconsult/running-wsl-gui-apps-on-windows-10/ba-p/1493242)
