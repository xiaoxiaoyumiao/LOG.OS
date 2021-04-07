# Basics

## USB Connection

* 将待调试的手机通过 USB 数据线连接开发计算机
* 在手机上打开开发者选项 - USB 调试
* 在开发计算机上 adb 所在目录下运行 `adb devices -l` ，此时 adb client 启动，并检测是否有 adb server 在运行，若没有则会自动启动，并将 client 连接至 server。
* 上面的命令会显示包含当前连接的所有模拟器和硬件设备的列表。

## File Transfer

> 如需从设备中复制某个文件或目录（及其子目录），请使用以下命令：

```text
adb pull remote local
```

> 如需将某个文件或目录（及其子目录）复制到设备，请使用以下命令：

```text
adb push local remote
```

> 将 `local` 和 `remote` 替换为开发机器（本地）和设备（远程）上的目标文件/目录的路径。

## Kill Server

> 如需停止 adb 服务器，请使用 `adb kill-server` 命令。然后，您可以通过发出其他任何 adb 命令来重启服务器。

## Reference

\[1\] [https://developer.android.google.cn/studio/command-line/adb](https://developer.android.google.cn/studio/command-line/adb)

