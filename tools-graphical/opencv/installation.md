# Installation

## Installation on Windows

* Download windows installer msi
* extract files to somewhere you want OpenCV to be installed
* set environment variables

## Version Check

Library files \(for example, .lib files under opencv\build\x64\vc15\lib\) has a suffix named after the version number. A ...452.lib represents an OpenCV version 4.5.2.

Another way to get the version is checking VERSION macros in opencv\sources\modules\core\include\opencv2\core\version.hpp. 

```text
#define CV_VERSION_MAJOR    4
#define CV_VERSION_MINOR    5
#define CV_VERSION_REVISION 2
```

## Reference

\[1\] [https://docs.opencv.org/master/d3/d52/tutorial\_windows\_install.html](https://docs.opencv.org/master/d3/d52/tutorial_windows_install.html)

\[2\] [https://stackoverflow.com/questions/11030640/how-to-determine-opencv-version](https://stackoverflow.com/questions/11030640/how-to-determine-opencv-version)

