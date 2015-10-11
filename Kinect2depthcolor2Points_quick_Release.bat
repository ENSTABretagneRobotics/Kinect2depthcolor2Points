cd /d "%~dp0"
copy /y /b %1 color.png
copy /y /b %2 depth.png
Release\Kinect2depthcolor2Points.exe 
pause
