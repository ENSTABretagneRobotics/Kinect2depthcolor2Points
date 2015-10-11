cd /d "%~dp0"
"C:\Program Files (x86)\IrfanView\i_view32.exe" %1 /resize=(160,120) /aspectratio /convert=%1
rem "C:\Program Files (x86)\IrfanView\i_view32.exe" %1 /resize=(160,120) /aspectratio /convert=%~pn1_small%~x1

rem pause

exit



 i_view32.exe c:\test.bmp /convert=c:\giftest.gif
  => Convert file: 'c:\test.bmp' to 'c:\giftest.gif' without GUI ;-)
  i_view32.exe c:\*.jpg /convert=d:\temp\*.gif
  i_view32.exe c:\*.jpg /resize=(500,300) /aspectratio /resample /convert=d:\temp\*.png
  i_view32.exe /filelist=c:\mypics.txt /resize=(500,300) /aspectratio /resample /convert=d:\temp\*.png
  i_view32.exe c:\test.bmp /convert=c:\test_$Wx$H.jpg
  i_view32.exe c:\test.bmp /resize=(100,100) /resample /aspectratio /convert=d:\$N_thumb.jpg

i_view32.exe c:\test.jpg /resize=(300,300) /aspectratio
  => Open 'c:\test.jpg' and resize: width = max. 300, height = max. 300, proportional
