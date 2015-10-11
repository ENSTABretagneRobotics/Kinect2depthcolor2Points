cd /d "%~dp0"

copy /Y /B %1 %1.bak
sleep 2
replaceinfile /infile %1.bak /outfile "%1" /searchstr "polylist" /replacestr "triangles"

pause
