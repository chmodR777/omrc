DEL *.bin /q
REM DEL *.nkvds /q
DEL *.db /q
DEL .\routing\* /s/q
DEL .\output-d\* /s/q
REM XCOPY .\worldmanager3\worldmanager3.nkvds ..\routing\ /s/e/y
perl ./omdb_and_routing_compile.pl
pause