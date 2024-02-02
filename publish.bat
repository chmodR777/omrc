nkvd.exe split china_refmap.db --nkvds -wmr worldmanagerv3.nkvds --suffix _refmap -o output -f
cd ./output
zgen compress *.nkvds -r

echo done!
