pushd applets\legacy\sam-ba_applets\dataflash
make  BOARD=at91sam9xe-ek CHIP=at91sam9xe512 MEMORIES=sdram -f Makefile clean all
popd

rem Do not forget copy tcl_lib\at91sam9xe512-ek\applet-dataflash-at91sam9xe512.bin 
