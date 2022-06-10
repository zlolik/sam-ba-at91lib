rem set PATH=%PATH%;c:\Progs\GNU Tools ARM Embedded\7.2.1 2017q4\bin

pushd applets\legacy\sam-ba_applets\dataflash
make  BOARD=at91sam9260-ek CHIP=at91sam9260 MEMORIES=sdram -f Makefile clean all
make  BOARD=at91sam9xe-ek CHIP=at91sam9xe256 MEMORIES=sdram -f Makefile clean all
make  BOARD=at91sam9xe-ek CHIP=at91sam9xe512 MEMORIES=sdram -f Makefile clean all
popd

rem Do not forget copy tcl_lib\at91sam9xe512-ek\applet-dataflash-at91sam9xe512.bin 
