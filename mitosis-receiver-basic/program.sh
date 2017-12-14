#!/bin/bash
echo '=============================== MAKING ================================'
cd custom/armgcc
make
if [[ $? -ne 0 ]] ; then
    exit 0
fi
sleep 0.1

echo
echo '============================= PROGRAMMING ============================='
{
	echo "reset halt";
	sleep 0.1;
	echo "flash write_image erase  ./mitosis-receiver-basic/custom/armgcc/_build/nrf51822_xxac.hex";
	sleep 11;
	echo "reset";
	sleep 0.1;
	exit;

} | telnet localhost 4444

echo
echo '============================== FINISHED ==============================='
