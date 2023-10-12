#!/bin/bash
#@Echo off
#make clr_rotmat COM=COM9
#make clr_accl_offset COM=COM9
#@Echo on
BITRATE=250000
TOOL_DIR="../tools_python/"
EXEC="can_g552pc1_logger.py"
IF='socketcan'
CH='can0'
NODEID=1

read -p "Config(See conf_can.txt for detail) update? (y/n) > " CONF
if [ ${CONF} = "y" ];then
    ./Input_conf_can.sh
fi
. ./conf_can.txt

OS=`./get_pfInfo.sh`
if [ ${OS} == 'Linux' ];then
    ./can_init.sh ${BITRATE}
fi
EXEC="can_a552_logger.py"

if [ $# = 0 ]; then
    if [ ${MODEL} = "0" ];then
        python ${TOOL_DIR}${EXEC} -b ${BITRATE} --bitrate_new ${BITRATE_NEW} ${CSV} ${SYNC} ${DRATE} --can_id ${NODEID} --can_id_new ${NODEID_NEW} -m ${SAMPLE} ${SAVECFG} ${TEMPC}
    else
        python ${TOOL_DIR}${EXEC} -b ${BITRATE} --bitrate_new ${BITRATE_NEW} ${CSV} ${SYNC} ${DRATE} --can_id ${NODEID} --can_id_new ${NODEID_NEW} -m ${SAMPLE} ${SAVECFG} ${TEMPC}
    fi
    echo "Complete !"
else
    NUM=0
    for i in "$@"
        do
            if [ ${MODEL} = "0" ];then
                python ${TOOL_DIR}${EXEC} -b ${BITRATE} ${CSV} ${SYNC} ${DRATE} --can_id $i -m ${SAMPLE} ${SAVECFG} ${TEMPC}
            else
                python ${TOOL_DIR}${EXEC} -b ${BITRATE} ${CSV} ${SYNC} ${DRATE} --can_id $i -m ${SAMPLE} ${SAVECFG}${TEMPC}
            fi
            NUM=${NUM+1}
        done
    echo "Complete !"
fi
