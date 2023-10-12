#!/bin/bash
#@Echo off
#make clr_rotmat COM=COM9
#make clr_accl_offset COM=COM9
#@Echo on
#echo "Input configuration parameter for multi IMU (MIU)"
NUM=0
COMX="/dev/ttyUSB0"
COM="/dev/ttyUSB"
TOOL_DIR="../tools_python/"

BITRATE=250000
TOOL_DIR="../tools_python/"
EXEC="can_g552pc1_logger.py"
IF='socketcan'
CH='can0'
NODEID=1

read -p "Model ? (0:A552 1:G552PC1 2:G552PJ )  > " MODEL
MODEL=${MODEL:-1}
read -p "CAN bitarate ? > " BITRATE
BITRATE=${BITRATE:-250000}
read -p "Change bitrate?(y/n) > " BITRATE_NEW
if [[ ${BITRATE_NEW} == "y" ]];then
    read -p "Bitrate?(Valid after save & reboot) > " BITRATE_NEW
    BITRATE_NEW=${BITRATE_NEW:-${BITRATE}}
else
    BITRATE_NEW=${BITRATE}
fi
read -p "Node ID ? > " NODEID
NODEID=${NODEID:-1}
read -p "Change Node ID?(y/n) > " NODEID_NEW
if [[ ${NODEID_NEW} == "y" ]];then
    read -p "Node ID?(Valid after save & reboot) > " NODEID_NEW
    NODEID_NEW=${NODEID_NEW:-${NODEID}}
else
    NODEID_NEW=${NODEID}
fi
read -p "Samples? > " SAMPLE
SAMPLE=${SAMPLE:-1000}
read -p "Data rate? > " DRATE
DRATE=${DRATE:-100}
read -p "Sync mode?(y/n) > " SYNC
if [[ -z ${SYNC} ]] || [[ ${SYNC} == "y" ]];then
    SYNC="--sync_hz"
else
    SYNC="--drate"
fi
read -p "csv?(y/n) > " CSV
if [[ -z ${CSV} ]] || [[ ${CSV} == "y" ]];then
    CSV="--outfile"
else
    CSV=
fi
read -p "temperature output?(y/n) > " TEMPC
if [[ -z ${TEMPC} ]] || [[ ${TEMPC} == "y" ]];then
    TEMPC="--tempc"
else
    TEMPC=
fi
read -p "Scaled data output?(y/n) > " NOSCALE
if [[ -z ${NOSCALE} ]] || [[ ${NOSCALE} == "y" ]];then
    NOSCALE="--noscale"
else
    NOSCALE=
fi
read -p "Configuration save ?(y/n) > " SAVECFG
if [[ -z ${SAVECFG} ]] || [[ ${SAVECFG} == "y" ]];then
    SAVECFG="--svcfg"
else
    SAVECFG=
fi
echo INTERFACE=${IF} > conf_can.txt
echo CHANNEL=&{CH} >> conf_can.txt
echo MODEL=${MODEL} >> conf_can.txt
echo BITRATE=${BITRATE} >> conf_can.txt
echo BITRATE_NEW=${BITRATE_NEW} >> conf_can.txt
echo NODEID=${NODEID} >> conf_can.txt
echo NODEID_NEW=${NODEID_NEW} >> conf_can.txt
echo SAMPLE=${SAMPLE} >> conf_can.txt
echo DRATE=${DRATE} >> conf_can.txt
echo SYNC=${SYNC} >> conf_can.txt
echo CSV=${CSV} >> conf_can.txt
echo TEMPC=${TEMPC} >> conf_can.txt
echo NOSCALE=${NOSCALE} >> conf_can.txt
echo SAVECFG=${SAVECFG} >> conf_can.txt

