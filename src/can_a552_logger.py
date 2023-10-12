#!/usr/bin/env python
# coding: utf-8

###############################################################################
##  Disclaimer:
##  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
##  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT,
##  SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT
##  SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR IN CONNECTION
##  WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.
###############################################################################

"""
This python utility will retrieve CAN messages from an Epson Sensor Unit of CAN interface
with the specified CAN interface & channel and convert CAN message data
to angular rate & linear acceleration formated to the console or CSV file.

The file output columns are:
IMU: 
Sample No., Angular xRate[deg/sec], Angular yRate[deg/sec], Angular zRate[deg/sec],
xAccel[mG], yAccel[mG], zAccel[mG], CAN_Recv_Time, Time_Msg
, counter[count]
Q-Acc
Sample No., xAccel[mG], yAccel[mG], zAccel[mG], CAN_Recv_Time, Time_Msg
, counter[count]

LIMITATIONS:
- Using the --sync_hz <RATE> parameter to generate periodic SYNC messages has
  performance latencies issues in Python. Due to latencies in Python and
  depending on system speed there may be ~500 microsecond delay overhead
  to send a SYNC message. Therefore, SYNC rate will not be accurate at rates
  faster than 100Hz. The user should experiment to specify a SYNC faster rate
  than actual to reach fast SYNC rates.
- Alternatively (recommended), for accurate SYNC messages, use the PCAN-VIEW
  software running in background to generate SYNC messages by sending a
  a 0x80 CAN_ID message with no data (DLC=0)
"""

from __future__ import print_function

import time
import sys
import struct
import argparse
import math
import datetime as dt
import can
import tqdm
# NOTE: uptime library is not used by this program.
#       But is necessary by python-can library to generate valid
#       timestamps which this program requires.
import uptime
# Use below library for Test mode(--tmode) if necessary. 
# https://ja.manuals.plus/fnirsi/dc6006l-cnc-dc-adjustable-power-supply-manual


def parse_cli(open_browser=True):
    parser = argparse.ArgumentParser(
    description="This program will use the specified CAN interface to \
    read data, process, and output to CSV file"
    )
    parser.add_argument("-s", "--serial_port",
                    help="specifies the serial port of DC6006Lcomxx or /dev/ttyUSBx",
                    type=str,
                    default='com5'
                    )
    parser.add_argument(
    "-i",
    "--interface",
    help="specifies the CAN device interface name, \
            default is 'pcan' on Windows \
            and 'socketcan' on Linux, \
            See [Interface Names] on https://python-can.readthedocs.io/en/stable/configuration.html",
    type=str,
    default=None,
    )
    parser.add_argument(
    "-c",
    "--channel",
    help="specifies the CAN channel name, \
            default is 'PCAN_USBBUS1' on Windows\
            and 'can0' on Linux, \
            See [Hardware Interfaces] of each on https://python-can.readthedocs.io/en/stable/interfaces.html",
    type=str,
    default=None,
    )
    parser.add_argument(
    "-b",
    "--bitrate",
    help="specifies the CAN bitrate, \
            default is 1000000",
    type=int,
    default=1000000,
    )
    parser.add_argument(
    "--bitrate_new",
    help="specifies the CAN bitrate, \
            default is 1000000",
    type=int,
    default=1000000,
    )
    parser.add_argument(
    "--can_id",
    help="specifies CAN ID of the IMU which must be between, \
            1 ~ 127, default is 1",
    type=int,
    default=1,
    )
    parser.add_argument(
    "--can_id_new",
    help="specifies CAN ID of the IMU which must be between, \
            1 ~ 127, default is 1",
    type=int,
    default=1,
    )
    parser.add_argument(
    "--node_num",
    help="specifies Node Number of the IMU which must be between, \
            default is 1 max 8",
    type=int,
    default=1,
    )
    parser.add_argument(
    #"-s",
    "--sync_hz",
    help="specifies to generate periodic SYNC message at \
            output rate in Hz. This is intended for synchronous \
            mode sampling. If not specified, the default \
            is off (no SYNC messages)",
    type=int,
    default=None,
    )
    parser.add_argument("--filter",
                    help="specifies the filter selection. If not specified, \
                    moving average based on selected output data rate \
                    will automatically be selected. \
                    NOTE: Confirm valid settings:\
                    Output Rate vs Filter in Table 5.3 of the Datasheet. \
                    When selecting UDF, user must ensure UDF tap matches \
                    the UDF coefficients loaded using --udf switch.",
                    type=str,
                    choices=['MV_AVG0', 'MV_AVG2', 'MV_AVG4', 'MV_AVG8',
                             'MV_AVG16', 'MV_AVG32', 'MV_AVG64', 'MV_AVG128',
                             'K32_FC50', 'K32_FC100', 'K32_FC200', 'K32_FC400',
                             'K64_FC50', 'K64_FC100', 'K64_FC200', 'K64_FC400',
                             'K128_FC50', 'K128_FC100', 'K128_FC200','K128_FC400',
                             'K32_FC25', 'K32_FC50', 'K32_FC100', 'K32_FC200',
                             'K64_FC25', 'K64_FC50', 'K64_FC100', 'K64_FC200',
                             'K128_FC25', 'K128_FC50', 'K128_FC100',
                             'K128_FC200','K64_FC83', 'K64_FC220', 'K128_FC36',
                             'K128_FC110', 'K128_FC350', 'K512_FC9',
                             'K512_FC16', 'K512_FC60', 'K512_FC210',
                             'K512_FC460', 'UDF4', 'UDF64',
                             'UDF128', 'UDF512'], default=None)
    parser.add_argument("--tempc",
                    help="specifies to enable temperature data in sensor data",
                    action="store_true")
    parser.add_argument(
    "-t",
    "--time_per_nsamples",
    help="specifies to generate a TIME message every \
            N samples as specified. This TIME message \
            will come from host clock. If not specified, \
            the default is off \
            (no TIME messages sent)",
    type=int,
    default=None,
    )
    parser.add_argument("--drate",
                    help="specifies ACC output data interval in Hz, \
                    default is 1000Hz", type=float,
                    default=1000)
    parser.add_argument(
    "-o",
    "--outfile",
    help="specifies to output to .csv file, \
            otherwise output to console. \
            The file will use Date & Time \
            to generate filename with .csv",
    action="store_true",
    )
    parser.add_argument(
    "-m",
    "--max_sample",
    help="specifies # of IMU samples to capture before exiting \
            , default is 100",
    type=int,
    default=100,
    )
    parser.add_argument(
    "--tm_posix",
    help="specifies to format time as POSIX time format. \
            Number of seconds since Unix epoch, \
            00:00:00 UTC on 1 January 1970. \
            If not specified, the default is to format as \
            'YYYY-MM-DD hh:mm:ss.ssssss'",
    action="store_true",
    )
    parser.add_argument(
    "--no_count",
    help="specifies to NOT read 16-bit sample counter \
            embedded in TPDO1 \
            , if not specified, the default is to read the counter \
            and append as right-side column in the .csv",
    action="store_true",
    )
    parser.add_argument(
    "--noscale",
    help="specifies to not convert the ACCL & GYRO \
            data and leave it as unscaled 16-bit integers in the \
            .csv, if not specified, the default is disabled",
    action="store_true",
    )
    parser.add_argument(
    "--tag",
    help="specifies optional extra tag to append \
            to the .csv filename",
    type=str,
    )
    parser.add_argument(
    "--svcfg",
    help="save configuration",
    action="store_true",
    )
    parser.add_argument(
    "--tmode",
    help="specifies Test mode \
            0: Software reset Cyc \
            1: Power on/off Cyc \
                ",
    type=int,
    default=0,
    )
    
    parser.add_argument("--secs",
                    help="specifies time intervals of sending messages in \
                    seconds, default 2.5 seconds. \
                    ",
                    type=float,
                    default=2.5)
    return parser.parse_args()

#args = parse_cli()
# CANopen era starts from Jan 1, 1984
CANOPEN_ERA_ = dt.datetime(1984, 1, 1)
# user-defined imports
#####################################

# define model
##################
#IMU
MODEL = {
    "NAME": "M-G550PC2",
    "GYRO_SCL": 0.008,
    "ACCEL_SCL": 0.2,
    "TEMPC_OFF": 2634,
    "TEMPC_SCL": -0.0037918,
}

MODEL2 = {
    "NAME": "M-G552PCx",
    "GYRO_SCL": 0.0151515,
    "ACCEL_SCL": 0.4,
    "TEMPC_OFF": 2634,
    "TEMPC_SCL": -0.0037918,
}
#Q-ACC
MODEL_ACC = {
    "NAME": "M-A552AC10",
    "ACCEL_SCL": 5.96046e-5,
    "TILT_SCL": 1.86265e-9,
    "TEMPC_OFF": 2634,
    "TEMPC_SCL": 0.001,
}
#Filter G365
FILTER_SEL = {
	'MV_AVG0':	0x00,
    'MV_AVG2':    0x01,
    'MV_AVG4':   0x02,
    'MV_AVG8':   0x03,
    'MV_AVG16':  0x04,
    'MV_AVG32':  0x05,
    'MV_AVG64':    0x06,
    'MV_AVG128':   0x07,
    'K32_FC50':   0x08,
    'K32_FC100':  0x09,
    'K32_FC200':  0x0A,
    'K32_FC400':        0x0B,
    'K64_FC50':       0x0C,
    'K64_FC100':      0x0D,
    'K64_FC200':      0x0E,
    'K64_FC400':	0x0F,
    'K128_FC50':       0x10,
    'K128_FC100':      0x11,
    'K128_FC200':      0x12,
    'K128_FC400':	0x13,
}
#Filter G370
FILTER_SEL2 = {
	'MV_AVG0':	0x00,
    'MV_AVG2':    0x01,
    'MV_AVG4':   0x02,
    'MV_AVG8':   0x03,
    'MV_AVG16':  0x04,
    'MV_AVG32':  0x05,
    'MV_AVG64':    0x06,
    'MV_AVG128':   0x07,
    'K32_FC25':   0x08,
    'K32_FC50':  0x09,
    'K32_FC100':  0x0A,
    'K32_FC200':        0x0B,
    'K64_FC25':       0x0C,
    'K64_FC50':      0x0D,
    'K64_FC100':      0x0E,
    'K64_FC200':	0x0F,
    'K128_FC25':       0x10,
    'K128_FC50':      0x11,
    'K128_FC100':      0x12,
    'K128_FC200':	0x13,
}
#Filter for Q-Acc
FILTER_SEL_ACC = {
    'K64_FC83':    0x01,
    'K64_FC220':   0x02,
    'K128_FC36':   0x03,
    'K128_FC110':  0x04,
    'K128_FC350':  0x05,
    'K512_FC9':    0x06,
    'K512_FC16':   0x07,
    'K512_FC60':   0x08,
    'K512_FC210':  0x09,
    'K512_FC460':  0x0A,
    'UDF4':        0x0B,
    'UDF64':       0x0C,
    'UDF128':      0x0D,
    'UDF512':      0x0E
}
# Drate_select IMU
DRATE_SEL = {
    "2000": 0x00,
    "1000": 0x01,
    "500": 0x02,
    "250": 0x03,
    "125": 0x04,
    "62.5": 0x05,
    "31.25": 0x06,
    "15.625": 0x07,
    "400": 0x08,
    "200": 0x09,
    "100": 0x0a,
    "80": 0x0b,
    "50": 0x0c,
    "40": 0x0d,
    "25": 0x0e,
    "20": 0x0f,
}
# Drate_select Q-Acc
DRATE_SEL2 = {
    "1000": 0x01,
    "500": 0x02,
    "200": 0x05,
    "10": 0x0a,
    "50": 0x32,
}
BRATE_SEL = {
    '1000000':    0x00,
    '800000':    0x01,
    '500000':   0x02,
    '250000':   0x03,
    '125000':  0x04,
    '50000':  0x05,
    '20000':    0x06,
    '10000':   0x07,
}
# CAN message ID field
cob_id = {
    "NMT": 0x000,
    "SYNC": 0x080,
    "TIME": 0x100,
    "TPDO1": 0x180,
    "TPDO2": 0x280,
    "TPDO3": 0x380,
    "TPDO4": 0x480,
    "RSDO": 0x580,
    "TSDO": 0x600,
    "HBx": 0x700,
    "HB": 0x700,
    "HB1": 0x701,
    "HB2": 0x702,
    }

hb_id = {
    
}

def set_hb_id(node_num):
    key = []
    value = []
    for i in range(node_num):
        id=i+1
        hbstr = "HB" + str(id)
        key.append(hbstr)
        value.append(0x700 + id)
        hb_id.update(zip(key, value))

# NMT command
nmt = {
    "START": 0x01,
    "STOP": 0x02,
    "PRE-OP": 0x80,
    "RES_NODE": 0x81,
    "RES_COMM": 0x82,
}

tpdo1_filter = {
    "can_id": cob_id["TPDO1"],
    "can_mask": cob_id["TPDO1"],
    "extended": False,
}
tpdo2_filter = {
    "can_id": cob_id["TPDO2"],
    "can_mask": cob_id["TPDO2"],
    "extended": False,
}
tpdo3_filter = {
    "can_id": cob_id["TPDO3"],
    "can_mask": cob_id["TPDO3"],
    "extended": False,
}
tpdo4_filter = {
    "can_id": cob_id["TPDO4"],
    "can_mask": cob_id["TPDO4"],
    "extended": False,
}
op_filter = {
    "can_id": cob_id["HB"],
    "can_mask": 0x780,
    "extended": False,
}
rsdo_filter = {
    "can_id": cob_id["RSDO"],
    "can_mask": 0x780,
    "extended": False,
}
tsdo_filter = {
    "can_id": cob_id["TSDO"],
    "can_mask": 0x780,
    "extended": False,
}
hb_filter = {
    "can_id": cob_id["HB"],
    "can_mask": 0x700,
    "extended": False,
}
hb1_filter = {
    "can_id": cob_id["HB1"],
    "can_mask": 0x700,
    "extended": False,
}
hb2_filter = {
    "can_id": cob_id["HB2"],
    "can_mask": 0x700,
    "extended": False,
}

# Misc Constants
#################
GRAVITY = 9.80665

# Scale factor for gyro and accel
#GYRO_SF = MODEL["GYRO_SCL"] * math.pi / 180
#ACCL_SF = MODEL["ACCEL_SCL"] * GRAVITY / 1000
GYRO_SF = MODEL["GYRO_SCL"]
ACCL_SF = MODEL["ACCEL_SCL"]
TEMPC_SF = MODEL["TEMPC_SCL"]
TEMPC_25C = 0

def setModel(imuModel):
    """Sets the global variable of the IMU Model
    """
    sf={}
    length=0
    model_epson = imuModel
    length=len(imuModel)
    #print(length)
    if length <= 4:
        if imuModel == 'G320':
            # G320 specific constants
            SF_GYRO = 0.008  # dps/bit
            SF_ACCL = 0.2  # mg/bit
            SF_TEMP = -0.0037918  # degC/bit
            TEMPC_25C = -2634  # offset in 16bit mode @ 25C
        elif imuModel == 'G354':
            # G354 specific constants
            SF_GYRO = 0.016  # dps/bit
            SF_ACCL = 0.20  # mg/bit
            SF_TEMP = -0.0037918  # degC/bit
            TEMPC_25C = -2634  # offset in 16bit mode @ 25C
        elif imuModel == 'G570':
            # G365/G370PDF0 specific constants
            SF_GYRO = 0.0151515  # dps/bit
            SF_ACCL = 0.4  # mg/bit
            SF_TEMP = -0.0037918  # degC/bit
            TEMPC_25C = -2634  # offset in 16bit mode @ 25C
        elif imuModel == 'MIU':
            # G365/G370PDF0 specific constants
            SF_GYRO = 0.0151515  # dps/bit
            SF_ACCL = 0.4  # mg/bit
            SF_TEMP = -0.0037918  # degC/bit
            TEMPC_25C = -2634  # offset in 16bit mode @ 25C
    elif length >= 8:
        if imuModel == 'G364PDC0':
            # G364PDC0 specific constants
            SF_GYRO = 0.0075  # dps/bit
            SF_ACCL = 0.125  # mg/bit
            SF_TEMP = -0.0037918  # degC/bit
            TEMPC_25C = -2634  # offset in 16bit mode @ 25C
        elif imuModel == 'G364PDCA':
            # G364PDCA specific constants
            SF_GYRO = 0.00375  # dps/bit
            SF_ACCL = 0.125  # mg/bit
            SF_TEMP = -0.0037918  # degC/bit
            TEMPC_25C = -2634  # offset in 16bit mode @ 25C
        elif imuModel[:7] in ('G365PDC', 'G370PDC'):
            # G365/G370PDC0 specific constants
            SF_GYRO = 0.0151515  # dps/bit
            SF_ACCL = 0.16  # mg/bit
            SF_TEMP = -0.0037918  # degC/bit
            TEMPC_25C = -2634  # offset in 16bit mode @ 25C
        elif imuModel[:7] in ('G365PDF', 'G370PDF','G552PR7', 'G552PR1','G570PR1','G552PC1','G552PC7'):
            # G365/G370PDF0 specific constants
            SF_GYRO = 0.0151515  # dps/bit
            SF_ACCL = 0.4  # mg/bit
            SF_TEMP = -0.0037918  # degC/bit
            TEMPC_25C = -2634  # offset in 16bit mode @ 25C
        elif imuModel[:7] in ('G570PR2', 'G370PDG'):
            SF_GYRO = 0.0151515  # dps/bit
            SF_ACCL = 0.5  # mg/bit
            SF_TEMP = 0.0039063  # degC/bit
            TEMPC_25C = 0  # offset in 16bit mode @ 25C
        elif imuModel[:7] in ('G550PC2','G55T2A0','G55P200'):
            # G320 specific constants
            SF_GYRO = 0.008  # dps/bit
            SF_ACCL = 0.2  # mg/bit
            SF_TEMP = -0.0037918  # degC/bit
            TEMPC_25C = -2634  # offset in 16bit mode @ 25C
    elif length == 7:
        if imuModel in ('G552PR7', 'G552PR1','G570PR1','G365PDF','G370PDF'):
            SF_GYRO = 0.0151515  # dps/bit
            SF_ACCL = 0.4  # mg/bit
            SF_TEMP = -0.0037918  # degC/bit
            TEMPC_25C = -2634  # offset in 16bit mode @ 25C
        elif imuModel in ('G570PR2', 'G370PDG'):
            SF_GYRO = 0.0151515  # dps/bit
            SF_ACCL = 0.5  # mg/bit
            SF_TEMP = 0.0039063  # degC/bit
            TEMPC_25C = 0  # offset in 16bit mode @ 25C

    else:
        # Other specific constants
        SF_GYRO = 1  # dps/bit
        SF_ACCL = 1  # mg/bit
        SF_TEMP = 1  # degC/bit
        TEMPC_25C = 0  # offset in 16bit mode @ 25C
    sf['model_epson']=model_epson
    sf['SF_GYRO']=SF_GYRO
    sf['SF_ACCL']=SF_ACCL
    sf['SF_TEMP']=SF_TEMP
    sf['TEMPC_25C']=TEMPC_25C
    
    return sf

def set_cobid(node_id):
    global cob_id

    cob_id["TPDO1"] += node_id
    cob_id["TPDO2"] += node_id
    cob_id["TPDO3"] += node_id
    cob_id["TPDO4"] += node_id
    cob_id["RSDO"] += node_id
    cob_id["TSDO"] += node_id
    cob_id["HBx"] += node_id

def set_bus_filter(node_id):
    global cob_id
    global op_filter
    global tpdo1_filter
    global tpdo2_filter
    global tpdo3_filter
    global tpdo4_filter

    tpdo1_filter["can_id"] = cob_id["TPDO1"]
    tpdo1_filter["can_mask"] = cob_id["TPDO1"]

    tpdo2_filter["can_id"] = cob_id["TPDO2"]
    tpdo2_filter["can_mask"] = cob_id["TPDO2"]

    tpdo3_filter["can_id"] = cob_id["TPDO3"]
    tpdo3_filter["can_mask"] = cob_id["TPDO3"]

    tpdo4_filter["can_id"] = cob_id["TPDO4"]
    tpdo4_filter["can_mask"] = cob_id["TPDO4"]

def set_SCL(model):
    global GYRO_SF
    global ACCL_SF
    global TEMPC_SF
    global TEMPC_25C

    sf={}
    if  model[:7] in ('A552AC1','A55A200'):
        sf['SF_GYRO']=0
        sf['SF_ACCL']=5.96046e-5
        sf['TILT_SCL']=1.86265e-9
        sf['SF_TEMP']=-0.0037918
        sf['TEMPC_25C']=34.987
    else:
        sf=setModel(model)
    GYRO_SF = sf['SF_GYRO']
    ACCL_SF = sf['SF_ACCL']
    TEMPC_SF = sf['SF_TEMP']
    TEMPC_25C =sf['TEMPC_25C']




# Struct format for TPDO
# < = Little endian, B = unsigned char
# i = int (4 byte), I = unsigned int (4 byte)
# h = short (2byte), H = unsigned short (2 byte)
STRUCT_TPDO1 = "<ii"
STRUCT_TPDO2 = "<iH"
STRUCT_TPDO3 = "<HI"
STRUCT_TPDO4="<i"
STRUCT_HB="<B"
# Struct format for SDO
STRUCT_SDO4I = "<BHBI"
STRUCT_SDO4i = "<BHBi"
STRUCT_SDO2H = "<BHBHBB"
STRUCT_SDO2h = "<BHBhBB"
STRUCT_SDO1B = "<BHBBBBB"

def get_pdo_struct_fmt(model):
    global STRUCT_TPDO1
    global STRUCT_TPDO2
    global STRUCT_TPDO3
    global STRUCT_TPDO4

    if model[:1] in ('A'):
        STRUCT_TPDO1 = "<ii"
        STRUCT_TPDO2 = "<iH"
        STRUCT_TPDO3 = "<HI"
        STRUCT_TPDO4 ="<i"
    else:
        STRUCT_TPDO1 = "<Hhhh"
        STRUCT_TPDO2 = "<Hhhh"
        STRUCT_TPDO3 = "<Hhhh"
        STRUCT_TPDO4 = "<HIH"

# CAN driver setting
def can_ports(args):
    """Select default between Windows and Linux

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        CAN interface, channel
    """
    if sys.platform.startswith("win"):
        if args.interface is None:
            args.interface = "pcan"
            #args.interface = "nixnet"
        if args.channel is None:
            args.channel = "PCAN_USBBUS1"
            #args.channel = "can1"
    elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
        if args.interface is None:
            args.interface = "socketcan"
        if args.channel is None:
            args.channel = "can0"
    else:
        raise EnvironmentError("Unsupported platform")


# Sub function for CSV output
def print_header(file,args,model,ver,SNum):

    """
    Outputs header information before sensor row data
    """

    # Create Header Row
    if args.time_per_nsamples:
        print(
            "Sending Time message every {} samples".format(args.time_per_nsamples),
            file=file,
        )
    print(file=file)
    print(str(args).replace("Namespace", "Parameters"), file=file)
    print(file=file)
    print(
        "Log created by Python", "Press CTRL-C to exit early", file=file,
    )
    print(
        "Creation Date:",
        str(dt.datetime.now()),
        ",PROD_ID=" + model,
        ",VERSION=" + ver,
        ",SERIAL_NUM=" + SNum,
        file=file,
    )
    # Identify if outputting scaled or Raw digital data
    if args.noscale:
        print("Unscaled Data", file=file)
    else:
        print("Scaled Data", file=file)
        print("SF_GYRO={:+01.8f} (deg/sec)/lsb".format(GYRO_SF),
              "SF_ACCL={:+01.8f} (mG)/lsb".format(ACCL_SF),
              file=file
              )
    # Customize the ROW heading based on types of data included in burst
    print("Sample No.", end="", file=file)

    if args.noscale:
        print(",Gx[dec],Gy[dec],Gz[dec]", end="", file=file)
        print(",Ax[dec],Ay[dec],Az[dec]", end="", file=file)
    else:
        #print(", Gx[rad/s], Gy[rad/s], Gz[rad/s]", end="", file=file)
        #print(", Ax[m/s^2], Ay[m/s^2], Az[m/s^2]", end="", file=file)
        print(",Gx[deg/s],Gy[deg/s],Gz[deg/s]", end="", file=file)
        print(",Ax[mG],Ay[mG],Az[mG]", end="", file=file)
    print(",CAN_Recv_Time", end="", file=file)
    print(",Time_Message", end="", file=file)
    if args.tempc:
        print(",Tempc", end="", file=file)
    if not args.no_count:
        print(",Counter[dec]", end="", file=file)
    print(file=file)


def print_row(
        index_,
        gx_, gy_, gz_,
        ax_, ay_, az_,
        sample_count_,
        time_can_,
        time_days_,
        time_millisecs_,
        tempc_,
        file,
        args
):
    """
    Outputs sensor row
    """
    print("{:08}".format(index_), end="", file=file)
    if args.noscale:
        print(", {:+06}, {:+06}, {:+06}".format(gx_, gy_, gz_),
              end="", file=file)
        print(", {:+06}, {:+06}, {:+06}".format(ax_, ay_, az_),
              end="", file=file)
    else:
        print(", {:+05.06f}, {:+05.06f}, {:+05.06f}".format(gx_, gy_, gz_),
              end="", file=file)
        print(", {:+05.02f}, {:+05.02f}, {:+05.02f}".format(ax_, ay_, az_),
              end="", file=file)

    # CAN_recv timestamp is UTC
    can_ = dt.datetime.utcfromtimestamp(time_can_)
    # Calculate Timestamp from TIME message
    day_ = dt.timedelta(days=time_days_)
    ms_ = dt.timedelta(microseconds=time_millisecs_ * 1000)
    time_stamp_ = CANOPEN_ERA_ + day_ + ms_
    if args.tm_posix:
        # NOTE: Convert to POSIX time (UTC)
        print(", {:.06f}".format(can_.timestamp()), end="", file=file)
        print(", {:.03f}".format(time_stamp_.timestamp()), end="", file=file)
    else:
        print(",", can_, end="", file=file)
        print(",", time_stamp_, end="", file=file)
    if args.tempc:
        print(", {:+05.06f}".format(tempc_),end="", file=file)
    if not args.no_count:
        print(", {:05}".format(sample_count_), end="", file=file)
    print(file=file)

# Sub function for CSV output
def print_header_acc(file,args,model,ver,SNum):

    """
    Outputs header information before sensor row data
    """

    # Create Header Row
    if args.time_per_nsamples:
        print(
            "Sending Time message every {} samples".format(args.time_per_nsamples),
            file=file,
        )
    print(file=file)
    print(str(args).replace("Namespace", "Parameters"), file=file)
    print(file=file)
    print(
        "Log created by Python", "Press CTRL-C to exit early", file=file,
    )
    print(
        "Creation Date:",
        str(dt.datetime.now()),
        ",PROD_ID=" + model,
        ",VERSION=" + ver,
        ",SERIAL_NUM=" + SNum,
        file=file,
    )
    # Identify if outputting scaled or Raw digital data
    if args.noscale:
        print("Unscaled Data", file=file)
    else:
        print("Scaled Data", file=file)
        print("SF_ACCL={:+01.8f} mG/lsb".format(ACCL_SF),
              file=file
              )
    # Customize the ROW heading based on types of data included in burst
    print("Sample No.", end="", file=file)

    if args.noscale:
        print(",Ax[dec],Ay[dec],Az[dec]", end="", file=file)
    else:
        print(",Ax[mG],Ay[mG],Az[mG]", end="", file=file)
    print(",CAN_Recv_Time", end="", file=file)
    print(",Time_Message", end="", file=file)
    if args.tempc:
        print(",Tempc", end="", file=file)
    if not args.no_count:
        print(",Counter[dec]", end="", file=file)
    print(file=file)


def print_row_acc(
        index_,
        ax_, ay_, az_,
        sample_count_,
        time_can_,
        time_days_,
        time_millisecs_,
        tempc_,
        file,
        args
):
    """
    Outputs sensor row
    """
    print("{:08}".format(index_), end="", file=file)
    if args.noscale:
        print(", {:+06}, {:+06}, {:+06}".format(ax_, ay_, az_),
              end="", file=file)
    else:
        print(", {:+05.06f}, {:+05.06f}, {:+05.06f}".format(ax_, ay_, az_),
              end="", file=file)

    # CAN_recv timestamp is UTC
    can_ = dt.datetime.utcfromtimestamp(time_can_)
    # Calculate Timestamp from TIME message
    day_ = dt.timedelta(days=time_days_)
    ms_ = dt.timedelta(microseconds=time_millisecs_ * 1000)
    time_stamp_ = CANOPEN_ERA_ + day_ + ms_

    if args.tm_posix:
        # NOTE: Convert to POSIX time (UTC)
        print(", {:.06f}".format(can_.timestamp()), end="", file=file)
        print(", {:.03f}".format(time_stamp_.timestamp()), end="", file=file)
    else:
        print(",", can_, end="", file=file)
        print(",", time_stamp_, end="", file=file)
    if args.tempc:
        print(", {:+05.06f}".format(tempc_),end="", file=file)
    if not args.no_count:
        print(", {:05}".format(sample_count_), end="", file=file)
    print(file=file)

# Sub function for configuration
def sync_send(bus_, file,args):
    """
    Sends a SYNC message at specified sync rate in Hz
    """
    print("Send a SYNC message @ {} Hz".format(args.sync_hz), file=file)
    sync_msg = can.Message(arbitration_id=cob_id["SYNC"],
                           data=[], is_extended_id=False)
    task = bus_.send_periodic(sync_msg, 1 / args.sync_hz)
    return task


def time_send(bus_):
    """
    Sends a TIME message based on host time
    """
    canopen_datetime = dt.datetime.now() - CANOPEN_ERA_
    day_ = canopen_datetime.days
    ms_ = (canopen_datetime.seconds * 1000
           + canopen_datetime.microseconds / 1000) * 16
    time_of_day = struct.pack("<HI", day_, int(ms_))
    time_msg = can.Message(
        arbitration_id=cob_id["TIME"], data=time_of_day, is_extended_id=False
    )
    bus_.send(time_msg)


def nmt_send(bus_, cmd,node=0x00):
    """
    Sends a NMT message with specified command
    """

    nmt_msg = can.Message(
        arbitration_id=cob_id["NMT"], data=[cmd, node], is_extended_id=False
    )
    bus_.send(nmt_msg)
    
def sdo_write(bus_, byte,index,subindex,dat):
    if byte == 1:
        cmd = 0x2f
        STRCT=STRUCT_SDO1B
        can_data =struct.pack(STRCT, cmd, index, subindex,dat,0,0,0)
    elif byte == 2:
        cmd = 0x2b
        STRCT=STRUCT_SDO2H
        can_data =struct.pack(STRCT, cmd, index, subindex,dat,0,0)
    elif byte == 4:
        cmd = 0x23
        STRCT=STRUCT_SDO4I
        can_data =struct.pack(STRCT, cmd, index, subindex,dat)
    else:
        cmd =0x23
        STRCT=STRUCT_SDO4I
        can_data =struct.pack(STRCT, cmd, index, subindex,dat)
    
    sdo_msg = can.Message(
        arbitration_id=cob_id["TSDO"], data=can_data, is_extended_id=False
    )
    bus_.send(sdo_msg)
    
    time.sleep(0.001)

    STRCT=STRUCT_SDO4I
    for msg in bus_:
        if (msg.arbitration_id) == cob_id["RSDO"]:
                rcmd,rindex,rsubindex,rdat = struct.unpack(STRCT, msg.data)
                print("RSDO detected CAN_ID: {:x},{:x},{:x},{:x}".format(rcmd,rindex,rsubindex,rdat))
                break

    return rdat
    

def sdo_read(bus_,byte,index,subindex):
    cmd =0x40
    STRCT=STRUCT_SDO4I
    can_data =struct.pack(STRCT, cmd, index, subindex,0)
    sdo_msg = can.Message(
        arbitration_id=cob_id["TSDO"], data=can_data, is_extended_id=False
    )
    bus_.send(sdo_msg)
    time.sleep(0.001)
    if byte == 1:
        STRCT=STRUCT_SDO4I
    elif byte == 2:
        STRCT=STRUCT_SDO4I
    elif byte == 4:
        STRCT=STRUCT_SDO4I
    else:
        STRCT=STRUCT_SDO4I
    for msg in bus_:
        if (msg.arbitration_id) == cob_id["RSDO"]:
                rcmd,rindex,rsubindex,rdat = struct.unpack(STRCT, msg.data)
                print("RSDO detected CAN_ID: {:x},{:x},{:x},{:x}".format(rcmd,rindex,rsubindex,rdat))
                break

    return rdat

def get_model(bus_):
    prodcode = []
    result = []
    tmp=(sdo_read(bus_,4,0x1008,0))
    result.append((tmp) & 0xff)
    result.append((tmp >> 8) & 0xff)
    result.append((tmp >> 16) & 0xff)
    result.append((tmp >> 24) & 0xff)
    tmp=(sdo_read(bus_,4,0x1009,0))
    result.append((tmp) & 0xff)
    result.append((tmp >> 8) & 0xff)
    result.append((tmp >> 16) & 0xff)
    result.append((tmp >> 24) & 0xff)
    for item in result:
        prodcode.append(item & 0xFF)
    array=''.join(chr(i) for i in prodcode)

    return array

def get_ver(bus_):
    prodcode = []
    result = []
    tmp=(sdo_read(bus_,4,0x100a,0))
    result.append((tmp) & 0xff)
    result.append((tmp >> 8) & 0xff)
    result.append((tmp >> 16) & 0xff)
    result.append((tmp >> 24) & 0xff)
    for item in result:
        prodcode.append(item & 0xFF)
    array=''.join(chr(i) for i in prodcode)

    return array

def get_SN(bus_):
    prodcode = []
    result = []
    tmp=(sdo_write(bus_,1,0x3000,0x7e,0x01))
    tmp=(sdo_read(bus_,2,0x3000,0x74))
    result.append((tmp) & 0xff)
    result.append((tmp >> 8) & 0xff)
    tmp=(sdo_read(bus_,2,0x3000,0x76))
    result.append((tmp) & 0xff)
    result.append((tmp >> 8) & 0xff)
    tmp=(sdo_read(bus_,2,0x3000,0x78))
    result.append((tmp) & 0xff)
    result.append((tmp >> 8) & 0xff)
    tmp=(sdo_read(bus_,2,0x3000,0x7a))
    result.append((tmp) & 0xff)
    result.append((tmp >> 8) & 0xff)
    tmp=(sdo_write(bus_,1,0x3000,0x7e,0x00))
    for item in result:
        prodcode.append(item & 0xFF)
    array=''.join(chr(i) for i in prodcode)

    return array

def sdo_seq(bus_,byte,index,subindex,dat):
    pass

def load_param(bus_):
    #dat=int("load")
    dat=0x64616f6c
    sdo_write(bus_,4,0x1011,1,dat)
    
def save_param(bus_):
    #dat="save"
    dat=0x65766173
    sdo_write(bus_,4,0x1010,1,dat)
   
def ev_mode(bus_,intvl,model):
    dat =  str(int(intvl))
    sdo_write(bus_,1,0x1800,0x02,0xFE)
    if  model[:7] in ('A552AC1'):
        param=DRATE_SEL2[dat]
        sdo_write(bus_,4,0x2001,0,param)
    else:
        param=DRATE_SEL[dat]
        sdo_write(bus_,1,0x2001,0,param)
    
def sync_mode(bus_,intvl):
    dat =  intvl
    sdo_write(bus_,1,0x1800,0x02,0x01)
    sdo_write(bus_,4,0x2001,0,dat)
    
def apply_param(bus_,dat):
    sdo_write(bus_,1,0x2005,0x00,dat)
    time.sleep(1)

def filter_set(bus_,dat,model):
    if model[:1] in('A'):
        param=FILTER_SEL_ACC[dat]
    elif model[:7] in ('G552PC7'):
        param=FILTER_SEL2[dat] 
    else:
        param=FILTER_SEL[dat]
    sdo_write(bus_,1,0x61A1,0x01,param)
    time.sleep(3)

def brate_set(bus_,dat):
    param=BRATE_SEL[dat]
    sdo_write(bus_,1,0x2000,0x02,param)

def canid_set(bus_,dat):
    sdo_write(bus_,1,0x2000,0x01,dat)

def debug():
    can_ports(args)
    # low-level canbus parameter
    interface = args.interface
    channel = args.channel
    bitrate = args.bitrate
    bus = can.Bus(interface=interface, channel=channel, bitrate=bitrate)
    save_param(bus)
        

# Main sample program        
def main(arg):
    global ACCL_SF
    global GYRO_SF
    global TEMPC_SF
    global TEMPC_25C
    
    time_stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    
    print("Start: \t\t" + dt.datetime.now().ctime())
    set_cobid(args.can_id)
    set_hb_id(args.node_num)
    set_bus_filter(args.can_id)
    # If not specified set canbus device defaults depedning on OS
    can_ports(args)

    # low-level canbus parameter
    interface = args.interface
    channel = args.channel
    bitrate = args.bitrate

    # Setup CAN_ID filters to capture TPDO1 & TPDO2 & TPDO4 & HB
    bus_filters = [op_filter,tpdo1_filter, tpdo2_filter, tpdo3_filter, tpdo4_filter]#,rsdo_filter,tsdo_filter] #,hb_filter]

    try:
        bus = can.Bus(interface=interface, channel=channel, bitrate=bitrate)
        bus.set_filters(bus_filters)

        # This delay is required for the bus to stabilize and
        # allow the G550PC2 to exit BUS_HEAVY since no other device
        # may be acknowledging is CAN adapter just initialized
        time.sleep(1)
        
        # Place all nodes to ResetNode
        nr_num=0
        nr_time=args.secs
        nr_num=nr_num+1
        # Place all nodes to Pre-Op
        print("NMT: Pre-Op")
        nmt_send(bus, nmt["PRE-OP"],args.can_id)
        time.sleep(1)

        # Send a TIME message
        print("TIME: Sent")
        time_send(bus)
        time.sleep(2)

        model=get_model(bus)
        ver=get_ver(bus)
        SNum=get_SN(bus)
        print('MODEL:\t'+model)
        print('VER:\t'+ver)
        print('S/N:\t'+SNum)
    
        if not (args.can_id == args.can_id_new) :
            canid_set(bus,args.can_id_new)

        if not (args.bitrate == args.bitrate_new) :
            brate_set(bus,str(args.bitrate_new))

        # Generate filename tag based on specified settings for file management
        if model[:1] in ('A'):
            if args.sync_hz:
                fn_end = str(int(args.sync_hz)) + "_" + str(args.filter) + "_32B"
            else:
                fn_end = str(int(args.drate))+ "_" + str(args.filter) + "_32B"
        else:
            if args.sync_hz:
                fn_end = str(int(args.sync_hz)) + "_" + str(args.filter) + "_16B"
            else:
                fn_end = str(int(args.drate))+ "_" + str(args.filter) + "_16B"

        if args.noscale:
            fn_end = fn_end + "_NSC"
            # If noscale option then set scale factor to 1
            GYRO_SF = 1
            ACCL_SF = 1
            TEMPC_SF = 1
        else:
            fn_end = fn_end + "_SCL"
            # If noscale option then set scale factor from set_SCL()
            set_SCL(model)
        if (args.tempc):
            fn_end = fn_end + "_TEMPC"
        fn_end = fn_end + "_ID" + str(args.can_id)
        if (args.tag is None):
            args.tag = fn_end
        else:
            args.tag = fn_end + "_" + args.tag

        out_fname = time_stamp + "_" + model+ "_" + args.tag + ".csv"
        if args.outfile:
            f = open(out_fname, "a")
            print("Output File: \t" + out_fname)
        else:
            f = None

        get_pdo_struct_fmt(model)

        
        # flag to determine if complete set of TPDOs captured and ready
        # for output formatting
        tpdo_flg = 0
        tpdo_captured = 0
        hb_captured = 0
        sample_count1 = 0
        sample_count2 = 0
        sample_count3 = 0
        sample_count4 = 0
        gx = 0
        gy = 0
        gz = 0
        ax = 0
        ay = 0
        az = 0
        time_days = 0
        time_millisecs = 0
        tempc = 0
        index = 0
        hb_msg = 0
        hb1_num = 0
        hb2_num = 0

        if args.sync_hz:
            sync_mode(bus,1)
        else:
            ev_mode(bus,args.drate,model)
        
        if args.filter is not None:
            filter_set(bus,args.filter,model)

        if args.tempc:
            if model[:1] in ('A'):
                rd4=sdo_read(bus,4,0x1803,1)
                rd4=rd4 & ~0x80000000
                sdo_write(bus,4,0x1803,1,rd4)
            else:
                rd3=sdo_read(bus,4,0x1802,1)
                rd3=rd3 & ~0x80000000
                sdo_write(bus,4,0x1802,1,rd3)

        rd1=sdo_read(bus,4,0x1800,1)
        rd2=sdo_read(bus,4,0x1801,1)
        rd3=sdo_read(bus,4,0x1802,1)
        rd4=sdo_read(bus,4,0x1803,1)
        if(not (rd1 & 0x80000000)):
            tpdo_flg |= 0x01
        if(not (rd2 & 0x80000000)):
            tpdo_flg |= 0x02
        if(not (rd3 & 0x80000000)):
            tpdo_flg |= 0x04
        if(not (rd4 & 0x80000000)):
            tpdo_flg |= 0x08
        apply_param(bus,0x01)
        time.sleep(2)
        if args.svcfg:
            save_param(bus)
            time.sleep(3)
        # Place all nodes to Operational
        print("NMT: Start")
        nmt_send(bus, nmt["START"],args.can_id)
        time.sleep(0.25)

        if args.sync_hz:
            # Start sending SYNCs
            print("Send a SYNC message @ {} Hz".format(args.sync_hz))
            sync_send(bus, f,args)
        else:
            print("Timer event mode @ {} Hz".format(args.drate), file=f)

        if model[:1] in ('A'):
            print_header_acc(f,args,model,ver,SNum)
        else:
            print_header(f,args,model,ver,SNum)

        if args.outfile:
            pbar = tqdm.tqdm(total=args.max_sample, unit="samples")
        # iterate over received CAN messages
        for msg in bus:
            if (msg.arbitration_id) == hb_id["HB1"]:
                hb_msg = struct.unpack(STRUCT_HB, msg.data)
                hb1_num = hb1_num+1
                print("HB detected CAN_ID: {}".format(hb_msg),", HB1 cyc=: {} times".format(hb1_num))
                hb_captured |= 1
            elif msg.arbitration_id == cob_id["TPDO1"]:
                # NOTE: python-can can.Message.timestamp is POSIX UTC Time!
                can_timestamp = msg.timestamp
                if model[:1] in ('A'):
                    ax,ay = struct.unpack(STRUCT_TPDO1,msg.data)
                else:
                    sample_count1, gx, gy, gz = struct.unpack(STRUCT_TPDO1,msg.data)
                tpdo_captured |= 1
            elif msg.arbitration_id == cob_id["TPDO2"]:
                if model[:1] in ('A'):
                    az,sample_count1 = struct.unpack(STRUCT_TPDO2,msg.data)
                else:
                    sample_count2, ax, ay, az = struct.unpack(STRUCT_TPDO2,msg.data)
                tpdo_captured |= 2
            elif msg.arbitration_id == cob_id["TPDO3"]:
                if model[:1] in ('A'):
                    time_days, time_millisecs = struct.unpack(STRUCT_TPDO3, msg.data)
                    time_millisecs /= 16
                else:
                    sample_count3,angx, angy, angz = struct.unpack(STRUCT_TPDO3, msg.data)
                tpdo_captured |= 4
            elif msg.arbitration_id == cob_id["TPDO4"]:
                if model[:1] in ('A'):
                    tempc, = struct.unpack(STRUCT_TPDO4, msg.data)
                else:
                    sample_count4, time_millisecs, time_days = struct.unpack(STRUCT_TPDO4, msg.data)
                    time_millisecs /= 16
                tpdo_captured |= 8
            else:
                print("Unrecognized CAN_ID: {}".format(msg))
            # Only print row if we have complete set of samples
            if tpdo_captured == tpdo_flg:
                if model[:1] in ('A'):
                    print_row_acc(
                        index,
                        ax * ACCL_SF,
                        ay * ACCL_SF,
                        az * ACCL_SF,
                        sample_count1,
                        can_timestamp,
                        time_days,
                        time_millisecs,
                        (tempc * TEMPC_SF) + TEMPC_25C,
                        f,
                        args
                    )
                    
                else:
                    print_row(
                        index,
                        gx * GYRO_SF,
                        gy * GYRO_SF,
                        gz * GYRO_SF,
                        ax * ACCL_SF,
                        ay * ACCL_SF,
                        az * ACCL_SF,
                        sample_count1,
                        can_timestamp,
                        time_days,
                        time_millisecs,
                        (angx+TEMPC_25C) * TEMPC_SF+25,
                        f,
                        args
                    )
                tpdo_captured = 0
                if args.time_per_nsamples:
                    if index % args.time_per_nsamples == 0:
                        pass
                if args.outfile and (index % 50 == 0):
                    pbar.update(50)
                index += 1
            if index == args.max_sample:
                break
            
            
            # Place all nodes to ResetNode
            if hb_captured == 0x01:
                hb_captured = 0
            
    except (can.interfaces.pcan.pcan.PcanError) as e:
        print("PcanError. Check CAN hardware connection. {}".format(e))
        if args.outfile:
            f.close()
        sys.exit()
    except (can.CanError) as e:
        print("CanError. Check CAN hardware connection. {}".format(e))
        if args.outfile:
            f.close()
        sys.exit()
    except KeyboardInterrupt:
        print("CTRL-C exit")
    # Place all nodes to Pre-Op
    print("NMT: Pre-Op")
    nmt_send(bus, nmt["PRE-OP"],args.can_id)
    time.sleep(1)
    print("\nStop: \t\t" + dt.datetime.now().ctime())
    bus.stop_all_periodic_tasks()
    time.sleep(0.5)
    bus.shutdown()
    time.sleep(0.5)
    if args.outfile:
        pbar.close()
        f.close()
        

if __name__ == "__main__":
    args = parse_cli()
    main(args)
    

    
