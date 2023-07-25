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
This python utility will retrieve CAN messages from an Epson M-A552 IMU
with the specified CAN interface & channel and convert CAN message data
to angular rate & linear acceleration formated to the console or CSV file.

The file output columns are:

Sample No., xAccel[m/sec^2], yAccel[m/sec^2], zAccel[m/sec^2], CAN_Recv_Time, Time_Msg
, counter[count]

LIMITATIONS:
- This program assumes that the A552 has already been configured with desired
  output rate & filter setting
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
#from dc6006l import *

def parse_cli(open_browser=True):
    parser = argparse.ArgumentParser(
    description="This program will use the specified CAN interface to \
        read Acc data, process, and output to CSV file"
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
            and 'socketcan' on Linux",
    type=str,
    default=None,
    )
    parser.add_argument(
    "-c",
    "--channel",
    help="specifies the CAN channel name, \
            default is 'PCAN_USBBUS1' on Windows\
            and 'can0' on Linux",
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
    "--can_id",
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
                    choices=['K64_FC83', 'K64_FC220', 'K128_FC36',
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
                    default=1)
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

args = parse_cli()
# CANopen era starts from Jan 1, 1984
CANOPEN_ERA_ = dt.datetime(1984, 1, 1)
# user-defined imports
#####################################

# define model
##################
#ACC
MODEL = {
    "NAME": "M-A552AC10",
    "ACCEL_SCL": 5.96046e-5,
    "TILT_SCL": 1.86265e-9,
    "TEMPC_OFF": 2634,
    "TEMPC_SCL": 0.001,
}
#Filter
FILTER_SEL = {
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
# CAN message ID field
cob_id = {
    "NMT": 0x000,
    "SYNC": 0x080,
    "TIME": 0x100,
    "TPDO1": 0x180 + args.can_id,
    "TPDO2": 0x280 + args.can_id,
    "TPDO3": 0x380 + args.can_id,
    "TPDO4": 0x480 + args.can_id,
    "RSDO": 0x580 + args.can_id,
    "TSDO": 0x600 + args.can_id,
    #"HB": 0x700 + args.can_id,
    "HB": 0x700,
    "HB1": 0x701,
    "HB2": 0x702,
    }

hb_id = {
    
}

for i in range(args.node_num):
    id=i+1
    hbstr = "HB" + str(id)
    hb_id[hbstr] = cob_id["HB"] +id

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
ACCL_SF = MODEL["ACCEL_SCL"] #* GRAVITY / 1000
TEMPC_SF = MODEL["TEMPC_SCL"]

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
def print_header(file,args):

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
        "PROD_ID=" + MODEL["NAME"],
        "VERSION=" + "NA",
        "SERIAL_NUM=" + "NA",
        file=file,
    )
    # Identify if outputting scaled or Raw digital data
    if args.noscale:
        print("Unscaled Data", file=file)
    else:
        print("Scaled Data", file=file)
        print("SF_ACCL={:+01.8f} (m/sec^2)/lsb".format(ACCL_SF),
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
    print(",Tempc", end="", file=file)
    if not args.no_count:
        print(",Counter[dec]", end="", file=file)
    print(file=file)


def print_row(
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
    #time.sleep(0.01)
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
    #time.sleep(0.01)
    return rdat

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
   
def ev_mode(bus_,intvl):
    dat =  intvl
    sdo_write(bus_,1,0x1800,0x02,0xFE)
    sdo_write(bus_,4,0x2001,0,dat)
    
def sync_mode(bus_,intvl):
    dat =  intvl
    sdo_write(bus_,1,0x1800,0x02,0x01)
    sdo_write(bus_,4,0x2001,0,dat)
    
def apply_param(bus_,dat):
    sdo_write(bus_,1,0x2005,0x00,dat)
    time.sleep(1)

def filter_set(bus_,dat):
    param=FILTER_SEL[dat]
    sdo_write(bus_,1,0x61A1,0x00,param)
    time.sleep(3)

def debug():
    can_ports(args)
    # low-level canbus parameter
    interface = args.interface
    channel = args.channel
    bitrate = args.bitrate
    bus = can.Bus(interface=interface, channel=channel, bitrate=bitrate)
    save_param(bus)
        

# Main sample program        
def main():
    global ACCL_SF
    global TEMPC_SF
    #if not args:
    #    
        
    if args.noscale:
    # If noscale option then set scale factor to 1
        #GYRO_SF = 1
        ACCL_SF = 1
        TEMPC_SF = 1
    
    # Generate filename tag based on specified settings for file management
    if args.sync_hz:
        fn_end = str(int(args.sync_hz)) + "_" + str(args.filter) + "_32B"
    else:
        fn_end = str(int(args.drate))+ "_" + str(args.filter) + "_32B"
    if args.noscale:
        fn_end = fn_end + "_NSC"
    else:
        fn_end = fn_end + "_SCL"
    if (args.tempc):
        fn_end = fn_end + "_TEMPC"
    fn_end = fn_end + "_ID" + str(args.can_id)
    if (args.tag is None):
        args.tag = fn_end
    else:
        args.tag = fn_end + "_" + args.tag
    
    time_stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")

    out_fname = time_stamp + "_A552AC_" + args.tag + ".csv"

    if args.outfile:
        f = open(out_fname, "a")
        print("Output File: \t" + out_fname)
    else:
        f = None
        
    print("Start: \t\t" + dt.datetime.now().ctime())

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
        nmt_send(bus, nmt["PRE-OP"])
        time.sleep(1)

        # Send a TIME message
        print("TIME: Sent")
        time_send(bus)
        time.sleep(0.5)
        
        if args.sync_hz:
            sync_mode(bus,1)
        else:
            ev_mode(bus,int(1000/args.drate))
        
        if args.filter is None:
            if args.drate == 1000:
                args.filter = "K512_FC460"
            elif args.drate == 500:
                args.filter = "K512_FC210"
            elif args.drate == 200:
                args.filter = "K512_FC60"
            elif args.drate == 100:
                args.filter = "K512_FC16"
            elif args.drate == 50:
                args.filter = "K512_FC9"
            else:
                if args.sync_hz:
                    args.filter = "K512_FC460"
                else:
                    args.filter = "K512_FC9"
            filter_set(bus,args.filter)
        else:
            filter_set(bus,args.filter)
        # flag to determine if complete set of TPDOs captured and ready
        # for output formatting
        tpdo_flg = 0
        tpdo_captured = 0
        hb_captured = 0
        sample_count1 = 0
        sample_count2 = 0
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

        if args.tempc:
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
        #print(tpdo_flg)
        apply_param(bus,0x01)
        if args.svcfg:
            save_param(bus)
            time.sleep(3)
        # Place all nodes to Operational
        print("NMT: Start")
        nmt_send(bus, nmt["START"])
        time.sleep(0.25)

        if args.sync_hz:
            # Start sending SYNCs
            print("Send a SYNC message @ {} Hz".format(args.sync_hz))
            sync_send(bus, f,args)
        else:
            print("Timer event mode @ {} Hz".format(args.drate), file=f)

        print_header(f,args)
        if args.outfile:
            pbar = tqdm.tqdm(total=args.max_sample, unit="samples")
        # iterate over received CAN messages
        for msg in bus:
            if (msg.arbitration_id) == hb_id["HB1"]:
                hb_msg = struct.unpack(STRUCT_HB, msg.data)
                hb1_num = hb1_num+1
                print("HB detected CAN_ID: {}".format(hb_msg),", HB1 cyc=: {} times".format(hb1_num))
                hb_captured |= 1
            #elif (msg.arbitration_id) == hb_id["HB2"]:
            #    hb_msg = struct.unpack(STRUCT_HB, msg.data)
            #    hb2_num = hb2_num+1
            #    print("HB detected CAN_ID: {}".format(hb_msg),", HB2 cyc=: {} times".format(hb2_num))
            #    hb_captured |= 2
            elif msg.arbitration_id == cob_id["TPDO1"]:
                # NOTE: python-can can.Message.timestamp is POSIX UTC Time!
                can_timestamp = msg.timestamp
                ax,ay = struct.unpack(STRUCT_TPDO1,
                                                          msg.data)
                tpdo_captured |= 1
            elif msg.arbitration_id == cob_id["TPDO2"]:
                az,sample_count1 = struct.unpack(STRUCT_TPDO2,
                                                          msg.data)
                tpdo_captured |= 2
            elif msg.arbitration_id == cob_id["TPDO3"]:
                time_days, time_millisecs = struct.unpack(
                    STRUCT_TPDO3, msg.data
                )
                time_millisecs /= 16
                tpdo_captured |= 4
            elif msg.arbitration_id == cob_id["TPDO4"]:
                tempc, = struct.unpack(
                    STRUCT_TPDO4, msg.data
                )
                tpdo_captured |= 8
            else:
                print("Unrecognized CAN_ID: {}".format(msg))
            # Only print row if we have complete set of samples
            if tpdo_captured == tpdo_flg:
                print_row(
                    index,
                    ax * ACCL_SF,
                    ay * ACCL_SF,
                    az * ACCL_SF,
                    sample_count1,
                    can_timestamp,
                    time_days,
                    time_millisecs,
                    tempc * TEMPC_SF,
                    f,
                    args
                )
                tpdo_captured = 0
                if args.time_per_nsamples:
                    if index % args.time_per_nsamples == 0:
                        #time_send(bus)
                        pass
                if args.outfile and (index % 50 == 0):
                    pbar.update(50)
                index += 1
            if index == args.max_sample:
                break
            
            
            # Place all nodes to ResetNode
            if hb_captured == 0x01:
                nmt_send(bus, nmt["RES_NODE"])
                nr_num=nr_num+1
                print("NMT: node reset cyc=: {} times".format(nr_num))
                hb_captured = 0
                time.sleep(nr_time)
            
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
    nmt_send(bus, nmt["PRE-OP"])
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
    #debug()
    main()
    

    
