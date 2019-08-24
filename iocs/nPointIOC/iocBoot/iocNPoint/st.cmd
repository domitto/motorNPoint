#!../../bin/linux-x86_64/nPoint

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/nPoint.dbd"
nPoint_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=nPoint:")

##
< LC400.cmd
#< C300.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("nPoint:")

# Boot complete