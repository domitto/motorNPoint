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
#dbpf nPoint:LC400:M1.NTM 0
#dbpf nPoint:LC400:M2.NTM 0
#dbpf nPoint:LC400:M3.NTM 0
#dbpf nPoint:LC400:M1.OMSL 1
#dbpf nPoint:LC400:M2.OMSL 1
#dbpf nPoint:LC400:M3.OMSL 1
