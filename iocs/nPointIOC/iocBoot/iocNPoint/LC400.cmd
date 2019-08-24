#emulated serial port (slower transfer rates)
#drvAsynSerialPortConfigure("serial1", "/dev/ttyUSB0", 0, 0, 0)
#asynSetOption("serial1",0,"baud","230400")
#asynSetOption("serial1",0,"bits","8")
#asynSetOption("serial1",0,"parity","none")
#asynSetOption("serial1",0,"stop","1")

#asynSetTraceIOMask("serial1", -1, 0x4)
#asynSetTraceMask("serial1", -1, 0x9)

#FTDI driver
drvAsynFTDIPortConfigure("LC400ftdi", "0x403", "0x6014","3000000", "2", "0", "0", "1")

dbLoadRecords("auxParameters.db","PORT=LC400,ADDR=0,P=nPoint:,R=LC400:")
dbLoadRecords("auxParameters.db","PORT=LC400,ADDR=1,P=nPoint:,R=LC400:")
dbLoadRecords("auxParameters.db","PORT=LC400,ADDR=2,P=nPoint:,R=LC400:")

dbLoadTemplate("LC400.substitutions")

# LC400CreateController(
#      port name,
#      serial port,
#      num axes,
#      moving poll period (ms),
#      idle poll period (ms))
LC400CreateController("LC400", "LC400ftdi", 3, 100, 1000)
#LC400CreateController("LC400", "serial1", 3, 100, 1000)
