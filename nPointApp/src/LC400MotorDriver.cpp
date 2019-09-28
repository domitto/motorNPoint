/*
FILENAME... LC400MotorDriver.cpp
USAGE...    Motor driver support for the nPoint LC400 series of controllers

Diego Omitto
August, 2019

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <assert.h>
#include <unistd.h>
#include <sys/time.h>

#include <iocsh.h>
#include <epicsThread.h>
#include <epicsMutex.h>
#include <epicsGuard.h>
#include <epicsTime.h>

#include <asynOctetSyncIO.h>

#include "LC400MotorDriver.h"
#include <epicsExport.h>

typedef epicsGuard<epicsMutex> Guard;

static const char *driverName = "LC400MotorDriver";

/** Get Motor Controller base address for an axis
  * \param[in] axis  The axis number
  */
epicsUInt32 getBaseAddress(epicsInt32 axis)
{
  if (axis == 0)
    return(CH1_BASE_ADDR);
  else if (axis == 1)
    return(CH2_BASE_ADDR);
  else if (axis == 2)
    return(CH3_BASE_ADDR);
  else
    return 0;
}

/** Get Motor Controller wavetable array address for an axis
  * \param[in] axis  The axis number
  */
epicsUInt32 getWavetableAddress(epicsInt32 axis)
{
  if (axis == 0)
    return(CH1_WAV_ADDR);
  else if (axis == 1)
    return(CH2_WAV_ADDR);
  else if (axis == 2)
    return(CH3_WAV_ADDR);
  else
    return 0;
}

/** Convert axis sensor cts to Position
  * \param[in] cts  Sensor cts
  */
epicsFloat64 getPosition(epicsInt32 cts)
{
  return static_cast<epicsFloat64>(cts*100.0/1048574.0);
}

/** Convert axis Position to sensor cts
  * \param[in] pos  Position
  */
epicsInt32 getCts(epicsFloat64 pos)
{
  return static_cast<epicsInt32>(pos*1048574.0/100.0);
}


/** Creates a new LC400Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] LC400PortName       The name of the drvAsynIPPPort that was created previously to connect to the C300 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
LC400Controller::LC400Controller(const char *portName, const char *LC400PortName, epicsInt32 numAxes, 
                             epicsFloat64 movingPollPeriod, epicsFloat64 idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_LC400_PARAMS, 
                         asynUInt32DigitalMask, 
                         asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  epicsInt32 axis;
  asynStatus status;
  static const char *functionName = "LC400Controller";
 
  // Create controller-specific parameters
  //static positioning parameters
  createParam(LC400RangeString,         asynParamInt32,   &LC400_Range_);
  createParam(LC400RangeTypeString,     asynParamInt32,   &LC400_Range_Type_);
  createParam(LC400DigitalPosString,    asynParamFloat64, &LC400_Digital_Pos_);
  createParam(LC400ServoStateString,    asynParamInt32,   &LC400_Servo_State_);
  createParam(LC400ProportionalGString, asynParamFloat64, &LC400_P_Gain_);
  createParam(LC400IntegralGString,     asynParamFloat64, &LC400_I_Gain_);
  createParam(LC400DerivativeGString,   asynParamFloat64, &LC400_D_Gain_);
  createParam(LC400FirmwareString,      asynParamOctet,   &LC400_FW_);
  //wavetable movement parameters
  createParam(LC400WavetableEnString,   asynParamInt32,   &LC400_W_Enable_);
  createParam(LC400WavetableIdxString,  asynParamInt32,   &LC400_W_Index_);
  createParam(LC400WavetableCDlyString, asynParamInt32,   &LC400_W_Delay_);
  createParam(LC400WavetableEIdxString, asynParamInt32,   &LC400_W_EndIdx_);
  createParam(LC400WavetableActString,  asynParamInt32,   &LC400_W_Active_);
  createParam(LC400WavetableItString,   asynParamInt32,   &LC400_W_Iter_);
  createParam(LC400WavetableItCtsString,asynParamInt32,   &LC400_W_IterCts_);
  createParam(LC400WavetableMaxPts,     asynParamInt32,   &LC400_W_MaxPts_);

  /* Connect to LC400 controller */
  status = pasynOctetSyncIO->connect(LC400PortName, 0, &pasynUserLC400_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s:%s: cannot connect to LC400 controller\n",
      driverName, functionName);
  }

  epicsInt32 fwVersion[5];

  status = readArray(0x11830028, 5, fwVersion, sizeof(fwVersion));
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s:%s: cannot read LC400 controller firmware version\n",
      driverName, functionName);
  }
  printf("Firmware Version: %s\n", (char*)fwVersion);
  setStringParam (LC400_FW_, (char*)fwVersion);

  this->numAxes=numAxes;
  // Create the axis objects
  for (axis=0; axis<numAxes; axis++) {
    new LC400Axis(this, axis);
  }
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Creates a new LC400Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] LC400PortName       The name of the drvAsynIPPPort that was created previously to connect to the C300 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" epicsInt32 LC400CreateController(const char *portName, const char *LC400PortName, epicsInt32 numAxes, 
                                   epicsInt32 movingPollPeriod, epicsInt32 idlePollPeriod)
{
  new LC400Controller(portName, LC400PortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void LC400Controller::report(FILE *fp, epicsInt32 level)
{
  epicsInt32 axis;
  LC400Axis *pAxis;

  fprintf(fp, "LC400 motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  if (level > 0) {
    for (axis=0; axis<numAxes_; axis++) {
      pAxis = getAxis(axis);
      fprintf(fp, " axis %d\n",
              pAxis->axisNo_);
    }
  }

  // Call the base class method
  asynMotorController::report(fp, level);
}

/*
 * Generate a waveform of maxpts positions with a trapezoidal velocity profile.
 * \param[in]  ipos   Initial Position (cts)
 * \param[in]  fpos   Final Position (cts)
 * \param[in]  vel    Velocity (positive cts/sec)
 * \param[in]  acc    Acceleration (positive cts/sec^2)
 * \param[in]  maxpts Max number of pts for the movement
 * \param[out] result Generated waveform with cycle count
 */
static void genTrapezoid(epicsFloat64 ipos, epicsFloat64 fpos, epicsFloat64 vel, epicsFloat64 acc, waveform_t *result, size_t maxpts=LC400_MAX_ARRAY)
{
    assert(result);
    assert(vel > 0);
    assert(acc > 0);

    // Calculate necessary times and distances

    epicsFloat64 d_tot = fabs(fpos-ipos);         // Total requested distance
    epicsFloat64 t_acc = vel/acc;                 // Time accelerating (sec)
    epicsFloat64 d_acc = (vel*vel) / (2.0*acc);   // Distance while accelerating

    epicsFloat64 d_cnt = d_tot - 2.0*d_acc;       // Distance at constant speed
    epicsFloat64 t_cnt = d_cnt / vel;             // Time at constant speed

    // Check if there is enough room for accel and deccel
    if (t_cnt < 0.0) {
        t_cnt = 0.0;
        d_cnt = 0.0;
        d_acc = d_tot / 2.0;
        t_acc = sqrt(2.0*d_acc / acc);
        vel = acc*t_acc;
    }

    epicsFloat64 t_tot = 2*t_acc + t_cnt;         // Total time (sec)

    // Convert to cycles
    size_t cycle_count = (size_t) ceil(t_tot / LC400_MIN_DELAY / maxpts);
    epicsFloat64 t_per_cycle = cycle_count*LC400_MIN_DELAY;
    size_t npts = (size_t) ceil(t_tot / t_per_cycle);

    assert(npts <= maxpts);

    // Fill the waveform
    epicsFloat64 dir = fpos > ipos ? 1.0 : -1.0;

    for (size_t i = 0; i < npts; ++i) {
        epicsFloat64 t = i*t_per_cycle;

        // Accelerating
        if (i < npts/2 && t < t_acc)
        {
            result->data[i+1] = ipos + dir*acc*pow(t, 2) / 2.0;
            result->dataProc[i+1]=(epicsInt32)(result->data[i+1]);
        }
        // Constant speed
        else if (t < t_acc + t_cnt) {
            epicsFloat64 tt = t - t_acc;
            result->data[i+1] = ipos + dir*(d_acc + vel*tt);
            result->dataProc[i+1]=(epicsInt32)(result->data[i+1]);
        }

        // Decelerating
        else {
            epicsFloat64 tt = t - t_acc - t_cnt;
            result->data[i<npts-1 ? i+1 : 0] =
                ipos + dir*(d_acc + d_cnt + vel*tt - acc*tt*tt / 2.0);
            result->dataProc[i<npts-1 ? i+1 : 0]=(epicsInt32)(result->data[i<npts-1 ? i+1 : 0]);
        }
    }

    result->cycle_count = cycle_count;
    result->data_len = npts;
}

/** Read Single Location Command
  * Reads one 32 bit value from the specified address
  * \param[in] addr address to read from 
  * \param[in] val return data value*/
asynStatus LC400Controller::readSingle(epicsUInt32 addr, epicsInt32 *val)
{
  asynStatus status = asynSuccess;

  struct __attribute__((__packed__))outData{
      unsigned char cmd;
      epicsUInt32 offset;
      unsigned char eos;
  };

  struct __attribute__((__packed__))inData{
      unsigned char cmd;
      epicsUInt32 offset;
      epicsInt32 response;
      unsigned char eos;
  };

  struct outData out;
  out.cmd=0xA0;
  out.offset=addr;
  out.eos=0x55;
  size_t nwrite;
  size_t nread;
  epicsInt32 eomReason;

  struct inData inp;
  {
    Guard guard(rwLock);
    status = pasynOctetSyncIO->write(pasynUserLC400_, (char*)&out, sizeof(out), LC400_TIMEOUT, &nwrite);
    if (status)
      return status;
    status = pasynOctetSyncIO->read(pasynUserLC400_, (char*)&inp, sizeof(inp), LC400_TIMEOUT, &nread, &eomReason);
    if (status)
      return status;
  }

  *val = inp.response;

  return asynSuccess;
}

/** Write Single Location Command
  * Write one 32 bit value from the specified address
  * \param[in] addr address to write
  * \param[in] val data value to write*/
asynStatus LC400Controller::writeSingle(epicsUInt32 addr,epicsInt32 value)
{
  asynStatus status;

  struct __attribute__((__packed__))outData{
      unsigned char cmd;
      epicsUInt32 offset;
      epicsInt32 value;
      unsigned char eos;
  };

  struct outData out;
  out.cmd=0xA2;
  out.offset=addr;
  out.value=value;
  out.eos=0x55;

  size_t nwrite;
  {
    Guard guard(rwLock);
    status = pasynOctetSyncIO->write(pasynUserLC400_, (char *)&out, sizeof(outData), LC400_TIMEOUT, &nwrite);
  }
  
  return status;
}

/** Write Array Command
  * Write multiple 32 bit values starting at the specified address
  * \param[in] addr address to write
  * \param[in] *array data array to write
  * \param[in] arraySize size of the data array*/
asynStatus LC400Controller::writeArray(epicsUInt32 addr, epicsInt32* array, size_t arraySize)
{
  asynStatus status;

  struct __attribute__((__packed__))outData{
    unsigned char cmd;
    epicsInt32 value;
    unsigned char eos;
  };
  struct outData out;
  out.cmd=0xA3;
  size_t nwrite;
  out.eos=0x55;

  size_t N = arraySize/sizeof(*array)-1;
  char *buffer = (char*)malloc(N*sizeof(out));
  for(epicsUInt32 i = 0; i < N; ++i) 
  {
    struct outData *t;
    t = (struct outData*) &buffer[sizeof(out)*i];
    t->cmd = 0xA3;
    t->value = array[i+1];
    t->eos=0x55;
  }

  {
    Guard guard(rwLock);
    if ((status = writeSingle(addr,array[0])) ) goto skip;
    status = pasynOctetSyncIO->write(pasynUserLC400_, (char *)buffer, N*sizeof(out), LC400_TIMEOUT, &nwrite);
  }
  skip:
  free(buffer);
  return status;
}

/** Read Array Command
  * Read multiple 32 bit values starting at the specified address
  * \param[in] addr     address to read from
  * \param[in] numReads number of elements to read
  * \param[in] *buf     array to store the data
  * \param[in] bufSize  size of the data array */
asynStatus LC400Controller::readArray(epicsUInt32 addr, epicsUInt32 numReads, epicsInt32  *buf, size_t bufSize)
{
  assert(numReads*4 == bufSize);
  asynStatus status = asynSuccess;

  struct __attribute__((__packed__))outData{
    unsigned char cmd;
    epicsUInt32 offset;
    epicsUInt32 numReads;
    unsigned char eos;
  };

  struct __attribute__((__packed__))inData{
    unsigned char cmd;
    epicsUInt32 offset;
  };

  struct outData out;
  out.cmd=0xA4;
  out.offset=addr;
  out.numReads=numReads;
  out.eos=0x55;

  struct inData inp;
  size_t nwrite;
  size_t nread;
  epicsInt32 eomReason;
  unsigned char eos;

  memset((char*)buf, 0xAA, bufSize);
  {
    Guard guard(rwLock);
    if ((status = pasynOctetSyncIO->writeRead(pasynUserLC400_, (char*)&out,
                                      sizeof(out), (char*)&inp, 
                                      sizeof(struct inData), 
                                      LC400_TIMEOUT, &nwrite, &nread, &eomReason)) )
      goto skip;
    if ((status = pasynOctetSyncIO->read(pasynUserLC400_, (char*)buf, bufSize, LC400_TIMEOUT, &nread, &eomReason)) )
      goto skip;
    status = pasynOctetSyncIO->read(pasynUserLC400_, (char*)&eos, sizeof(eos), LC400_TIMEOUT, &nread, &eomReason);
  }
  skip:
  return status;
}

/** updateAuxParams
  * Read all the auxiliary variables for a given axis
  * \param[in] axis axis number*/
asynStatus LC400Controller::updateAuxParams(epicsInt32 axis)
{
  asynStatus status = asynSuccess;
  epicsInt32 done;
  //only update if axes are idle
  for(epicsInt32 i=0; i < numAxes; i++)
  {
    getIntegerParam(i,motorStatusDone_,&done);
    if(!done)
      return asynSuccess;
  }
  union doubleToInt{
    epicsInt32 vInt[2];
    epicsFloat64 vDouble;
  };
  union doubleToInt v;
  epicsUInt32 address = getBaseAddress(axis);
  
  if (address)
  {
    //read static parameters
    epicsInt32 val;
    if ((status = readSingle(address+ST_RANGE_CMD, &val)) ) goto skip;
    setIntegerParam(axis, LC400_Range_,       val);
    
    if ((status = readSingle(address+ST_RANGE_TYPE, &val)) ) goto skip;
    setIntegerParam(axis, LC400_Range_Type_,  val);

    if ((status = readSingle(address+SERVO_STATE, &val)) ) goto skip;
    setIntegerParam(axis, LC400_Servo_State_, val);
    
    if ((status = readSingle(address+P_GAIN, &v.vInt[0])) ) goto skip;
    if ((status = readSingle(address+P_GAIN+sizeof(epicsInt32),&v.vInt[1])) ) goto skip;
    setDoubleParam(axis, LC400_P_Gain_,v.vDouble);
    
    if ((status = readSingle(address+I_GAIN, &v.vInt[0])) ) goto skip;
    if ((status = readSingle(address+I_GAIN+sizeof(epicsInt32), &v.vInt[1])) ) goto skip;
    setDoubleParam(axis, LC400_I_Gain_,v.vDouble);
    
    if ((status = readSingle(address+D_GAIN, &v.vInt[0])) ) goto skip;
    if ((status = readSingle(address+D_GAIN+sizeof(epicsInt32), &v.vInt[1])) ) goto skip;
    setDoubleParam(axis, LC400_D_Gain_, v.vDouble);
  }
  skip:
  return status;
}

/** Returns a pointer to an LC400MotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
LC400Axis* LC400Controller::getAxis(asynUser *pasynUser)
{
  return static_cast<LC400Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an LC400MotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
LC400Axis* LC400Controller::getAxis(epicsInt32 axisNo)
{
  return static_cast<LC400Axis*>(asynMotorController::getAxis(axisNo));
}

/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus LC400Controller::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  epicsInt32 function = pasynUser->reason;
  asynStatus status = asynSuccess;
  LC400Axis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeInt32";

  epicsUInt32 address = getBaseAddress(pAxis->axisNo_);
  
  if (address)
  {
    //write command
    if (function == LC400_Servo_State_)
      status = writeSingle(address+SERVO_STATE,value);
    else if (function == LC400_W_Enable_)
      status = writeSingle(address+WAV_ENABLE,value);
    else if (function == LC400_W_Index_)
      status = writeSingle(address+WAV_INDEX,value);
    else if (function == LC400_W_Delay_)
      status = writeSingle(address+WAV_DELAY,value);
    else if (function == LC400_W_EndIdx_)
      status = writeSingle(address+WAV_END,value);
    else if (function == LC400_W_Active_)
      status = writeSingle(address+WAV_ACTIVE,value);
    else if (function == LC400_W_Iter_)
      status = writeSingle(address+WAV_ITERATIONS,value);
    else
      /* Call base class method */
      status = asynMotorController::writeInt32(pasynUser, value);
  }
  else
    status = asynError;
  
  skip:
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%d, value=%d\n", 
        driverName, functionName, status, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%d, value=%d\n", 
        driverName, functionName, function, value);
  return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus LC400Controller::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  epicsInt32 function = pasynUser->reason;
  asynStatus status = asynSuccess;
  LC400Axis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeFloat64";
  union doubleToInt{
    epicsInt32 vInt[2];
    epicsFloat64 vDouble;
  };
  union doubleToInt v;
  
  v.vDouble = value;
  epicsUInt32 address = getBaseAddress(pAxis->axisNo_);
  if (address)
  {
    if (function == LC400_Digital_Pos_)
    {
      status = writeSingle(address+ST_DIGITAL_SP, (epicsInt32)value);
    }
    else if (function == LC400_P_Gain_)
    {
      if ((status = writeSingle(address+P_GAIN,v.vInt[0])) ) goto skip;
      status = writeSingle(address+P_GAIN+sizeof(epicsInt32),v.vInt[1]);
    }
    else if (function == LC400_I_Gain_)
    {
      if ((status = writeSingle(address+I_GAIN,v.vInt[0])) ) goto skip;
      status = writeSingle(address+I_GAIN+sizeof(epicsInt32),v.vInt[1]);
    }
    else if (function == LC400_D_Gain_)
    {
      if ((status = writeSingle(address+D_GAIN,v.vInt[0])) ) goto skip;
      status = writeSingle(address+D_GAIN+sizeof(epicsInt32),v.vInt[1]);
    }
    else
      /* Call base class method */
      status = asynMotorController::writeFloat64(pasynUser, value);
  }
  else
      status = asynError;
  
  skip:
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%d, value=%d\n", 
        driverName, functionName, status, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%d, value=%d\n", 
        driverName, functionName, function, value);
  return status;
}

// These are the LC400Axis methods
/** Creates a new LC400Axis object.
  * \param[in] pC Pointer to the LC400Controller to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
LC400Axis::LC400Axis(LC400Controller *pC, epicsInt32 axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  epicsUInt32 address = getBaseAddress(axisNo);

  epicsInt32 val;
  pC_->readSingle(address+ST_RANGE_CMD,&val);
  this->range=val;
  setIntegerParam(pC_->LC400_Range_, range);

  callParamCallbacks();
}

asynStatus LC400Axis::stop(epicsFloat64 acceleration)
{
  asynStatus status;
  epicsUInt32 chAddr = getBaseAddress(axisNo_);
  if (chAddr)
    status = pC_->writeSingle(chAddr+WAV_ACTIVE,0);
  else
    status = asynError;
  
  return status;
}

asynStatus LC400Axis::move(epicsFloat64 position, epicsInt32 relative, epicsFloat64 minVelocity, epicsFloat64 maxVelocity, epicsFloat64 acceleration)
{
  asynStatus status=asynSuccess;
  epicsFloat64 initialPos;
  epicsUInt32 chAddr;
  
  pC_->getDoubleParam(axisNo_,pC_->LC400_Digital_Pos_,&initialPos);
  if(relative)
    position=initialPos+position;

  //generate trapezoidal movement from command and maxPts from PV
  epicsInt32 maxpts;
  pC_->getIntegerParam(axisNo_,pC_->LC400_W_MaxPts_, &maxpts);
  waveform_t *movement = (waveform_t*)malloc(sizeof(*movement));
  genTrapezoid(initialPos,position,maxVelocity,acceleration,movement,(size_t)maxpts);
  
  //write array to controller
  epicsUInt32 wavAddr = getWavetableAddress(axisNo_);
  if (wavAddr)
  {
    if ((status = pC_->writeArray(wavAddr,movement->dataProc,(size_t)movement->data_len*sizeof(epicsInt32))) )
    {
      free(movement);
      return status;
    }
  }
  else
  {
    free(movement);
    return asynError;
  }
  //necessary to make sure the controller has time to write array
  epicsThreadSleep(0.01);

  //readback array from controller 
  waveform_t *readArrray = (waveform_t*)malloc(sizeof(*readArrray));
  if ((status = pC_->readArray(wavAddr,(size_t)movement->data_len,readArrray->dataProc,movement->data_len*sizeof(epicsInt32))) )
    goto skip;
  //check if array from controller is the same as the generated array
  for (epicsUInt32 i=0; i<movement->data_len; i++)
  {
    if(movement->dataProc[i] != readArrray->dataProc[i])
    {
      status = asynError;
      printf("something is wrong with array pos[%d] wrote %d read %d\n", 
        i, movement->dataProc[i],readArrray->dataProc[i]);
      goto skip;
    }
  }

  //configure wavetable movement
  chAddr = getBaseAddress(axisNo_);
  if (chAddr)
  {
    if ((status = pC_->writeSingle(chAddr+WAV_ACTIVE,0)) ) goto skip;
    if ((status = pC_->writeSingle(chAddr+WAV_ENABLE,2)) ) goto skip;
    if ((status = pC_->writeSingle(chAddr+WAV_INDEX,1)) ) goto skip;
    if ((status = pC_->writeSingle(chAddr+WAV_DELAY,(epicsInt32)movement->cycle_count)) ) goto skip;
    if ((status = pC_->writeSingle(chAddr+WAV_END,((epicsInt32)movement->data_len-1))) ) goto skip;
    if ((status = pC_->writeSingle(chAddr+WAV_ITERATIONS,1)) ) goto skip;
    if ((status = pC_->writeSingle(chAddr+WAV_COUNT,1)) ) goto skip;

    //readback wavetable parameters and update PVs
    epicsInt32 val;
    if ((status = pC_->readSingle(chAddr+WAV_ACTIVE, &val)) ) goto skip;
    setIntegerParam(pC_->LC400_W_Active_, val);
    if ((status = pC_->readSingle(chAddr+WAV_ACTIVE, &val)) ) goto skip;
    setIntegerParam(pC_->LC400_W_Enable_, val);
    if ((status = pC_->readSingle(chAddr+WAV_INDEX, &val)) ) goto skip;
    setIntegerParam(pC_->LC400_W_Index_, val);
    if ((status = pC_->readSingle(chAddr+WAV_DELAY, &val)) ) goto skip;
    setIntegerParam(pC_->LC400_W_Delay_, val);
    if ((status = pC_->readSingle(chAddr+WAV_END, &val)) ) goto skip;
    setIntegerParam(pC_->LC400_W_EndIdx_, val);
    if ((status = pC_->readSingle(chAddr+WAV_ITERATIONS, &val)) ) goto skip;
    setIntegerParam(pC_->LC400_W_Iter_, val);
    if ((status = pC_->readSingle(chAddr+WAV_COUNT, &val)) ) goto skip;
    setIntegerParam(pC_->LC400_W_IterCts_, val);
    
    //start movement
    status = pC_->writeSingle(chAddr+WAV_ACTIVE,1);
  }
  else
    status = asynError;

  skip:
  free(movement);
  free(readArrray);
  return status;
}

asynStatus LC400Axis::moveVelocity(epicsFloat64 min_velocity, epicsFloat64 max_velocity, epicsFloat64 acceleration)
{
  asynStatus status;
  epicsInt32 range;
  if ((status = pC_->getIntegerParam(axisNo_,pC_->LC400_Range_,&range)) ) goto skip;
  range = range/2;

  if (max_velocity < 0) {
    max_velocity = -max_velocity;
    range = -range;
  }
  status = move(getCts((epicsFloat64)range),0,0,max_velocity,acceleration);

  skip:
  return status;
}


/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus LC400Axis::poll(bool *moving)
{ 
  epicsInt32 done;
  epicsInt32 currentPos;
  epicsInt32 busy;
  asynStatus status = asynSuccess;
  
  epicsUInt32 address = getBaseAddress(axisNo_);

  if (address)
  {
    //update position
    if ((status = pC_->readSingle(address+ST_DIGITAL_POS, &currentPos)) ) goto skip;
    setDoubleParam(pC_->LC400_Digital_Pos_,currentPos);
    setDoubleParam(pC_->motorPosition_, currentPos);

    //check if motor is done moving
    if ((status = pC_->readSingle(address+WAV_ACTIVE, &busy)) ) goto skip;
    if(busy == 0)
    {
      done = 1;
      if ((status = pC_->updateAuxParams(axisNo_)) ) goto skip;
    }
    else
      done = 0;
    setIntegerParam(pC_->motorStatusDone_, done);
    *moving = (!done);
  }
  else
  {
    status = asynError;
    goto skip;
  }
  // Read the limit status
  if(getPosition(currentPos) >= (float)(this->range/2.0))
    setIntegerParam(pC_->motorStatusHighLimit_,1);
  else
    setIntegerParam(pC_->motorStatusHighLimit_,0);
  if(getPosition(currentPos) <= (float)((this->range/2*-1.0)))
    setIntegerParam(pC_->motorStatusLowLimit_,1);
  else
    setIntegerParam(pC_->motorStatusLowLimit_,0);

  skip:
  //set motorStatusProblem
  setIntegerParam(pC_->motorStatusProblem_, !!status);
  setIntegerParam(pC_->motorStatusPowerOn_, 1);
  callParamCallbacks();

  return status;
}

/** Code for iocsh registration */
static const iocshArg LC400CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg LC400CreateControllerArg1 = {"LC400 port name", iocshArgString};
static const iocshArg LC400CreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg LC400CreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg LC400CreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const LC400CreateControllerArgs[] = {&LC400CreateControllerArg0,
                                                           &LC400CreateControllerArg1,
                                                           &LC400CreateControllerArg2,
                                                           &LC400CreateControllerArg3,
                                                           &LC400CreateControllerArg4};
static const iocshFuncDef LC400CreateControllerDef = {"LC400CreateController", 5, LC400CreateControllerArgs};
static void LC400CreateContollerCallFunc(const iocshArgBuf *args)
{
  LC400CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void LC400MotorRegister(void)
{
  iocshRegister(&LC400CreateControllerDef, LC400CreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(LC400MotorRegister);
}
