/*
FILENAME...   LC400MotorDriver.h
USAGE...      Motor driver support for the nPoint LC400 series of controllers

Diego Omitto
August, 2019

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "asynPortDriver.h"
#include "LC400MemoryAddr.h"

#define LC400_TIMEOUT 5.0
#define LC400_MAX_ARRAY 83300
#define LC400_MIN_DELAY 24e-6

/** drvInfo strings for extra parameters that the LC400 Axis Supports */
#define LC400RangeString          "LC400_RANGE"
#define LC400RangeTypeString      "LC400_RANGE_TYPE"
#define LC400DigitalPosString     "LC400_DIGITAL_POS"
#define LC400ServoStateString     "LC400_SERVO_STATE"
#define LC400ProportionalGString  "LC400_P_GAIN"
#define LC400IntegralGString      "LC400_I_GAIN"
#define LC400DerivativeGString    "LC400_D_GAIN"
#define LC400FirmwareString       "LC400_FW"
#define LC400WavetableEnString    "LC400_W_ENABLE"
#define LC400WavetableIdxString   "LC400_W_INDEX"
#define LC400WavetableCDlyString  "LC400_W_DELAY"
#define LC400WavetableEIdxString  "LC400_W_ENDIDX"
#define LC400WavetableActString   "LC400_W_ACTIVE"
#define LC400WavetableItString    "LC400_W_ITER"
#define LC400WavetableItCtsString "LC400_W_ITERCTS"
#define LC400WavetableMaxPts      "LC400_W_MAXPTS"

typedef struct {
    size_t cycle_count;
    size_t data_len;
    double data[LC400_MAX_ARRAY];
    epicsInt32 dataProc[LC400_MAX_ARRAY];
} waveform_t;

class LC400Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  LC400Axis(class LC400Controller *pC, epicsInt32 axis);
  asynStatus move(epicsFloat64 position, epicsInt32 relative, epicsFloat64 min_velocity, epicsFloat64 max_velocity, epicsFloat64 acceleration);
  asynStatus moveVelocity(epicsFloat64 min_velocity, epicsFloat64 max_velocity, epicsFloat64 acceleration);
  asynStatus poll(bool *moving);
  asynStatus stop(epicsFloat64 acceleration);


private:
  LC400Controller *pC_;      /**< Pointer to the asynMotorController to which this axis belongs.
                                *   Abbreviated because it is used very frequently */
  epicsInt32 range;

friend class LC400Controller;
};

class LC400Controller : public asynMotorController {
public:
  LC400Controller(const char *portName, const char *LC400PortName, epicsInt32 numAxes, epicsFloat64 movingPollPeriod, epicsFloat64 idlePollPeriod);

  /* These are the methods that we override from asynMotorDriver */
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  void report(FILE *fp, epicsInt32 level);
  LC400Axis* getAxis(asynUser *pasynUser);
  LC400Axis* getAxis(epicsInt32 axisNo);

  /* These are the methods that are new to this class */
  //main functions to manipulate controller registers
  asynStatus readSingle(epicsUInt32 addr, epicsInt32 *val);
  asynStatus readArray(epicsUInt32 addr, epicsUInt32 numReads, epicsInt32 *buf, size_t bufSize);
  asynStatus writeSingle(epicsUInt32 addr, epicsInt32 value);
  asynStatus writeArray(epicsUInt32 addr, epicsInt32* array, size_t arraySize);
  asynStatus updateAuxParams(epicsInt32 axis);

protected:
  int LC400_Range_;
  int LC400_Range_Type_;
  int LC400_Digital_Pos_;
  int LC400_Servo_State_;
  int LC400_P_Gain_;
  int LC400_I_Gain_;
  int LC400_D_Gain_;
  int LC400_FW_;
  int LC400_W_Enable_;
  int LC400_W_Index_;
  int LC400_W_Delay_;
  int LC400_W_EndIdx_;
  int LC400_W_Active_;
  int LC400_W_Iter_;
  int LC400_W_IterCts_;
  int LC400_W_MaxPts_;
#define FIRST_LC400_PARAM LC400_Range_
#define LAST_LC400_PARAM LC400_W_MaxPts_

#define NUM_LC400_PARAMS (&LAST_LC400_PARAM - &FIRST_LC400_PARAM + 1)

private:
  asynUser *pasynUserLC400_;
  epicsMutex rwLock;
  epicsInt32 numAxes;

friend class LC400Axis;
};
