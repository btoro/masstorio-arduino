
// ***** INCLUDES *****
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <eRCaGuy_NewAnalogRead.h>

//ADC
ADC_prescaler_t ADCSpeed = ADC_FAST;
byte bitsOfResolution = 12; //commanded oversampled resolution
unsigned long numSamplesToAvg = 5; //number of samples AT THE OVERSAMPLED RESOLUTION that you want to take and average

// AUTO Tune
byte ATuneModeRemember = 1;

double aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 100;
unsigned int aTuneLookBack = 20;

boolean tuning = false;

// ***** TYPE DEFINITIONS *****
typedef enum STATE
{
  STATE_INACTIVE, // nothing, cannot run or do anything
  STATE_ISO, // Isothermal control
  STATE_ACTIVE, // ready for Ramp soak program
  STATE_RAMP, // is ramping
  STATE_SOAK, // is soaking
  STATE_PAUSE,
  STATE_ERROR, // no T read
  STATE_AUTOTUNE, //6
  STATE_HIGHTEMP
} State_t;

typedef enum STATUS
{
  STATUS_OFF, // normaly off
  STATUS_ON, // switches to ON during run
  STATUS_INITIATE, // starts a cycle
  STATUS_COMPLETE // cycle is completed
} Status_t;

typedef enum SEQUENCE_PARAMS
{
  PARAM_SETPOINT,
  PARAM_RAMPTIME,
  PARAM_PAUSETIME,
  PARAM_SOAKTIME
} Sequence_t;


// ***** CONSTANTS *****
#define SENSOR_SAMPLING_TIME 200
#define CYCLE_TIME 5000
#define SOAK_TEMPERATURE_STEP 5
#define MAX_SEQUENCE_LENGTH 50
#define MAX_TEMP 105
#define MIN_TEMP -200
// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP 20
#define PID_KI 0.1
#define PID_KD 0


// ***** PIN ASSIGNMENT *****
int ssrPin = 6;
int ssrPin2 = 7;

int tempPin = A0;
int ledRedPin = A1;
int ledGreenPin = A0;


// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP;
double ki = PID_KI;
double kd = PID_KD;
int windowSize = CYCLE_TIME;
unsigned long windowStartTime;
unsigned long nextRead;
unsigned long timerCycle;


// Controller State
State_t State;
// Ramp soak status
Status_t Status;

// Specify PID control interface
PID controllerPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
PID_ATune aTune(&input, &output);


// Temperature Reading Constants
const float A_value = 1.285E-3;
const float B_value = 2.362E-4;
const float C_value = 9.285E-8;
const double SERIESRESISTOR = 10000;

//Serial
const size_t READ_BUF_SIZE = 64;
// Forward declarations
void processSerial();

char readBuf[READ_BUF_SIZE];
size_t readBufOffset = 0;


//Ramp Soak

double sequence[MAX_SEQUENCE_LENGTH][4];
double sequenceGains[MAX_SEQUENCE_LENGTH][3];
int startStep = 0;
int currentStep;
int totalSteps = 0;

unsigned long timerStep;

int rampStep;
int maxrampSteps;
double rampInterval;
double rampStartInput;
bool direction; // 0 RISE, 1 COOl
bool customGainsON = false;



void setup()
{

  // SSR pin initialization to ensure heater is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  digitalWrite(ssrPin2, LOW);
  pinMode(ssrPin2, OUTPUT);

  // LED pins initialization and turn on upon start-up (active low)
  digitalWrite(ledRedPin, LOW);
  pinMode(ledRedPin, OUTPUT);

  // Serial communication at 57600 bps
  Serial.begin(57600);

  // Initialize thermocouple reading variable
  nextRead = millis();
  timerCycle = millis();

  windowStartTime = millis();

  controllerPID.SetOutputLimits(0, windowSize);
  controllerPID.SetSampleTime( CYCLE_TIME );
  controllerPID.SetMode(AUTOMATIC);

  controllerPID.SetTunings( kp, ki, kd, P_ON_E );

  //ADC
  adc.setADCSpeed(ADCSpeed);
  adc.setBitsOfResolution(bitsOfResolution);
  adc.setNumSamplesToAvg(numSamplesToAvg);


}

void loop()
{

  // Read data from serial
  while (Serial.available()) {
    if (readBufOffset < READ_BUF_SIZE) {
      char c = Serial.read();
      if (c != '\n') {
        // Add character to buffer
        readBuf[readBufOffset++] = c;
      }
      else {
        // End of line character found, process line
        readBuf[readBufOffset] = 0;
        processSerial();
        readBufOffset = 0;
      }
    }
    else {
      readBufOffset = 0;
    }
  }
  // Current time
  unsigned long now;

  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature
    input = read_temps();

    // If thermocouple problem detected
    if (isnan(input))
    {
      // Illegal operation
      State = STATE_ERROR;
      Status = STATUS_OFF;
      //Serial.println("Error No read");
    }

    if( input > MAX_TEMP )
    {
      State = STATE_HIGHTEMP;
      Status = STATUS_OFF;
    }
    if( input < MIN_TEMP )
    {
      State = STATE_HIGHTEMP;
      Status = STATUS_OFF;
    }
  }

  if (Status == STATUS_ON)
  {
    // Reflow oven controller state machine
    switch (State)
    {
      case STATE_RAMP:
        if (millis() > timerStep) // ramp is complete
        {
          output = 0; // go to next step
          //controllerPID.SetMode( AUTOMATIC );
          initatePause();
          //initiateSoak();
        }
        break;
      case STATE_PAUSE:
        if (millis() > timerStep) // Pause is complete
        {
          controllerPID.SetMode( AUTOMATIC );
          initiateSoak();
        }
        break;
      case STATE_SOAK:
        // If micro soak temperature is achieved
        if (millis() > timerStep) // Soak is complete
        {
          currentStep++; // go to next step
          initiateManualRamp();
        }
        break;
      case STATE_ERROR:
        // If thermocouple problem is still present
        if (isnan(input))
        {
          // Wait until thermocouple wire is connected
          State = STATE_ERROR;
        }
        else
        {
          // Clear to perform reflow process
          State = STATE_INACTIVE;
        }
        break;
    }

    // PID computation and SSR control

    if (tuning)
    {
      byte val = (aTune.Runtime());
      if (val != 0)
      {
        tuning = false;
        State = STATE_INACTIVE;
      }
      if (!tuning)
      { //we're done, set the tuning parameters
        kp = aTune.GetKp();
        ki = aTune.GetKi();
        kd = aTune.GetKd();
        controllerPID.SetTunings(kp, ki, kd , P_ON_E );
        AutoTuneHelper(false);
      }
    }
    else 
    {
      if( millis() > timerCycle)
      {
            timerCycle += CYCLE_TIME;
            controllerPID.Compute();
      }
    }


    now = millis();


    if ((now - windowStartTime) > windowSize)
    {
      // Time to shift the Relay Window
      windowStartTime += windowSize;

    }
    if (output > (now - windowStartTime)) 
    {
      digitalWrite(ssrPin, HIGH);
//      digitalWrite(ssrPin2, HIGH);
    }
    else
    {
      digitalWrite(ssrPin, LOW);
      digitalWrite(ssrPin2, LOW);

    }

  }
  // Make sure heaters are off
  else
  {
    digitalWrite(ssrPin, LOW);
    digitalWrite(ssrPin2, LOW);
  }
}

void initiateRun()
{
  if ( Status == STATUS_INITIATE )
  {
    windowStartTime = millis();
    output = 0;
    switch ( State )
    {
      case STATE_ISO: // Here we start a new setpoint
        Status = STATUS_ON;
        break;

      case STATE_ACTIVE:  // We begin a new RAMP SOAK Cycle
        Status = STATUS_ON;
        currentStep = startStep;
        initiateManualRamp();

        break;
    }
  }
}

void initatePause()
{
    State = STATE_PAUSE;
    timerStep = millis() + (sequence[currentStep][PARAM_PAUSETIME] * 1000);
}

void initiateManualRamp()
{
  if( currentStep <= totalSteps )
  {
    controllerPID.SetMode( MANUAL );
  
    timerStep = millis() + (sequence[currentStep][PARAM_RAMPTIME] * 1000);
    setpoint = sequence[currentStep][PARAM_SETPOINT];
  
    output = windowSize;
  
    State = STATE_RAMP;
  
    if ( customGainsON )
    {
      kp = sequenceGains[currentStep][0];
      ki = sequenceGains[currentStep][1];
      kd = sequenceGains[currentStep][2];
  
      controllerPID.SetTunings( kp, ki, kd );
    }
  }
  else
  {
    Status = STATUS_COMPLETE;
  }
}

void initiateSoak()
{
//  if ( sequence[currentStep][PARAM_SOAKTIME] == 0 )
//  {
//    State = STATE_ISO;
//    timerStep = millis();
//  }
//  else
//  {
    State = STATE_SOAK;
    timerStep = millis() + (sequence[currentStep][PARAM_SOAKTIME] * 1000);
//  }
}

void calculateOutput()
{
  if( input > upperDB ) output = 0;
  else if(input < lowerDB) output = 100;
}

void processSerial() {
  //int receivedValue = atoi(readBuf);

  // This program just increments the value sent by the Photon and returns it
  //Serial.print(receivedValue + 1, DEC);
  char* command = strtok(readBuf, ",");

  int action = atoi(command);

  switch (action)
  {
    case 3: //read variable
      //       Serial.print("read");
      //       Serial.print('\n');
      command = strtok(0, ",");
      action = atoi(command);
      switch (action)
      {
        case 1: //PID input
          Serial.print("3,1,");
          Serial.print(input);
          Serial.print('\n');
          break;
        case 2: //PID Output
          Serial.print("3,2,");
          Serial.print(output);
          Serial.print('\n');
          break;
        case 3:  // State INACTIVE, ...
          Serial.print("3,3,");
          Serial.print(State);
          Serial.print('\n');
          break;
        case 4: //Ramp Soak Time
          Serial.print("3,4,");
          if ( millis() > timerStep )
          {
            Serial.print( (millis() - timerStep) );
          }
          else
          {
            Serial.print( (timerStep - millis()) );
          }
          Serial.print('\n');
          break;
        case 5:  // Status ON or oFF
          Serial.print("3,5,");
          Serial.print(Status);
          Serial.print('\n');
          break;
        case 7: //setpoint
          Serial.print("3,7,");
          Serial.print(setpoint);
          Serial.print('\n');
          break;

        case 8: //PID
          Serial.print("3,8,");
          Serial.print(kp);
          Serial.print(",");
          Serial.print(ki);
          Serial.print(",");
          Serial.print(kd);
          Serial.print('\n');
          break;
        case 10:
          int id, param;

          id = atoi(strtok(0, ","));
          param = atoi(strtok(0, ","));

          Serial.print("3,10,");
          Serial.print(id);
          Serial.print(",");
          Serial.print(param);
          Serial.print(",");
          Serial.print( sequence[id][param] );
          Serial.print('\n');

          break;
        case 11:
          int id2, param2;
          id2 = atoi(strtok(0, ","));
          param2 = atoi(strtok(0, ","));

          Serial.print("3,11,");
          Serial.print(id);
          Serial.print(",");
          Serial.print(param);
          Serial.print(",");
          Serial.print( sequenceGains[id][param] );
          Serial.print('\n');

          break;
        case 12: //get current steps
          Serial.print("3,12,");
          Serial.print(currentStep);
          Serial.print('\n');
          break;
      }



      break;
    case 6: //write variable
      command = strtok(0, ",");
      action = atoi(command);
      switch (action)
      {
        case 3: // State INACTIVE, ...
          command = strtok(0, ",");
          State = atoi(command);
          break;
        case 5: // Status ON or oFF
          action = atoi( strtok(0, ",") );

          Status =  action;
          if ( Status == STATUS_INITIATE )
          {
            initiateRun();
          }
          else if ( Status == STATUS_OFF) output = 0;
          break;
        case 6: // startStep
          command = strtok(0, ",");
          startStep = atoi(command);
          break;

        case 7: //customGains
          command = strtok(0, ",");
          customGainsON = atoi(command);
          break;
        case 8: //autotune
          command = strtok(0, ",");
          switchAutoTune( atoi(command) );
          break;
        case 9: //setpoint
          setpoint =  strtod(strtok(0, ","), NULL);
          break;
        case 10: //Ramp Soak Program
          int id, param;
          double value;

          id = atoi(strtok(0, ","));
          param = atoi(strtok(0, ","));

          value = strtod(strtok(0, ","), NULL);

          sequence[id][param] = value;
          break;
        case 11: //Ramp Soak Program Custom PID
          int id2, param2;
          double value2;

          id2 = atoi(strtok(0, ","));
          param2 = atoi(strtok(0, ","));

          value2 = strtod(strtok(0, ","), NULL);

          sequenceGains[id2][param2] = value2;
          break;
        case 12: //PID Parameters

          kp = strtod(strtok(0, ","), NULL);
          ki = strtod(strtok(0, ","), NULL);
          kd = strtod(strtok(0, ","), NULL);

          controllerPID.SetTunings( kp, ki, kd, P_ON_E );
          break;

        case 13: // auto /man modes
          command = strtok(0, ",");
          controllerPID.SetMode( atoi(command) );
          break;
        case 14: // set output
          output = strtod(strtok(0, ","), NULL);
          break;
        case 15: // set total step
          totalSteps = strtod(strtok(0, ","), NULL);
          break;
      }
      break;
  }
}

double read_temps(void)
{
  // int input = analogRead(tempPin);
  double input = adc.newAnalogRead( tempPin );

  double a = 1.0 - (input /  adc.getMaxPossibleReading() );
  double R = (SERIESRESISTOR - (a * SERIESRESISTOR)) / a;


  double t_read = A_value + (B_value * log(R)) + (C_value * pow(log(R), 3));
  t_read = 1 / t_read;

  t_read = t_read - 273.15;
  return t_read;
}

void switchAutoTune( int mode )
{
  if ( mode == 1 && !tuning)
  {
    Status = STATUS_ON;
    State = STATE_AUTOTUNE;
    //Set the output to the desired starting frequency.
    output = aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else if ( mode == 0 )
  { //cancel autotune
    Status = STATUS_OFF;
    State = STATE_AUTOTUNE;
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if (start)
    ATuneModeRemember = controllerPID.GetMode();
  else
    controllerPID.SetMode(ATuneModeRemember);
}
