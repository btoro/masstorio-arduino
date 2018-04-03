
// ***** INCLUDES *****
#include <PID_v1.h>
#include <eRCaGuy_NewAnalogRead.h>

//ADC
ADC_prescaler_t ADCSpeed = ADC_FAST;
byte bitsOfResolution = 12; //commanded oversampled resolution
unsigned long numSamplesToAvg = 5; //number of samples AT THE OVERSAMPLED RESOLUTION that you want to take and average

//#define AUTOTUNE

// AUTO Tune
#if defined( AUTOTUNE )
#include <PID_AutoTune_v0.h>

byte ATuneModeRemember = 1;

double aTuneStep = 50, aTuneNoise = 1, aTuneStartValue = 100;
unsigned int aTuneLookBack = 20;

// Specify PID control interface
PID_ATune aTune(&input, &output);


#endif
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
  STATE_AUTOTUNE, //7
  STATE_HIGHTEMP
} State_t;

typedef enum STATUS
{
  STATUS_OFF, // normaly off
  STATUS_ON, // switches to ON during run
  STATUS_INITIATE, // starts a cycle
  STATUS_COMPLETE // cycle is completed
} Status_t;

typedef enum MODE
{
  MODE_ISOTHERMAL, // isothermal operating mode
  MODE_RAMPSOAK, // ramp soak 
  MODE_RATE, // constant rate mode
} Mode_t;


typedef enum SEQUENCE_PARAMS
{
  PARAM_SETPOINT,
  PARAM_RC1,
  PARAM_RC2,
  PARAM_PAUSETIME,
  PARAM_SOAKTIME
} Sequence_t;


// ***** CONSTANTS *****
#define SENSOR_SAMPLING_TIME 200
#define CYCLE_TIME 1000
#define MAX_SEQUENCE_LENGTH 20  
#define MAX_TEMP 105
#define MIN_TEMP -200
#define RATE_SAMPLE_POINTS 10
// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP 20
#define PID_KI 0.1
#define PID_KD 0

// ***** PIN ASSIGNMENT *****
const int ssrPin = 6;
const int ssrPin2 = 7;
const int chillerPin = 13;
const int tempPin = A1;
const int ledRedPin = A1;
const int ledGreenPin = A0;


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

//derivative
double lastInput;
double rate;
int counter = 0;

// Controller State
State_t State;
// Ramp soak status
Status_t Status;
Mode_t Mode = MODE_RAMPSOAK;


PID controllerPID(&input, &output, &setpoint, kp, ki, kd, DIRECT, P_ON_M);


// Temperature Reading Constants
const float A_value = 1.285E-3;
const float B_value = 2.362E-4;
const float C_value = 9.285E-8;
const double SERIESRESISTOR = 10000;
double calibrationFactor = 0.0;

//Serial
const size_t READ_BUF_SIZE = 64;
// Forward declarations
void processSerial();

char readBuf[READ_BUF_SIZE];
size_t readBufOffset = 0;


//Ramp Soak

double sequence[MAX_SEQUENCE_LENGTH][5];
double sequenceGains[MAX_SEQUENCE_LENGTH][3];
int startStep = 0;
int currentStep;
int totalSteps = 0;

unsigned long timerStep;

int rampStep;
int maxrampSteps;
double rampInterval;
double rampStartInput;
bool ramp_direction; // 0 RISE, 1 COOl
bool customGainsON = false;

double error;


// On Off Control
double upperDB = 0.1;
double lowerDB = 0.3;


void setup()
{

  // SSR pin initialization to ensure heater is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  digitalWrite(ssrPin2, LOW);
  pinMode(ssrPin2, OUTPUT);

  // LED pins initialization and turn on upon start-up (active low)
//  digitalWrite(ledRedPin, LOW);
//  pinMode(ledRedPin, OUTPUT);

  pinMode(chillerPin, OUTPUT);
  digitalWrite(chillerPin, LOW);

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
    counter = counter++;

    if ( counter % RATE_SAMPLE_POINTS == 0 )
    {
      lastInput = input;
      input = read_temps();
      rate = (input - lastInput) / ((SENSOR_SAMPLING_TIME * RATE_SAMPLE_POINTS) / 1000); // c/s
      counter = 0;
    }
    else input = read_temps();


    // If thermocouple problem detected
    if (isnan(input))
    {
      // Illegal operation
      State = STATE_ERROR;
      Status = STATUS_OFF;
      //Serial.println("Error No read");
    }

    if ( input > MAX_TEMP )
    {
      State = STATE_HIGHTEMP;
      Status = STATUS_OFF;
    }
    if ( input < MIN_TEMP )
    {
      State = STATE_HIGHTEMP;
      Status = STATUS_OFF;
    }
  }

  if (Status == STATUS_ON)
  {
    switch(Mode)
    {
      case MODE_RAMPSOAK:
      
        switch(State)
        {
          case STATE_RAMP:
            error =  setpoint - input;
    
            if ( abs(error) < sequence[currentStep][PARAM_RC2] )
            {
                output = 0;
    //
    //          if( sequence[currentStep][PARAM_PAUSETIME] > 0 )
    //          {
                initatePause( sequence[currentStep][PARAM_PAUSETIME] );
    //          }
    //          else
    //          {
    //            if( ramp_direction ) // Going UP
    //            {
    //              controllerPID.SetMode( AUTOMATIC );
    //              ramp_direction = true;
    //              initiateSoak();
    //            }
    //            else // If we are cooling, then extend pause until we are at or below SP
    //            {
    //              error = input - setpoint;
    //              if( error <= 0.5 )
    //              {
    //                controllerPID.SetMode( AUTOMATIC );
    //                ramp_direction = true;
    //                initiateSoak();
    //              }            
    //            }
    //          }
    
                
    
    //          if( ~ramp_direction ) // if we were cooling and reached the difference
    //          {
    //            digitalWrite(chillerPin, LOW);
    //          }
              
            }
            else if ( abs(error) < sequence[currentStep][PARAM_RC1])
            {
              if( ramp_direction )
              {
                output = windowSize / 2;
              }
              else // Cooling; No 50% 
              {
                output = 0;
                digitalWrite(chillerPin, LOW);
                initatePause( sequence[currentStep][PARAM_PAUSETIME] );
    
              }
    
            }
            break;
          case STATE_PAUSE:
            if (millis() > timerStep) // Pause is complete
            {
              if( ramp_direction ) // Going UP
              {
                controllerPID.SetMode( AUTOMATIC );
                ramp_direction = true;
                initiateSoak();
              }
              else // If we are cooling, then extend pause until we are at or below SP
              {
                error = input - setpoint;
                if( error <= 0.5 )
                {
                  controllerPID.SetMode( AUTOMATIC );
                  ramp_direction = true;
                  initiateSoak();
                }            
              }
            }
            break;
          case STATE_SOAK:
            // If micro soak temperature is achieved
            //controllerPID.SetMode( AUTOMATIC );
            if (millis() > timerStep) // Soak is complete
            {
              if ( (currentStep+1) <= totalSteps )
              {
                currentStep++; // go to next step
                initiateAutoRamp();
              }
              else
              {
                Status = STATUS_COMPLETE;
                State = STATE_INACTIVE;
              }
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
      break;
      case MODE_RATE:
        if( State == STATE_RAMP )
        {

          
        }
      break;
    }
    //SSR control
      if ( millis() > timerCycle)
      {
        timerCycle += CYCLE_TIME;

        controllerPID.Compute();
      }
    now = millis();


    if ((now - windowStartTime) >= windowSize)
    {
      // Time to shift the Relay Window
      windowStartTime += windowSize;

    }
    if (output > (now - windowStartTime))
    {
      if( ramp_direction ) // HEATING
      {
        digitalWrite(ssrPin, HIGH);
        //      digitalWrite(ssrPin2, HIGH);
      }
      else // COOLING
      {
        digitalWrite(chillerPin, HIGH);
      }
    }
    else
    {
//      if( ~direction ) // HEATING
//      {
//        digitalWrite(chillerPin, LOW);
//      }
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
        initiateAutoRamp();

        break;
    }
  }
}

void initatePause( double t )
{
//  if ( t )
//  {
   timerStep = millis() + (t * 1000);
//  }
//  else
//  {
//    timerStep = millis() + (sequence[currentStep][PARAM_PAUSETIME] * 1000);
//  }
  State = STATE_PAUSE;
}


void initiateAutoRamp()
{
  if ( currentStep <= totalSteps )
  {
    setpoint = sequence[currentStep][PARAM_SETPOINT];

    if( abs(setpoint-sequence[currentStep-1][PARAM_SETPOINT]) < 0.1 )
    {
      State = STATE_SOAK;
      timerStep = millis() + (sequence[currentStep][PARAM_SOAKTIME] * 1000);
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
      controllerPID.SetMode( MANUAL );
      timerStep = millis(); 
      //timerStep = millis() + (sequence[currentStep][PARAM_RAMPTIME] * 1000);
  
  
      if( currentStep == 1 )
      {
        if( (setpoint- input) > 0 )
        {
          ramp_direction = true;
        }
        else
        {
          ramp_direction = false;
        }
      }
      else if( (setpoint - sequence[currentStep-1][PARAM_SETPOINT]) >= 0 )
      {
        ramp_direction = true;
      }
      else
      {
        ramp_direction = false;
      }
        
  
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
          Serial.print((output / windowSize) * 100);
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
        case 13: //get rate
          Serial.print("3,13,");
          Serial.print(rate);
          Serial.print('\n');
          break;
        case 14: //get direction
          Serial.print("3,14,");
          Serial.print(ramp_direction);
          Serial.print('\n');
          break;
      }



      break;
    case 6: //write variable
      command = strtok(0, ",");
      action = atoi(command);
      switch (action)
      {
        case 2: // Mode 
          command = strtok(0, ",");
          Mode = atoi(command);
          break;
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
          #if defined( AUTOTUNE )
          command = strtok(0, ",");
          switchAutoTune( atoi(command) );
          #endif
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
        case 16: // set DB
          upperDB = strtod(strtok(0, ","), NULL);
          lowerDB = strtod(strtok(0, ","), NULL);
          break;
        case 17: // CHILLER
          action = atoi(strtok(0, ","));
          digitalWrite(chillerPin, action);
          break;
        case 18: // calibration factor
          calibrationFactor = strtod(strtok(0, ","), NULL);
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

  t_read = t_read - 273.15 + calibrationFactor;
  return t_read;
}

#if defined( AUTOTUNE )

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

#endif
