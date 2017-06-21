
// ***** INCLUDES *****
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum STATE
{
  STATE_INACTIVE, // nothing, cannot run or do anything
  STATE_ISO, // Isothermal control
  STATE_ACTIVE, // ready for Ramp soak program
  STATE_RAMP, // is ramping
  STATE_SOAK, // is soaking
  STATE_ERROR // no T read
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
  PARAM_SOAKTIME
} Sequence_t;



typedef enum SWITCH
{
  SWITCH_NONE,
  SWITCH_1,
  SWITCH_2
} switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

// ***** CONSTANTS *****
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 0
#define TEMPERATURE_SOAK_MAX 100
#define TEMPERATURE_REFLOW_MAX 250
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5
#define SOAK_MICRO_PERIOD 9000
#define DEBOUNCE_PERIOD_MIN 50
#define MAX_SEQUENCE_LENGTH 50

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP 20
#define PID_KI 0.1
#define PID_KD 0
// ***** SOAKING STAGE *****
//#define PID_KP_SOAK 300
//#define PID_KI_SOAK 0.05
//#define PID_KD_SOAK 250
//// ***** REFLOW STAGE *****
//#define PID_KP_REFLOW 300
//#define PID_KI_REFLOW 0.05
//#define PID_KD_REFLOW 350

#define PID_SAMPLE_TIME 1000

// ***** PIN ASSIGNMENT *****
int ssrPin = 5;
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
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;

unsigned long buzzerPeriod;

// Controller State
State_t State;
// Ramp soak status
Status_t Status;


// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
// Seconds timer
int timerSeconds;

// Specify PID control interface
PID controllerPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

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

double sequence[MAX_SEQUENCE_LENGTH][3];
int startStep = 1;
int currentStep;
unsigned long timerStep; 
unsigned long cycleTime = 5000; // 5sec cycle time

int rampStep;
int maxrampSteps;
double rampInterval;
double rampStartInput;
bool direction; // 0 RISE, 1 COOl

void setup()
{

  // SSR pin initialization to ensure heater is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  // LED pins initialization and turn on upon start-up (active low)
  digitalWrite(ledRedPin, LOW);
  pinMode(ledRedPin, OUTPUT);

  // Serial communication at 57600 bps
  Serial.begin(57600);

  // Set window size
  windowSize = 2000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();
  
    controllerPID.SetOutputLimits(0, windowSize);
    controllerPID.SetMode(AUTOMATIC);
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
    //      Serial.println(input);

    // If thermocouple problem detected
    if (isnan(input))
    {
      // Illegal operation
      State = STATE_ERROR;
      Status = STATUS_OFF;
      //Serial.println("Error No read");
    }
  }

//  if (millis() > nextCheck)
//  {
//    //          Serial.print("Test 2  ");
//    //          Serial.println(reflowStatus);
//
//
//    // Check input in the next seconds
//    nextCheck += 1000;
//    // If reflow process is on going
//    if (Status == STATUS_ON)
//    {
//      // Toggle red LED as system heart beat
//      digitalWrite(ledRedPin, !(digitalRead(ledRedPin)));
//      // Increase seconds timer for reflow curve analysis
//      timerSeconds++;
//      // Send temperature and time stamp to serial
//      //      Serial.print(timerSeconds);
//      //      Serial.print(" ");
//      //      Serial.print(setpoint);
//      //      Serial.print(" ");
//      //      Serial.print(input);
//      //      Serial.print(" ");
//      //      Serial.println(output);
//    }
//    else
//    {
//      // Turn off red LED
//      digitalWrite(ledRedPin, HIGH);
//    }
//  }


  if (Status == STATUS_ON)
  {
    // Reflow oven controller state machine
    switch (State)
    {
      case STATE_RAMP:
        // check if cycle is passed
        if (millis() > nextCheck)
        {
          nextCheck = millis() + cycleTime;
          if ( (rampStep + 1) <= maxrampSteps )
          {
            rampStep++;
            setpoint = rampStartInput + (rampInterval*rampStep);
          }
          // Check to see if Soak temperature is reached. If so, then soak.
          if ( ( ( input > sequence[currentStep][PARAM_SETPOINT] ) && direction == 0 ) || ( ( input < sequence[currentStep][PARAM_SETPOINT] ) && direction == 1 )  )
          {
            initiateSoak();
          }
        }
        break;

      case STATE_SOAK:
        // If micro soak temperature is achieved
        if (millis() > timerStep) // Soak is complete
        {
          currentStep++; // go to next step
          initiateRamp();
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

    now = millis();

//    bool didcalc;
//    didcalc = 
    controllerPID.Compute();
//
//    if( didcalc )
//    {
//
//       Serial.print("Calculating!!\n");
//    }else
//    {
//       Serial.print("Nope!!\n");
//    }

    if ((now - windowStartTime) > windowSize)
    {
      // Time to shift the Relay Window
      windowStartTime += windowSize;

    }
    if (output > (now - windowStartTime)) digitalWrite(ssrPin, HIGH);
    else digitalWrite(ssrPin, LOW);
  }
  // Make sure heaters are off
  else
  {
    digitalWrite(ssrPin, LOW);
  }


}

void initiateRun()
{
  if ( Status == STATUS_INITIATE )
  {
    switch ( State )
    {
      case STATE_ISO: // Here we start a new setpoint
        Status = STATUS_ON;
        break;

      case STATE_ACTIVE:  // We begin a new RAMP SOAK Cycle
        Status = STATUS_ON;

        initiateRamp();

        break;
    }
  }
}

void initiateRamp()
{
  double deltaSetpoint;
  rampStartInput = input;

  deltaSetpoint =  sequence[currentStep][PARAM_SETPOINT] - rampStartInput;
  maxrampSteps = (sequence[currentStep][PARAM_RAMPTIME] * 1000) / cycleTime;

  rampInterval =  deltaSetpoint / maxrampSteps;
  rampStep = 0;

  setpoint = rampStartInput;
  
  timerStep = millis() + (sequence[currentStep][PARAM_RAMPTIME] * 1000);

  State = STATE_RAMP;
}

void initiateSoak()
{
  State = STATE_SOAK;
  timerStep = millis() + (sequence[currentStep][PARAM_SOAKTIME] * 1000);
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
        case 3: //State
          Serial.print("3,3,");
          Serial.print(State);
          Serial.print('\n');
          break;
        case 4: //Ramp Soak Time
          Serial.print("3,4,");
          Serial.print( (timerStep-millis()) );
          Serial.print('\n');
          break;
        case 5: //Status
          Serial.print("3,5,");
          Serial.print(Status);
          Serial.print('\n');
          break;
        case 7: //setpoint
          Serial.print("3,7,");
          Serial.print(setpoint);
          Serial.print('\n');
          break;
        case 10:
          int id, param;
          double value;

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
      }



      break;
    case 6: //write variable
      command = strtok(0, ",");
      action = atoi(command);
      switch (action)
      {
        case 3: // State
          command = strtok(0, ",");
          State = atoi(command);
          break;
        case 5: // Status
          action = atoi( strtok(0, ",") );
          
          Status =  action;
          if( Status == STATUS_INITIATE )
          {
            initiateRun();
          }
          break;
        case 6: // startStep
          command = strtok(0, ",");
          startStep = atoi(command);
          break;
        case 10: //Ramp Soak Program
          int id, param;
          double value;

          id = atoi(strtok(0, ","));
          param = atoi(strtok(0, ","));

          value = strtod(strtok(0, ","), NULL);

          sequence[id][param] = value;
          break;
        case 11: //PID Parameters

          kp = strtod(strtok(0, ","), NULL);
          ki = strtod(strtok(0, ","), NULL);
          kd = strtod(strtok(0, ","), NULL);

          controllerPID.SetTunings( kp, ki, kd);
          break;
      }
      break;
  }
}

double read_temps(void)
{
  int input = analogRead(tempPin);

  double a = 1.0 - ((double)input / 1024.0);
  double R = (SERIESRESISTOR - (a * SERIESRESISTOR)) / a;


  double t_read = A_value + (B_value * log(R)) + (C_value * pow(log(R), 3));
  t_read = 1 / t_read;

  t_read = t_read - 273.15;
  return t_read;
}
