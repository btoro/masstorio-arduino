# Masstorio-arduino
Arduino temperature control software with programs, and ramping

## General Information
Masstorio-arduino was used to regulate a temperature-controlled nanospray mass spectrometry source. The controller code allows for isothermal regulation (single temperature), ramp-soak programs, and constant temperature ramping. The code here is only for the Arudino and was generally ran alongside a controller software on a PC. 

While, this software was built for a specialized task, it can be quickly modified for other temperature-regulation functions.

## Hardware considerations

Temperature is read using a thermistor circuit.
Solid-state relays were used to control cartridge heaters. Only on-off control is supported.

Another relay was used to regulate a water circulator for cooling applications. You can ignore/omit this. It is not integral to the temperature-regulation operations. 

## Operating modes
Three main operation modes are present:

### 1. Isothermal control 

![image](https://user-images.githubusercontent.com/15056753/46376943-5bdce580-c665-11e8-9255-f4c3d025f42e.png)

### 2. Ramp-soak control

![image](https://user-images.githubusercontent.com/15056753/46376947-60090300-c665-11e8-8e58-3e708f890928.png)

### 3. Constant-rate ramp control



## Communication with the controller

Serial communication with the controller can be established with a baudrate of 57600
Serial commands are sent as strings ending with '\n'

Command structure:

    6,2,X,Y,Z\n
 
 The first integer specifies a request (3) or write (6) command.
 The second integer specifies the commmand type
 The later are variables, only applicable for write commands

### Request commands

Process value (temperature)

    3,1

Output value (0-100% heater output)

    3,2   

State (enum STATE)

    3,3     

    typedef enum STATE
    {
      STATE_INACTIVE, // nothing, cannot run or do anything
      STATE_ISO, // Isothermal control
      STATE_ACTIVE, // ready for Ramp soak program
      STATE_RAMP, // is ramping
      STATE_SOAK, // is soaking
      STATE_PAUSE,
      STATE_ERROR, // no T read
      STATE_HIGHTEMP
    } State_t;

Ramp soak time (Time left for current step in msec)

    3,4   

Status (enum STATUS)

    3,5   
    
    typedef enum STATUS
    {
      STATUS_OFF, // normaly off
      STATUS_ON, // switches to ON during run
      STATUS_INITIATE, // starts a cycle
      STATUS_COMPLETE // cycle is completed
    } Status_t;

Setpoint value (set temperature)

    3,7   


PID gain values (returns three values)

    3,8   

Mode (enum MODE)

    3,9   

    typedef enum MODE
    {
      MODE_ISOTHERMAL, // isothermal operating mode
      MODE_RAMPSOAK, // ramp soak 
      MODE_RATE, // constant rate mode
    } Mode_t;

Sequence details ( returns the parameter for a step in the uploaded program sequence)

    3,10,ID,PARAM   

Sequence gains ( returns the gain for a step in the uploaded program sequence)

    3,11,ID,PARAM   

Current step ( returns step in the running sequence program)

    3,12   

Rate value ( returns constant ramp rate, if applicable)

    3,13 

Direction ( returns temp control direction. 0 -> Heating; 1 -> cooling ) 

    3,14  



    


### Write commmands





 
## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgements

This was influenced by ...

