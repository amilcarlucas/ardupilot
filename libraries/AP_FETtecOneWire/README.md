==============
FETtec OneWire
==============

This is a half duplex serial protocol with a 2Mbit/s Baudrate created by Felix Niessen (former Flyduino KISS developer) from `FETtec <https://fettec.net/>`_.


Ardupilot to ESC protocol
=========================

How many bytes per message?
There are two types of messages:
	- Configuration message consists of six frame bytes + payload bytes.
	Byte 0 is the transfer direction(e.g. 0x01 Master to Slave)
	Byte 1 is the ID
	Byte 2 is the frame type (Low byte)
	Byte 3 is the frame type (High byte)
	Byte 4 is the frame length
	Byte 5-254 is the payload
	Byte 6-255 is CRC (last Byte after the Payload)
	
	- Fast Throttle Signal
	The signal is used to transfer the eleven bit throttle signals with as few bytes as possible
	The structure is a bit more complex. In the first two bytes are frame header and telemetry request as well as the first parts of the throttle
	signal. The following bytes are transmitting the throttle signals for the ESCs (11bit per ESC) followed by the CRC. 
	
	If telemetry is requested the ESCs will answer them in the ESC-ID order. See comments in onewire.c for details.
	
Which CRC? What proprieties has this CRC?
	The CRC is a well known bytewise 8bit CRC not bitwise as DSHOT is. 


How many ESCs are supported? At which Rate?
	Four ESCs need 90uS for the throttle request and telemetry receiption. With four ESCs 11kHz are possible. As each additional ESC adds 11bits
	and 16 telemetry bits the rate is lowered by each ESC. 
	
	8 ESCs need 160uS including telemetry response, so 5.8kHz are possible. 
	
	
What is the pause time between messages?
	250ms then the Motors will disarm.
	
What can we configure in the ESCs?
	Every setting that the ESCs offer. The FETtec configurator uses OneWire too. Even the firmware updater bootloader uses onewire. 
	
What is the signal resolution?
	11bits per throttle signal, 16 bits per telemetry type.
	
Can the motors rotate in both directions? How is that done ?
	yes this is standard. (1020-2000 is throttle forward, 980-0 is throttle backward)
	
	
ESC to Ardupilot protocol
=========================

It supports ESC telemetry, so information from the ESC status can be sent back to the autopilot:

- Electronic rotations per minute (eRPM) (must be divided by number of motor poles to translate to propeller RPM)
- Input voltage (V/10)
- Current draw (A/10)
- Power consumption (mAh)
- Temperature (°C)

At which Rate?
 As described above. 
This information allows the autopilot to:

- dynamically change the center frequency of the notch filters used to reduce frame vibration noise in the gyros
- log the status of each ESC to the SDCard or internal Flash, for post flight analysis
- send the status of each ESC to the Ground Station or companion computer for real-time monitoring
- 

Extra features
==============

The ESC can beep and have lights. To control this you must add this code snippet to the header file:

```
public:
/**
    makes all connected ESCs beep
    @param beepFrequency a 8 bit value from 0-255. higher make a higher beep
*/
    void Beep(uint8_t beepFrequency);

/**
    sets the racewire color for all ESCs
    R, G, B = 8bit colors
*/
    void RW_LEDcolor(uint8_t R, uint8_t G, uint8_t B);
```

And this code snippet to the .cpp file:

```
/**
    makes all connected ESCs beep
    @param beepFrequency a 8 bit value from 0-255. higher make a higher beep
*/
void AP_FETtecOneWire::Beep(uint8_t beepFrequency)
{
    if (_IDcount > 0) {
        uint8_t request[2] = {OW_BEEP, beepFrequency};
        uint8_t spacer[2] = {0, 0};
        for (uint8_t i = _minID; i < _maxID + 1; i++) {
            Transmit(i, request, FETtecOneWire_RequestLength[request[0]]);
            // add two zeros to make sure all ESCs can catch their command as we don't wait for a response here
            _uart->write(spacer, 2);
            _IgnoreOwnBytes += 2;
        }
    }
}

/**
    sets the racewire color for all ESCs
    @param R red brightness
    @param G green brightness
    @param B blue brightness
*/
void AP_FETtecOneWire::RW_LEDcolor(uint8_t R, uint8_t G, uint8_t B)
{
    if (_IDcount > 0) {
        uint8_t request[4] = {OW_SET_LED_TMP_COLOR, R, G, B};
        uint8_t spacer[2] = {0, 0};
        for (uint8_t i = _minID; i < _maxID + 1; i++) {
            Transmit(i, request, FETtecOneWire_RequestLength[request[0]]);
            // add two zeros to make sure all ESCs can catch their command as we don't wait for a response here
            _uart->write(spacer, 2);
            _IgnoreOwnBytes += 2;
        }
    }
}
```

Connection and Halfduplex with STM
==================================
Normally halfduplex can be used for the communication. 
To have reliable 2Mbit/s Baudrate the GPIO should be set to PushPull.
On STM it is a special mode that is initialized like:

gpioInit.mode = LL_GPIO_MODE_ALTERNATE;
gpioInit.Pull = LL_GPIO_MODE_ALTERNATE;
gpio.Pull = LL_GPIO_PULL_UP;
gpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
gpioInit.Alternate = LL_GPIO_AF_1;



FullDuplex with Diode
==================================
If halfduplex is not available or working there is a workaround with a diode using the normal full duplex.
This is also working with other "non-halfduplex" Controller like ESP32.

The TLM-Pin (ESC) and RX (FC) connects to the anode (black side) of the diode.
TX (FC) connects to cathode (white striped side) of the diode.

 (TX --|<-- RX, TLM)
 
 
OneWire Configuration:
==================================
This settings have to be changed under CONFIG -> Full Parameter List to get it working.
MOT_PWM_MAX 2000
MOT_PWM_MIN 1000
Serial2_OPTIONS 160 #FOR full duplex - Use with diode
Serial2_OPTIONS 164 #FOR half duplex - Use without diode -> TX from FC to TLM of FETtec ESC

#FOR QUADCOPTER -> Sum of binary values
SERVO_FTW_MASK 15 
MOTOR 		1 | 2 | 3 | 4 | 5 | 6  ...
BIN VAL	    1 | 2 | 4 | 8 | 16| 32 ...
ACTIV		X | X | X | X | 0 | 0  ...
SUM     	1 + 2 + 4 + 8 = 15

Change SERVO*_FUNCTION to 33-38 according to the needed motor order e.g.:
SERVO1_FUNCTION 33
SERVO2_FUNCTION 34
SERVO3_FUNCTION 35
SERVO4_FUNCTION 36

