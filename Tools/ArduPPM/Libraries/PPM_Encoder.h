// -------------------------------------------------------------
// PPM ENCODER V3.0.1 Beta 3 (18-11-2013)
// -------------------------------------------------------------
// Improved servo to ppm for ArduPilot MEGA v1.x (ATmega328p),
// PhoneDrone and APM2.x (ATmega32u2)

// By: John Arne Birkeland - 2012 : PWM servo mode
//                                  PPM passthrough mode
// By: Olivier ADLER : PPM redundancy mode - 2013
//                     Tests - 2012 - 2013
// 

    // Todo PPM redundancy mode
    // In the validation checker do not allow validation before two valid frames.
    // Reverse ppm output polarity when PPM2 input is selected (need support from APM code)
    // The goal is that the APM can detect witch receiver input is active and log it
        
    // Todo Servo mode
    
    // Add compatibility with failsafe methods
    // #define FAILSAFE_MUTE  
    // #define FAILSAFE_THROTTLE
    // #define FAILSAFE_EXTRA_LOW

// ---------------------------------------------------------------------------------
// ARDUPPM OPERATIONAL DESCRIPTION
// ---------------------------------------------------------------------------------

// APM 2.x LED STATUS:
// --------------------------------------------------------------------------------

// SERVO PWM or PPM PASSTHROUGH MODES

// RX - OFF         = No input signal detected
// RX - SLOW TOGGLE = Input signal OK
// RX - FAST TOGGLE = Invalid input signal(s) detected
// RX - ON          = Input signal(s) lost during flight and fail-safe activated

// TX - OFF         = PPM output disabled
// TX - FAST TOGGLE = PPM output enabled
// TX - SLOW TOGGLE = PPM pass-trough mode

// PPM REDUNDANCY MODE
// In PPM redundancy mode TX and RX LEDs have a different meaning :

// TX - OFF         = PPM1 input valid
// TX - ON          = PPM1 sync or channel error

// RX - OFF         = PPM2 input valid
// RX - ON          = PPM2 sync or channel error

// C7 - ON          = PPM redundancy mode active (Green LED)
// C7 - SLOW TOGGLE = PPM redundancy mode : PPM2 switchover
// --------------------------------------------------------------------------------

// SERVO INPUT (PWM) MODE:
// -----------------------
// - PPM output will not be enabled unless a input signal has been detected and verified
// - Verified inputs are lost during operation (lose servo wire or receiver malfunction):
//   + The last known value of the lost input channel is kept for ~1 second
//   + If the lost input channel is not restored within ~1 second, it will be set to the default fail-safe value
// - Lost channel signal is restored:
//   + Normal channel operation is restored using the valid input signal

// PPM PASS-THROUGH MODE (signal pin 2&3 shorted):
// -----------------------------------------------
// - PPM output will not be enabled unless a input signal has been detected
// - Active signal on input channel 1 has been detected:
//   + Any input level changes will be passed directly to the PPM output (PPM pass-trough)
//   + If no input level changes are detected withing 250ms:
//     + PPM output is enabled and default fail-safe values for all eight channels transmitted
//     + Input level change detected again, PPM fail-safe output is terminated and normal PPM pass-through operation is restored

// PPM REDUNDANCY MODE (signal pin 7&8 shorted):
// -----------------------------------------------
// PPM redundancy mode do allow connection of two receivers in PPM mode.
// It will continously check the incoming signals and will isolate the APM from wrong signals.

// - PPM1 input is selected if it is valid
// - PPM2 output will be selected if PPM1 is not valid and PPM2 is valid
// - PPM1 is selected if both signals are valid, except if PPM2 is forced through the PPM1 force switchover channel
// - If both inputs are lost during more than 250 ms
// - or if both inputs are invalid then the selected failsafe method will be enabled :

// FAILSAFE_MUTE        // Failsafe method muting ppm output ( mainly for use with other systems )
// FAILSAFE_THROTTLE    // Legacy throtlle failsafe method using a low value (900 us) on the throttle channel
// FAILSAFE_EXTRA_LOW   // Prefered method using extra low PWM values on missing channels

// Hardware setup for PPM REDUNDANCY MODE :
//-----------------
// - PPM1 input on channel 1 (PB0)
// - PPM2 input on channel 5 (PB4)
// - Jumper between PWM signal input pins 7 and 8


// Changelog:
// ------------------------------------------------------------------------------------------------------------------------

// 01-08-2011
// V2.2.3 - Changed back to BLOCKING interrupts.
//          Assembly PPM compare interrupt can be switch back to non-blocking, but not recommended.
// V2.2.3 - Implemented 0.5us cut filter to remove servo input capture jitter.

// 04-08-2011
// V2.2.4 - Implemented PPM passthrough function.
//          Shorting channel 2&3 enabled ppm passthrough on channel 1.

// 04-08-2011
// V2.2.5 - Implemented simple average filter to smooth servo input capture jitter.
//          Takes fewer clocks to execute and has better performance then cut filter.

// 05-08-2011
// V2.2.51 - Minor bug fixes.

// 06-08-2011
// V2.2.6 - PPM passthrough failsafe implemented.
//          The PPM generator will be activated and output failsafe values while ppm passthrough signal is missing.

// 01-09-2011
// V2.2.61 - Temporary MUX pin always high patch for APM beta board

// 22-09-2011
// V2.2.62 - ATmegaXXU2 USB connection status pin (PC2) for APM UART MUX selection (removed temporary high patch)
//         - Removed assembly optimized PPM generator (not usable for production release)

// 23-09-2011
// V2.2.63 - Average filter disabled

// 24-09-2011
// V2.2.64 - Added distinct Power on / Failsafe PPM values
//         - Changed CH5 (mode selection) PPM Power on and Failsafe values to 1555 (Flight mode 4)
//         - Added brownout detection : Failsafe values are copied after a brownout reset instead of power on values

// 25-09-2011
// V2.2.65 - Implemented PPM output delay until input signal is detected (PWM and PPM pass-trough mode)
//         - Changed brownout detection and FailSafe handling to work with XXU2 chips
//         - Minor variable and define naming changes to enhance readability

// 15-03-2012
// V2.2.66  - Added APM2 (ATmega32U2) support for using TX and RX status leds to indicate PWM and PPM traffic 
//            - <RX>: <OFF> = no pwm input detected, <TOGGLE> = speed of toggle indicate how many channel are active, <ON> = input lost (failsafe)
//            - <TX>: <OFF> = ppm output not started, <FAST TOGGLE> = normal PWM->PPM output or PPM passtrough failsafe, <SLOW TOGGLE> = PPM passtrough

// 03-06-2012
// V2.2.67 - Implemented detection and failsafe (throttle = 900us) for missing throttle signal. 

// 04-06-2012
// V2.2.68 - Fixed possible logic flaw in throttle failsafe reset if  _JITTER_FILTER_ is enabled

// 02-11-2012
// V2.2.69 - Added PPM output positive polarity - mainly for standalone PPM encoder board use

// 03-11-2012
// V2.3.0 - Implemented single channel fail-safe detection for all inputs

// 16-11-2012
// V2.3.1 - Improved watchdog timer reset, so that only valid input signals will prevent the watchdog timer from triggering

// 22-11-2012
// V2.3.11 - Very experimental test forcing throttle fail-safe (RTL) on single channel loss. !DO NOT RELEASE TO PUBLIC!
//         - Test for active input channels during init

// 23-11-2012 !VERY EXPERIMENTAL! 
// V2.3.11 - Nested interrupt PWM output interrupt (PPM generator) for greatly improved jitter performance
//         - Active input channel test during init removed
//         - Implemented dynamic testing of active input channels

// 23-12-2012
// V2.3.12 - New improved fail-safe detection and handling for single or multiple signal loss and receiver malfunction
//         - Improved LED status for APM 2.x
//         - Improved jitter performance (PPM output using nested interrupts)

// 30-11-2012
// V2.3.13pre - Internal dev only pre-release
//            - Improved active channel detection requiring 100 valid pulses before channel is marked active
//            - Removed forced throttle fail-safe after channel loss
//            - Changed fail-safe values to 800us, this will be used by APM code to detect channel loss and decide fail safe actions

// 16-12-2012
// V2.3.13 - Official release
//         - Fail-safe vales changed back to normal fail-safe values. Use until APM code knows how to handle lost channel signal (800us)

// 10-01-2013
// V2.3.14pre - Internal test release
//            - If one or more or all channel(s) are disconnected, throttle is set to fail-safe low (RTL)
//            - If the missing channel(s) are regained, throttle control is regained

// 11-01-2013
// V2.3.14 - temporary release for ArduCopter 2.9
//         - fail-safe throttle low can be set with a define
//         - recovery from error condition can also be set with a define

// 13-01-2013
// V2.3.15 - very small bug fix: speedup

// 27-02-2013
// V2.3.16 - if channel 1, 2 or 4 is disconnected: set it to center (1500us)
//         - if channel 3 (throttle) is disconnected: set it to low (900us)
//         - if channel 5-8 are disconnected: set it to last value
//         - permanent LED error condition indication is only triggered when throttle is set low (or all channels are disconnected)
//
// 12-10-2012
// V3.0.0 - Added dual input PPM redundancy mode with auto switchover. This is mainly for dual PPM receivers setup.

// 03-11-2013
// V3.0.1 - PPM redundancy mode : cleaning to allow compilation.
//        - PPM redundancy mode : logic errors corrected inside decoding algorithm
//        - PPM redundancy mode :  - use input pins 1 and 5 for PPM1 and PPM2 
//                                 - use jumper between pin 7 and 8 to select PPM redundancy mode
//                                 / this is to avoid SPI lines conflicts with ISP programmers.
//        - PPM redundancy mode : frame formats definition simplification, new values for better sync detection compatibility
//        - PPM redundancy mode : implemented sync PPM output
//        - PPM redundancy mode : adding specific LED control
//        - Watchdog interrupt : add code for redundancy mode
//        - Watchdog interrupt : removed the legacy failsafe method for servo mode (throttle = 900 us and load all other channels with default values)
//        - PPM encoder init : add code for PPM redundancy mode
//        - arduino USB code : disabling LED control to allow PPM debugging with Mission Planner connected
//        - Failsafe changes : see manual
// FAILSAFE_MUTE        // Failsafe method muting ppm output ( mainly for use with other systems )
// FAILSAFE_THROTTLE    // Legacy throtlle failsafe method using a low value (900 us) on the throttle channel
// FAILSAFE_EXTRA_LOW   // Prefered method using extra low PWM values on missing channels

// ------------------------------------------------------------------------------------------------------------------------


// -----------------------------------------------------------------------------------------
// MACRO DEFINITIONS / GLOBAL VARIABLES
// -----------------------------------------------------------------------------------------

#ifndef _PPM_ENCODER_H_
#define _PPM_ENCODER_H_

#include <avr/io.h>

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

// -------------------------------------------------------------
// SERVO INPUT FILTERS AND PARAMETERS
// -------------------------------------------------------------
// Using both filters is not recommended and may reduce servo input resolution

//#define _AVERAGE_FILTER_            // Average filter to smooth servo input capture jitter
#define _JITTER_FILTER_ 2           // Cut filter to remove servo input capture jitter (1 unit = 0.5us)
#define _INPUT_ERROR_TRIGGER_ 100   // Number of invalid input signals allowed before triggering alarm
// -------------------------------------------------------------

#ifndef F_CPU
#define F_CPU             16000000UL
#endif

#ifndef true
#define true                1
#endif

#ifndef false
#define false               0
#endif

#ifndef bool
#define bool                _Bool
#endif

// Version stamp for firmware hex file ( decode hex file using <avr-objdump -s file.hex> and look for "ArduPPM" string )
const char ver[15] = "ArduPPMv3.0.1";

// Licence scheme stamp( decode hex file using <avr-objdump -s file.hex> and look for "GPLv3-licencing" string )
const char licence[19] = "-GPLv3-licencing-";


// -------------------------------------------------------------
// INPUT MODE (by jumper)
// -------------------------------------------------------------

#define JUMPER_SELECT_MODE    0    // Jumper selected
#define SERVO_PWM_MODE        1    // Normal 8 channel servo (pwm) input
#define PPM_PASSTROUGH_MODE   2    // PPM signal passthrough on channel 1 if input pins 2&3 shorted
#define PPM_REDUNDANCY_MODE   3    // PPM redundancy on channels 1 and 2 if input pins 7&8 shorted

// Force input mode default = selection by jumper
volatile uint8_t input_mode = JUMPER_SELECT_MODE;


// -------------------------------------------------------------
// FAILSAFE MODE
// -------------------------------------------------------------

// FAILSAFE_EXTRA_LOW is the prefered new failsafe method
// Check compatibility with your APM board firmware
// Channel failsafe value for each lost or invalid channel
// will be the extra low value specified with PPM_CHANNEL_LOSS setting
// available in the "PPM output special values" section

#define FAILSAFE_MUTE      0  // Failsafe method muting ppm output ( mainly for use with other systems )
#define FAILSAFE_THROTTLE  1  // Legacy throtlle failsafe method using a low value (900 us) on the throttle channel
#define FAILSAFE_EXTRA_LOW 2  // Prefered method using extra low PWM values on missing channels

#define _FAILSAFE_METHOD_ FAILSAFE_THROTTLE  // Select the failsafe method here

// -------------------------------------------------------------
// SERVO LIMIT VALUES
// -------------------------------------------------------------

// Timer1 ticks for 1 microsecond (2 timer ticks = 1 us at 16 MHz)
#define TICKS_FOR_ONE_US                F_CPU / 8 / 1000 / 1000
// 400us PPM pre pulse
#define PPM_PRE_PULSE         TICKS_FOR_ONE_US * 400
// Servo minimum position
#define PPM_SERVO_MIN         TICKS_FOR_ONE_US * 900 - PPM_PRE_PULSE
// Servo center position
#define PPM_SERVO_CENTER      TICKS_FOR_ONE_US * 1500 - PPM_PRE_PULSE
// Servo maximum position
#define PPM_SERVO_MAX         TICKS_FOR_ONE_US * 2100 - PPM_PRE_PULSE
// Servo input channels
#define SERVO_CHANNELS        8
// PWM channels minimum values
#define PWM_VAL_MIN         TICKS_FOR_ONE_US * 900 - PPM_PRE_PULSE
// PWM channels maximum values
#define PWM_VAL_MAX         TICKS_FOR_ONE_US * 2100 - PPM_PRE_PULSE

// -------------------------------------------------------------
// PPM output special values
// -------------------------------------------------------------

// Throttle default at power on
#define PPM_THROTTLE_DEFAULT  TICKS_FOR_ONE_US * 1100 - PPM_PRE_PULSE
// Throttle during failsafe
#define PPM_THROTTLE_FAILSAFE TICKS_FOR_ONE_US * 900 - PPM_PRE_PULSE
// Channel loss failsafe
#define PPM_CHANNEL_LOSS TICKS_FOR_ONE_US * 700 - PPM_PRE_PULSE
// CH5 power on values (mode selection channel)
#define PPM_CH5_MODE_4        TICKS_FOR_ONE_US * 1555 - PPM_PRE_PULSE

// -------------------------------------------------------------
// PPM OUTPUT SETTINGS
// -------------------------------------------------------------

//#define _POSITIVE_PPM_FRAME_    // PPM frame positive

// PPM negative is the more common format :
//
// Negative PPM frame : ¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯
//                      --->|<---ch7--->|<---ch8--->|<-CH0--frame sync symbol-->|----ch1----|----ch2----|----ch3----|----ch4----|----ch5----|<--

//
// 
// Positive PPM frame : _|¯¯|________|¯¯|________|¯¯|________________________|¯¯|________|¯¯|________|¯¯|________|¯¯|________|¯¯|________|¯¯|____
//                      --->|<---ch7--->|<---ch8--->|<-CH0--frame sync symbol-->|----ch1----|----ch2----|----ch3----|----ch4----|----ch5----|<--

// Output PPM channels
#define PPM_CHANNELS        8
// PPM period 18.5ms - 26.5ms (54hz - 37Hz) 
#define PPM_PERIOD            TICKS_FOR_ONE_US * ( 22500 - ( PPM_CHANNELS * 1500 ) )
// Size of ppm[..] data array
#define PPM_ARRAY_MAX         PPM_CHANNELS * 2 + 2

// -----------------------------------------------------------------------------------------------------------
// PWM SERVO MODE SETTINGS
// -----------------------------------------------------------------------------------------------------------

//#define _THROTTLE_LOW_FAILSAFE_INDICATION //if set, throttle is set to low when a single channel is lost
//#define _THROTTLE_LOW_RECOVERY_POSSIBLE //if set, throttle low recovers from being low when the single channel comes back, only makes sense together with _THROTTLE_LOW_FAILSAFE_INDICATION

#if defined _THROTTLE_LOW_RECOVERY_POSSIBLE && !defined _THROTTLE_LOW_FAILSAFE_INDICATION
#error failsafe recovery is only possible with throttle_low_failsafe_indication defined as well
#endif

// -----------------------------------------------------------------------------------------------------------
// PPM REDUNDANCY MODE SETTINGS
// -----------------------------------------------------------------------------------------------------------

#define AUTODETECTED_LAST_PPM1_CHANNEL   255 //Do not change this.

// SWITCHOVER FORCE PPM VALUE
#define PPM1_SWITCHOVER_FORCE_VAL_MIN	TICKS_FOR_ONE_US * 2000
#define PPM_SWITCHOVER_CHANNEL		AUTODETECTED_LAST_PPM1_CHANNEL	// Receiver 1 PPM channel to force input 2.
                                    // Use "AUTODETECTED_LAST_PPM1_CHANNEL" to use the last channel of PPM1 input (automatically detected)
                                    // Use "0" to disable switchover control from PPM1 input
                                    // Use a number for selecting a specific channel.
                                    // Should be between 6 to 16 because the APM needs at least channels 1 - 5 for model control
									// Preferably between 9 to 16 so that channels 1 to 8 are free for APM use.
                                    
#define RESTART_POLARITY_DETECTION_AFTER_WATCHDOG       // Comment to disable. Allow adaptation to a new channel count during normal operation.
#define RESTART_CHANNEL_COUNT_DETECTION_AFTER_WATCHDOG  // Comment to disable. Allow adaptation to a new channel count after a watchdog interrupt
                                                        // ! The watchdog trigger only if PPM1 and PPM2 signals are both fully dead during more than 250 ms (0.25s).

//#define DYNAMIC_CHANNEL_COUNT_DETECTION                 // Comment to disable. Allow adaptation to a new channel count during normal operation.

#define SWITCHOVER_2_to_1_DELAY		100	// Frame count delay for switching back to receiver 1 if both inputs are valid

#define MISSING_CHANNELS                    10  // Each PPM input is checking the validity of other one.
                                                // This value represente the missing PPM channels of the opposite PPM input before declaring it invalid.
                                                // This method is used because a dead input cannot update anymore its validity checks (wrong validity report).
                                                // If the PPM inputs are not synchronous (different transmitters) this value should not be set too low
                                                // Or this could cause wrong invalid detections in extreme cases.
                                                
#define POLARITY_DETECTION_THRESHOLD        80  // Valid needed prepulses before polarity detection validation
#define CHANNEL_COUNT_DETECTION_THRESHOLD	10  // Identical frames needed before channel count validation


// ----------------------------
// PPM1 input : frame format
// ----------------------------
 
// PPM channel count limits

#define PPM1_MIN_CHANNELS        5
#define PPM1_MAX_CHANNELS        16

// Values are expressed in microseconds

// PPM channel value limits
#define PPM1_CHANNEL_VALUE_MIN           TICKS_FOR_ONE_US * 725
#define PPM1_CHANNEL_VALUE_MAX           TICKS_FOR_ONE_US * 2275

//------------------------------------------------------------------------------------------------------
// Change this only if you know what your are doing !

// Sync detection
#define PPM1_FRAME_SYNC_DETECTION_MARGIN         TICKS_FOR_ONE_US * 425
#define PPM1_CHANNEL_SYNC_DETECTION_MARGIN       TICKS_FOR_ONE_US * 200

#define PPM1_SYNC_LENGTH_MAX		 TICKS_FOR_ONE_US * 30000

//Values below are automatically computed

// PPM prepulses length
#define PPM1_PREPULSE_MIN            PPM1_CHANNEL_SYNC_DETECTION_MARGIN
#define PPM1_PREPULSE_MAX            PPM1_CHANNEL_VALUE_MIN - PPM1_CHANNEL_SYNC_DETECTION_MARGIN
// Frame sync minimum lenght
#define PPM1_SYNC_LENGTH_MIN		 PPM1_CHANNEL_VALUE_MAX + PPM1_FRAME_SYNC_DETECTION_MARGIN

// ----------------------------
// PPM2 input : frame format
// ----------------------------
 
// PPM channel count limits
#define PPM2_MIN_CHANNELS     5
#define PPM2_MAX_CHANNELS     16

// Values are expressed in microseconds

// PPM channel value limits
#define PPM2_CHANNEL_VALUE_MIN           TICKS_FOR_ONE_US * 725
#define PPM2_CHANNEL_VALUE_MAX           TICKS_FOR_ONE_US * 2275

//------------------------------------------------------------------------------------------------------
// Change this only if you know what your are doing !

// Sync detection
#define PPM2_FRAME_SYNC_DETECTION_MARGIN         TICKS_FOR_ONE_US * 425
#define PPM2_CHANNEL_SYNC_DETECTION_MARGIN       TICKS_FOR_ONE_US * 200
#define PPM2_SYNC_LENGTH_MAX		 TICKS_FOR_ONE_US * 30000

//Values below are automatically computed

// PPM prepulses length
#define PPM2_PREPULSE_MIN            PPM2_CHANNEL_SYNC_DETECTION_MARGIN
#define PPM2_PREPULSE_MAX            PPM2_CHANNEL_VALUE_MIN - PPM2_CHANNEL_SYNC_DETECTION_MARGIN

// Frame sync minimum lenght
#define PPM2_SYNC_LENGTH_MIN		 PPM2_CHANNEL_VALUE_MAX + PPM2_FRAME_SYNC_DETECTION_MARGIN

// ---------------------------------------------------------------------------------------------------------------
// APM FAILSAFE VALUES
// ---------------------------------------------------------------------------------------------------------------

#if ( _FAILSAFE_METHOD_ == FAILSAFE_EXTRA_LOW )
volatile uint16_t failsafe_ppm[ PPM_ARRAY_MAX ] =                               
{
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 1
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 2
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 3
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 4
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 5
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 6
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 7
    PPM_PRE_PULSE,
    PPM_CHANNEL_LOSS,         // Channel 8
    PPM_PRE_PULSE,
    PPM_PERIOD
};
#else
// -------------------------------------------------------------
// SERVO FAILSAFE VALUES
// -------------------------------------------------------------
volatile uint16_t failsafe_ppm[ PPM_ARRAY_MAX ] =                               
{
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 1
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 2
    PPM_PRE_PULSE,
    PPM_THROTTLE_FAILSAFE,    // Channel 3 (throttle)
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 4
    PPM_PRE_PULSE,
    PPM_CH5_MODE_4,           // Channel 5
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 6
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 7
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 8
    PPM_PRE_PULSE,
    PPM_PERIOD
};
#endif

// -------------------------------------------------------------
// Data array for storing ppm (8 channels) pulse widths.
// -------------------------------------------------------------
volatile uint16_t ppm[ PPM_ARRAY_MAX ] =                                
{
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 1 
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 2
    PPM_PRE_PULSE,
    PPM_THROTTLE_DEFAULT,     // Channel 3 (throttle)
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 4
    PPM_PRE_PULSE,
    PPM_CH5_MODE_4,           // Channel 5
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 6
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 7
    PPM_PRE_PULSE,
    PPM_SERVO_CENTER,         // Channel 8
    PPM_PRE_PULSE,
    PPM_PERIOD
};

// ---------------------------------------------------------------
// Data array for storing ppm timeout (missing channel detection)
// ---------------------------------------------------------------

#define PPM_TIMEOUT_VALUE 40 // ~1sec before triggering missing channel detection
volatile uint8_t ppm_timeout[ PPM_ARRAY_MAX ];

// Servo input channel connected status
#define SERVO_INPUT_CONNECTED_VALUE 100
volatile uint8_t servo_input_connected[ PPM_ARRAY_MAX ];

#ifdef _THROTTLE_LOW_RECOVERY_POSSIBLE
// count the channels which have been once connected but then got disconnected
volatile uint8_t disconnected_channels;
#endif

// ---------------------------------------------------------------------------
// AVR hardware assignments for PhoneDrone and APM2 boards using ATmega32u2
// ---------------------------------------------------------------------------

#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)

#define SERVO_DDR             DDRB
#define SERVO_PORT            PORTB
#define SERVO_INPUT           PINB
#define SERVO_INT_VECTOR      PCINT0_vect
#define SERVO_INT_MASK        PCMSK0
#define SERVO_INT_CLEAR_FLAG  PCIF0
#define SERVO_INT_ENABLE      PCIE0
#define SERVO_TIMER_CNT       TCNT1

#define PPM_DDR               DDRC
#define PPM_PORT              PORTC
#define PPM_OUTPUT_PIN        PC6
#define PPM_INT_VECTOR        TIMER1_COMPA_vect
#define PPM_COMPARE           OCR1A
#define PPM_COMPARE_MODE_BIT_0      COM1A0
#define PPM_COMPARE_MODE_BIT_1      COM1A1
#define PPM_COMPARE_ENABLE    OCIE1A
#define PPM_COMPARE_FORCE_MATCH    FOC1A

#define USB_DDR            DDRC
#define USB_PORT           PORTC
#define USB_PIN            PC2

#define LED_REDUNDANCY     PC7
#define PPM1_PIN           PB0
#define PPM2_PIN           PB4

// ---------------------------------------------------------------------------
// AVR parameters for ArduPilot MEGA v1.4 PPM Encoder (ATmega328P)
// ---------------------------------------------------------------------------
#elif defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328__)

#define SERVO_DDR             DDRD
#define SERVO_PORT            PORTD
#define SERVO_INPUT           PIND
#define SERVO_INT_VECTOR      PCINT2_vect
#define SERVO_INT_MASK        PCMSK2
#define SERVO_INT_CLEAR_FLAG  PCIF2
#define SERVO_INT_ENABLE      PCIE2
#define SERVO_TIMER_CNT       TCNT1

#define PPM_DDR               DDRB
#define PPM_PORT              PORTB
#define PPM_OUTPUT_PIN        PB2
#define PPM_INT_VECTOR        TIMER1_COMPB_vect
#define PPM_COMPARE           OCR1B
#define PPM_COMPARE_MODE_BIT_0      COM1B0
#define PPM_COMPARE_MODE_BIT_1      COM1B1
#define PPM_COMPARE_ENABLE    OCIE1B
#define PPM_COMPARE_FORCE_MATCH    FOC1B

#else
#error NO SUPPORTED DEVICE FOUND! (ATmega16u2 / ATmega32u2 / ATmega328p)
#endif

// -------------------------------
// USB cable connection detection
// -------------------------------

// true if we have received a USB device connect event
static bool usb_connected;

// USB connected event
void EVENT_USB_Device_Connect(void)
{
    // Toggle USB pin high if USB is connected
    USB_PORT |= (1 << USB_PIN);

    usb_connected = true;

    // this unsets the pin connected to PA1 on the 2560
    // when the bit is clear, USB is connected
    PORTD &= ~1;
}

// USB disconnect event
void EVENT_USB_Device_Disconnect(void)
{
    // toggle USB pin low if USB is disconnected
    USB_PORT &= ~(1 << USB_PIN);

    usb_connected = false;

    // this sets the pin connected to PA1 on the 2560
    // when the bit is clear, USB is connected
    PORTD |= 1;
}

// ---------------------------------------------------------------------------
// Global management flags and counters
// ---------------------------------------------------------------------------

// Used to indicate invalid SERVO input signals
volatile uint8_t servo_input_errors = 0;

// Used to indicate that watchdog has been triggered (all input signals are missing : there is no more capture interrupt)
volatile bool watchdog_triggered = true;

// Used to indicate if PPM generator is active
volatile bool ppm_generator_active = false;

// Used to indicate a brownout restart
volatile bool brownout_reset = false;

// Current active ppm channel
volatile uint8_t ppm_out_channel = PPM_ARRAY_MAX - 1;

#ifdef _THROTTLE_LOW_FAILSAFE_INDICATION
// Used to force throttle fail-safe mode (RTL)
volatile bool throttle_failsafe_force = false;
#endif

// ------------------------------------------------------------------------------
// PPM GENERATOR START - "TOGGLE ON COMPARE" INTERRUPT ENABLING
// ------------------------------------------------------------------------------
void ppm_start( void )
{
        // Prevent enabling an already active PPM generator
        if( ppm_generator_active ) return;

        // Store interrupt status and register flags
        volatile uint8_t SREG_tmp = SREG;

        // Stop interrupts
        cli();

        // Set initial compare toggle to maximum (32ms) to give other parts of the system time to start
        SERVO_TIMER_CNT = 0;
        PPM_COMPARE = 0xFFFF;

        if( input_mode != SERVO_PWM_MODE ) // If we are in redundancy or passthrough mode
                                           // we need to force PPM output pin to the right state
        {                                  // before starting the PPM generator
            TCCR1B = 0; // Stop timer 1
            
            #if defined (_POSITIVE_PPM_FRAME_) // Select the compare output mode
                TCCR1A = ( 1 << PPM_COMPARE_MODE_BIT_1 ); // // Set compare output mode to "clear on compare match"
            #else
                TCCR1A = ( ( 1 << PPM_COMPARE_MODE_BIT_1 ) | ( 1 << PPM_COMPARE_MODE_BIT_0 ) ); // Set compare output mode to "set on compare match"
            #endif
            
            TCCR1C |= (1 << PPM_COMPARE_FORCE_MATCH); // Issue a force compare match to set the PPM output pin state
            
            // Reset ppm generator current channel to start generation on a frame sync symbol
            ppm_out_channel = PPM_ARRAY_MAX - 1;
        }
        else // We are in PWM servo mode
        {
            #if defined (_POSITIVE_PPM_FRAME_)
                // Force output compare to reverse polarity
                TCCR1C |= (1 << PPM_COMPARE_FORCE_MATCH); // Todo : works only for the first ppm generator start
            #endif
        }

        // Set compare output mode to "toggle on compare"
        TCCR1A = (1 << PPM_COMPARE_MODE_BIT_0);

        // Start TIMER1 with 8x prescaler
        TCCR1B = ( 1 << CS11 );

        // Enable output compare interrupt
        TIMSK1 |= (1 << PPM_COMPARE_ENABLE);

        // Indicate that PPM generator is active
        ppm_generator_active = true;

        #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)        
        // Turn on TX led if we are not in PPM redundancy mode
        if( input_mode != PPM_REDUNDANCY_MODE )
        {
            PORTD &= ~( 1<< PD5 );
        }
        #endif

        // Restore interrupt status and register flags
        SREG = SREG_tmp;
}

// ------------------------------------------------------------------------------
// PPM GENERATOR STOP - "TOGGLE ON COMPARE" INTERRUPT DISABLING
// ------------------------------------------------------------------------------
void ppm_stop( void )
{
        // Prevent stopping an already stopped PPM generator
        if( !ppm_generator_active ) return;
        
        // Store interrupt status and register flags
        volatile uint8_t SREG_tmp = SREG;

        // Stop interrupts
        cli();

        // Disable output compare interrupt
        TIMSK1 &= ~(1 << PPM_COMPARE_ENABLE);

        // Reset toggle on compare output
        TCCR1A = 0;
               
        if( input_mode != PPM_REDUNDANCY_MODE ) // If PPM redundancy mode leave timer 1 running for PPM input capture
        {
            TCCR1B = 0; // Stop timer 1
        }
        
        // Indicate that PPM generator is not active
        ppm_generator_active = false;

        #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)        
        // Turn off TX led if we are not in PPM redundancy mode
        if( input_mode != PPM_REDUNDANCY_MODE )
        {
            PORTD |= ( 1<< PD5 );
        }
        #endif
        
        // Restore interrupt status and register flags
        SREG = SREG_tmp;
}

// ------------------------------------------------------------------------------
// Watchdog  (interrupt mode, no reboot)
// ------------------------------------------------------------------------------
ISR( WDT_vect ) // If watchdog is triggered then enable Watchdog triggered flag and manage failsafe
{
    
    // If we are in the first watchdog timeout interrupt to be serviced or the AVR has experienced a brownout reset
    if ( watchdog_triggered == false || brownout_reset )
    {
        
        // If we are in PWM SERVO mode, 
        if ( input_mode == SERVO_PWM_MODE )
        {
                        
            #if ( _FAILSAFE_METHOD_ == FAILSAFE_THROTTLE )
            
                // Todo :
                // older method not used anymore ?
                /*
                // Load failsafe values on all channels
                for( uint8_t i = 0; i < PPM_ARRAY_MAX; i++ )
                {
                    ppm[ i ] = failsafe_ppm[ i ];
                }
                */
                
                // Set Throttle Low & leave other channels at last value
                ppm[5] = failsafe_ppm[ 5 ];
            
            #elif ( _FAILSAFE_METHOD_ == FAILSAFE_EXTRA_LOW )
            
                // Set Throttle extra Low & leave other channels at last value
                ppm[5] = failsafe_ppm[ 5 ];
            
            #else // We are in FAILSAFE_MUTE mode
            
                ppm_stop(); // Stop PPM generator during receiver lost period
            
            #endif
        }
        else // We are in PPM passthrough or PPM redundancy mode
        {
            #if ( _FAILSAFE_METHOD_ != FAILSAFE_MUTE ) // We are in a failsafe mode requiring the ppm generator
            
                // Load failsafe values on all channels
                for( uint8_t i = 0; i < PPM_ARRAY_MAX; i++ )
                {
                    ppm[ i ] = failsafe_ppm[ i ];
                }
                // Start the ppm generator
                ppm_start();
                
            #endif
        }
        
        
        //----------------------------
        // LED control
        //----------------------------

        #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
        
            // Turn on RX led if we are not in PPM redundancy mode
            if( input_mode != PPM_REDUNDANCY_MODE )
            {
                PORTD &= ~( 1<< PD4 );
            }
            else // if we are in PPM redundancy mode turn on both RX and TX LED to signal missing PPM on both inputs
            {
                PORTD &= ~( 1<< PD4 );
                PORTD &= ~( 1<< PD5 );
                PORTC &= ~( 1<< PC7 ); // Switch green LED ON (necessary if it was blinking)
            }
        #endif
        
        // Reset management variables
        
        watchdog_triggered = true; // Set Watchdog triggered flag
        brownout_reset = false; // Reset brownout flag
        servo_input_errors = 0; // Reset servo input error flag
    }
    
} 


// ------------------------------------------------------------------------------
// SERVO/PPM INPUT - PIN CHANGE INTERRUPT
// ------------------------------------------------------------------------------
ISR( SERVO_INT_VECTOR )
{
    // Servo pulse start timing
    static uint16_t servo_start[ PPM_ARRAY_MAX ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    // We sacrifice some memory but save instructions by working with ppm index count (18) instead of input channel count (8)
    
    #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
    
        // Toggle LED delay
        static uint8_t led_RX_delay = 0;
        static uint8_t led_TX_delay = 0;
        static uint8_t led_redundancy_delay = 0;
    #endif
    
    // Servo input pin storage 
    static uint8_t servo_pins_old = 0;
    
    // Used to store current servo input pins
    uint8_t servo_pins;
    uint8_t servo_change;
    uint8_t servo_pin;
    uint8_t ppm_channel;

    // Read current servo pulse change time
    uint16_t servo_time = SERVO_TIMER_CNT;
	
    

// ------------------------------------------------------------------------------
// PPM passthrough mode ( signal passthrough from channel 1 to ppm output pin)
// ------------------------------------------------------------------------------
    if( input_mode == PPM_PASSTROUGH_MODE )
    {
        // Has watchdog timer failsafe started PPM generator? If so we need to stop it.
        if( ppm_generator_active )
        {
            // Stop PPM generator
            ppm_stop();
        }
        // PPM Output driver (direct control ) :
        //-----------------------------------------------
        // PPM (channel 1) input pin is high
        if( SERVO_INPUT & 1 ) 
        {
            // Set PPM output pin high
            PPM_PORT |= (1 << PPM_OUTPUT_PIN);
        }
        // PPM (channel 1) input pin is low
        else
        {
            // Set PPM output pin low
            PPM_PORT &= ~(1 << PPM_OUTPUT_PIN);
        }
        //-----------------------------------------------
        // Reset Watchdog Timer
        wdt_reset(); 

        // Reset watchdog trigerred flag to indicate that we have received a ppm input signal
        watchdog_triggered = false;

        #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
        // Toggle TX LED at PPM passtrough
        if( ++led_TX_delay > 128 ) // Toggle every 128th pulse
        {
            // Toggle TX led
            PIND |= ( 1<< PD5 ); 
            led_TX_delay = 0;
        }
        #endif
        
        // Leave interrupt
        return;
    }
		
 // ------------------------------------------------------------------------------
 // SERVO PWM MODE
 // ------------------------------------------------------------------------------
    if( input_mode == SERVO_PWM_MODE )
    {
    //CHECK_PINS_START: // Start of servo input check

        // Store current servo input pins
        servo_pins = SERVO_INPUT;

        // Calculate servo input pin change mask
        servo_change = servo_pins ^ servo_pins_old;
        
        // Set initial servo pin and ppm[..]  index
        servo_pin = 1;
        ppm_channel = 1;

    CHECK_PINS_LOOP: // Input servo pin check loop

        // Check for pin change on current servo channel
        if( servo_change & servo_pin )
        {
            // Remove processed pin change from bitmask
            servo_change &= ~servo_pin;

            // High (raising edge)
            if( servo_pins & servo_pin )
            {
                servo_start[ ppm_channel ] = servo_time;
            }
            else
            {
                // Get servo pulse width
                uint16_t servo_width = servo_time - servo_start[ ppm_channel ] - PPM_PRE_PULSE;
                
                // Check that servo pulse signal is valid before sending to ppm encoder
                if( servo_width > PPM_SERVO_MAX ) goto CHECK_PINS_ERROR;
                if( servo_width < PPM_SERVO_MIN ) goto CHECK_PINS_ERROR;

                // Reset Watchdog Timer
                wdt_reset(); 

                // Reset Watchdog triggered flag to indicate that we have received servo input signals
                watchdog_triggered = false;
                
                // Count valid signals to mark channel active
                if( servo_input_connected[ ppm_channel ] < SERVO_INPUT_CONNECTED_VALUE )
                {
                    servo_input_connected[ ppm_channel ]++;
                }
                
                //Reset ppm single channel fail-safe timeout
                ppm_timeout[ ppm_channel ] = 0;

            #ifdef _THROTTLE_LOW_FAILSAFE_INDICATION
                // Check for forced throttle fail-safe
                if( throttle_failsafe_force )
                {
                    if( ppm_channel == 5 )
                    {
                        // Force throttle fail-safe
                        servo_width = PPM_THROTTLE_FAILSAFE;
                    }
                }
            #endif
        
            #ifdef _AVERAGE_FILTER_
                // Average filter to smooth input jitter
                servo_width += ppm[ ppm_channel ];
                servo_width >>= 1;
            #endif

            #ifdef _JITTER_FILTER_
                // 0.5us cut filter to remove input jitter
                int16_t ppm_tmp = ppm[ ppm_channel ] - servo_width;
                if( ppm_tmp <= _JITTER_FILTER_ && ppm_tmp >= -_JITTER_FILTER_ ) goto CHECK_PINS_NEXT;
            #endif

                // Update ppm[..]
                ppm[ ppm_channel ] = servo_width;
            }
        }
        
    CHECK_PINS_NEXT:
     
        // Are we done processing pins?
        if( servo_change )
        {
            // Select next ppm[..] index
            ppm_channel += 2;

            // Select next servo pin
            servo_pin <<= 1;
         
            // Check next pin
            goto CHECK_PINS_LOOP;
        }

        // All changed pins have been checked
        goto CHECK_PINS_DONE;

    CHECK_PINS_ERROR:
        
        // Used to indicate invalid servo input signals
        servo_input_errors++;

        goto CHECK_PINS_NEXT;
        
    // Processing done, cleanup and exit
    CHECK_PINS_DONE:

        // Start PPM generator if not already running
        if( !ppm_generator_active ) ppm_start();

        #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
       
        // Toggle RX LED when finished receiving servo pulses
        if( ++led_RX_delay == 0 ) // Toggle led every 128th or 255th interval
        {
            PIND |= ( 1<< PD4 );

            // Fast toggle on servo input errors
            if( servo_input_errors > _INPUT_ERROR_TRIGGER_ )
            {
                led_RX_delay = 200;
            }
        }
        #endif    

        // Store current servo input pins for next check
        servo_pins_old = servo_pins;

        //Has servo input changed while processing pins, if so we need to re-check pins
        //if( servo_pins != SERVO_INPUT ) goto CHECK_PINS_START;

        // Clear interrupt event from already processed pin changes
        //PCIFR |= (1 << SERVO_INT_CLEAR_FLAG);
        
        // Leave interrupt
        return;
    }    



// ------------------------------------------------------------------------------
// PPM redundancy mode
// ------------------------------------------------------------------------------
			
	if( input_mode == PPM_REDUNDANCY_MODE )
    {
		// -------------------------------------
		// PPM redundancy mode - variables Init
		// -------------------------------------
        
        // array indexes
        #define PPM_CH1    0
        #define PPM_CH2    1
        
        // ppm polarity
        #define NEGATIVE_POLARITY    0
        #define POSITIVE_POLARITY    1
        
        struct RM_macros_t
        {
            uint16_t ppm_channel_value_min;
            uint16_t ppm_channel_value_max;
            uint16_t ppm_prepulse_min;
            uint16_t ppm_prepulse_max;
            uint16_t ppm_sync_length_min;
            uint16_t ppm_sync_length_max;
            uint8_t ppm_min_channels;
            uint8_t ppm_max_channels;
            uint8_t ppm_pin;
        };
        // constant structured array for macro expansion in the decoder function
        const struct RM_macros_t ppm_defn[2] = \
        {{ PPM1_CHANNEL_VALUE_MIN, PPM1_CHANNEL_VALUE_MAX, PPM1_PREPULSE_MIN, PPM1_PREPULSE_MAX, PPM1_SYNC_LENGTH_MIN, PPM1_SYNC_LENGTH_MAX, PPM1_MIN_CHANNELS, PPM1_MAX_CHANNELS, PPM1_PIN }, \
        { PPM2_CHANNEL_VALUE_MIN, PPM2_CHANNEL_VALUE_MAX, PPM2_PREPULSE_MIN, PPM2_PREPULSE_MAX, PPM2_SYNC_LENGTH_MIN, PPM2_SYNC_LENGTH_MAX, PPM2_MIN_CHANNELS, PPM2_MAX_CHANNELS, PPM2_PIN }};
  
        struct RM_variables_t
        {
        // PPM pulses start times
		uint16_t ppm_start[17];
        // PPM channels widths
		uint16_t ppm_channel_width[17];
        // PPM prepulse start
		uint16_t ppm_prepulse_start;
		// PPM prepulse width
		uint16_t ppm_prepulse_width;
		// PPM current channel ( Channel 0 = frame sync symbol )
		uint8_t ppm_channel;
        // Previous PPM channel ( Channel 0 = frame sync symbol )
		uint8_t ppm_dead_detector_previous_channel;
        // Dead counter (for dead ppm input detection)
        uint8_t ppm_dead_counter;
        // Polarity detection counter
		uint8_t ppm_polarity_detection_counter;
        // Detected Channel counts
		uint8_t ppm_channel_count;
		// Detected Channel count (previous value)
		uint8_t ppm_previous_channel_count;
		// Channel counts detection : detection counter
		uint8_t ppm_channel_count_detection_counter;
        };
        static struct RM_variables_t ppm_var[2];

        
        // PPM redundancy mode - Flags
        struct RM_flags_t
        {
            // Frame sync flag
            uint8_t ppm_sync                       : 1; // Bit 0  = false
            // Sync error flag
            uint8_t ppm_sync_error                 : 1; // Bit 1  = true
            // Channel error flag
            uint8_t ppm_channel_error              : 1; // Bit 2  = true
            // Frame completed Flag
            uint8_t ppm_frame_completed            : 1; // Bit 3  = false
            // ppm validity flag
            uint8_t ppm_valid                      : 1; // Bit 4  = false
            // Polarity detection started flag
            uint8_t ppm_polarity_detection_started : 1; // Bit 5  = false
            // Polarity deteted flag
            uint8_t ppm_polarity_detected          : 1; // Bit 6  = false
            // Polarity deteted flag
            uint8_t ppm_polarity                   : 1; // Bit 7  = false
            // Channel count detection ready flag
            uint8_t ppm_count_detection_started    : 1; // Bit 8  = false
            // Channel count detected flag
            uint8_t ppm_channel_count_detected     : 1; // Bit 9  = false
        };
        static struct RM_flags_t ppm_flag[2] = \
        {{ false, true, true, false, false, false, false, false, false, false }, \
        { false, true, true, false, false, false, false, false, false, false }};
        
        // PPM2 switchover flag
        static uint8_t ppm2_switchover = false;
        
        // PPM switchover force channel
        static uint8_t ppm2_switchover_force_channel = 0;
        
        // PPM switchover_delay_2_to_1 counter
        static uint8_t switchover_delay_2_to_1 = 0;
        
        // ----------------------------------------------
		// PPM redundancy mode - inputs states analyzis
		// ----------------------------------------------

        // Store current ppm inputs pins states
		servo_pins = SERVO_INPUT;

		// Calculate ppm input pin change mask
		uint8_t servo_change = servo_pins ^ servo_pins_old;
        
        // Store current ppm input pins states for next check
		servo_pins_old = servo_pins;

        // ------------------------------------------------------
		// PPM redundancy mode - Resets
		// ------------------------------------------------------
        
        // Reset flags and variables if we come out of a watchdog condition
        inline void reset_vars( uint8_t ppm_input )
        {
            ppm_flag[ppm_input].ppm_valid = false;
            ppm_flag[ppm_input].ppm_sync_error = true;
            ppm_flag[ppm_input].ppm_sync = false;
            ppm_flag[ppm_input].ppm_channel_error = true;
            ppm_flag[ppm_input].ppm_frame_completed = false;
            ppm_flag[ppm_input].ppm_count_detection_started = false;
            
            ppm_var[ppm_input].ppm_channel = 0;
            ppm_var[ppm_input].ppm_dead_detector_previous_channel = 255;
            ppm_var[ppm_input].ppm_dead_counter = 0;
            ppm_var[ppm_input].ppm_channel_count_detection_counter = 0;
            ppm_var[ppm_input].ppm_previous_channel_count = 255;
            
            #ifdef RESTART_POLARITY_DETECTION_AFTER_WATCHDOG
            ppm_flag[ppm_input].ppm_polarity_detection_started = false;
            ppm_flag[ppm_input].ppm_polarity_detected = false;
            ppm_flag[ppm_input].ppm_polarity = false;
            ppm_var[ppm_input].ppm_polarity_detection_counter = 0;
            #endif
            
            #ifdef RESTART_CHANNEL_COUNT_DETECTION_AFTER_WATCHDOG
            ppm_flag[ppm_input].ppm_channel_count_detected = false;
            ppm_var[ppm_input].ppm_channel_count = 0;
            #endif
        }
        
        if ( watchdog_triggered == true )
        {
            reset_vars( PPM_CH1 ); // reset variables and flags for PPM1 input
            reset_vars( PPM_CH2 ); // reset variables and flags for PPM2 input
            ppm2_switchover = false; // reset switchover flag
            // Reset watchdog_triggered flag false to indicate that we have received at least one ppm front
            switchover_delay_2_to_1 = 0;
            watchdog_triggered = false;
        }
        // some macros for easier reading
        
        #define CHANNEL_WIDTH   ppm_var[ppm_input].ppm_channel_width[ppm_var[ppm_input].ppm_channel]
        #define PREPULSE_WIDTH  ppm_var[ppm_input].ppm_prepulse_width
        
        // ------------------------------------------------------
		// PPM redundancy mode - Polarity detector function
		// ------------------------------------------------------
        inline void polarity_detection( uint8_t ppm_input )
		{
            if ( ppm_flag[ppm_input].ppm_polarity_detection_started == false ) // If polarity detection is not yet started
            {
                ppm_flag[ppm_input].ppm_sync_error = true;	                // Set sync error flag to signal that PPM output is not reliable
                ppm_flag[ppm_input].ppm_polarity_detection_started = true;  // Set polarity detection started flag
                ppm_var[ppm_input].ppm_polarity_detection_counter = 0;      // Reset polarity detection counter
            }
            else // Polarity detection is started
            {
                // Do we have a valid prepulse length ?
                if( ( PREPULSE_WIDTH > ppm_defn[ppm_input].ppm_prepulse_min ) && ( PREPULSE_WIDTH < ppm_defn[ppm_input].ppm_prepulse_max ) )
                { // We have a valid prepulse
                    
                    ppm_var[ppm_input].ppm_polarity_detection_counter++; // Increment polarity detection counter
                    
                    if ( ppm_var[ppm_input].ppm_polarity_detection_counter >= POLARITY_DETECTION_THRESHOLD ) // Check polarity detection counter
                    { // If counter is >= THRESHOLD
                        ppm_flag[ppm_input].ppm_polarity_detected = true;           // Set polarity detected flag
                        ppm_flag[ppm_input].ppm_polarity_detection_started = false; // Reset polarity detection started flag
                    }
                }
                else // We do not have a valid prepulse
                {
                    ppm_var[ppm_input].ppm_polarity_detection_counter = 0; // Reset polarity detection counter
                    ppm_flag[ppm_input].ppm_polarity ^= true;              // Reverse ppm polarity
                }
            }
            
 
		} // End : polarity_detection function
        
        // ------------------------------------------------------
		// PPM redundancy mode - Channel count detector function
		// ------------------------------------------------------
        inline void channel_count_detection( uint8_t ppm_input )
		{
            if ( ppm_flag[ppm_input].ppm_count_detection_started == false) // If count detection is not started
            {
                ppm_flag[ppm_input].ppm_sync_error = true;	          // Set sync error flag to signal that PPM output is not reliable
                if ( ppm_flag[ppm_input].ppm_channel_error == false ) // If we have a valid channel
                {
                    if ( ppm_var[ppm_input].ppm_channel == 1 ) // If PPM channel is 1 (we are at the pulse end of first channel)
                    {
                        ppm_var[ppm_input].ppm_channel_count_detection_counter = 0; // Reset detection counter
                        ppm_flag[ppm_input].ppm_count_detection_started = true;     // Set count detection started flag
                        ppm_var[ppm_input].ppm_channel_count = 1;                   // Set channel count to 1. This counter is used to increment and store the channel count value
                        ppm_var[ppm_input].ppm_previous_channel_count = 255;        // Set ppm_previous_channel_count to 255 for first iteration inhibit
                        
                        ppm_var[ppm_input].ppm_channel++;                           // Increment PPM channel
                        return;
                    }
                }
            } // Reset variables
            else // If detection is started
            {
                if ( ppm_flag[ppm_input].ppm_channel_error == false ) // If we have a valid channel
                {
                    ppm_var[ppm_input].ppm_channel_count = ppm_var[ppm_input].ppm_channel;              // Make ppm channel count and ppm channel identical
                    if ( ppm_var[ppm_input].ppm_channel_count <= ppm_defn[ppm_input].ppm_max_channels ) // If ppm channel count <= MAX_CHANNELS
                    {
                        ppm_var[ppm_input].ppm_channel++; // Increment ppm channel
                        return;                           // Bypass variables reset
                    }
                }
                else // We do not have a valid channel
                {
                    // Check for a valid sync symbol
                    if( ( CHANNEL_WIDTH > ppm_defn[ppm_input].ppm_sync_length_min ) && ( CHANNEL_WIDTH < ppm_defn[ppm_input].ppm_sync_length_max ) )
                    {   // We have a valid sync symbol
                        if ( ppm_var[ppm_input].ppm_channel_count >= ppm_defn[ppm_input].ppm_min_channels ) // If ppm channel count >= MIN CHANNELS
                        {
                            ppm_var[ppm_input].ppm_channel = 1; // Set ppm channel to 1
                            if ( ppm_var[ppm_input].ppm_previous_channel_count != 255 ) // We have a previous channel count value to compare with
                            {
                                if ( ppm_var[ppm_input].ppm_previous_channel_count == ppm_var[ppm_input].ppm_channel_count ) // Check channel count
                                { // Channel count did not change we could have detected it
                                    
                                    ppm_var[ppm_input].ppm_channel_count_detection_counter++; // Increment detection counter
                                    ppm_var[ppm_input].ppm_previous_channel_count = ppm_var[ppm_input].ppm_channel_count; // store ppm channel count inside ppm previous channel count
                                    
                                    if ( ppm_var[ppm_input].ppm_channel_count_detection_counter >= CHANNEL_COUNT_DETECTION_THRESHOLD ) // Check detection counter
                                    { // Detection counter reached detection level
                                        //-------------------------------------------------------------------------------------------------------
                                        ppm_flag[ppm_input].ppm_channel_count_detected = true; // Channel count is now detected on this PPM input
                                        //-------------------------------------------------------------------------------------------------------
                                        ppm_flag[ppm_input].ppm_count_detection_started = false;        // Reset ppm count detection started flag
                                        
                                        // update ppm switchover force channel for input 1
                                        if ( ppm_input == PPM_CH1 && ( PPM_SWITCHOVER_CHANNEL == AUTODETECTED_LAST_PPM1_CHANNEL ) ) // We are on input 1 and we are in autodection mode
                                        {
                                            ppm2_switchover_force_channel = ppm_var[PPM_CH1].ppm_channel_count; // assign detected channel count for input 1 to switchover channel
                                        }
                                    }
                                    return; // Channel count is not yet validated - bypass variables reset
                                }           // Else channel count did change, detection is wrong : reset detector variables and let the main the decoder resync.
                            }
                            else // We do not yet have a previous channel count value to compare with
                            {
                                ppm_var[ppm_input].ppm_previous_channel_count = ppm_var[ppm_input].ppm_channel_count; // store ppm channel count inside ppm previous channel count
                                return;                                                                               // Bypass variables reset
                            }
                        }
                    }
                }
            }
            // Reset variables to restart detection
            ppm_flag[ppm_input].ppm_count_detection_started = false;        // Reset ppm count detection started flag
            ppm_flag[ppm_input].ppm_sync = false;			                // Reset ppm sync flag
            ppm_var[ppm_input].ppm_channel = 0;                             // Reset ppm channel
           
		} // End : channel_count_detection function
        
        // ------------------------------
        // PPM validity checker function
        // ------------------------------
        inline void check_ppm_validity( uint8_t ppm_input )
        {
            // Check PPM validity
            if ( ppm_flag[ppm_input].ppm_sync_error == false && ppm_flag[ppm_input].ppm_channel_error == false && ppm_flag[ppm_input].ppm_frame_completed == true )
            { // PPM is valid
                // Set channel validity flag
                ppm_flag[ppm_input].ppm_valid = true;
                // At least one input is valid so we need to stop the ppm generator because one of them will be pushed to the PPM output
                if ( ppm_generator_active ) ppm_stop();
            }
            else if ( ppm_flag[ppm_input].ppm_sync_error == true || ppm_flag[ppm_input].ppm_channel_error == true ) // PPM is not valid
            {
                // Reset channel validity flag
                ppm_flag[ppm_input].ppm_valid = false;
            }
        } // End : Validity checker function
        
        // ------------------------------------------------------
		// PPM redundancy mode - Dead input detector
		// ------------------------------------------------------
        inline void dead_input_detector ( uint8_t ppm_input )
        {
            if ( ppm_flag[!ppm_input].ppm_valid == true ) // run dead input detector if input marked as valid
            {
                if( ppm_var[ppm_input].ppm_dead_detector_previous_channel == ppm_var[ppm_input].ppm_channel ) // if current channel did not change
                {
                    ppm_var[ppm_input].ppm_dead_counter++;                       // Increment channel dead counter
                    if( ppm_var[ppm_input].ppm_dead_counter > MISSING_CHANNELS ) // If other channel dead counter rise over detection threshold then reset ppm input and declare it invalid.
                    {
                        ppm_flag[ppm_input].ppm_valid = false;
                        ppm_flag[ppm_input].ppm_sync_error = true;
                        ppm_flag[ppm_input].ppm_sync = false;
                        ppm_flag[ppm_input].ppm_channel_error = true;
                        ppm_flag[ppm_input].ppm_frame_completed = false;
                        
                        ppm_var[ppm_input].ppm_channel = 0;
                        ppm_var[ppm_input].ppm_dead_counter = 0;
                        ppm_var[ppm_input].ppm_dead_detector_previous_channel = 255;
                    }
                }
                else // if channel index is changing then reset dead counter
                {
                    ppm_var[ppm_input].ppm_dead_counter  = 0;
                }
                ppm_var[ppm_input].ppm_dead_detector_previous_channel = ppm_var[ppm_input].ppm_channel; // Store previous channel index
            }
            check_ppm_validity ( ! ppm_input ); // Run validity checker on other input
        } // End : Dead input detector
        
        //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        // Decoder function
        // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        // The code in the decoder function is representative of a PPM negative polarity decoding :
        //
        // Negative PPM frame : ¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯¯¯¯¯|__|¯¯¯¯
        //                      --->|<---ch7--->|<---ch8--->|<-CH0--frame sync symbol-->|----ch1----|----ch2----|----ch3----|----ch4----|----ch5----|<--


        // Positive decoding is achieved through polarity inversion during the level check
        // 
        // Positive PPM frame : _|¯¯|________|¯¯|________|¯¯|________________________|¯¯|________|¯¯|________|¯¯|________|¯¯|________|¯¯|________|¯¯|____
        //                      --->|<---ch7--->|<---ch8--->|<-CH0--frame sync symbol-->|----ch1----|----ch2----|----ch3----|----ch4----|----ch5----|<--

        inline void ppm_decoder ( uint8_t ppm_input ) 
        {
            if ( !!( servo_pins & ( 1 << ppm_defn[ppm_input].ppm_pin ) ) ^ ppm_flag[ppm_input].ppm_polarity ) // Check if we've got a high level
            //----------------------------------------------------------------------------------------------------------------------------------
                      // PPM polarity is reversed here if needed to keep the same decoding code
                      // This is a raising edge for negative ppm polarity : ___|¯¯¯
                      // Logically this is a channel end, a prepulse end and a channel start
                      // Or a frame sync symbol end (channel 0), a prepulse end and a channel 1 start
            {
                PREPULSE_WIDTH = servo_time - ppm_var[ppm_input].ppm_prepulse_start; // Calculate prepulse length
                
                if ( ppm_flag[ppm_input].ppm_polarity_detected == true ) // Ckeck for polarity detected flag
                {
                    // Do we have a valid prepulse length ?
                    if( ( PREPULSE_WIDTH > ppm_defn[ppm_input].ppm_prepulse_min ) && ( PREPULSE_WIDTH < ppm_defn[ppm_input].ppm_prepulse_max ) )
                    {
                        // Calculate channel length
                        CHANNEL_WIDTH = servo_time - ppm_var[ppm_input].ppm_start[ppm_var[ppm_input].ppm_channel];
                        
                        if( ppm_flag[ppm_input].ppm_sync == true ) // If sync flag is set, we are synchronized - watch for channel length
                        {
                            // Check channel length validity
                            if( ( CHANNEL_WIDTH < ppm_defn[ppm_input].ppm_channel_value_max ) && ( CHANNEL_WIDTH > ppm_defn[ppm_input].ppm_channel_value_min ) )
                            // If we have a valid pulse length
                            {
                                ppm_flag[ppm_input].ppm_channel_error = false; // Reset channel error flag
                            }
                            else	// We do not have a valid channel length
                            {
                                ppm_flag[ppm_input].ppm_channel_error = true;	// Set channel error flag
                            }
                            // Check channel count detected
                            if( ppm_flag[ppm_input].ppm_channel_count_detected == true ) // If channel count is detected
                            {
                                // Check for last channel
                                if( ppm_var[ppm_input].ppm_channel ==  ppm_var[ppm_input].ppm_channel_count )
                                {
                                    // We are at latest PPM channel - we have a frame sync symbol start
                                    ppm_flag[ppm_input].ppm_frame_completed = true; //set frame completed flag
                                    ppm_flag[ppm_input].ppm_sync = false; 		// Reset sync flag
                                    ppm_var[ppm_input].ppm_channel = 0; 		// Reset PPM channel counter						
                                }
                                else // We have a new channel start
                                {
                                    ppm_var[ppm_input].ppm_channel++; // Increment channel counter
                                }						
                            }
                            else // channel count is not yet detected
                            {
                                channel_count_detection( ppm_input ); // Run channel detection
                            }
                        }
                        else // Sync flag is not set, we are not yet synchronized
                        {
                            // Check for frame sync symbol length
                            if( ( CHANNEL_WIDTH > ppm_defn[ppm_input].ppm_sync_length_min ) && ( CHANNEL_WIDTH < ppm_defn[ppm_input].ppm_sync_length_max ) )
                            {
                                // We have a valid sync symbol
                                ppm_flag[ppm_input].ppm_sync_error = false;   // Reset sync error flag
                                ppm_flag[ppm_input].ppm_sync = true;		  // Set sync flag
                                ppm_var[ppm_input].ppm_channel = 1;           // Set PPM channel counter to channel 1
                            }
                            else // We did not find a valid sync symbol
                            {
                                ppm_flag[ppm_input].ppm_sync_error = true; 	// Set sync error flag
                                ppm_flag[ppm_input].ppm_sync = false;		// Reset sync flag
                                ppm_var[ppm_input].ppm_channel = 0;         // Reset PPM channel counter
                                #ifdef DYNAMIC_CHANNEL_COUNT_DETECTION 
                                ppm_flag[ppm_input].ppm_channel_count_detected = false; // Reset PPM channel count detected flag
                                #endif
                            }
                            ppm_flag[ppm_input].ppm_frame_completed = false; //Reset frame completed flag
                        }
                    }
                    else
                    {
                        //We do not have a valid pre pulse
                        ppm_flag[ppm_input].ppm_sync_error = true;	 	         // Set sync error flag
                        ppm_flag[ppm_input].ppm_sync = false;			         // Reset sync flag
                        ppm_flag[ppm_input].ppm_frame_completed = false;         // Reset frame completed flag
                        ppm_var[ppm_input].ppm_channel = 0; 			         // Reset PPM channel counter
                        ppm_flag[ppm_input].ppm_count_detection_started = false; // Reset count detection started flag
                    }
                }
                else // Polarity is not yet detected
                {
                     polarity_detection(ppm_input); // Run polarity detector
                }
                ppm_var[ppm_input].ppm_start[ppm_var[ppm_input].ppm_channel] = servo_time; // Store pulse start time for PPM input
               
                dead_input_detector (!ppm_input); // run dead input detector function against other input
            }
            // --------------------------------------------------------------------------------------------------------------------
            else // We've got a low level (falling edge, channel end or frame sync symbol end)
            {
                // Store prepulse start time
                ppm_var[ppm_input].ppm_prepulse_start = servo_time;
            }
        } // End : Decoder function

        // --------------------------------------------------------------------------------------------------------------------------------
        // PPM redundancy mode - Task optimizer
        // --------------------------------------------------------------------------------------------------------------------------------

        // PPM Output is drived first for the lowest jitter
        
        #ifdef _POSITIVE_PPM_FRAME_
            #define INVERT_PPM_OUTPUT   1
        #else
            #define INVERT_PPM_OUTPUT   0
        #endif

        if ( ppm2_switchover == false ) // PPM1 is selected
        {
            if ( ppm_flag[PPM_CH1].ppm_valid == true ) // PPM1 is valid
            {
                if( servo_change & ( 1 << ppm_defn[PPM_CH1].ppm_pin ) ) // Check if we have a pin change on PPM1 input
                {
                    // Check if we've got a high level on PPM1 (reverse polarity according to detected polarity and PPM output setup)
                    if ( !!( servo_pins & ( 1 << ppm_defn[PPM_CH1].ppm_pin ) ) ^ ppm_flag[PPM_CH1].ppm_polarity ^ INVERT_PPM_OUTPUT )
                    {
                        PPM_PORT |= (1 << PPM_OUTPUT_PIN); // Set PPM output pin high
                    }
                    else // We've got a low level on PPM1
                    {
                        PPM_PORT &= ~(1 << PPM_OUTPUT_PIN); // Set PPM output pin low
                    }
                }
            }
        }
        else // PPM2 is selected
        {
            if ( ppm_flag[PPM_CH2].ppm_valid == true ) // PPM2 is valid
            {
                if( servo_change & ( 1 << ppm_defn[PPM_CH2].ppm_pin ) ) // Check if we have a pin change on PPM2 input
                {
                    // Check if we've got a high level on PPM2 (reverse polarity according to detected polarity and PPM output setup)
                    if ( !!( servo_pins & ( 1 << ppm_defn[PPM_CH2].ppm_pin ) ) ^ ppm_flag[PPM_CH2].ppm_polarity ^ INVERT_PPM_OUTPUT )
                    {
                        PPM_PORT |= (1 << PPM_OUTPUT_PIN); // Set PPM output pin high
                    }
                    else // We've got a low level on PPM1
                    {
                        PPM_PORT &= ~(1 << PPM_OUTPUT_PIN); // Set PPM output pin low
                    }
                }
            }
        }
        // We have now some time to relax, start the PPM decoder and auxiliary tasks
        
        if ( ( servo_change & ( 1 << ppm_defn[PPM_CH1].ppm_pin ) ) && ( servo_change & ( 1 << ppm_defn[PPM_CH2].ppm_pin ) ) ) // Check if we have a pin change on both PPM inputs
        {
            ppm_var[PPM_CH1].ppm_dead_counter = 0; // Reset dead input detector variables for PPM input 1
            ppm_var[PPM_CH1].ppm_dead_detector_previous_channel = 255;
            ppm_var[PPM_CH2].ppm_dead_counter = 0; // Reset dead input detector variables for PPM input 2
            ppm_var[PPM_CH2].ppm_dead_detector_previous_channel = 255;
            
            ppm_decoder ( PPM_CH1 );   // run decoder function for each PPM input
            ppm_decoder ( PPM_CH2 );
        }
        else if ( servo_change & ( 1 << ppm_defn[PPM_CH1].ppm_pin ) ) // If we have only a change on input 1
        {
            ppm_decoder ( PPM_CH1 );   // run decoder function PPM1 input
        }
        else // We have only a change on input 2
        {
            ppm_decoder ( PPM_CH2 );   // run decoder function PPM2 input
        }
        // End : Task optimizer

		// ------------------------------------------------------
		// PPM redundancy mode - Switchover control
		// ------------------------------------------------------
        
		// Check for PPM1 validity
		if ( ppm_flag[PPM_CH1].ppm_valid == true ) // If PPM1 is valid
		{
            #if ( PPM_SWITCHOVER_CHANNEL != 0 ) // Enable switchover forcing only if switchover function enabled
            
                // check for PPM2 forcing (through PPM1 force channel)
                #if ( PPM_SWITCHOVER_CHANNEL == AUTODETECTED_LAST_PPM1_CHANNEL ) // Use the correct method (auto detection or manually set switchover channel)
                    if ( ppm_var[PPM_CH1].ppm_channel_width[ppm2_switchover_force_channel] > PPM1_SWITCHOVER_FORCE_VAL_MIN ) // If forcing is valid
                #else
                    if ( ppm_var[PPM_CH1].ppm_channel_width[PPM_SWITCHOVER_CHANNEL] > PPM1_SWITCHOVER_FORCE_VAL_MIN ) // If forcing is valid
                #endif
                    {
                        if ( ppm_flag[PPM_CH2].ppm_valid == true ) // If PPM2 is valid
                        {
                            if ( ppm2_switchover == false ) // If PPM2 is not yet selected
                            {
                                ppm2_switchover = true; // Switch to PPM2
                                switchover_delay_2_to_1 = SWITCHOVER_2_to_1_DELAY; // Preset SWITCHOVER_2_to_1_DELAY delay to cancel it (fast return to PPM1 if needed)
                            } // Else if PPM2 is selected : return
                        }
                        else // PPM2 is not valid
                        {
                            if ( ppm2_switchover == true ) // If PPM2 is selected
                            {
                                if ( ppm_flag[PPM_CH1].ppm_frame_completed == true )	// Wait for last PPM1 channel before switching
                                {
                                    ppm2_switchover = false; // Switch to PPM1
                                }
                            }
                        }
                    }
                    else // Channel 2 forcing is not active
                    {
            #endif
                        if ( ppm2_switchover == true ) // If PPM1 is not selected
                        {
                            if ( ppm_flag[PPM_CH1].ppm_frame_completed == true ) // Wait for each frame completion before to count and switch
                            {
                                // wait for SWITCHOVER_2_to_1_DELAY before switching to PPM1. If PPM2 is invalid then switch immediately
                                if ( ( ++switchover_delay_2_to_1 >= SWITCHOVER_2_to_1_DELAY ) || ( ppm_flag[PPM_CH2].ppm_valid == false ) )
                                {
                                    ppm2_switchover = false; // Switch to PPM1
                                    switchover_delay_2_to_1 = 0; // reset switchover delay
                                }
                            }
                        }
                        else // If PPM1 is selected
                        {
                            switchover_delay_2_to_1 = 0; // reset switchover delay
                        }
            #if ( PPM_SWITCHOVER_CHANNEL != 0 ) // Check for switchover forcing only if switchover function enabled
                    }
            #endif
		}
		else // PPM1 is not valid
		{
			if ( ppm_flag[PPM_CH2].ppm_valid == true ) // If PPM2 is valid
			{
				if ( ppm2_switchover == false ) // If PPM2 is not yet selected
				{
					// Todo : Optional switchover delay 1 to 2 here
                    if ( ppm_flag[PPM_CH2].ppm_frame_completed == true )	// Wait for last PPM2 channel before switching
                    {
                        ppm2_switchover = true; // Switch to PPM2
                    }
				}
			}
			else // PPM2 is not valid, both inputs are invalid
			{
                #if ( _FAILSAFE_METHOD_ != ( FAILSAFE_MUTE ) ) // We can start the ppm generator if we are not in FAILSAFE_MUTE mode
                                        
                    if( !ppm_generator_active ) // Avoid restarting
                    {
                        // Load failsafe values on all channels
                        for( uint8_t i = 0; i < PPM_ARRAY_MAX; i++ )
                        {
                            ppm[ i ] = failsafe_ppm[ i ];
                        }
                        // Start the ppm generator
                        ppm_start();
                    }
                #endif
                ppm2_switchover = false; // Reset Switchover
			}
		}
      
        // ------------------------------------------
		// PPM redundancy mode - service management 
		// ------------------------------------------
        
		// Reset Watchdog Timer
		wdt_reset(); 

        inline void ppm_redundancy_led_control( void )
        {
            // --------------------------------------------------------
            // PPM redundancy mode - LED Control for status reporting
            // --------------------------------------------------------

            #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
          
            if( ppm_flag[PPM_CH1].ppm_valid == false ) // Switch on TX LED if PPM1 is not valid
            {
                PORTD &= ~( 1<< PD5 );
            }
            else // Switch off TX LED if PPM1 is valid
            {
                PORTD |= ( 1<< PD5 );
            }
          
            if( ppm_flag[PPM_CH2].ppm_valid == false ) // Switch on RX LED if PPM2 is not valid
            {
                PORTD &= ~( 1<< PD4 );
            }
            else // Switch off RX LED if PPM2 is valid
            {
                PORTD |= ( 1<< PD4 );
            }
                            
            // Green Redundancy LED control
            /*
            if( ( ppm_flag[ppm_input].ppm_valid == false) && ( ppm_flag[PPM_CH2].ppm_valid = false ) ) // If both PPM inputs are invalid
            {
                if( ++led_redundancy_delay > 63 ) // Fast Toggle green led every 64th interrupts
                {
                    PINC |= ( 1<< PC7 );
                    led_redundancy_delay = 0;
                }
            }
            
            // Slow Toggle REDUNDANCY LED if switchover to PPM2 is active (PPM1 is not valid or PPM2 is forced)
            else */ if( ppm2_switchover == true )
            {
                if( ++led_redundancy_delay > 253 ) // Slow Toggle green led every 254th interrupts
                {
                    PINC |= ( 1<< PC7 );
                    led_redundancy_delay = 0;
                }
            }
            else  // There is no switchover.
            {
                // Switch green LED ON
                PORTC &= ~( 1<< PC7 );
            }
            #endif
        }
		// run LED control code
        ppm_redundancy_led_control();
        
 	} // PPM REDUNDANCY MODE END
    
} // SERVO/PPM INPUT PIN CHANGE INTERRUPT END


	
// ------------------------------------------------------------------------------
// PPM OUTPUT - TIMER1 COMPARE INTERRUPT
// ------------------------------------------------------------------------------

// TX Led Delay
//volatile uint8_t led_delay2 = 0;

ISR( PPM_INT_VECTOR, ISR_NOBLOCK )  
{
    // ------------------------------------------------------------------------------
    // !! NESTED INTERRUPT !!
    // - ALL VARIABLES SHOULD BE GLOBAL VOLATILE 
    // - ACCESSING VARIABLES >8BIT MUST BE DONE ATOMIC USING CLI/SEI
    // ------------------------------------------------------------------------------   

    // Update timing for next compare toggle with either current ppm input value, or fail-safe value if there is a channel timeout.
        
    if( ppm_timeout[ ppm_out_channel ] > PPM_TIMEOUT_VALUE ) // We have a timeout on a channel
    {
         
        if( ppm_out_channel < 8 ) // For channels 1-4
        {
            // Channel 1-4 - Use fail-safe value
            cli();
            PPM_COMPARE += failsafe_ppm[ ppm_out_channel ]; // Copy failsafe value on the channel
            sei();

            // Channel 3
            if( ppm_out_channel == 5 ) // Check channel 3
            {
                // report this to the LED indication
                
                // Todo : this flag should not be set for LED control
                watchdog_triggered = true;
            }
        }
        else // For channels 5-8
        {
            // Channel 5-8 - Use last known value
            cli();
            PPM_COMPARE += ppm[ ppm_out_channel ];
            sei();
        }
       
    #if defined _THROTTLE_LOW_RECOVERY_POSSIBLE && defined _THROTTLE_LOW_FAILSAFE_INDICATION
        // Count the channel that we have lost
        disconnected_channels++;
    #elif defined _THROTTLE_LOW_FAILSAFE_INDICATION
        throttle_failsafe_force = true; 
    #endif

    
    #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
        if( input_mode != PPM_REDUNDANCY_MODE ) // If we are in PWM servo mode or PPM Passthrough mode
        {
            // Turn on RX LED to indicate a fail-safe condition
            PORTD &= ~(1 << PD4);
        }
    #endif    
    }
    else // We don't have a channel timeout - normal timer compare update
    {
        // Use latest ppm input value   
        cli();
        PPM_COMPARE += ppm[ ppm_out_channel ];
        sei();
        
        // Increment active channel timeout (reset to zero in input interrupt each time a valid signal is detected)
        if( servo_input_connected[ ppm_out_channel ] >= SERVO_INPUT_CONNECTED_VALUE )
        {
            ppm_timeout[ ppm_out_channel ]++;
        }
    }

    if( ++ppm_out_channel >= PPM_ARRAY_MAX ) 
    {
        ppm_out_channel = 0;

    #ifdef _THROTTLE_LOW_RECOVERY_POSSIBLE
        // Did we lose one or more active servo input channel? If so force throttle fail-safe (RTL)
        if( disconnected_channels > 0 )
        {
            throttle_failsafe_force = true;
            disconnected_channels = 0;
        }
        else
        {
            throttle_failsafe_force = false;
        }
    #endif

        #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
        // Fast toggle TX LED when PPM generator has finished a pulse train
        if( input_mode != PPM_REDUNDANCY_MODE )
        {
            PIND |= ( 1<< PD5 );
        /*
        if( ++led_delay2 > 5 ) // Fast Toggle (every 5 pulses)
        {
            // Toggle TX led
            PIND |= ( 1<< PD5 ); 
            led_delay2 = 0;
        }
        */
        }
        #endif
        
    }
}

// ------------------------------------------------------------------------------
// PPM READ - INTERRUPT SAFE PPM SERVO CHANNEL READ
// ------------------------------------------------------------------------------
uint16_t ppm_read_channel( uint8_t channel )
{
    // Limit channel to valid value
    uint8_t _channel = channel;
    if( _channel == 0 ) _channel = 1;
    if( _channel > SERVO_CHANNELS ) _channel = SERVO_CHANNELS;

    // Calculate ppm[..] position
    uint8_t ppm_index = ( _channel << 1 ) + 1;
    
    // Read ppm[..] in a non blocking interrupt safe manner
    uint16_t ppm_tmp = ppm[ ppm_index ];
    while( ppm_tmp != ppm[ ppm_index ] ) ppm_tmp = ppm[ ppm_index ];

    // Return as normal servo value
    return ppm_tmp + PPM_PRE_PULSE;    
}

// ------------------------------------------------------------------------------
// PPM ENCODER INIT
// ------------------------------------------------------------------------------
void ppm_encoder_init( void )
{
    // ATmegaXXU2 only init code
    // ------------------------------------------------------------------------------    
    #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
        // ------------------------------------------------------------------------------    
        // Reset Source checkings
        // ------------------------------------------------------------------------------
        if (MCUSR & 1)    // Power-on Reset
        {
            MCUSR=0; // Clear MCU Status register
            // custom code here
        }
        else if (MCUSR & 2)    // External Reset
        {
           MCUSR=0; // Clear MCU Status register
           // custom code here
        }
        else if (MCUSR & 4)    // Brown-Out Reset
        {
           MCUSR=0; // Clear MCU Status register
           brownout_reset=true;
        }
        else    // Watchdog Reset
        {
           MCUSR=0; // Clear MCU Status register
           // custom code here
        }

        // ------------------------------------------------------------------------------
        // APM USB connection status UART MUX selector pin
        // ------------------------------------------------------------------------------
        USB_DDR |= (1 << USB_PIN); // Set USB pin to output
        
        // ------------------------------------------------------------------------------
        // APM LEDS pins
        // ------------------------------------------------------------------------------
        
        DDRD |= (1<<PD4); // RX LED OUTPUT
        DDRD |= (1<<PD5); // TX LED OUTPUT
        DDRC |= (1<<PC7); // REDUNDANCY LED OUTPUT
                
        PORTD |= (1<<PD4); // RX LED OFF
        PORTD |= (1<<PD5); // TX LED OFF
        PORTC |= (1<<PC7); // REDUNDANCY LED OFF
    #endif
     

    // USE JUMPER TO CHECK FOR INPUT MODE (pins 2&3 or 7&8 shorted)
    // ------------------------------------------------------------------------------
    if( input_mode == JUMPER_SELECT_MODE )
    {
        // Input pins 2 and 4 status counters
        uint8_t channel_2_status = 0;
		uint8_t channel_7_status = 0;
		
		// Set channel 2 to input
        SERVO_DDR &= ~(1 << 1);
		// Enable channel 2 pullup
        SERVO_PORT |= (1 << 1);
		
		// Set channel 7 to input
        SERVO_DDR &= ~(1 << 6);
		// Enable channel 7 pullup
        SERVO_PORT |= (1 << 6);
		
		
		// Set channel 3 to output
        SERVO_DDR |= (1 << 2);
        // Set channel 3 output low
        SERVO_PORT &= ~(1 << 2);
        
        // Set channel 8 to output
        SERVO_DDR |= (1 << 7);
        // Set channel 8 output low
        SERVO_PORT &= ~(1 << 7);
		
		_delay_us (2);
        		
				
        // Increment channel_2_status if channel 2 is set low by channel 3
        if( ( SERVO_INPUT & (1 << 1) ) == 0 ) channel_2_status++;
		// Increment channel_7_status if channel 7 is set low by channel 8
        if( ( SERVO_INPUT & (1 << 6) ) == 0 ) channel_7_status++;

        // Set channel 3 output high
        SERVO_PORT |= (1 << 2);
        // Set channel 8 output high
        SERVO_PORT |= (1 << 7);
        
        _delay_us (2);
        
        // Increment channel_2_status if channel 2 is set high by channel 3
        if( ( SERVO_INPUT & (1 << 1) ) != 0 ) channel_2_status++;
		// Increment channel_7_status if channel 7 is set high by channel 8
		if( ( SERVO_INPUT & (1 << 6) ) != 0 ) channel_7_status++;

        // Set channel 3 output low
        SERVO_PORT &= ~(1 << 2);
        // Set channel 8 output low
        SERVO_PORT &= ~(1 << 7);

        _delay_us (2);

        // Increment channel_2_status if channel 2 is set low by channel 3
        if( ( SERVO_INPUT & (1 << 1) ) == 0 ) channel_2_status++;
		// Increment channel_7_status if channel 7 is set low by channel 8
        if( ( SERVO_INPUT & (1 << 6) ) == 0 ) channel_7_status++;
		
		
        // Set servo input mode based on channels status
        if( channel_2_status == 3 )
        {
            input_mode = PPM_PASSTROUGH_MODE;
        }
		else if( channel_7_status == 3 )
        {
            input_mode = PPM_REDUNDANCY_MODE;
            #if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)        
            // Turn on Redundancy led
            PORTC &= ~(1 << PC7);
            #endif
        }
        else
        {
            input_mode = SERVO_PWM_MODE;
        }

    }

    // SERVO/PPM INPUT PINS
    // ------------------------------------------------------------------------------
    // Set all servo input pins to inputs
    SERVO_DDR = 0;

    // Activate pullups on all input pins
    SERVO_PORT |= 0xFF;

	#if defined (__AVR_ATmega16U2__) || defined (__AVR_ATmega32U2__)
		// on 32U2 set PD0 to be an output, and clear the bit. This tells
		// the 2560 that USB is connected. The USB connection event fires
		// later to set the right value
		DDRD |= 1;
		if (usb_connected) {
			PORTD     &= ~1;
		} else {
			PORTD     |= 1;
		}
	#endif

    // SERVO/PPM INPUT - PIN CHANGE INTERRUPT
    // ------------------------------------------------------------------------------
    if( input_mode == SERVO_PWM_MODE )
    {
        // Set servo input interrupt pin mask to all 8 servo input channels
        SERVO_INT_MASK = 0b11111111;
    }

    if( input_mode == PPM_PASSTROUGH_MODE )
    {
        // Set servo input interrupt pin mask to servo input channel 1 (pin PB0)
        SERVO_INT_MASK = 0b00000001;
    }

    if( input_mode == PPM_REDUNDANCY_MODE )
    {
        // Set servo input interrupt pin mask to servo input channel 1 and 5 (pins PB0 and PB4)
        SERVO_INT_MASK = 0b00010001;
        
        // Start TIMER1 with 8x prescaler
        TCCR1B = ( 1 << CS11 );
    }
            
	// Enable servo input interrupt
    PCICR |= (1 << SERVO_INT_ENABLE);

    // PPM OUTPUT
    // ------------------------------------------------------------------------------
    // PPM generator (PWM output timer/counter) is started either by pin change interrupt or by watchdog interrupt
    
    // Set PPM pin to output
    PPM_DDR |= (1 << PPM_OUTPUT_PIN);
    
    // Enable output compare interrupt
    // TIMSK1 |= (1 << PPM_COMPARE_ENABLE);

    // ------------------------------------------------------------------------------
    // Enable watchdog interrupt mode
    // ------------------------------------------------------------------------------
    // Disable watchdog
    wdt_disable();
     // Reset watchdog timer
    wdt_reset();
     // Start timed watchdog setup sequence
    WDTCSR |= (1<<WDCE) | (1<<WDE );
    // Set 250 ms watchdog timeout and enable interrupt
    WDTCSR = (1<<WDIE) | (1<<WDP2);
}

#endif // _PPM_ENCODER_H_
