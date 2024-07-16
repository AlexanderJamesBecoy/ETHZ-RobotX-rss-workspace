#ifndef SMB_2_CONTROLLER_H__
#define SMB_2_CONTROLLER_H__
#define USE_ROS

#ifdef USE_ROS
#define USE_USBCON          // Use native USB Serial as COM port for ROS
#endif

/************* DEFINES *************/

// Timer 
#define TIMER_MILLIS    1000
bool timer_flag = false;

// ADS1015 and ADC conversion defines
#define ADC_RESISTOR_DIVIDER_RATIO    ((6.8 + 91)/6.8)

// Battery define
#define BATTERY_LOW_INDICATION      3.2 * 7 
#define POWER_PRESENT_VOLTAGE       18       // minimum voltage, to be sure that a battery is plugged in
#define BATTERY_MAXIMUM             29.4
#define BATTERY_MINUMUM             19.8
#define POWER_PRESENT(voltage)    voltage > POWER_PRESENT_VOLTAGE

// TLI4971 (current sensor connected to ADC port 4)
#define TLI4971_SENSITIVITY          0.0545 // volt / Ampere, 0.048 according to spec sheet
#define TLI4971_VREF                 1.65 // volt


// Define struct for the voltages
struct power_status_struct
{
    // Voltages from ADS1015 
    float v_acdc;
    float v_bat1;
    float v_bat2;
    float v_aout;
    // current from TLI4971
    float i_out;
    // Battery use
    bool acdc_use;
    bool bat1_use;
    bool bat2_use;
    // Battery valid
    bool acdc_val;
    bool bat1_val;
    bool bat2_val;

};

/************* FUNCTIONS DEFINES *************/

#ifdef USE_ROS
// Publish the data to ROS
void publishROS();
#else
// Print the data int the seril console
void publishSerial();
#endif

// Set timer flag
bool setTimerFlag();

// Read sensors data
void readSensorsData();

// Convert voltage to percentage
float mapVoltageToPercentage(float voltage);

//Set the leds
void setLeds();

void powerSourceUsed();


#endif