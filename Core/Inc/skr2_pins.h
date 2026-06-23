// Defining pins for BigTreeTech SKR2

// Heater is the BED, and thermistor is TH0
// Thermistor must be 100k NTC
#define HEATER_POWER PD7
#define HEATER_THERM PA2

// Preheater is HE1, thermistor is TH1
// Thermistor must be 100k NTC
#define PREHEAT_POWER PB4
#define PREHEAT_THERM PA3

// Extruder is the E0 stepper
#define EXTRUDE_STEP_PIN PD15
#define EXTRUDE_DIR_PIN PD14
#define EXTRUDE_EN_PIN PC7
#define EXTRUDE_UART PC6

// Puller is the X stepper
#define PULLER_STEP_PIN PE2
#define PULLER_DIR_PIN PE1
#define PULLER_EN_PIN PE3
#define PULLER_UART PE0

// Spool is the Y stepper
#define SPOOL_STEP_PIN PD5
#define SPOOL_DIR_PIN PD4
#define SPOOL_EN_PIN PD6
#define SPOOL_UART PD3

// Fans, standard PWM
#define FAN_0 PB7
#define FAN_1 PB6
#define FAN_2 PB5

// CYD interface, broken out from 10pin EXP1 connector to 2 4pin Molex PicoBlade
#define CYD_VIN "EXP1 5v"
#define CYD_GND "EXP1 GND" // Connect to CYD P5 (or P1) GND and P3 GND
#define CYD_TX PE12 // P5 (P1) TX
#define CYD_RX PE13 // P5 (P1) RX
#define CYD_IO35 PE11 // Input only on CYD, output only on SKR2
#define CYD_IO22 PE10
#define CYD_IO21 // Backlight control, NC on SKR2

// I2C interface, from BTT I2C pins
#define I2C_SCL PB8
#define I2C_SDA PB9 