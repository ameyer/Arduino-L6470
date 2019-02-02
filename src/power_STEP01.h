////////////////////////////////////////////////////////////////////
//                                                                //
// ORIGINAL CODE 12 Dec 2011 Mike Hord, SparkFun Electronics      //
//                                                                //
// LIBRARY Created by Adam Meyer (@ameyer) of bildr 18 Aug 2012   //
//   Modified by Scott Lahteine (@thinkyhead) 6 Mar 2018          //
//   Chain and SPI updates by Bob Kuhn (@bob-the-kuhn) 6 Jan 2019 //
//   Released as MIT license                                      //
//                                                                //
////////////////////////////////////////////////////////////////////

/**
 * This library is aimed at the L6470 but it also can be used
 * for other L647x devices, L648x devices and the powerSTEP01.
 *
 * Page numbers are for the L6470 data sheet.
 */

#ifndef _POWER_STEP01_H_
#define _POWER_STEP01_H_

#include <Arduino.h>

#define L6470_LIBRARY_VERSION 0x000701

//#define L647X         // enable this if using the L647x family
#define POWER_STEP      // enable this if using the powerSTEP01 or L648x family rather than the L647x

#ifdef L647X
  #define L64XX L6470
#elif defined(POWER_STEP)
  #define L64XX powerSTEP01
#endif

//#define SCK    10  // Wire this to the CSN pin
//#define MOSI   11  // Wire this to the SDI pin
//#define MISO   12  // Wire this to the SDO pin
//#define SS_PIN 16  // Wire this to the CK pin
#define PIN_RESET   6  // Wire this to the STBY line
#define PIN_BUSYN   4  // Wire this to the BSYN line

#define STAT1    14  // Hooked to an LED on the test jig
#define STAT2    15  // Hooked to an LED on the test jig
#define SWITCH    8  // Hooked to the switch input and a pB on the jig

// Constant definitions for L6470 overcurrent thresholds. Write these values to
//  register dSPIN_OCD_TH to set the level at which an overcurrent even occurs.
#define OCD_TH_375mA  0x00
#define OCD_TH_750mA  0x01
#define OCD_TH_1125mA 0x02
#define OCD_TH_1500mA 0x03
#define OCD_TH_1875mA 0x04
#define OCD_TH_2250mA 0x05
#define OCD_TH_2625mA 0x06
#define OCD_TH_3000mA 0x07
#define OCD_TH_3375mA 0x08
#define OCD_TH_3750mA 0x09
#define OCD_TH_4125mA 0x0A
#define OCD_TH_4500mA 0x0B
#define OCD_TH_4875mA 0x0C
#define OCD_TH_5250mA 0x0D
#define OCD_TH_5625mA 0x0E
#define OCD_TH_6000mA 0x0F

// L6470_STEP_MODE option values.
// First comes the "microsteps per step" options...
#define STEP_MODE_STEP_SEL 0x07  // Mask for these bits only.
#define STEP_SEL_1     0x00
#define STEP_SEL_1_2   0x01
#define STEP_SEL_1_4   0x02
#define STEP_SEL_1_8   0x03
#define STEP_SEL_1_16  0x04
#define STEP_SEL_1_32  0x05
#define STEP_SEL_1_64  0x06
#define STEP_SEL_1_128 0x07

// ...next, define the SYNC_EN bit. When set, the BUSYN pin will instead
//  output a clock related to the full-step frequency as defined by the
//  SYNC_SEL bits below.
#define STEP_MODE_SYNC_EN  0x80  // Mask for this bit
#define SYNC_EN 0x80

// ...last, define the SYNC_SEL modes. The clock output is defined by
//  the full-step frequency and the value in these bits- see the datasheet
//  for a matrix describing that relationship (page 46).
#define STEP_MODE_SYNC_SEL 0x70
#define SYNC_SEL_1_2 0x00
#define SYNC_SEL_1   0x10
#define SYNC_SEL_2   0x20
#define SYNC_SEL_4   0x30
#define SYNC_SEL_8   0x40
#define SYNC_SEL_16  0x50
#define SYNC_SEL_32  0x60
#define SYNC_SEL_64  0x70

// Bit names for the L6470_ALARM_EN register.
//  Each of these bits defines one potential alarm condition.
//  When one of these conditions occurs and the respective bit in L6470_ALARM_EN is set,
//  the FLAG pin will go low. The register must be queried to determine which event
//  caused the alarm.
#define ALARM_EN_OVERCURRENT       0x01
#define ALARM_EN_THERMAL_SHUTDOWN  0x02
#define ALARM_EN_THERMAL_WARNING   0x04
#define ALARM_EN_UNDER_VOLTAGE     0x08
#define ALARM_EN_STALL_DET_A       0x10
#define ALARM_EN_STALL_DET_B       0x20
#define ALARM_EN_SW_TURN_ON        0x40
#define ALARM_EN_WRONG_NPERF_CMD   0x80

// L6470_CONFIG register renames.

// Oscillator options.
// The dSPIN needs to know what the clock frequency is because it uses that for some
//  calculations during operation.
#define CONFIG_OSC_SEL                 0x000F // Mask for this bit field.
#define CONFIG_INT_16MHZ               0x0000 // Internal 16MHz, no output
#define CONFIG_INT_16MHZ_OSCOUT_2MHZ   0x0008 // Default; internal 16MHz, 2MHz output
#define CONFIG_INT_16MHZ_OSCOUT_4MHZ   0x0009 // Internal 16MHz, 4MHz output
#define CONFIG_INT_16MHZ_OSCOUT_8MHZ   0x000A // Internal 16MHz, 8MHz output
#define CONFIG_INT_16MHZ_OSCOUT_16MHZ  0x000B // Internal 16MHz, 16MHz output
#define CONFIG_EXT_8MHZ_XTAL_DRIVE     0x0004 // External 8MHz crystal
#define CONFIG_EXT_16MHZ_XTAL_DRIVE    0x0005 // External 16MHz crystal
#define CONFIG_EXT_24MHZ_XTAL_DRIVE    0x0006 // External 24MHz crystal
#define CONFIG_EXT_32MHZ_XTAL_DRIVE    0x0007 // External 32MHz crystal
#define CONFIG_EXT_8MHZ_OSCOUT_INVERT  0x000C // External 8MHz crystal, output inverted
#define CONFIG_EXT_16MHZ_OSCOUT_INVERT 0x000D // External 16MHz crystal, output inverted
#define CONFIG_EXT_24MHZ_OSCOUT_INVERT 0x000E // External 24MHz crystal, output inverted
#define CONFIG_EXT_32MHZ_OSCOUT_INVERT 0x000F // External 32MHz crystal, output inverted

// Configure the functionality of the external switch input
#define CONFIG_SW_MODE                 0x0010 // Mask for this bit.
#define CONFIG_SW_HARD_STOP            0x0000 // Default; hard stop motor on switch.
#define CONFIG_SW_USER                 0x0010 // Tie to the GoUntil and ReleaseSW
                                              //  commands to provide jog function.
                                              //  See page 25 of datasheet.

// Configure the motor voltage compensation mode (see page 34 of datasheet)
#define CONFIG_EN_VSCOMP               0x0020  // Mask for this bit.
#define CONFIG_VS_COMP_DISABLE         0x0000  // Disable motor voltage compensation.
#define CONFIG_VS_COMP_ENABLE          0x0020  // Enable motor voltage compensation.

// Configure overcurrent detection event handling
#define CONFIG_OC_SD                   0x0080  // Mask for this bit.
#define CONFIG_OC_SD_DISABLE           0x0000  // Bridges do NOT shutdown on OC detect
#define CONFIG_OC_SD_ENABLE            0x0080  // Bridges shutdown on OC detect

// Configure the slew rate of the power bridge output
#define CONFIG_POW_SR                  0x0300  // Mask for this bit field.
#define CONFIG_SR_180V_us              0x0000  // 180V/us
#define CONFIG_SR_290V_us              0x0200  // 290V/us
#define CONFIG_SR_530V_us              0x0300  // 530V/us

#ifdef POWER_STEP
  #define CONFIG1_SR_220V_us           0x006C
  #define CONFIG1_SR_400V_us           0x0087
  #define CONFIG1_SR_520V_us           0x00A6
  #define CONFIG1_SR_980V_us           0x00E2

  #define CONFIG2_SR_220V_us           0x10
  #define CONFIG2_SR_400V_us           0x10
  #define CONFIG2_SR_520V_us           0x10
  #define CONFIG2_SR_980V_us           0x30
#endif

// Integer divisors for PWM sinewave generation
//  See page 32 of the datasheet for more information on this.
#define CONFIG_F_PWM_DEC               0x1C00      // mask for this bit field
#define CONFIG_PWM_MUL_0_625           (0x00<<10)
#define CONFIG_PWM_MUL_0_75            (0x01<<10)
#define CONFIG_PWM_MUL_0_875           (0x02<<10)
#define CONFIG_PWM_MUL_1               (0x03<<10)
#define CONFIG_PWM_MUL_1_25            (0x04<<10)
#define CONFIG_PWM_MUL_1_5             (0x05<<10)
#define CONFIG_PWM_MUL_1_75            (0x06<<10)
#define CONFIG_PWM_MUL_2               (0x07<<10)

// Multiplier for the PWM sinewave frequency
#define CONFIG_F_PWM_INT               0xE000     // mask for this bit field.
#define CONFIG_PWM_DIV_1               (0x00<<13)
#define CONFIG_PWM_DIV_2               (0x01<<13)
#define CONFIG_PWM_DIV_3               (0x02<<13)
#define CONFIG_PWM_DIV_4               (0x03<<13)
#define CONFIG_PWM_DIV_5               (0x04<<13)
#define CONFIG_PWM_DIV_6               (0x05<<13)
#define CONFIG_PWM_DIV_7               (0x06<<13)

// Status register bit renames- read-only bits conferring information about the
//  device to the user.
#define STATUS_HIZ                     0x0001 // high when bridges are in HiZ mode
#define STATUS_BUSY                    0x0002 // mirrors BUSY pin
#define STATUS_SW_F                    0x0004 // low when switch open, high when closed
#define STATUS_SW_EVN                  0x0008 // active high, set on switch falling edge,
                                              //  cleared by reading L6470_STATUS
#define STATUS_DIR                     0x0010 // Indicates current motor direction.
                                              //  High is dSPIN_FWD, Low is dSPIN_REV.

#ifdef L647X
  #define STATUS_NOTPERF_CMD           0x0080 // Last command not performed.
  #define STATUS_WRONG_CMD             0x0100 // Last command not valid.
  #define STATUS_UVLO                  0x0200 // Undervoltage lockout is active
  #define STATUS_TH_WRN                0x0400 // Thermal warning
  #define STATUS_TH_SD                 0x0800 // Thermal shutdown
  #define STATUS_OCD                   0x1000 // Overcurrent detected
  #define STATUS_STEP_LOSS_A           0x2000 // Stall detected on A bridge
  #define STATUS_STEP_LOSS_B           0x4000 // Stall detected on B bridge
  #define STATUS_SCK_MOD               0x8000 // Step clock mode is active
#else
  #define STATUS_NOTPERF_CMD           0x0080 // Last command not performed.
  #define STATUS_WRONG_CMD             0x0080 // Last command not valid.
  #define STATUS_CMD_ERR               0x0080 // Command error
  #define STATUS_UVLO                  0x0200 // Undervoltage lockout is active
  #define UVLO_ADC                     0x0400 // ADC undervoltage event
  #define STATUS_TH_WRN                0x0800 // Thermal warning
  #define STATUS_TH_SD                 0x1000 // Thermal shutdown
  #define STATUS_OCD                   0x2000 // Overcurrent detected
  #define STATUS_STEP_LOSS_A           0x4000 // Stall detected on A bridge
  #define STATUS_STEP_LOSS_B           0x8000 // Stall detected on B bridge
  #define STATUS_SCK_MOD               0x0100 // Step clock mode is active
#endif

// Status register motor status field
#define STATUS_MOT_STATUS                0x0060      // field mask
#define STATUS_MOT_STATUS_STOPPED       (0x0000<<13) // Motor stopped
#define STATUS_MOT_STATUS_ACCELERATION  (0x0001<<13) // Motor accelerating
#define STATUS_MOT_STATUS_DECELERATION  (0x0002<<13) // Motor decelerating
#define STATUS_MOT_STATUS_CONST_SPD     (0x0003<<13) // Motor at constant speed

// Register address redefines.
//  See the Param_Handler() function for more info about these.
#define L6470_ABS_POS         0x01
#define L6470_EL_POS          0x02
#define L6470_MARK            0x03
#define L6470_SPEED           0x04
#define L6470_ACC             0x05
#define L6470_DEC             0x06
#define L6470_MAX_SPEED       0x07
#define L6470_MIN_SPEED       0x08
#define L6470_FS_SPD          0x15
#define L6470_KVAL_HOLD       0x09
#define L6470_KVAL_RUN        0x0A
#define L6470_KVAL_ACC        0x0B
#define L6470_KVAL_DEC        0x0C
#define L6470_INT_SPD         0x0D
#define L6470_ST_SLP          0x0E
#define L6470_FN_SLP_ACC      0x0F
#define L6470_FN_SLP_DEC      0x10
#define L6470_K_THERM         0x11
#define L6470_ADC_OUT         0x12
#define L6470_OCD_TH          0x13
#define L6470_STALL_TH        0x14
#define L6470_STEP_MODE       0x16
#define L6470_ALARM_EN        0x17
#ifdef L647X
  #define L6470_CONFIG        0x18
  #define L6470_STATUS        0x19
#else
  #define L6470_GATECFG1      0x18
  #define L6470_GATECFG2      0x19
  #define L6470_CONFIG        0x1A
  #define L6470_STATUS        0x1B
#endif
#define PWR_TVAL_HOLD         0X09  // powerSTEP01 current mode register names
#define PWR_TVAL_RUN          0X0A
#define PWR_TVAL_ACC          0X0B
#define PWR_TVAL_DEC          0X0C
#define PWR_T_FAST            0X0E
#define PWR_TON_MIN           0X0F
#define PWR_TOFF_MIN          0X10

// dSPIN commands
#define dSPIN_NOP             0x00
#define dSPIN_SET_PARAM       0x00
#define dSPIN_GET_PARAM       0x20
#define dSPIN_RUN             0x50
#define dSPIN_STEP_CLOCK      0x58
#define dSPIN_MOVE            0x40
#define dSPIN_GOTO            0x60
#define dSPIN_GOTO_DIR        0x68
#define dSPIN_GO_UNTIL        0x82
#define dSPIN_RELEASE_SW      0x92
#define dSPIN_GO_HOME         0x70
#define dSPIN_GO_MARK         0x78
#define dSPIN_RESET_POS       0xD8
#define dSPIN_RESET_DEVICE    0xC0
#define dSPIN_SOFT_STOP       0xB0
#define dSPIN_HARD_STOP       0xB8
#define dSPIN_SOFT_HIZ        0xA0
#define dSPIN_HARD_HIZ        0xA8
#define dSPIN_GET_STATUS      0xD0

// dSPIN direction options
#define dSPIN_FWD             0x01
#define dSPIN_REV             0x00

// dSPIN action options
#define dSPIN_ACTION_RESET    0x00
#define dSPIN_ACTION_COPY     0x01

#ifdef L647X
  #define OCD_TH_MAX 15
  #define STALL_TH_MAX 127
  #define OCD_CURRENT_CONSTANT_INV 375                      //  mA per count
  #define OCD_CURRENT_CONSTANT  1/OCD_CURRENT_CONSTANT_INV  //  counts per mA
  #define STALL_CURRENT_CONSTANT_INV 31.25                      //  mA per count
  #define STALL_CURRENT_CONSTANT  1/STALL_CURRENT_CONSTANT_INV  //  counts per mA
#elif defined(POWER_STEP)
  #define OCD_TH_MAX 31
  #define STALL_TH_MAX 31
  //#define POWERSTEP_AVERAGE_RDS 0.016                                       //  Ohms - L648x use external FETs so this may be user modified
  //#define OCD_CURRENT_CONSTANT  (POWERSTEP_AVERAGE_RDS/0.03125)/1000        //  counts per mA (calc per data sheet - definitely wrong)
  //#define OCD_CURRENT_CONSTANT_INV (1000 * 0.03125)/(POWERSTEP_AVERAGE_RDS) //  mA per count  (calc per data sheet - definitely wrong)
  //#define STALL_CURRENT_CONSTANT  OCD_CURRENT_CONSTANT                      //  counts per mA (calc per data sheet - definitely wrong)
  //#define STALL_CURRENT_CONSTANT_INV OCD_CURRENT_CONSTANT_INV               //  mA per count  (calc per data sheet - definitely wrong)
  #define OCD_CURRENT_CONSTANT        0.001                                   //  counts per mA (empirically derived for powerSTEP01)
  #define OCD_CURRENT_CONSTANT_INV    1000                                    //  mA per count  (empirically derived for powerSTEP01)
  #define STALL_CURRENT_CONSTANT      0.005                                   //  counts per mA (empirically derived for powerSTEP01)
  #define STALL_CURRENT_CONSTANT_INV  200                                     //  mA per count  (empirically derived for powerSTEP01)
#endif

class L64XX {
public:

  static uint8_t chain[21]; // 0 - number of drivers in chain, 1... axis index for first device in the chain (closest to MOSI)

  L64XX(const int16_t pin_SS);

  void init();
  void spi_init();

  void setMicroSteps(int16_t microSteps);
  void setCurrent(const int16_t current); // NOT IMPLEMENTED
  void setMaxSpeed(const int16_t speed);
  void setMinSpeed(const int16_t speed);
  void setAcc(const float acceleration);
  void setDec(const float deceleration);
  void setOverCurrent(float ma_current);
  void setThresholdSpeed(const float threshold);
  void setStallCurrent(float ma_current);

  uint32_t ParamHandler(const uint8_t param, const uint32_t value);
  void SetLowSpeedOpt(boolean enable);

  void run(const uint8_t dir, const float spd);
  void Step_Clock(const uint8_t dir);

  void goHome();
  void setAsHome();

  void goMark();
  void move(const long n_step);
  void goTo(long pos);
  void goTo_DIR(const uint8_t dir, long pos);
  void goUntil(const uint8_t act, const uint8_t dir, uint32_t spd);

  boolean isBusy();

  void releaseSW(const uint8_t act, const uint8_t dir);

  float getSpeed();
  long getPos();
  void setMark();
  void setMark(long value);

  void resetPos();
  void resetDev();
  void softStop();
  void hardStop();
  void softFree();
  void free();
  int16_t getStatus();

  void SetParam(const uint8_t param, const uint32_t value);
  uint32_t GetParam(const uint8_t param);

  void set_chain_info(const uint8_t axis_index, const uint8_t position);

  void set_pins(const int16_t SCK, const int16_t MOSI, const int16_t MISO, const int16_t RESET, const int16_t BUSYN);

private:

  long convert(uint32_t val);

  uint32_t AccCalc(const float stepsPerSecPerSec);
  uint32_t DecCalc(const float stepsPerSecPerSec);
  uint32_t MaxSpdCalc(const float stepsPerSec);
  uint32_t MinSpdCalc(const float stepsPerSec);
  uint32_t FSCalc(const float stepsPerSec);
  uint32_t IntSpdCalc(const float stepsPerSec);
  uint32_t SpdCalc(const float stepsPerSec);
  uint32_t Param(uint32_t value, const uint8_t bit_len);
  uint8_t Xfer(uint8_t data);
  uint8_t Xfer(uint8_t data, int16_t ss_pin, uint8_t position);

  int16_t pin_SS    = -1,
          pin_SCK   = -1,
          pin_MOSI  = -1,
          pin_MISO  = -1,
          pin_RESET = -1,
          pin_BUSYN = -1;

  uint8_t axis_index;
  uint8_t position = 0;  // 0 - not part of a chain
};

#endif // _POWER_STEP01_H_
