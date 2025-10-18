#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <VectorXf.h>

#include "MPU6500_Raw.h"
MPU6500 mpu;

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See datasheet for Address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET, 3400000, 400000);


#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

uint32_t interval, last_cycle;
uint32_t loop_micros;
uint32_t cycle_count;

#define SOK_BUTTON 2
#define SNEXT_BUTTON 3
#define SESC_BUTTON 4

// button states
bool SOK, SOKprev;
bool SNEXT, SNEXTprev;
bool SESC, SESCprev;
uint8_t long_press_sec = 2;


void set_interval(float new_interval)
{
  interval = new_interval * 1000000L;   // In microseconds
}  

typedef struct {
  int state, new_state;
  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

fsm_t fsmOLED,fsmSOK;

// enum of states for OLED
/*enum {
  oled_running,
  oled_menu,
  oled_menu_calibrate_IMU,
  oled_menu_dice_range,
  oled_menu_dice_counter,
  oled_menu_dice_spin_time
};*/


enum{
	running,
	menu_imu_calibration,
  calibrating,
	menu_number_of_dices,
	menu_number_of_dices_selection,
	menu_dice_range,
	menu_dice_range_selection,
	menu_spin_time,
	menu_spin_time_selection	
};

struct dice_range{
  uint8_t min_value;
  uint8_t max_value;
};

const struct dice_range dice_set[4] = {
    {1, 4},
    {1, 6},
    {1, 10},
    {1, 20}
};
uint8_t dice_range_selection = 1; // deafult option is 1-6 - index to array dice_set
uint8_t tmp_dice_range_selection; // tmp variable for selection in menu
uint8_t number_of_dices = 1; // deafult option is 1
uint8_t tmp_number_of_dices; // tmp variable for selection in menu

#define SPIN_TIME_MIN 2
#define SPIN_TIME_MAX 10
uint8_t dice_spin_time_sec = SPIN_TIME_MIN; // default 3 seconds
uint8_t tmp_dice_spin_time_sec; // default 3 seconds

// enum of states for buttons
enum {
  button_off,
  button_on
};


// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}


void setup() 
{
  // Builtin LED
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  // Our cycle time
  set_interval(20e-3); // 20 ms -> 50 Hz

  // IMU initalization
  const int I2C0_SDA = 20;
  const int I2C0_SCL = 21;
  pinMode(I2C0_SDA, INPUT_PULLUP);
  pinMode(I2C0_SCL, INPUT_PULLUP);
  
  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();
  
  delay(20);
  
  MPU6500Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
  
  while (!mpu.setup(0x68, setting)) { 
    Serial.println("MPU connection failed.");
    delay(500); // Wait to try again     
  }

  // OLED initalization
  const int I2C1_SDA = 18;
  const int I2C1_SCL = 19;
  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  while(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    delay(500);  // Wait to try again
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Configure buttons (use internal pull-ups; buttons -> GND)
  pinMode(SOK_BUTTON, INPUT_PULLUP);
  
  // Initialize stable states
  SOK = (digitalRead(SOK_BUTTON));
  SNEXT = (digitalRead(SNEXT_BUTTON) == LOW);
  SESC = (digitalRead(SESC_BUTTON) == LOW);
  SOKprev = SOK;
  SNEXTprev = SNEXT;
  SESCprev = SESC;

  // Set state machines to initial states
  set_state(fsmOLED, running);
  set_state(fsmSOK, button_off);
}

// Struct to store IMU readings
typedef struct {
  Vec3f w;
  Vec3f a;
  uint32_t cycle_time, last_cycle_time; // IMU cycle tracking
} imu_values_t;

imu_values_t imu;

void loop() 
{
  uint8_t b;
  if (Serial.available()) {  // Only do this if there is serial data to be read
  
    b = Serial.read();    
    //Serial.write(b);
  } 

  // Do this only every "interval" microseconds 
  uint32_t now = micros();
  uint32_t delta = now - last_cycle; 
  if (delta >= interval) {
    loop_micros = micros();
    last_cycle = now;
    cycle_count++;

    // Read, if ready, the MPU
    if (mpu.update()) {
      imu.last_cycle_time = imu.cycle_time;
      imu.cycle_time = micros();

      imu.w.x = mpu.getGyroX();
      imu.w.y = mpu.getGyroY();
      imu.w.z = mpu.getGyroZ();

      imu.a.x = mpu.getAccX();
      imu.a.y = mpu.getAccY();
      imu.a.z = mpu.getAccZ();
    }

    // Read the buttons
    SOKprev = SOK;
    SNEXTprev = SNEXT;
    SESCprev = SESC;

    SOK = !digitalRead(SOK_BUTTON);
    SNEXT = !digitalRead(SNEXT_BUTTON);
    SESC = !digitalRead(SESC_BUTTON);


    // SOK button handiling
    if(fsmSOK.state == button_off) {
      if(SOK == LOW && SOKprev == HIGH) { // Button pressed
        set_state(fsmSOK, button_on);
      }
    } else if(fsmSOK.state == button_on) {
      if(SOK == HIGH && SOKprev == LOW) { // Button released
        set_state(fsmSOK, button_off);
      }
    }


    // State machine for SOK button -> setting states of fsmOLED
    if(fsmSOK.state == button_on && (fsmSOK.tis-fsmSOK.tes>long_press_sec*1000)) {
      if(fsmOLED.state == running){
        set_state(fsmOLED, menu_imu_calibration);
      }
    }else if(fsmSOK.state == button_off && fsmSOK.new_state == button_on) {
      // Approval (short press) handling for all menu states:
      if (fsmOLED.state == menu_imu_calibration) {
        // approve IMU calibration (start calibration or confirm)
        set_state(fsmOLED, calibrating);
      } else if (fsmOLED.state == menu_number_of_dices) {
        // enter selection for number of dices
        tmp_number_of_dices = number_of_dices; // load current value to tmp variable
        set_state(fsmOLED, menu_number_of_dices_selection);
      } else if (fsmOLED.state == menu_number_of_dices_selection) {
        // confirm selected number_of_dices (apply selection)
        number_of_dices = tmp_number_of_dices;
        set_state(fsmOLED, menu_number_of_dices);
      } else if (fsmOLED.state == menu_dice_range) {
        // enter dice range selection
        tmp_dice_range_selection = dice_range_selection; // load current value to tmp variable
        set_state(fsmOLED, menu_dice_range_selection);
      } else if (fsmOLED.state == menu_dice_range_selection) {
        // confirm selected dice range (apply selection)
        dice_range_selection = tmp_dice_range_selection;
        set_state(fsmOLED, menu_dice_range);
      } else if (fsmOLED.state == menu_spin_time) {
        // enter spin time selection
        tmp_dice_spin_time_sec = dice_spin_time_sec; // load current value to tmp variable
        set_state(fsmOLED, menu_spin_time_selection);
      } else if (fsmOLED.state == menu_spin_time_selection) {
        // confirm selected spin time (apply selection)
        dice_spin_time_sec = tmp_dice_spin_time_sec;
        set_state(fsmOLED, menu_spin_time);
      }
    }

    // SNEXT button handling
    if(SNEXT == LOW && SNEXTprev == HIGH) {
      // if state is oled_menu change to next option
      if(fsmOLED.state == menu_imu_calibration){
        set_state(fsmOLED, menu_number_of_dices);
      }else if(fsmOLED.state == menu_number_of_dices){
        set_state(fsmOLED, menu_dice_range);
      }else if(fsmOLED.state == menu_dice_range){
        set_state(fsmOLED, menu_spin_time);
      }else if(fsmOLED.state == menu_spin_time){
        set_state(fsmOLED, menu_imu_calibration);

      }else if(fsmOLED.state == menu_number_of_dices_selection){
        tmp_number_of_dices++;
        if(tmp_number_of_dices>4) tmp_number_of_dices=1; // wrap around
      }else if(fsmOLED.state == menu_dice_range_selection){
        tmp_dice_range_selection++;
        if(tmp_dice_range_selection>3) tmp_dice_range_selection=0; // wrap around
      }else if(fsmOLED.state == menu_spin_time_selection){
        tmp_dice_spin_time_sec++;
        if(tmp_dice_spin_time_sec>SPIN_TIME_MAX) tmp_dice_spin_time_sec=SPIN_TIME_MIN; // max 10 seconds 
      }
    }
  

    // SESC button handling
    if (SESC == LOW && SESCprev == HIGH) {
      if (fsmOLED.state == menu_imu_calibration ||
          fsmOLED.state == menu_number_of_dices ||
          fsmOLED.state == menu_number_of_dices_selection ||
          fsmOLED.state == menu_dice_range ||
          fsmOLED.state == menu_dice_range_selection ||
          fsmOLED.state == menu_spin_time ||
          fsmOLED.state == menu_spin_time_selection) {
        set_state(fsmOLED, running);
      }
    }

    uint32_t cur_time = millis();   // Just one call to millis()
    fsmSOK.tis = cur_time - fsmSOK.tes;
    fsmOLED.tis = cur_time - fsmOLED.tes;





    

    // OLED output
    display.clearDisplay();

    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    
    display.printf("Wx %.2f\n", imu.w.x);
    display.printf("Wy %.2f\n", imu.w.y);
    display.printf("Wz %.2f\n", imu.w.z);

    display.setCursor(64, 0);     // Start at top-left corner
    display.printf("Ax %.2f", imu.a.x);
    display.setCursor(64, 8);
    display.printf("Ay %.2f", imu.a.y);
    display.setCursor(64, 16);
    display.printf("Az %.2f", imu.a.z);

    display.display();

    // Serial output
    Serial.printf("IMU_dt %d; ", imu.cycle_time - imu.last_cycle_time);

    Serial.printf("Wx %.2f; ", imu.w.x);
    Serial.printf("Wy %.2f; ", imu.w.y);
    Serial.printf("Wz %.2f; ", imu.w.z);

    Serial.printf("Ax %.2f; ", imu.a.x);
    Serial.printf("Ay %.2f; ", imu.a.y);
    Serial.printf("Az %.2f; ", imu.a.z);

    //Serial.print("T ");
    //Serial.print(mpu.getTemperature(), 2);

    Serial.print("loop ");
    Serial.print(micros() - now);

    Serial.println();
  }

}
