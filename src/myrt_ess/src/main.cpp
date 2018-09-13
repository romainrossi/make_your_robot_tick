#include <Arduino.h>
#include <SoftwareSerial.h>

// Types, enums and structures definitions
typedef enum pins_e {
  PIN_IN_START_SW = 2,
  PIN_IN_CURRENT_SENSE = PIN_A0,

  PIN_OUT_LEDSTRIP_MOSI = 11,
  PIN_OUT_LEDSTRIP_SCK = 13,
  PIN_OUT_DISPLAY_UART = 10,

  PIN_OUT_SRC_REGULAR1 = 3,
  PIN_OUT_SRC_REGULAR2 = 4,
  PIN_OUT_SRC_ACCELERATED = 5,
  PIN_OUT_SRC_FAST = 6,

  PIN_OUT_SPARE1 = 7,
  PIN_OUT_SPARE2 = 8,
  PIN_OUT_SPARE3 = 9,
  PIN_OUT_SPARE4 = PIN_A7
} pins;

typedef enum states_e {
  STATE_RESET,
  STATE_INIT,
  STATE_SETUP,
  STATE_READY,
  STATE_RUNNING,
  STATE_FINISHED
} states;

typedef struct globals_s {
  states current_state;
  uint16_t score;
  uint32_t current_match_time_ms;
} data;

// Constants
const float score_calib_multiplier = 0.0001;


const uint32_t time_slot_duration_ms = 15000; // 15 s = 15000 ms
// State (ON=1, OFF=0) of sources at each time slot, MSB is the beginning of the match
const uint32_t regular_source_activation_slots      = 0b11110000111100001111000000000000;
const uint32_t accelerated_source_activation_slots  = 0b00010001000100010001000000000000;
const uint32_t fast_source_activation_slots         = 0b00000100010001000000000000000000;


// Variables
data Data;
SoftwareSerial led_display(0, PIN_OUT_DISPLAY_UART);

// Prototypes
void update_display(void);
void read_energy(void);
void update_source_activation(void);

// Core
void setup()
{
  Data.current_state = STATE_RESET;
  Data.score = 0;

  // Init GPIO
  pinMode(PIN_IN_START_SW, INPUT_PULLUP);
  pinMode(PIN_OUT_LEDSTRIP_MOSI, OUTPUT);
  pinMode(PIN_OUT_LEDSTRIP_SCK, OUTPUT);
  pinMode(PIN_OUT_DISPLAY_UART, OUTPUT);
  pinMode(PIN_OUT_SRC_REGULAR1, OUTPUT);
  pinMode(PIN_OUT_SRC_REGULAR2, OUTPUT);
  pinMode(PIN_OUT_SRC_ACCELERATED, OUTPUT);
  pinMode(PIN_OUT_SRC_FAST, OUTPUT);
  pinMode(PIN_OUT_SPARE1, OUTPUT);
  pinMode(PIN_OUT_SPARE2, OUTPUT);
  pinMode(PIN_OUT_SPARE3, OUTPUT);
  pinMode(PIN_OUT_SPARE4, OUTPUT);

  digitalWrite(PIN_IN_START_SW, LOW);
  digitalWrite(PIN_OUT_LEDSTRIP_MOSI, LOW);
  digitalWrite(PIN_OUT_LEDSTRIP_SCK, LOW);
  digitalWrite(PIN_OUT_DISPLAY_UART, LOW);
  digitalWrite(PIN_OUT_SRC_REGULAR1, LOW);
  digitalWrite(PIN_OUT_SRC_REGULAR2, LOW);
  digitalWrite(PIN_OUT_SRC_ACCELERATED, LOW);
  digitalWrite(PIN_OUT_SRC_FAST, LOW);
  digitalWrite(PIN_OUT_SPARE1, LOW);
  digitalWrite(PIN_OUT_SPARE2, LOW);
  digitalWrite(PIN_OUT_SPARE3, LOW);
  digitalWrite(PIN_OUT_SPARE4, LOW);

  // Init ADC
  analogReference(DEFAULT);
  pinMode(PIN_IN_CURRENT_SENSE, INPUT);

  // Init Display
  led_display.begin(9600);
  led_display.stopListening();
  led_display.write(0x76);
  led_display.print("----");
  delay(2000);
  led_display.write(0x76);

  // Init Timers

  // Init Interrupts

  // Init Led strip

  // Init debug serial port
  Serial.begin(9600);

#if 0
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(sensor_pin), water_level_threshold_reached_ISR, FALLING);
#endif

}

void loop()
{
  Data.current_match_time_ms = millis();
  update_display();
  read_energy();
  update_source_activation();
  delay(100);
}

void update_display(void)
{
  static uint16_t previous_score = -1;
  if ( Data.score != previous_score ) {
    previous_score = Data.score;
    if ( previous_score <= 9999 ){
      if ( previous_score < 1000 )
        led_display.write("0");
      if ( previous_score < 100 )
        led_display.write("0");
      if ( previous_score < 10 )
        led_display.write("0");
      led_display.print(Data.score);
    } else {
      led_display.print("OUFL");
    }
  }
}

void read_energy(void) // 500 pts pour Fast; 250 pour medium; 66 pour regular
{
  static uint32_t last_read_time = 0;
  uint32_t read_time = millis();
  float adc_read = analogRead(PIN_IN_CURRENT_SENSE);

  float delta_t = read_time - last_read_time;
  float new_points = adc_read * delta_t * score_calib_multiplier;

  Data.score += (uint16_t)round(new_points);
  last_read_time = read_time;
}

void update_source_activation(void)
{
  uint8_t current_match_time_slot = Data.current_match_time_ms / time_slot_duration_ms;
  uint8_t regular1_state = (regular_source_activation_slots << current_match_time_slot) >> 31;
  uint8_t regular2_state = regular1_state;
  uint8_t accelerated_state = (accelerated_source_activation_slots << current_match_time_slot) >> 31;
  uint8_t fast_state = (fast_source_activation_slots << current_match_time_slot) >> 31;

  // Update GPIO
  digitalWrite(PIN_OUT_SRC_REGULAR1, regular1_state);
  digitalWrite(PIN_OUT_SRC_REGULAR2, regular2_state);
  digitalWrite(PIN_OUT_SRC_ACCELERATED, accelerated_state);
  digitalWrite(PIN_OUT_SRC_FAST, fast_state);

  // Print sources states on the display using decimal points
  led_display.write(0x77);
  uint8_t display_dots = (fast_state << 3) | (accelerated_state << 2) | (regular2_state << 1) | (regular1_state << 0);
  led_display.write(display_dots);
}
