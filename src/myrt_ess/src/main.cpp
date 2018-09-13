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
  uint32_t current_time;
} data;

// Constants

// Variables
data Data;
SoftwareSerial led_display(0, PIN_OUT_DISPLAY_UART);

// Prototypes
void update_display(void);
void read_energy(void);

// Core
void setup() {
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

void loop() {
  update_display();
  read_energy();
  delay(1000);
}

void update_display(void)
{
  static uint16_t previous_score = -1;
  if ( Data.score != previous_score ) {
    previous_score = Data.score;
    led_display.write(0x76);
    if ( previous_score < 1000 )
      led_display.write("0");
    if ( previous_score < 100 )
      led_display.write("0");
    if ( previous_score < 10 )
      led_display.write("0");
    led_display.print(Data.score);
  }
}

void read_energy(void)
{
  static uint32_t last_read_time = 0;
  uint32_t read_time = millis();

  uint16_t adc_read = analogRead(PIN_IN_CURRENT_SENSE);
  Data.score = adc_read;
}
