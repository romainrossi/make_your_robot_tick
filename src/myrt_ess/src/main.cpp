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
  STATE_INIT,
  STATE_SETUP,
  STATE_READY,
  STATE_RUNNING,
  STATE_FINISHED
} states;

typedef struct globals_s {
  states current_state;
  uint32_t score;
  uint32_t match_start_time_ms;
  uint32_t current_match_time_ms;
} data;

// Constants
const uint32_t match_duration_ms = 5ul*60ul*1000ul;

const uint32_t time_slot_duration_ms = 15000; // 15 s = 15000 ms
// State (ON=1, OFF=0) of sources at each time slot, MSB is the beginning of the match
const uint32_t regular_source_activation_slots      = 0b11110000111100001111000000000000;
const uint32_t accelerated_source_activation_slots  = 0b00010001000100010001000000000000;
const uint32_t fast_source_activation_slots         = 0b00000100010001000000000000000000;


// Variables
data Data;
SoftwareSerial led_display(0, PIN_OUT_DISPLAY_UART);

// Prototypes
void display_score(bool force = false);
void display_message(const char * msg, uint16_t delay_ms=0);
void read_energy(void);
void update_source_activation(void);
uint8_t start_sw_state(void);
uint8_t read_start_sw(void);
void start_match_timer(void);
void update_match_timer(void);
bool is_match_finished(void);

void init_to_ready(void);
void init_to_setup(void);
void ready_to_running(void);
void running_to_finished(void);

// Core
void setup()
{
  Data.current_state = STATE_INIT;
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

  digitalWrite(PIN_OUT_LEDSTRIP_MOSI, LOW);
  digitalWrite(PIN_OUT_LEDSTRIP_SCK, LOW);
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

  // Init debug serial port
  Serial.begin(9600);

  // Init Display
  led_display.begin(9600);
  led_display.stopListening();
  led_display.write(0x76); // Clear
  led_display.write(0x79); // Move cursor
  led_display.write("\0"); // .. to position 0
  delay(500);
}

void loop()
{
  switch ( Data.current_state )
  {
    case STATE_INIT:
    {
      display_message("init", 2000);
      if ( start_sw_state() )
      {
        init_to_setup();
      }
      else
      {
        init_to_ready();
      }
    }
    break;

    case STATE_READY :
    {
        if ( read_start_sw() )
        {
          ready_to_running();
        }
        delay(100);
    }
    break;

    case STATE_RUNNING :
    {
        update_match_timer();
        display_score();
        read_energy();
        update_source_activation();
        if ( read_start_sw() || is_match_finished() )
        {
            running_to_finished();
        }
        delay(100);
    }
    break;

    case STATE_FINISHED:
    {
        if ( read_start_sw() )
        {
          running_to_finished();
        }
        delay(100);
    }
    break;

    default:
      Data.current_state = STATE_READY;
    break;
  }
}

// Transitions functions

/** Make the transition from STATE_INIT to STATE_READY
*/
void init_to_ready()
{
  Data.current_state = STATE_READY;
  display_message("REDY");
  Serial.print("Ready");
}

void init_to_setup(void)
{
  Data.current_state = STATE_SETUP;
  display_message("SET ");
  Serial.print("Setup");
}

/** Make the transition from STATE_READY to STATE_RUNNING
*/
void ready_to_running(void)
{
  Data.current_state = STATE_RUNNING;
  start_match_timer();
}

/** Make the transition from STATE_RUNNING to STATE_FINISHED
*/
void running_to_finished(void)
{
  Data.current_state = STATE_FINISHED;
  // switch off all sources
  digitalWrite(PIN_OUT_SRC_REGULAR1, LOW);
  digitalWrite(PIN_OUT_SRC_REGULAR2, LOW);
  digitalWrite(PIN_OUT_SRC_ACCELERATED, LOW);
  digitalWrite(PIN_OUT_SRC_FAST, LOW);
  Serial.println("Finished");
  // Switch of sources states on LED display and switch on Apostrophe
  led_display.write(0x77);
  led_display.write(32);
  // Display "End" for 3 seconds, then the final score
  display_message("END ", 3000);
  display_score(true);
}

/** Display the current score if it has changed.
  \param force Update the display even if the score hasn't changed
*/
void display_score(bool force)
{
  static uint32_t previous_score = -1;
  const uint32_t new_score = Data.score / 1000;
  if ( (new_score != previous_score) || (force) ) {
    previous_score = new_score;
    if ( previous_score <= 9999 ){
      if ( previous_score < 1000 )
        led_display.write("0");
      if ( previous_score < 100 )
        led_display.write("0");
      if ( previous_score < 10 )
        led_display.write("0");
      led_display.print(new_score);
    } else {
      led_display.print("OUFL");
    }
  }
}

void display_message(const char * msg, uint16_t delay_ms)
{
  led_display.write(0x76); // Clear
  led_display.print(msg);
  if ( delay_ms > 0 )
    delay(delay_ms);
}

/** Measure the quantity of energy delivered by the robot and update the score.
*/
void read_energy(void)
{
  static uint32_t last_read_time = 0;
  const uint32_t read_time = millis();
  const uint16_t adc_read = analogRead(PIN_IN_CURRENT_SENSE);
  const uint16_t delta_t_ms = read_time - last_read_time;
  const uint32_t new_points = adc_read * delta_t_ms;

  Data.score += new_points;
  Serial.print("NRJ\n\tadc="); Serial.println(adc_read);
  Serial.print("\tdt="); Serial.println(delta_t_ms);
  Serial.print("\tpts="); Serial.println(new_points);
  Serial.print("\tscore_raw="); Serial.println(Data.score);

  last_read_time = read_time;
}

/** Enable the power sources based on the chonogram of the Rule book.
*/
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
  static uint8_t previous_dot = 0;
  uint8_t display_dots = (fast_state << 3) | (accelerated_state << 2) | (regular2_state << 1) | (regular1_state << 0);
  if ( previous_dot != display_dots )
  {
    led_display.write(0x77);
    led_display.write(display_dots);
    previous_dot = display_dots;
  }
}

/** Read the state of the Start Switch.
  \return 1 if pressed else 0
*/
uint8_t start_sw_state(void)
{
  return (digitalRead(PIN_IN_START_SW)==LOW); //sw_state == 1 if Start_SW pressed;
}

/** Read and debounce the Start Switch state.
  \return 1 if pressed else 0
*/
uint8_t read_start_sw(void)
{
  static uint16_t sw_states_history = 0;
  const uint16_t trigger = 0b0000000011111111;
  const uint8_t sw_state = start_sw_state();
  sw_states_history = (sw_states_history << 1) | (sw_state & 1);
  if ( sw_states_history == trigger )
  {
    return 1;
  }
  return 0;
}

/** Start counting the match time.
*/
void start_match_timer(void)
{
  Data.match_start_time_ms = millis();
}

/** Update the match timer to keep accurate timing.
*/
void update_match_timer(void)
{
  Data.current_match_time_ms = millis() - Data.match_start_time_ms;
}

/** Check if the match is finished (match time expired).
*/
bool is_match_finished(void)
{
  if ( Data.current_match_time_ms >= match_duration_ms )
  {
    return true;
  }
  return false;
}
