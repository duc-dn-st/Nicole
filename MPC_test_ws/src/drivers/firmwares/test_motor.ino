#define PULSE_PIN 7
#define INPUT_PIN 5
#define VOLTAGE_RANGE 1023
#define FREQUENCY_RANGE 100000

const float ratio = FREQUENCY_RANGE / VOLTAGE_RANGE;
// lowest allowed frequency is 40Hz (double the frequency of the controller)
const int min_frequency = 40;
int timer = 1.0f / min_frequency * 1000000; // in microsec
const int stop_delay = timer / 2;
int pulse_analog_value;
unsigned long frequency;
int duration;
int total_time;

void setup() {
  pinMode(PULSE_PIN, OUTPUT);
  PORTD = B00000000;
  Serial.begin(9600);
}

void loop() {
  /*
  int pulse_analog_value = analogRead(5);
  unsigned int frequency;
  if (pulse_analog_value > 5){
    // NOTE: 625Hz = pi/4 [rad/s]
    frequency = map(pulse_analog_value, 0, 1023, 40, 100000);
  }
  else
  {
    frequency = 40;
  }
  int duration = (1.0f/(float)frequency) * 500000.0f;  // in millisecond
  PORTD = B10000000;
  delayMicroseconds(duration);
  PORTD = B00000000;
  delayMicroseconds(duration);
*/
  pulse_analog_value = analogRead(INPUT_PIN);
//  pulse_analog_value = 3;
  //pulse_analog_value = 1000;
  frequency = pulse_analog_value * ratio;
//  frequency = 100000;
  duration = (1.0f/(float)frequency) * 500000.0f;  // in microsecond
  total_time = duration * 2;
  if (pulse_analog_value > 2)
  {
    while(timer >= total_time)
    {
      PORTD = B10000000;
      delayMicroseconds(duration);
      PORTD = B00000000;
      delayMicroseconds(duration);
      total_time += duration * 2;
    }
  }
  else
  {
    PORTD = B10000000;
    delayMicroseconds(stop_delay);
    PORTD = B00000000;
    delayMicroseconds(stop_delay);
  }
}
