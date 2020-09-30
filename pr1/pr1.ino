int pin = 32;
float voltage;

//Declaro variables necesarias para los timers:
volatile int interruptCounter;
int totalInterruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void setup() {
 pinMode(pin, INPUT);
 Serial.begin(115200);
 timer = timerBegin(0, 80, true);
}

void loop() {
  delay(1000);
  voltage = analogRead(pin)*3.3/4095;
  Serial.println(voltage);
}
