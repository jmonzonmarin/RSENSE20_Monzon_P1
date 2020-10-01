
int pin = 32;
int pwm = 22;
float voltage;
float duty = 0;

//Declaro variables necesarias para los timers:
volatile int interruptCounter;
int totalInterruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; //variable encargada de sincronizar el loop principal y el ISR

//Declaro variables necesarias para el PWM (https://randomnerdtutorials.com/esp32-pwm-arduino-ide/)
const int freq = 5000;                //Frecuencia de 5kHz 
const int channel = 0;                //Canal del PWM. Opciones disponibles 0-15
const int resolution = 12;            //Escojo una resolución de 12 bits para el duty. Con esto la resolución es de 0 a 4095, ahorrandome la conversión de la salida del ADC.

void IRAM_ATTR onTimer() {              //Esta funión incrementa el contador de interrupciones. Al aumentar el contador, el loop principal sabe que ha ocurrido una interrupción
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  pinMode(pin, INPUT);
  pinMode(pwm, OUTPUT);
  Serial.begin(115200);
  timer = timerBegin(0, 80, true); // Inicializo el timer. 
                                   // 1ºArg: El cero indica el hardware que utilizamos (hay 4). 
                                   // 2ºArg: Apartado 2: 80 es el factor por el que dividimos la frecuencia para obtener una frecuencia de 1MHz. 
                                   // 3ºArg: True indica al timer que debe contar en sentido ascendente.                                 
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000000, true); //Puntero para señalar al timer donde debe generar la interrumpción. 
                                         //1ºArg: el objeto timer en el que estoy señalando la interrupción. 
                                         //2ºArg: Momento en el que quiero realizar la interrupción (10000000 micro s = 10s). 
                                         //3ºArg  True: El timer se vuelve a cargar consiguiendo una interrupción automatica.
  timerAlarmEnable(timer);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(pwm, channel);
  
}

void loop() {
  //delay(1000);  //Desactivado en el ejercicio 2

  interrupcion();
  
  ledcWrite(channel, duty);
  
}

void interrupcion(){
  if (interruptCounter > 0) {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
 
    totalInterruptCounter++;                //Numero de interrupciones total que ocurren
    
    voltage = analogRead(pin)*3.3/4095;   //Obtiene el voltaje de salida. 
    duty = analogRead(pin);
    
  }
}
