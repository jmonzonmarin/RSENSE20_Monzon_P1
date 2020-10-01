int pin = 32;
float voltage;

//Declaro variables necesarias para los timers:
volatile int interruptCounter;
int totalInterruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; //variable encargada de sincronizar el loop principal y el ISR

void IRAM_ATTR onTimer() {              //Esta funión incrementa el contador de interrupciones. Al aumentar el contador, el loop principal sabe que ha ocurrido una interrupción
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
 pinMode(pin, INPUT);
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
}

void loop() {
  
  if (interruptCounter > 0) {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
 
    totalInterruptCounter++;                //Numero de interrupciones total que ocurren
    
    voltage = analogRead(pin)*3.3/4095;   //Obtiene el voltaje de salida. 
    //duty = analogRead(pin);
    Serial.println(voltage);
  }
}
