
int pin = 32;
int pwm = 22;
float voltage;
float duty = 0;
int x = 0;

//Declaro variables necesarias para los timers:
volatile int interruptCounter;
int totalInterruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; //variable encargada de sincronizar el loop principal y el ISR

//Declaro variables necesarias para el PWM (https://randomnerdtutorials.com/esp32-pwm-arduino-ide/)
const int freq = 5000;                //Frecuencia de 5kHz 
const int channel = 0;                //Canal del PWM. Opciones disponibles 0-15
const int resolution = 12;            //Escojo una resolución de 12 bits. Con esto la resolución es de 0 a 4095, ahorrandome la conversión de la salida del ADC.

void IRAM_ATTR onTimer() {              //Esta función incrementa el contador de interrupciones. Al aumentar el contador, el loop principal sabe que ha ocurrido una interrupción
  portENTER_CRITICAL_ISR(&timerMux);    //Aquí entro en la sección crítica de la interrupción
  interruptCounter++;                   //Se ejecuta lo que me interesa dentro del timer
  voltage = analogRead(pin)*3.3/4095;     //Obtiene el voltaje de salida. 
  portEXIT_CRITICAL_ISR(&timerMux);     //Sale de la sección crítica del timer
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
  timerAlarmWrite(timer, 1000000, true); //Puntero para señalar al timer donde debe generar la interrumpción. 
                                         //1ºArg: el objeto timer en el que estoy señalando la interrupción. 
                                         //2ºArg: Momento en el que quiero realizar la interrupción (1000000 micro s = 1s). 
                                         //3ºArg  True: El timer se vuelve a cargar consiguiendo una interrupción automatica.
  timerAlarmEnable(timer);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(pwm, channel);
  
}

void loop() {
  //delay(1000);  //Desactivado en el ejercicio 2

  interrupcion();
  
  ledcWrite(channel, duty);

  recibeComando();
  //Serial.println(voltage); 
  
}

void interrupcion(){
  
  if (interruptCounter > 0) {

    interruptCounter = 0;
    
    totalInterruptCounter++;                //Numero de interrupciones total que ocurren
    
    //Serial.print(totalInterruptCounter);
    //duty = analogRead(pin);

    if (x > 0) {                            //Solo le permito mostrar el valor del voltaje si el valor de x es mayor que cero.
      Serial.println(voltage);
    }
  }
}

void recibeComando(){
  String comando = "";
  if (Serial.available()) {
   comando = Serial.readString();
  }

  if (comando == "ADC") {
    //Serial.println("Nombre del comando: " + comando);
    //Serial.println("Comando ADC()");
    comando = "";
    //delay(2);
    //Serial.println("Nombre del comando: " + comando);
    Serial.println(voltage);
    
    
  } else if (comando.startsWith("ADC(")){
    String comando_1 = comando;
    comando.remove(-1);
    comando.remove(0, 4);
    x = comando.toInt();
    //Serial.println(x);
    if (x > 0){                                       //Si el número de segundos es mayor que cero, el timer generara una interrupción cada x * 1s. Si es menor o igual seguira como hasta ahora
      timerAlarmWrite(timer, 1000000 * x, true);
    }
    
  } else if (comando.startsWith("PWM(")) {
    comando.remove(-1);
    comando.remove(0, 4);
    int y = comando.toInt();
      if (y > 9 || y < 0){
        Serial.println("Número no valido. Por favor introduzca un número del 0 al 9");
      } else {
        duty = y * 4095 / 9;
        ledcWrite(channel, duty);
      }
  } else {
    
  }
}

