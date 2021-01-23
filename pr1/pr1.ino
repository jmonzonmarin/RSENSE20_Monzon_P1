#include <Time.h>

//Incluyo la libreria necesaria para el I2C
#include <Wire.h>

int pin = 32;
int pwm = 17;
float voltage;
float duty = 0;
int x = 0;
int ledPIN = 0;
int contadorLED = 0;

//Declaro variables necesarias para los timers:
volatile int interruptCounter;
int totalInterruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; //variable encargada de sincronizar el loop principal y el ISR

//Declaro variables necesarias para el PWM (https://randomnerdtutorials.com/esp32-pwm-arduino-ide/)
const int freq = 5000;                //Frecuencia de 5kHz 
const int channel = 0;                //Canal del PWM. Opciones disponibles 0-15
const int resolution = 12;            //Escojo una resolución de 12 bits. Con esto la resolución es de 0 a 4095, ahorrandome la conversión de la salida del ADC.

//Comunicación I2c
int pinSCL = 5;
int pinSDA = 17;

//Defino la dirección del sensor y la resolución del acelerometro y giroscopo
#define    MPU9250_ADDRESS            0x68    //Direccion del MPU
#define    GYRO_FULL_SCALE_2000_DPS   0x18    //Escala del giroscopo de 2000º/s
#define    ACC_FULL_SCALE_2_G         0x00    //Escala del acelerometro de +/-2g
#define    A_R         ((32768.0/2.0)/9.8)    //Ratio de conversion

//Variables de aceleración y giro en los distintos ejes

int16_t aX, aY, aZ, gX, gY, gZ, aX_offset, aY_offset, aZ_offset, gX_offset, gY_offset, gZ_offset = 0.0;
int contadorADC, contadorSensor = 0; 
int periodoADC = -1;

//Funcion auxiliar lectura. Referencia: https://www.luisllamas.es/usar-arduino-con-los-imu-de-9dof-mpu-9150-y-mpu-9250/
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
   Wire.beginTransmission(Address);       //Indica la dirección del esclavo al que me quiero dirigir
   Wire.write(Register);                  //Indico el registro con el que me quiero comunicar
   Wire.endTransmission();
 
   Wire.requestFrom(Address, Nbytes);     //Leo del esclavo que he seleccionado Nbytes
   uint8_t index = 0;
   while (Wire.available())
      Data[index++] = Wire.read();        //Almaceno la lectura en un vector (Data).
}
 
 
// Funcion auxiliar de escritura. Referencia: https://www.luisllamas.es/usar-arduino-con-los-imu-de-9dof-mpu-9150-y-mpu-9250/
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
   Wire.beginTransmission(Address);       //Inicio la comunicación con el esclavo que quiero
   Wire.write(Register);                  //Accedo al registro en el que quiero escribir
   Wire.write(Data);                      //Escribo los datos pertinentes. (Normalmente la configuración del registro)
   Wire.endTransmission();
}

//Entrada en el timer
void IRAM_ATTR onTimer() {              //Esta función incrementa el contador de interrupciones. Al aumentar el contador, el loop principal sabe que ha ocurrido una interrupción
  portENTER_CRITICAL_ISR(&timerMux);    //Aquí entro en la sección crítica de la interrupción
  interruptCounter++;                   //Se ejecuta lo que me interesa dentro del timer
  voltage = analogRead(pin)*3.3/4095;   //Obtiene el voltaje de salida. 
  contadorADC++;
  contadorSensor++;
  portEXIT_CRITICAL_ISR(&timerMux);     //Sale de la sección crítica del timer
  contadorLED--;
}

void setup() {
  pinMode(pin, INPUT);
  pinMode(pwm, OUTPUT);  
  pinMode(ledPIN , OUTPUT);
  Serial.begin(115200); 
  delay(200);
  timer = timerBegin(0, 80, true); // Inicializo el timer. 
                                   // 1ºArg: El cero indica el hardware que utilizamos (hay 4). 
                                   // 2ºArg: Apartado 2: 80 es el factor por el que dividimos la frecuencia para obtener una frecuencia de 1MHz. 
                                   // 3ºArg: True indica al timer que debe contar en sentido ascendente.                                 
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100000, true); //Puntero para señalar al timer donde debe generar la interrumpción. 
                                         //1ºArg: el objeto timer en el que estoy señalando la interrupción. 
                                         //2ºArg: Momento en el que quiero realizar la interrupción (100000 micro s = 100ms). 
                                         //3ºArg  True: El timer se vuelve a cargar consiguiendo una interrupción automatica.
  timerAlarmEnable(timer);
  
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(pwm, channel);

  pinMode(ledPIN , OUTPUT);

  Wire.begin(pinSDA, pinSCL);
  // Configura la escala del acelerometro
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_2_G);        //El registro 28 (escrito en decimal) se utiliza para configurar el acelerometro
  // Configura la escala del giroscopio
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);   //El registro 27 (escrito en decimal) se utiliza para configurar el giroscopio
  
  Serial.print("Aceleracion en X");
  Serial.print(";");
  Serial.print("Aceleracion en Y");
  Serial.print(";");
  Serial.print("Aceleracion en Z");
  Serial.print(";");
  Serial.print("Giro en X");
  Serial.print(";");
  Serial.print("Giro en Y");
  Serial.print(";");
  Serial.println("Giro en Z");

  uint8_t aceleracion_off[6];                               //Creo una cadena de 6 bytes para almacenar las distintas lecturas
  I2Cread(MPU9250_ADDRESS, 0x3B, 6, aceleracion_off);
    aX_offset = (aceleracion_off[0] << 8 | aceleracion_off[1])/A_R;    //Desplazo el bit 0 a la izquierda 8 posiciones y aplico el operador OR a la posición 1
    aY_offset = (aceleracion_off[2] << 8 | aceleracion_off[3])/A_R;
    aZ_offset = (aceleracion_off[4] << 8 | aceleracion_off[5])/A_R;

  uint8_t giroscopo_off[6];
  I2Cread(MPU9250_ADDRESS, 0x43, 6, giroscopo_off);
    gX_offset = giroscopo_off[0] << 8 | giroscopo_off[1];
    gY_offset = giroscopo_off[2] << 8 | giroscopo_off[3];
    gZ_offset = giroscopo_off[4] << 8 | giroscopo_off[5];
}

void loop() {
  //delay(1000);  //Desactivado en el ejercicio 2

  interrupcion();
  ledcWrite(channel, duty);
  recibeComando();

  if (periodoADC == contadorADC){
    contadorADC = 0;
    Serial.println(voltage);
  }

  if (contadorLED > 0){
    digitalWrite(ledPIN , HIGH);
  } else {
    digitalWrite(ledPIN , LOW);
  }
  
  if (contadorSensor == 10){
    contadorLED = 2;
    contadorSensor = 0;
    Serial.print(aX,DEC);
    Serial.print(";");
    Serial.print(aY,DEC);
    Serial.print(";");
    Serial.print(aZ,DEC);
    Serial.print(";");
    Serial.print(gX,DEC);
    Serial.print(";");
    Serial.print(gY,DEC);
    Serial.print(";");
    Serial.println(gZ,DEC);
  }
}

void interrupcion(){
  
  if (interruptCounter > 0) {
    interruptCounter = 0;
    totalInterruptCounter++;                //Numero de interrupciones total que ocurren
 
    // Convertir registros acelerometro
    uint8_t aceleracion[6];                               //Creo una cadena de 6 bytes para almacenar las distintas lecturas
    I2Cread(MPU9250_ADDRESS, 0x3B, 6, aceleracion);
      aX = (aceleracion[0] << 8 | aceleracion[1])/A_R; //  - aX_offset;    //Desplazo el bit 0 a la izquierda 8 posiciones y aplico el operador OR a la posición 1
      aY = (aceleracion[2] << 8 | aceleracion[3])/A_R; //  - aY_offset;
      aZ = (aceleracion[4] << 8 | aceleracion[5])/A_R; //  - aZ_offset;
      
    uint8_t giroscopo[6];
    I2Cread(MPU9250_ADDRESS, 0x43, 6, giroscopo);
      gX =  (giroscopo[0] << 8 | giroscopo[1]); // - gX_offset;
      gY =  (giroscopo[2] << 8 | giroscopo[3]); // - gX_offset;
      gZ =  (giroscopo[4] << 8 | giroscopo[5]); // - gX_offset;
  }
}

void recibeComando(){
  String comando = "";
  if (Serial.available()) {
   comando = Serial.readString();
  }

  if (comando == "ADC") {
    comando = "";
    Serial.println(voltage);
    
  } else if (comando.startsWith("ADC(")){
    String comando_1 = comando;
    comando.remove(-1);
    comando.remove(0, 4);
    x = comando.toInt();
    if (x > 0){                                       //Si el número de segundos es mayor que cero, el timer generara una interrupción cada x * 1s. Si es menor o igual seguira como hasta ahora
      periodoADC = x;
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
  }
}
