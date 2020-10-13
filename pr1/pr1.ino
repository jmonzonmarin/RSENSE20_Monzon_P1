#include <Time.h>

//Incluyo la libreria necesaria para el I2C
#include <Wire.h>
//#include <TimeLib.h> 

int pin = 32;
int pwm = 17;
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

//Comunicación I2c
int pinSCL = 22;
int pinSDA = 21;

//Defino la dirección del sensor y la resolución del acelerometro y giroscopo
#define    MPU9250_ADDRESS            0x68    //Direccion del MPU
#define    GYRO_FULL_SCALE_2000_DPS   0x18    //Escala del giroscopo de 2000º/s
#define    ACC_FULL_SCALE_16_G        0x18    //Escala del acelerometro de +/-16g

//Variables de aceleración y giro en los distintos ejes

int16_t aX, aY, aZ, gX, gY, gZ, aX_offset, aY_offset, aZ_offset, gX_offset, gY_offset, gZ_offset;
int contadorADC, contadorSensor = 0; 
int periodoADC = -1;
time_t t, t_cero;

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

uint8_t Buf[14];

//Entrada en el timer
void IRAM_ATTR onTimer() {              //Esta función incrementa el contador de interrupciones. Al aumentar el contador, el loop principal sabe que ha ocurrido una interrupción
  portENTER_CRITICAL_ISR(&timerMux);    //Aquí entro en la sección crítica de la interrupción
  interruptCounter++;                   //Se ejecuta lo que me interesa dentro del timer
  voltage = analogRead(pin)*3.3/4095;   //Obtiene el voltaje de salida. 
  //aX, aY, aZ, gX, gY, gZ = acellSensor();  ¿Por que no funciona?
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
 
  // Convertir registros acelerometro
  aX = (Buf[0] << 8 | Buf[1]);    //Desplazo el bit 0 a la izquierda 8 posiciones y aplico el operador OR a la posición 1
  aY = (Buf[2] << 8 | Buf[3]);
  aZ = Buf[4] << 8 | Buf[5];

  //Serial.println(aX);
  
   // Convertir registros giroscopio
  gX = (Buf[8] << 8 | Buf[9]);
  gY = (Buf[10] << 8 | Buf[11]);
  gZ = Buf[12] << 8 | Buf[13];
  contadorADC++;
  contadorSensor++;
  portEXIT_CRITICAL_ISR(&timerMux);     //Sale de la sección crítica del timer
}

void setup() {
  pinMode(pin, INPUT);
  pinMode(pwm, OUTPUT);
  
  Serial.begin(115200); 

  delay(2000);
  
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

  delay(1000);
  
  Wire.begin();
  // Configurar acelerometro
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);        //El registro 28 (escrito en decimal) se utiliza para configurar el acelerometro
  // Configurar giroscopio
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);   //El registro 27 (escrito en decimal) se utiliza para configurar el giroscopio
  
  //Serial.print("Tiempo");
  //Serial.print(";");
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
  //t_cero = now();

  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

  aX_offset = -(Buf[0] << 8 | Buf[1]);    //Desplazo el bit 0 a la izquierda 8 posiciones y aplico el operador OR a la posición 1
  aY_offset = -(Buf[2] << 8 | Buf[3]);
  aZ_offset = Buf[4] << 8 | Buf[5];

   // Convertir registros giroscopio
  gX_offset = -(Buf[8] << 8 | Buf[9]);
  gY_offset = -(Buf[10] << 8 | Buf[11]);
  gZ_offset = Buf[12] << 8 | Buf[13];
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
  
  if (contadorSensor == 10){
    contadorSensor = 0;
    //time_t t = now(); 
    //Serial.print(t - t_cero);
    //Serial.print(";");
    Serial.print((aX - aX_offset),DEC);
    Serial.print(";");
    Serial.print((aY - aY_offset),DEC);
    Serial.print(";");
    Serial.print((aZ - aZ_offset),DEC);
    Serial.print(";");
    Serial.print((gX - gX_offset),DEC);
    Serial.print(";");
    Serial.print((gY - gY_offset),DEC);
    Serial.print(";");
    Serial.println((gZ - gZ_offset),DEC);
  }
  
}

void interrupcion(){
  
  if (interruptCounter > 0) {
    interruptCounter = 0;
    totalInterruptCounter++;                //Numero de interrupciones total que ocurren
    //Serial.print(totalInterruptCounter);
    //duty = analogRead(pin);
//    if (x > 0) {                            //Solo le permito mostrar el valor del voltaje si el valor de x es mayor que cero.
//      Serial.println(voltage);
//    }
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
  } else {
    
  }
}

float acellSensor(){
   uint8_t Buf[14];
   I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
 
   // Convertir registros acelerometro
   int16_t ax = -(Buf[0] << 8 | Buf[1]);    //Desplazo el bit 0 a la izquierda 8 posiciones y aplico el operador OR a la posición 1
   int16_t ay = -(Buf[2] << 8 | Buf[3]);
   int16_t az = Buf[4] << 8 | Buf[5];
 
   // Convertir registros giroscopio
   int16_t gx = -(Buf[8] << 8 | Buf[9]);
   int16_t gy = -(Buf[10] << 8 | Buf[11]);
   int16_t gz = Buf[12] << 8 | Buf[13];
   return ax, ay, az, gx, gy, gz;
}


