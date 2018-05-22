#include <SoftwareSerial.h>
#include <LCD03.h>
#include <Wire.h>
#include "Time.h"
#include "TimeLib.h"
#include <EEPROM.h>
#include <Sleep_n0m1.h>

//MIPROGRAMA
#define CMD        (byte)0x00            // MD49 command address of 0           
#define GET_SPEED1   0x21 //returns the current requested speed of motor 1
#define GET_SPEED2   0x22 //returns the current requested speed of motor 2
#define GET_ENC1     0x23 //motor 1 encoder count, 4 bytes returned high byte first (signed)
#define GET_ENC2     0x24 //motor 2 encoder count, 4 bytes returned high byte first (signed)
#define GET_ENCS     0x25 //returns 8 bytes -  encoder1 count, encoder2 count
#define GET_VOLT     0x26 //returns the input battery voltage level
#define GET_CUR1     0x27 //returns the current drawn by motor 1
#define GET_CUR2     0x28 //returns the current drawn by motor 2
#define GET_VER      0x29 //returns the MD49 software version
#define GET_ACE      0x2A //returns the current acceleration level
#define GET_MOD      0x2B //returns the currently selected mode
#define GET_VI       0x2C //returns battery volts, motor1 current and then motor2 current 
#define GET_ERR      0x2D //returns a byte within which the bits indicate errors on the MD49
#define SET_SPEED1   0x31 //set new speed1
#define SET_SPEED2   0x32 //set new speed2 or turn
#define SET_ACCEL    0x33 //set new acceleration
#define SET_MOD      0x34 //set the mode
#define RESET_ENC    0x35 //zero both of the encoder counts
#define DISABLE_REG  0x36 //power output not changed by encoder feedback
#define ENABLE_REG   0x37 //power output is regulated by encoder feedback
#define DISABLE_TOUT  0x38 //MD49 will continuously output with no regular commands
#define ENABLE_TOUT  0x39 //MD49 output will stop after 2 segundos without communication
#define DELAY_MSECS 50
#define M_PI 3.14159265358979323846
#define TICSV 980
#define RADIO 0.06
#define LCD05  0x63                   // LCD05 address


SoftwareSerial motors = SoftwareSerial(0x02, 0x03);    // Creates a serial port for the motors
volatile byte enc1a, enc1b, enc1c, enc1d;
volatile byte enc2a, enc2b, enc2c, enc2d;
volatile int32_t en1=0, enAnt1=0;
volatile int32_t en2=0, enAnt2=0;
volatile int32_t tf1=0, tb1=0, tf2=0, tb2=0, tk1=0,tk2=0;
volatile float velocidad1=0, velocidad2=0;
const int32_t minimum = -2147483648;
const int32_t maximum = 2147483647;
const int ticspv = 980; //tics por vuelta
LCD03 lcd;
int muestra = 0;                  //para contar hasta un seg.
int entero = 0;                   //para acotar valor de la velocidad
int cero = 0;                     //para corregir la hora
Sleep sleep;                      //Sleep mode
int tiempo =0;          //Variable para el sleep mode
unsigned long sleepTime = 5000;   //Sleep time (ms)
volatile unsigned int seconds = 00;  //segundos iniciales
volatile unsigned int minutes = 30;  //minutos iniciales
volatile unsigned int hours = 10;    //hora inicial
String command = "";
int horas, minutos, segundos;   //variables recogidas de la EEPROM
int eeAddress=0;                //Direccion EEPROM

void setup(){
  
  lcd.begin(16,2);
  lcd.backlight();    // Turn on the backlight
  // Write to the LCD
  lcd.print("Starting...");
  
  EEPROM.put( eeAddress, hours );  //Grabamos el valor
  eeAddress += sizeof(int);  //Obtener la siguiente posicion para escribir
  EEPROM.put( eeAddress, minutes );  //Grabamos el valor
  eeAddress += sizeof(int);  //Obtener la siguiente posicion para escribir
  EEPROM.put( eeAddress, seconds );  //Grabamos el valor
 
  if(eeAddress >= EEPROM.length()) eeAddress = 0;  //Comprobar que no hay desbordamiento
 
  EEPROM.get( 0, horas);
  EEPROM.get( 0+sizeof(int), minutos);
  EEPROM.get( 0+sizeof(int)+sizeof(int), segundos);
  
  setTime(horas,minutos,segundos,22,05,18);
  Wire.begin();
  Serial.begin(9600);
  motors.begin(9600);
  motors.write(CMD);   // command byte
  motors.write(0x34);
  //delay(2);
  motors.write(1);      //modo 1
  //delay(2);
  motors.write(CMD);      // command byte
  motors.write(0x35);
  //delay(2);
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  TCCR1B |=  (1<<(WGM12));
  //TCCR1B |=  (1<<(WGM13));
  OCR1A = 3124; // cada 50 ms
  TCCR1B |= (1<<(CS12)); //prescaler 256
  TIMSK1 |= (1<<(OCIE1A)); //por desbordamiento
  delay(1500);
  lcd.clear();
  setSpeed1(100);     //velocidad inicial 1
  setSpeed2(50);      //velocidad inicial 2
}

void loop(){ 

  leerEncoders();
  printTime();
  printVel();
  if(Serial.available()>0){
    
    command=Serial.readStringUntil('\n');
  if(command.substring(0,6)=="duerme"){
    
    String numero=command.substring(6,command.length());
    tiempo=numero.toInt();
    duerme(tiempo);
  }
  }
}
ISR (TIMER1_COMPA_vect){
  
  muestra ++;
  if(muestra >=19){
    
  update_clock();
    muestra=0;    
  }
    //leerEncoders();
}
// Función Sleep 
void duerme(unsigned long sleepTime ){

    //Serial.print("Me voy a dormir durante ");
    
    
    sleep.pwrDownMode();
    //sleepTime =  tiempo * 1000;
    sleep.sleepDelay(sleepTime);

    setSpeed1(100);     //velocidad inicial 1
    setSpeed2(50);      //velocidad inicial 2
    Serial.print("Revivo después de ");
  Serial.print(sleepTime);
  Serial.println(" milisegundos");
} 
  //Imprime en el LCD Fecha y Hora
void printTime(){
  //fecha
   time_t t = now(); // Fecha y hora actuales   
   lcd.home();
   lcd.print(day(t));
   lcd.print(+ "/") ;
   lcd.print(month(t));
   lcd.print(+ "/") ;
   lcd.print(year(t) - 2000);
   //hora
   lcd.setCursor(8, 0);
  if (horas<10){
    
    lcd.print(cero);
    }
  lcd.print(horas);
  lcd.setCursor(10,0);
  lcd.print(":");
  lcd.setCursor(11,0);
    if (minutos<10){
    
    lcd.print(cero);
    }  
  lcd.print(minutos);
  lcd.setCursor(13,0);
  lcd.print(":");
  lcd.setCursor(14,0);
    if (segundos<10){
    
    lcd.print(cero);
    }
  lcd.print(segundos);
}
//Imprime la velocidad en el LCD
void printVel(){
  
  delay(300);
    lcd.setCursor(0,1);
  
    lcd.print("v1:");
  entero = velocidad1*100;
    lcd.print(entero/100);
    lcd.setCursor(8,1);
  
    lcd.print("v2:");
  entero = velocidad2*100;
    lcd.print(entero/100);
}
  
void update_clock(){
  
    segundos++;
    if (segundos == 60){
    
        segundos = 0;
        minutos++;
    }
    if(minutos==60){
    
    minutos=0;
    horas++;
    }
    if(horas>23){
    
    horas=0;
    }
}
    

//Establecer velocidad1
void setSpeed1(int velocidad){
    
  motors.write(CMD); 
    motors.write(SET_SPEED1);
    motors.write(velocidad);
}
//Establecer velocidad2
void setSpeed2(int velocidad){
    
  motors.write(CMD); 
    motors.write(SET_SPEED2);
    motors.write(velocidad);
}
//Establecer aceleración 1
void setAcel(int acel){
  
  motors.write(CMD);  
  motors.write(SET_ACCEL);
  motors.write(acel);
}
//Establecer modo
void setMode(int mode){
  
  motors.write(CMD);  
  motors.write(SET_MOD);
  motors.write(mode);
}
//Obtener Encoder1
int32_t getEncoder1(){
  
  int32_t en1=0;
  byte enc1a, enc1b, enc1c, enc1d;
  motors.write(CMD);
  motors.write(GET_ENC1);
  delay(2);
  if(motors.available() > 3){
    
    enc1a = motors.read();
    enc1b = motors.read();
    enc1c = motors.read();
    enc1d = motors.read();
    } 
  en1=(enc1a << 24) | (enc1b <<16 ) | (enc1c <<8) | enc1d;
  return en1;
}
//Obtener Encoder 2
int32_t getEncoder2(){
  int32_t en2=0;
  byte enc2a, enc2b, enc2c, enc2d;
  motors.write(CMD);
  motors.write(GET_ENC2);
  delay(2);
  if(motors.available() > 3)
    {
      enc2a = motors.read();
      enc2b = motors.read();
      enc2c = motors.read();
      enc2d = motors.read();
    } 
  en2=(enc2a << 24) | (enc2b <<16 ) | (enc2c <<8) | enc2d;
  return en2;
}
//Obtener Velocidad 1
int getSpeed1(){
  
  int speed=0;
  motors.write(CMD);
  motors.write(GET_SPEED1);
  if(motors.available() > 0){
    
    speed = motors.read();
    }
  return speed; 
}
//Obtener Velocidad 2
int getSpeed2(){
  
  int speed=0;
  motors.write(CMD);
  motors.write(GET_SPEED2);
  delay(2);
  if(motors.available() > 0){
    
    speed = motors.read();
    }
  return speed; 
}

//Obtener voltaje
byte getVoltaje(){
    
  byte v=0;
    motors.write(CMD);
    motors.write(GET_VOLT);
    if(Serial.available()>0){
    
    v=Serial.read();
    }
    return v;
}
 
byte getCurrent1(){
   
  byte c=0;
    motors.write(CMD);
    motors.write(GET_CUR1);
    if(Serial.available()>0){
    
    c=Serial.read();
    }
    return c;
}
 
byte getCurrent2(){
    
  byte c=0;
    motors.write(CMD);
    motors.write(GET_CUR2);
    if(Serial.available()>0){
    
    c=Serial.read();
    }
    return c;
}
// Obtener versión
byte getVersion(){
    
  byte ver=0;
    motors.write(CMD);
    motors.write(GET_VER);
    if(Serial.available()>0){
    
    ver=Serial.read();
    }
    return ver;
}
 //Obtener aceleración
byte getAceleracion(){
    
  byte ac=0;
    motors.write(CMD);
    motors.write(GET_ACE);
    if(Serial.available()>0){
    
    ac=Serial.read();
    }
    return ac;
}
 //Obtener modo
byte getMode() {
    
  byte m=0;
    motors.write(CMD);
    motors.write(GET_MOD);
    if(Serial.available()>0){
    m=Serial.read();
    }
    return m;
}
 //Obtener el error
byte getError(){
    
  byte err=0;
    motors.write(CMD);
    motors.write(GET_ERR);
    if(Serial.available()>0){
    err=Serial.read();
    }
    return err;
}
 //Resetea los Encoders
void resetEncoders(){
  
    motors.write(CMD);
    motors.write(RESET_ENC);
}
 //Desabilitar regulador
void disableRegulator(){
  
    motors.write(CMD);
    motors.write(DISABLE_REG);
}
 //Habilitar Regulador
void enableRegulator(){
  
    motors.write(CMD);
    motors.write(ENABLE_REG);
}
 //desabilita el Timeout
void disableTimeout(){
  
    motors.write(CMD);
    motors.write(DISABLE_TOUT);
}
 //Habilitar Timeout
void enableTimeout(){
    motors.write(CMD);
    motors.write(ENABLE_TOUT);
}

void leerEncoders(){
   // TIMSK1 |= (0<< OCIE1A);
    motors.write(CMD);    
    motors.write(GET_ENC1);                   // Recieve encoder 1 value
    delay(2);    
    if(motors.available() > 3){
    
      enc1a = motors.read();
      enc1b = motors.read();
      enc1c = motors.read();
      enc1d = motors.read();
    } 
    motors.write(CMD);    
    motors.write(GET_ENC2);                   // Recieve encoder 2 value
    delay(2);    
    if(motors.available() > 3){
    
      enc2a = motors.read();
      enc2b = motors.read();
      enc2c = motors.read();
      enc2d = motors.read();
    } 
    /*Serial.println(enc1a);
    Serial.println(enc1b);
    Serial.println(enc1c);
    Serial.println(enc1d);*/
    en1 = (static_cast<uint32_t>(enc1a) << 24) + 
    (static_cast<uint32_t>(enc1b)<<16) + 
    (static_cast<uint32_t>(enc1c)<<8) + 
    static_cast<uint32_t>(enc1d);
    en2 = (static_cast<uint32_t>(enc2a) << 24) + 
    (static_cast<uint32_t>(enc2b)<<16) + 
    (static_cast<uint32_t>(enc2c)<<8) + 
    static_cast<uint32_t>(enc2d);
    /*
  Serial.print("enc1: ["); 
    Serial.print(enc1a,HEX); Serial.print(",");
    Serial.print(enc1b,HEX); Serial.print(",");
    Serial.print(enc1c,HEX); Serial.print(",");
    Serial.print(enc1d,HEX); 
    Serial.println("]");
    Serial.print("enc1: "); Serial.print(en1); Serial.print(" (0x"); Serial.print(en1,HEX); Serial.println(")");
    Serial.print("enc2: ["); 
    Serial.print(enc2a,HEX); Serial.print(",");
    Serial.print(enc2b,HEX); Serial.print(",");
    Serial.print(enc2c,HEX); Serial.print(",");
    Serial.print(enc2d,HEX); 
    Serial.println("]");
    Serial.print("enc2: "); Serial.print(en2); Serial.print(" (0x"); Serial.print(en2,HEX); Serial.println(")");
  */
    //TIMSK1 |= (1<< OCIE1A);
        
    //FORWARD
    if(en1>=enAnt1){
    
    tf1=en1-enAnt1;
    }else{
    
    tf1=maximum-minimum+en1-enAnt1;
    }
    if(en2>=enAnt2){
    
    tf2=en2-enAnt2;
    }else{
    
    tf2=maximum-minimum+en2-enAnt2;
    }

    //BACKWARD
    if(en1<=enAnt1){
    
    tb1=enAnt1-en1;
    }else{
    
    tb1=maximum-minimum+enAnt1-en1;
    }
    if(en2<=enAnt2){
    
    tb2=enAnt2-en2;
    }else{
    
    tb2=maximum-minimum+enAnt2-en1;
    }
    //OBTENER TICKS POR PERIODO
    if(tb1<tf1){
    
    tk1=-tb1;
    }else{
    
    tk1=tf1;
    }
    if(tb2<tf2){
     
    tk2=-tb2;
    }else{
      
    tk2=tf2;
    }   
    velocidad1=((tk1*2*M_PI/TICSV)/0.05)*RADIO;
    velocidad2=((tk2*2*M_PI/TICSV)/0.05)*RADIO;
    enAnt1=en1;
    enAnt2=en2;
   // Serial.println(velocidad1);
  }
