#include <AFMotor.h>
 AF_DCMotor Sol_On_Motor(4);
 AF_DCMotor Sag_On_Motor(3);
 AF_DCMotor Sol_Arka_Motor(1);
 AF_DCMotor Sag_Arka_Motor(2);
#include <Ultrasonic.h>

Ultrasonic ultrasonic_arka(40,41),ultrasonic_sol_arka(38,39),ultrasonic_sol_on(36,37),ultrasonic_on(34,35),ultrasonic_orta(45,44),ultrasonic_sag_on(48,47),ultrasonic_sag_orta(32,33),ultrasonic_sag_arka(30,31);
#define minimum_limit 15 //Arabanın genişliği (cm)
#define minimum_limit1 28 //arabanın uzunluğu (cm)
#define Sol 0 //sol yön komutu 
#define Sag 1 //sağ yön komutu
#define Ileri 2 //ileri yön komutu
#define Geri 3 //geri yön komutu
int park_durumu=0;// park durumunun kontrolu
int donus=0;// donus yaptıgının kontrolu
int donussonrasi=0;// donuş sonrası aradaki mesafenin ölçümü
int serit=0;// sağa yaslı sola yaslı kontrol
void motor_pinSetup()// motor durumu
{
 Sol_On_Motor.run(RELEASE);
  Sag_On_Motor.run(RELEASE);
  Sol_Arka_Motor.run(RELEASE);
  Sag_Arka_Motor.run(RELEASE);
}
void Robot_Dur()//robot durdurma
{
   Sol_On_Motor.run(RELEASE);
  Sag_On_Motor.run(RELEASE);
  Sol_Arka_Motor.run(RELEASE);
  Sag_Arka_Motor.run(RELEASE);
}

void Robot_Hareket(byte motor, byte spd)// robot hareketleri
{
  if (motor == Ileri)
  {
     Sol_On_Motor.setSpeed(spd);
   Sag_On_Motor.setSpeed(spd);
   Sol_Arka_Motor.setSpeed(spd);
   Sag_Arka_Motor.setSpeed(spd);
  Sol_On_Motor.run(FORWARD);
  Sag_On_Motor.run(FORWARD);
  Sol_Arka_Motor.run(FORWARD);
  Sag_Arka_Motor.run(FORWARD);

  }
    if (motor == Geri)
  {
     Sol_On_Motor.setSpeed(spd);
   Sag_On_Motor.setSpeed(spd);
   Sol_Arka_Motor.setSpeed(spd);
   Sag_Arka_Motor.setSpeed(spd);
  Sol_On_Motor.run(BACKWARD);
  Sag_On_Motor.run(BACKWARD);
  Sol_Arka_Motor.run(BACKWARD);
  Sag_Arka_Motor.run(BACKWARD);

  }
    if (motor == Sol)
  {
     Sol_On_Motor.setSpeed(spd);
   Sag_On_Motor.setSpeed(spd);
   Sol_Arka_Motor.setSpeed(spd);
   Sag_Arka_Motor.setSpeed(spd);
  Sol_On_Motor.run(BACKWARD);
  Sag_On_Motor.run(FORWARD);
  Sol_Arka_Motor.run(BACKWARD);
  Sag_Arka_Motor.run(FORWARD);

  }

    if (motor == Sag)
  {
     Sol_On_Motor.setSpeed(spd);
   Sag_On_Motor.setSpeed(spd);
   Sol_Arka_Motor.setSpeed(spd);
   Sag_Arka_Motor.setSpeed(spd);
  Sol_On_Motor.run(FORWARD);
  Sag_On_Motor.run(BACKWARD);
  Sol_Arka_Motor.run(FORWARD);
  Sag_Arka_Motor.run(BACKWARD);

  }
    
}   

int park_Durum(){// park durumu kontrolleri
  long on_Sensor  = ultrasonic_on.Ranging(CM); // sensor tanımları
    long sag_Sensor = ultrasonic_sol_on.Ranging(CM); // sensor tanımları
long sag_arka_Sensor   = ultrasonic_sol_arka.Ranging(CM); // sensor tanımları
  long sag_orta   = ultrasonic_orta.Ranging(CM); // sensor tanımları
    
   if((sag_Sensor <= minimum_limit)&&(sag_arka_Sensor <= minimum_limit)&&park_durumu==0 )// duvara yakın ve park 0 sa ilerle
     {
       Robot_Hareket(Ileri, 75);
       
         if(sag_Sensor-sag_arka_Sensor<-2 &&sag_Sensor-sag_arka_Sensor>=-20  ){// araba yamuksa düzelt
          sag_rotasyon();
 
  
  return 10;
}
else if(sag_Sensor-sag_arka_Sensor>2 &&sag_Sensor-sag_arka_Sensor<20 && sag_arka_Sensor-sag_orta<-1){// araba yamuksa düzelt
  sol_rotasyon();
 
  return 10;
} 
}
  

     if(donus==0){//araba dönmemişse
       if((sag_Sensor > minimum_limit)&&(sag_Sensor < minimum_limit1)&&(sag_arka_Sensor > minimum_limit)&&(sag_arka_Sensor < minimum_limit1))
    {
      park_durumu=1;
    }
    if( (sag_Sensor <= minimum_limit)&&(sag_arka_Sensor <= minimum_limit)&& park_durumu==1){
      return 1;//paralel park
      
    }
    if((sag_Sensor <= minimum_limit1)&&(sag_arka_Sensor >= minimum_limit1)){/**/
      park_durumu=1;
      return 2;//seri park
       
    }
    
     }
     else{/// araba dönüş yapmıssa
        if((sag_Sensor > minimum_limit+donussonrasi)&&(sag_Sensor < minimum_limit1+donussonrasi)&&(sag_arka_Sensor > minimum_limit+donussonrasi)&&(sag_arka_Sensor < minimum_limit1+donussonrasi))
    {
     
      park_durumu=1;
    }
    if( (sag_Sensor <= minimum_limit+donussonrasi)&&(sag_arka_Sensor <= minimum_limit+donussonrasi)&& park_durumu==1){
      return 4;// donusten sonraki paralel park
      
    }
    if((sag_Sensor >= minimum_limit1+donussonrasi)&&(sag_arka_Sensor >= minimum_limit1+donussonrasi)){
      park_durumu=1;
      return 5;//donusten sonraki seri park
       
    }
     }
   if(on_Sensor<=5 && park_durumu==0){// araba park yapmadan bir engele varmışsa dön
 Robot_Dur();
       delay(1000);
      
       donus=1;
     
      return 3;
    }
   

   return 6;
//}
  
}

void park_et(){
  long arka_Sensor   = ultrasonic_arka.Ranging(CM);
  int park=park_Durum();
  if(park==1){
   paralel(320);
  }
if(park==2){
 dik();
}
if(park==3){
 sag_donus();
}
if(park==4){
dik_paralel();
}
if(park==5){
  dik();
}
if(park_durumu==5){
  Robot_Dur();
}
}
void dik_paralel(){ // dik girip paralel park eden fonksiyon
    Robot_Dur();
       Robot_Hareket(Geri,110);
      delay(700);
      Robot_Hareket(Sag,150);
      delay(600);
      long arka_Sensor   = ultrasonic_arka.Ranging(CM);////Serial.println(arka_Sensor);
      while(arka_Sensor>7){
         Robot_Hareket(Geri,100);
         delay(50);
         arka_Sensor   = ultrasonic_arka.Ranging(CM);
      }
     Robot_Dur();
     delay(500);
     Robot_Hareket(Sol,150);
      delay(700);
     Robot_Dur();
     delay(500);
     long sag_Sensor = ultrasonic_sol_on.Ranging(CM); 
long sag_arka_Sensor   = ultrasonic_sol_arka.Ranging(CM); 
       while(sag_Sensor-sag_arka_Sensor<-2 &&sag_Sensor-sag_arka_Sensor>=-20 ){// rotasyon
  Robot_Hareket(Sag, 110);
  delay(100);
  sag_Sensor = ultrasonic_sol_on.Ranging(CM); 
  sag_arka_Sensor   = ultrasonic_sol_arka.Ranging(CM); 
}
while(sag_Sensor-sag_arka_Sensor>2 &&sag_Sensor-sag_arka_Sensor<20  ){// rotasyon
  Robot_Hareket(Sol, 110);
  delay(100);
  sag_Sensor = ultrasonic_sol_on.Ranging(CM); 
  sag_arka_Sensor   = ultrasonic_sol_arka.Ranging(CM); 
} 
long arka_Sensor2   = ultrasonic_arka.Ranging(CM);
 long arka_Sensor3   = ultrasonic_on.Ranging(CM);

      while(arka_Sensor2<5 || arka_Sensor3>5 ){// park ettikten sonra duvara cok yakınsa aradaki mesafeyi açar
         Robot_Hareket(Ileri,85);
         delay(50);
         arka_Sensor2   = ultrasonic_arka.Ranging(CM);
         arka_Sensor3   = ultrasonic_on.Ranging(CM);
      }
       Robot_Dur();
       delay(500);
      park_durumu=5;

}
void dik_paralel_sag(){// sağ yaslı dik paralel park
    Robot_Dur();
       Robot_Hareket(Geri,110);
      delay(850);
      Robot_Hareket(Sol,150);
      delay(600);
      long arka_Sensor   = ultrasonic_arka.Ranging(CM);////Serial.println(arka_Sensor);
      while(arka_Sensor>7){
         Robot_Hareket(Geri,100);
         delay(50);
         arka_Sensor   = ultrasonic_arka.Ranging(CM);
      }
     Robot_Dur();
     delay(500);
     Robot_Hareket(Sag,150);
      delay(640);
     Robot_Dur();
     delay(500);
     long sag_Sensor = ultrasonic_sag_on.Ranging(CM); 
long sag_arka_Sensor   = ultrasonic_sag_arka.Ranging(CM); 
       while(sag_Sensor-sag_arka_Sensor<-2 &&sag_Sensor-sag_arka_Sensor>=-20 ){
  Robot_Hareket(Sol, 110);
  delay(100);
  sag_Sensor = ultrasonic_sag_on.Ranging(CM); 
  sag_arka_Sensor   = ultrasonic_sag_arka.Ranging(CM); 
}
while(sag_Sensor-sag_arka_Sensor>2 &&sag_Sensor-sag_arka_Sensor<20  ){
  Robot_Hareket(Sag, 110);
  delay(100);
  sag_Sensor = ultrasonic_sag_on.Ranging(CM); 
  sag_arka_Sensor   = ultrasonic_sag_arka.Ranging(CM); 
} 
long arka_Sensor2   = ultrasonic_arka.Ranging(CM);
 long arka_Sensor3   = ultrasonic_on.Ranging(CM);
      while(arka_Sensor2<12 || arka_Sensor3>5 ){
         Robot_Hareket(Ileri,85);
         delay(50);
         arka_Sensor2   = ultrasonic_arka.Ranging(CM);
         arka_Sensor3   = ultrasonic_on.Ranging(CM);
      }
       Robot_Dur();
       delay(500);
      park_durumu=5;
}

void paralel(int del){
    Robot_Dur();
      Robot_Hareket(Geri,85);//75
      delay(900);
       Robot_Dur();
      Robot_Hareket(Sag,150);
      delay(320);
      Robot_Dur();
   long arka_Sensor2   = ultrasonic_arka.Ranging(CM);
      while(arka_Sensor2>13){
         Robot_Hareket(Geri,85);
         delay(50);
         arka_Sensor2   = ultrasonic_arka.Ranging(CM);
      }
      Robot_Dur();
       delay(500);
        Robot_Hareket(Sol,130);
      delay(500);
      Robot_Dur();
       delay(500);
        long sag_Sensor = ultrasonic_sol_on.Ranging(CM); 
long sag_arka_Sensor   = ultrasonic_sol_arka.Ranging(CM); 
       while(sag_Sensor-sag_arka_Sensor<-2 &&sag_Sensor-sag_arka_Sensor>=-20 ){
  Robot_Hareket(Sag, 110);
  delay(100);
  sag_Sensor = ultrasonic_sol_on.Ranging(CM); 
  sag_arka_Sensor   = ultrasonic_sol_arka.Ranging(CM); 
}
while(sag_Sensor-sag_arka_Sensor>2 &&sag_Sensor-sag_arka_Sensor<20  ){
  Robot_Hareket(Sol, 110);
  delay(100);
  sag_Sensor = ultrasonic_sol_on.Ranging(CM); 
  sag_arka_Sensor   = ultrasonic_sol_arka.Ranging(CM); 
} 
arka_Sensor2   = ultrasonic_arka.Ranging(CM);
 long arka_Sensor3   = ultrasonic_on.Ranging(CM);
      while(arka_Sensor2<10 || arka_Sensor3>5 ){
         Robot_Hareket(Ileri,85);
         delay(50);
         arka_Sensor2   = ultrasonic_arka.Ranging(CM);
         arka_Sensor3   = ultrasonic_on.Ranging(CM);
      }
       Robot_Dur();
       delay(500);
       park_durumu=5;      
}
void paralel_sag(int del){// sag yaslı paralel park
    Robot_Dur();
    delay(300);
      Robot_Hareket(Geri,105);//75
      delay(900);
       Robot_Dur();
      Robot_Hareket(Sol,150);
      delay(420);
      Robot_Dur();
   long arka_Sensor2   = ultrasonic_arka.Ranging(CM);
      while(arka_Sensor2>10){
         Robot_Hareket(Geri,85);
         delay(50);
         arka_Sensor2   = ultrasonic_arka.Ranging(CM);
      }
      Robot_Dur();
       delay(500);
        Robot_Hareket(Sag,150);
      delay(500);
      Robot_Dur();
       delay(500);
        long sag_Sensor = ultrasonic_sag_on.Ranging(CM); 
long sag_arka_Sensor   = ultrasonic_sag_arka.Ranging(CM); 
       while(sag_Sensor-sag_arka_Sensor<-2 &&sag_Sensor-sag_arka_Sensor>=-20 ){
  Robot_Hareket(Sol, 110);
  delay(100);
  sag_Sensor = ultrasonic_sag_on.Ranging(CM); 
  sag_arka_Sensor   = ultrasonic_sag_arka.Ranging(CM); 
}
while(sag_Sensor-sag_arka_Sensor>2 &&sag_Sensor-sag_arka_Sensor<20  ){
  Robot_Hareket(Sag, 110);
  delay(100);
  sag_Sensor = ultrasonic_sag_on.Ranging(CM); 
  sag_arka_Sensor   = ultrasonic_sag_arka.Ranging(CM); 
} 
arka_Sensor2   = ultrasonic_arka.Ranging(CM);
 long arka_Sensor3   = ultrasonic_on.Ranging(CM);

      while(arka_Sensor2<10 /*|| arka_Sensor3>10 */){
         Robot_Hareket(Ileri,85);
         delay(50);
         arka_Sensor2   = ultrasonic_arka.Ranging(CM);
         arka_Sensor3   = ultrasonic_on.Ranging(CM);
      }
       Robot_Dur();
       delay(500);
       park_durumu=5;
}
void dik(){// seri park
       Robot_Dur();
        delay(100);
      Robot_Hareket(Geri,75);
      delay(300);
      Robot_Dur();
      delay(600);
      Robot_Hareket(Sag,150);
      delay(600);
      long arka_Sensor   = ultrasonic_arka.Ranging(CM);////Serial.println(arka_Sensor);
      while(arka_Sensor>10){
         Robot_Hareket(Geri,100);
         delay(50);
         arka_Sensor   = ultrasonic_arka.Ranging(CM);
      }
     Robot_Dur();
     delay(500);
      park_durumu=5;
}
void sag_dik(){
  delay(350);
       Robot_Dur();
        delay(100);
      Robot_Hareket(Geri,100);
      delay(350);
      Robot_Dur();
      delay(600);    
      Robot_Hareket(Sol,150);
      delay(700);
      long arka_Sensor   = ultrasonic_arka.Ranging(CM);////Serial.println(arka_Sensor);
      while(arka_Sensor>10){
         Robot_Hareket(Geri,100);
         delay(50);
         arka_Sensor   = ultrasonic_arka.Ranging(CM);
      }
     Robot_Dur();
     delay(500);
      park_durumu=5;
}
void sag_donus(){// 
  delay(500);
  Robot_Hareket(Sag,150);
      delay(633);
   Robot_Dur();
    delay(500);
    long sag_Sensor = ultrasonic_sol_on.Ranging(CM); 
  delay(500);
    sag_Sensor = ultrasonic_sol_on.Ranging(CM); 
   donussonrasi=sag_Sensor;
   park_durumu=0;
}
void sol_donus(){
  delay(500);
  Robot_Hareket(Sol,150);
      delay(700);
   Robot_Dur();
    delay(500);
    long sag_Sensor = ultrasonic_sag_on.Ranging(CM); 
  delay(500);
    sag_Sensor = ultrasonic_sag_on.Ranging(CM); 
   donussonrasi=sag_Sensor;
   park_durumu=0;
}
void sol_rotasyon(){
  Robot_Hareket(Sol, 100);
  delay(100);
}
void sag_rotasyon(){
   Robot_Hareket(Sag, 100);
  delay(100);
}
void park_et_sag(){
  long arka_Sensor   = ultrasonic_arka.Ranging(CM);
  int park=park_Durum_sag();
  if(park==1){
   paralel_sag(320);
  }
if(park==2){
 sag_dik();
}
if(park==3){
 sol_donus();
}
if(park==4){
dik_paralel_sag();
}
if(park==5){
  sag_dik();
}
if(park_durumu==5){
  Robot_Dur();
}
}
int park_Durum_sag(){
  long on_Sensor  = ultrasonic_on.Ranging(CM); 
    long sag_Sensor = ultrasonic_sag_on.Ranging(CM); 
long sag_arka_Sensor   = ultrasonic_sag_arka.Ranging(CM); 
  long sag_orta   = ultrasonic_sag_orta.Ranging(CM); 
   if((sag_Sensor <= minimum_limit)&&(sag_arka_Sensor <= minimum_limit)&&park_durumu==0 )
     {
       Robot_Hareket(Ileri, 75);
         if(sag_Sensor-sag_arka_Sensor<-2 &&sag_Sensor-sag_arka_Sensor>=-20  ){
          sol_rotasyon();
  return 10;
}
else if(sag_Sensor-sag_arka_Sensor>2 &&sag_Sensor-sag_arka_Sensor<20 && sag_arka_Sensor-sag_orta<-1){
  sag_rotasyon();
  return 10;
} 
}
     if(donus==0){
       if((sag_Sensor > minimum_limit+3)&&(sag_Sensor < minimum_limit1+3)&&(sag_arka_Sensor > minimum_limit+3)&&(sag_arka_Sensor < minimum_limit1+3))
    {
      park_durumu=1;
    }
    if( (sag_Sensor <= minimum_limit)&&(sag_arka_Sensor <= minimum_limit)&& park_durumu==1){
      return 1;
    }
    if((sag_Sensor <= minimum_limit1)&&(sag_arka_Sensor >= minimum_limit1)){/**/
      park_durumu=1;
      return 2;
    }
     }
     else{///
        if((sag_Sensor > minimum_limit+donussonrasi)&&(sag_Sensor < minimum_limit1+donussonrasi)&&(sag_arka_Sensor > minimum_limit+donussonrasi)&&(sag_arka_Sensor < minimum_limit1+donussonrasi))
    {
      park_durumu=1;
    }
    if( (sag_Sensor <= minimum_limit+donussonrasi)&&(sag_arka_Sensor <= minimum_limit+donussonrasi)&& park_durumu==1){
      return 4;
    }
    if((sag_Sensor >= minimum_limit1+donussonrasi)&&(sag_arka_Sensor >= minimum_limit1+donussonrasi)){
      park_durumu=1;
      return 5;
    }
     }
   if(on_Sensor<=5 && park_durumu==0){
 Robot_Dur();
       delay(1000);
       donus=1;
      return 3;
    }
   return 6;
}
void setup() {
Serial.begin(9600); 
  motor_pinSetup();  
}
void loop() {
  long sol_Sensor = ultrasonic_sol_on.Ranging(CM);
long sag_Sensor = ultrasonic_sag_on.Ranging(CM); 
if(sol_Sensor>sag_Sensor && serit==0){
  Serial.println("sag ");
 delay(500);
  serit=1;
}
else if(sol_Sensor<sag_Sensor && serit==0){
 Serial.println("sol ");
 delay(500);
 serit=2;
}
 if(serit==1){
  park_et_sag();
 }
 else{
   park_et();
 }
 
}
