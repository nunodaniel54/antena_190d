#include <avr/sleep.h>
/************************************************************************/
/*                            Pin Defenition                            */
/************************************************************************/
//////////////////////////////// Digital /////////////////////////////////
////  Input ////
#define RD_ON 2   // Imput pin 1 while radio is on 0 when off
//// Output ////
#define IN1   10   // Data to driver
#define IN2   8   // Data to driver
#define standby 9  
#define Pwma 3
//////////////////////////////// Analog //////////////////////////////////
////  Inputs ////
#define current_input A1   //
/************************************************************************/
/*                              Constants                               */
/************************************************************************/
/////////////////////////// Maquina de estados ///////////////////////////
#define max_pwm      255  // Pwm maximo
#define min_pwm      150   // PWM limite (limita a corrente)
#define pwm_delta    50
//#define pwm_period   50   // Period in milliseconds
//#define inc_time     100e3 // Time in millisecond to increment PWM
//#define inc_pwm      1    // Pwm increment
//#define slow_down_dw 38e5  // 
//#define slow_down_up 32e5  // 
//#define reverse_time 1e5  //
#define max_current  2.5    // Motor current im amps to triger a stop
#define max_time 5e3
#define max_time_retract 12e3
int state=0;
int next_state = 0;
float current = 0;
int pwm = 0;
// time_passed()
unsigned long time = 0;
unsigned long time_secundary = 0;
unsigned long time_up = 0;
unsigned long time_passed_var = 0;

void setup() {
  // Declaration of digital outputs and analog inputs
  pinMode(RD_ON, INPUT_PULLUP);
  //pinMode(amps, INPUT);
  // Declaration of digital outputs and analog outputs (pwm)
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(standby, OUTPUT);
  pinMode(Pwma, OUTPUT);
  pinMode(current_input, INPUT_PULLUP);

  Serial.begin(9600);
  //attachInterrupt(digitalPinToInterrupt(RD_ON), goToSleep, HIGH);

}

void loop() {
  //delay(500);
  time_passed_var = time_passed();
  current = read_current();
  Serial.print("current:");
  Serial.println(current);
  if(state == 0) {
    if(digitalRead(RD_ON)) {
      next_state = 1;
    }
  }
  else if(state == 1) {
    if(time_passed_var > max_time/2){
      next_state = 2;
    }
    if (current > max_current && time_passed_var > 200) {
      next_state = 3;
    }
    if (!digitalRead(RD_ON)) {
      next_state = 4;
    }
  }
  else if(state == 2) {
    if((time_passed_var > max_time || current > max_current) && digitalRead(RD_ON)){
      next_state = 3;
    }
    else if(!digitalRead(RD_ON)){
      next_state = 4;
    }
  }
  else if(state == 3){
    if(!digitalRead(RD_ON)) {
      next_state = 4;
    }
  }
  else if(state == 4) {
    if(time_passed_var > max_time/2){
      next_state = 5;
    }
    if (current > max_current) {
      next_state = 0;
    }
    if( time_passed_var > time_up){
      next_state = 0;
    }
  }
  else if(state == 5) {
    if(time_passed_var > (max_time + 150) || current > max_current){
      next_state = 0;
    }
    else if( time_passed_var > (time_up + 150)){
      next_state = 0;
    }
  }/*
  else if(state == 6){
    if(time_passed_var > max_time_retract || current > max_current){
      next_state = 0;
    }
  }*/
  


  if(next_state == 0){
    // descansa -- to do
    time_up = 0;
    pwm = 0;
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    analogWrite(Pwma,0);
    Serial.print("foi dormir");
    if(!digitalRead(RD_ON)) {
      goToSleep();
    }
  }
  else if (next_state == 1) {
    if(state != next_state){
      pwm = min_pwm;
      time_secundary = millis();
      time = time_secundary;
    } 
    else if(time_passed_sec() > 700){
      time_secundary = millis();
      pwm = pwm + pwm_delta;
      if(pwm > max_pwm){
        pwm = max_pwm;
      }
    }
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(Pwma,pwm);
  }
  else if (next_state == 2) {
    if(state != next_state){
      pwm = max_pwm;
      time_secundary = millis();
    } 
    else if(time_passed_sec() > 700){
      time_secundary = millis();
      pwm = pwm - pwm_delta;
      if(pwm < min_pwm){
        pwm = min_pwm;
      }
    }
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(Pwma,pwm);
  }
  else if (next_state == 3) {
    if(state != next_state){
      time_up = time_passed();
      /*Serial.print("time_up");
      Serial.println(time_up);*/
    } 
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    analogWrite(Pwma,0);
  }
  else if (next_state == 4) {
    
    if(state != next_state){
     if(state == 1 || state == 2){
        time_up = time_passed();
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,HIGH);
        analogWrite(Pwma,pwm);
        delay(5);
        pwm = min_pwm;
      }
      time_secundary = millis();
      time = time_secundary;

      
    } 
    else if(time_passed_sec() > 700){
      time_secundary = millis();
      pwm = pwm + pwm_delta;
      if(pwm > max_pwm){
        pwm = max_pwm;
      }
    }
    if(!(state == 1 || state == 2)){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(Pwma,pwm);}
  }
  else if (next_state == 5) {
    if(state != next_state){
      pwm = max_pwm;
      time_secundary = millis();
    } 
    else if(time_passed_sec() > 700){
      time_secundary = millis();
      pwm = pwm - pwm_delta;
      if(pwm < min_pwm){
        pwm = min_pwm;
      }
    }
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(Pwma,pwm);
  }
    else if (next_state == 6) {
    pwm = min_pwm;
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(Pwma,pwm);
  }
  if(next_state != state){
     Serial.print("state:  ");
     Serial.print(state);
     Serial.print("    next state: ");
     Serial.println(next_state);
    state = next_state;
  }

  if (!(state == 3 || state ==0)) {
    digitalWrite(standby, HIGH);
  }

}

float read_current() {
  current = 0;
  for (int x = 0; x < 5; x++){ //Get 10 samples
    current = current + analogRead(current_input);  //Add samples together
    delay (3); // let ADC settle before next sample 3ms
  }
 // Serial.println(current);
  current = current/5.0;
  return abs(2.5 - (current * (5.0 / 1024.0)) )/0.100;
}

float time_passed(){
  return ((millis() - time));
}
float time_passed_sec(){
  return ((millis() - time_secundary));
}

void goToSleep()
{ 
  attachInterrupt(digitalPinToInterrupt(RD_ON), wakeUp, HIGH);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  delay(100);
  sleep_cpu();
}
void wakeUp(){
  sleep_disable();
  //detachInterrupt(digitalPinToInterrupt(RD_ON));
  Serial.println("acordou");
}