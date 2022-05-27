//#include <Adafruit_SleepyDog.h>
#include <PS3BT.h>
#include <SPI.h>
#include <usbhub.h>
#include <avr/wdt.h>

USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);

short stik_data[6] = {0, 0, 0, 0, 0, 0};
char stik_status = 0;

uint16_t joystick_buff;
uint16_t prev_joystick_buff;
char joystick_buttons[16];
enum JOY_BTN
{
  btn_select = 0,
  btn_l3,
  btn_r3,
  btn_start,
  btn_atas,
  btn_kanan,
  btn_bawah,
  btn_kiri,
  btn_l2,
  btn_r2,
  btn_l1,
  btn_r1,
  btn_segitiga,
  btn_bulat,
  btn_x,
  btn_kotak
} joy_btn;

char analog = 1;
char ps = 0;

long int waktu_skrg;
long int waktu_dulu;


int8_t MOTORSHIELD_PWM = 9;
int8_t MOTORSHIELD_DIR_A = 7;
int8_t MOTORSHIELD_DIR_B = 8;

int8_t MOTORSHIELD_ENC_A = 2;
int8_t MOTORSHIELD_ENC_B = 3;

short int targetShield, outputShield;
short int batasSpeedShield = 100;
int encoderShield;

short int kpShield = 5;
short int kiShield = 0;
short int kdShield = 0;

short int bacaOutput[5];

//void joystick_parsing();
//void motor_shield_read_enc();
//void motor_shield_control(int16_t target_shield, int16_t enc_shield);
//void motor_shield_pwm(short int pwm_output_shield);


void setup(void)
{
  Serial.begin(38400);
  
  //JOYSTICK
  digitalWrite(A0, HIGH);
  digitalWrite(13, HIGH);
  pinMode(A0, OUTPUT);
  pinMode(13, OUTPUT);

  //MOTOR
  pinMode(MOTORSHIELD_DIR_A, OUTPUT);    //D7
  pinMode(MOTORSHIELD_DIR_B, OUTPUT);   //D8
  pinMode(MOTORSHIELD_ENC_A, INPUT);    //D5
  pinMode(MOTORSHIELD_ENC_B, INPUT);    //D6

  Serial.println("Program Start");
  delay(500);

	Usb.Init();
}

void loop(void)
{
	Usb.Task();

	if (PS3.PS3Connected){
    joystick_parsing();

    if(prev_joystick_buff != joystick_buff)
    {
      if(!joystick_buttons[btn_bulat]){
        Serial.println("bulat euy"); 
      }
      
      if(!joystick_buttons[btn_x]){
        Serial.println("eks euy"); 
      }
      
      if(!joystick_buttons[btn_kotak]){
        Serial.println("kotak euy"); 
      }

      if(!joystick_buttons[btn_segitiga]){
        Serial.println("segitiga euy"); 
      }

      if(!joystick_buttons[btn_l1]){
        Serial.println("l1 euy");
        targetShield -= 10;
      }
     
      if(!joystick_buttons[btn_r1]){
        Serial.println("r1 euy");
        targetShield += 10;
      }
      
      prev_joystick_buff = joystick_buff;
    }
  }

  motor_shield_read_enc();
  motor_shield_control(targetShield, encoderShield, outputShield);

  Serial.print("enc = ");
  Serial.print(encoderShield);
  Serial.print(" | target = ");
  Serial.print(targetShield);
  Serial.print(" | pwm = ");
  Serial.println(bacaOutput[2]);

  if(stik_status == 0 && millis() > 10000)
  {
    digitalWrite(A0, LOW);
  }

}



void joystick_parsing(){
  
    stik_status = 1;
    
    stik_data[0] = 255 - PS3.l2capinbuf[11];
    stik_data[1] = 255 - PS3.l2capinbuf[12];
    stik_data[2] = PS3.l2capinbuf[17];
    stik_data[3] = PS3.l2capinbuf[18];
    stik_data[4] = PS3.l2capinbuf[15];
    stik_data[5] = PS3.l2capinbuf[16];
    
    joystick_buff = (uint16_t)stik_data[0] + ((uint16_t) (stik_data[1]) << 8);

    for(int i=0 ; i<=15 ; i++){
      joystick_buttons[i] = (joystick_buff >> i) & 0b1;
    }

    ps = PS3.getButtonClick(PS);

    if (ps && analog == 1) { analog = 0; PS3.setLedOn(LED2); }
    else if (ps && analog == 0) { analog = 1; PS3.setLedOff(LED2); }

    if (analog == 1)
    {
      stik_data[2] = 255;
      stik_data[3] = 255;
      stik_data[4] = 255;
      stik_data[5] = 255;
    }

}

void motor_shield_read_enc(){
  static int stateA, lastStateA;
  
  stateA = digitalRead(MOTORSHIELD_ENC_A);

  if(stateA != lastStateA){
    if(digitalRead(MOTORSHIELD_ENC_B) != stateA){
      encoderShield++;
    }
    else{
      encoderShield--;
    }
  }
  lastStateA = stateA;
}

void motor_shield_control(int16_t target_shield, int16_t enc_shield, short int output){
  static float proportional, integral, derivative;
  static int   error, prev_error, sum_error;

  error = target_shield - enc_shield;
  sum_error += error;

  proportional  = kpShield * error;
  integral      = kiShield * sum_error;
  derivative    = kdShield * (error - prev_error);

  if(integral > 50) integral = 50;
  else if (integral < -50) integral = -50;

  output = (short int) (proportional + integral + derivative);

  if(output >= batasSpeedShield) 
    output = batasSpeedShield;
  else if (output <= -batasSpeedShield) 
    output = -batasSpeedShield;
  
  prev_error = error;

  bacaOutput[0] = error;
  bacaOutput[1] = output;

  motor_shield_pwm(output);
}

void motor_shield_pwm(short int pwm_output_shield){
  if (pwm_output_shield >= 0)
  {
    digitalWrite(MOTORSHIELD_DIR_A, HIGH);
    digitalWrite(MOTORSHIELD_DIR_B, LOW);
  }
  else if (pwm_output_shield < 0)
  {
    digitalWrite(MOTORSHIELD_DIR_A, LOW);
    digitalWrite(MOTORSHIELD_DIR_B, HIGH);
  }
  analogWrite(MOTORSHIELD_PWM, pwm_output_shield);
  bacaOutput[2] = pwm_output_shield;
}


//  Serial.print(" | error = ");
//  Serial.print(bacaOutput[0]);
//  Serial.print(" | output = ");
//  Serial.print(bacaOutput[1]);
//  Serial.print(" | output2 = ");
//  Serial.println(bacaOutput[2]);


//    Serial.print(joystick_buff);
//    Serial.print(" | ");        
//    Serial.print(stik_data[2]);
//    Serial.print(" | ");
//    Serial.print(stik_data[3]);
//    Serial.print(" | ");
//    Serial.print(stik_data[4]);
//    Serial.print(" | ");
//    Serial.println(stik_data[5]);

    // CHECK JOYSTICK
//    Serial.print(joystick_buttons[btn_bulat]);
//    Serial.print(" | ");        
//    Serial.print(joystick_buttons[btn_x]);
//    Serial.print(" | ");
//    Serial.print(joystick_buttons[btn_r1]);
//    Serial.print(" | ");
//    Serial.print(joystick_buttons[btn_l2]);
//    Serial.print(" | ");
//    Serial.println(joystick_buttons[btn_kotak]);
