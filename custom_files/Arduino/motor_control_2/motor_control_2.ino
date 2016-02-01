#define BOT_ADDRESS 'a'

#define LED_PIN 13

//Motor 1 pin assigment
#define MOTOR1_PIN1 4
#define MOTOR1_PIN2 5
#define MOTOR1_SPEED_PIN 9

//Motor 2 pin assigment
#define MOTOR2_PIN1 6
#define MOTOR2_PIN2 7
#define MOTOR2_SPEED_PIN 10

//Motor bounds
#define MIN_MOTOR_SPEED 20  //speed at which motor starts running
#define MAX_MOTOR_SPEED 255

#define SENSOR1_PIN A0
#define SENSOR2_PIN A1
#define SENSOR3_PIN A2
#define SENSOR4_PIN A3
#define SENSOR5_PIN A4
#define SENSOR6_PIN A5

int sensor_pins[]={SENSOR1_PIN, SENSOR2_PIN, SENSOR3_PIN, SENSOR4_PIN, SENSOR5_PIN, SENSOR6_PIN};

class Motor{
  private:
    int _motor_pin1, _motor_pin2, _motor_speed_pin;
  public:
    void attachPins(int mp1, int mp2, int msp){
      _motor_pin1=mp1;    
      _motor_pin2=mp2;
      _motor_speed_pin=msp;
      pinMode(_motor_pin1, OUTPUT);
      pinMode(_motor_pin2, OUTPUT);
    }
    void setMotorSpeed(int motor_speed){  //TODO: check if the condition has to be <= or >=
      if(motor_speed<=0){
        digitalWrite(_motor_pin1, HIGH);
        digitalWrite(_motor_pin2, LOW);
      }
      else{
        digitalWrite(_motor_pin1, LOW);
        digitalWrite(_motor_pin2, HIGH);      
      }
      analogWrite(_motor_speed_pin, motor_speed);    
    }
};

/*
Motor::Motor(int mp1, int mp2, int msp){
  _motor_pin1=mp1;
  _motor_pin2=mp2;
  _motor_speed_pin=msp;
  pinMode(_motor_pin1, OUTPUT);
  pinMode(_motor_pin2, OUTPUT);
}
*/

class Sensor{
  private:
    int _sensor_pin, _sensor_value;
  public:
    void attachPin(int sp){
      _sensor_pin=sp;
    }
    int readData(){
      return analogRead(_sensor_pin);
    }
      
};

class Robot{
  private:
    Motor motor_left, motor_right;
    Sensor sensor[6];
  public:
    Robot(){
      motor_left.attachPins(MOTOR1_PIN1, MOTOR1_PIN2, MOTOR1_SPEED_PIN);
      motor_right.attachPins(MOTOR2_PIN1, MOTOR2_PIN2, MOTOR2_SPEED_PIN);
      for(int i=0;i<6;i++)  
        sensor[i].attachPin(sensor_pins[i]);
    }
    void translate(int cmd){
      //int normalised_speed=MIN_MOTOR_SPEED+(cmd*(MAX_MOTOR_SPEED-MIN_MOTOR_SPEED));
      int normalised_speed=cmd;
      motor_left.setMotorSpeed(normalised_speed);
      motor_right.setMotorSpeed(normalised_speed);
    }
    void rotate(int cmd){
      //int normalised_speed=MIN_MOTOR_SPEED+(abs(cmd)*(MAX_MOTOR_SPEED-MIN_MOTOR_SPEED));
      int normalised_speed=cmd-128;  
      if(cmd<0){  
        motor_left.setMotorSpeed(-normalised_speed);
        motor_right.setMotorSpeed(normalised_speed);
      }
      else{
        motor_left.setMotorSpeed(normalised_speed);
        motor_right.setMotorSpeed(-normalised_speed);
      } 
    }
    int getMaxSensorReading(){
      int max_reading=0;
      for(int i=0;i<6;i++)
      {
        int sensor_reading=sensor[i].readData();
        //Serial.println(sensor_reading);
        if(max_reading<sensor_reading){
          max_reading=sensor_reading;
        }
      }
      return max_reading;
    }
    
};

Robot swarm_bot;

// the setup routine runs once when you press reset:
void setup()  { 
  Serial.begin(115200);
//  Serial.println("Board initialised");
  digitalWrite(LED_PIN, HIGH);
} 
char cmd[2];
uint8_t value; 
// the loop routine runs over and over again forever:
void loop()  { 
  /*
  if (Serial.available() > 0) {
    Serial.readBytes(cmd, 2);
      for(int i=0;i<2;i++)
  {
    Serial.println(cmd[i]);
  }
  }
*/
  /* 
  while (Serial.available() > 0) {
    // get incoming byte:
    cmd = Serial.read();
    value=cmd;
    Serial.println(cmd);
    Serial.println(value);
   }
   */

  if (Serial.available() > 0) {
    Serial.readBytes(cmd, 3);
//    Serial.println(cmd);
    if (cmd[0]==BOT_ADDRESS){
//      Serial.println("Bot a");
      if(cmd[1]=='t'){
        value=cmd[2];
//        Serial.print(value);
//        Serial.println("translate");
        swarm_bot.translate(value);
        /*
        digitalWrite(4, LOW);
        digitalWrite(5, HIGH);      
        analogWrite(9, value);
  */
      }
      else if(cmd[1]=='r'){
        value=cmd[2];
//        Serial.print(value);
//        Serial.println("rotate");
        swarm_bot.rotate(value);
      }
      else if(cmd[1]=='i'){
        //Serial.print(value);
        Serial.println(swarm_bot.getMaxSensorReading());
      }
    }  
  }
}


