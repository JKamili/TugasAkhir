// rotary encoder demo by 'jurs' for Arduino Forum
struct rotary_t {
  byte pinA;
  byte pinB;
  int count;
};
  uint32_t millis_ref;

#include <EnableInterrupt.h>
#include <Servo.h>
#include <Wire.h>

Servo myservo;  //create servo object to control a servo

int Tunggu = 50; //Variable for bypassing motor's safety procedure to prevent sudden acceleration
int Jalan = 73; //Variable for moving
int Lurus = 90; //Variable for straight angle steering
int Kiri = 180; //Variable for turning left steering
int Kanan = 6; //Variable for turning right steering
int Stop = 0; //Variable for stopping

double xt = 5; // Target; X Coordinate
double yt = 0; // Target; Y Coordinate

double dtarget = 0;
double dsekarang = 0;

double x0 = 0;
double y0 = 0;

double deltatheta = 0;
  
  double previousPosA = 0;
  double previousPosB = 0;
  double currentPosA, currentPosB, X, Y, THETA;
  int i;


rotary_t encoder[] = { // define 2 pins for each rotary encoder
  {2, 3}, // encoder[0].pinA, encoder[0].pinB
  {18, 19}, // encoder[1].pinA, encoder[1].pinB
  {6, 7}, // encoder[2].pinA, encoder[2].pinB
  {8, 9}, // encoder[3].pinA, encoder[3].pinB
};
#define NUMENCODERS (sizeof(encoder)/sizeof(encoder[0]))

volatile byte state_ISR[NUMENCODERS];
volatile int8_t count_ISR[NUMENCODERS];


void beginEncoders()
{ // active internal pullup resistors on each encoder pin and start timer2
  for (int i = 0; i < NUMENCODERS; i++)
  {
    pinMode(encoder[i].pinA, INPUT_PULLUP);
    pinMode(encoder[i].pinB, INPUT_PULLUP);
    readEncoder(i); // Initialize start condition
  }
  startTimer2();
}

boolean updateEncoders()
{ // read all the 'volatile' ISR variables and copy them into normal variables
  boolean changeState = false;
  for (int i = 0; i < NUMENCODERS; i++)
  {
    if (count_ISR[i] != 0)
    {
      changeState = true;
      noInterrupts();
      encoder[i].count += count_ISR[i];
      count_ISR[i] = 0;
      interrupts();
    }
  }
  return changeState;
}

volatile double printEncodersA()
{
  double deltatetaA;
  previousPosA = currentPosA;
  currentPosA = encoder[0].count * 0.0628;
  deltatetaA = (currentPosA - previousPosA);
  return deltatetaA;
}

double printEncodersB() 
{
  double deltatetaB;
  previousPosB = currentPosB;
  currentPosB = encoder[1].count * 0.0628;
  deltatetaB = (currentPosB - previousPosB);
  return deltatetaB;
}

int8_t readEncoder(byte i)
{ // this function is called within timer interrupt to read one encoder!
  int8_t result = 0;
  byte state = state_ISR[i];
  state = state << 2 | (byte)digitalRead(encoder[i].pinA) << 1 | (byte)digitalRead(encoder[i].pinB);
  state = state & 0xF;  // keep only the lower 4 bits
  /* // next two lines would be code to read 'quarter steps'
    if (state==0b0001 || state==0b0111 || state==0b1110 || state==0b1000) result= -1;
    else if (state==0b0010 || state==0b1011 || state==0b1101 || state==0b0100) result= 1;
  */
  // next two lines is code to read 'full steps'
  if (state == 0b0001) result = -1;
  else if (state == 0b0010) result = 1;
  state_ISR[i] = state;
  return result;
}


void startTimer2()  // start TIMER2 interrupts
{
  noInterrupts();
  // Timer 2 CTC mode
  TCCR2B = (1 << WGM22) | (1 << CS22)  | (1 << CS20);
  TCCR2A = (1 << WGM21);
  OCR2A = 124;   // 249==500,  124==1000 interrupts per second
  // 63 ==2000,  31==4000
  // 15 ==8000,   7==16000
  TIMSK2 = (1 << OCIE2A); // enable Timer 2 interrupts
  interrupts();
}

void stopTimer2() // stop TIMER2 interrupts
{
  noInterrupts();
  TIMSK2 = 0;
  interrupts();
}


ISR(TIMER2_COMPA_vect)  // handling of TIMER2 interrupts
{
  for (int i = 0; i < NUMENCODERS; i++)
  {
    count_ISR[i] += readEncoder(i);
  }
}


#define BAUDRATE 9600L // serial baud rate

void setup() {
  Serial.begin(BAUDRATE);
//  Serial.println();
  Serial.println("A       B"); // print some Test-Message at beginning
  millis_ref = millis() - 100;
  beginEncoders();

  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  currentPosA = currentPosB = X = Y = THETA = 0.0;
  Wire.begin(); //start transmitting command to motor controller
  Wire.beginTransmission(44); // transmit to device #44 (0x2c)00, device address is specified in datasheet
  Wire.write(byte(0x00)); // instruction byte
  Wire.write(Tunggu);        // sends potentiometer value byte
  Serial.println("Go1");
  Wire.endTransmission(); // stop transmitting

  Wire.beginTransmission(45); // transmit to device #44 (0x2c)00, device address is specified in datasheet
  Wire.write(byte(0x00)); // instruction byte
  Wire.write(Tunggu);        // sends potentiometer value byte
  Serial.println("Go2");
  Wire.endTransmission(); // stop transmitting
  delay(100);
  
  Wire.beginTransmission(44); // transmit to device #44 (0x2c)00, device address is specified in datasheet
  Wire.write(byte(0x00)); // instruction byte
  Wire.write(Jalan);        // sends potentiometer value byte
  Serial.println("Go1");
  Wire.endTransmission(); // stop transmitting

  Wire.beginTransmission(45); // transmit to device #44 (0x2c)00, device address is specified in datasheet
  Wire.write(byte(0x00)); // instruction byte
  Wire.write(Jalan);        // sends potentiometer value byte
  Serial.println("Go2");
  Wire.endTransmission(); // stop transmitting
}

void loop() {
  
  if (millis() - millis_ref >= 100)
  {
    millis_ref += 100;
    unsigned long waktuSekarang = (millis() + 100)/100;
    unsigned long t = (millis_ref/100);
    unsigned long duration = (waktuSekarang - t);
    
    if (updateEncoders()) {

      // Dapatkan informasi
      double a = printEncodersA();
      double b = printEncodersB();
      double VR = ((a*0.2)/duration); 
      double VL = ((b*0.2)/duration);
      double R = (0.265*(VR+VL)/(VR-VL));
      float omega= ((VR - VL)/0.265);

      // Artinya sedang jalan lurus
      double eps = abs(a - b);
//      Serial.print(eps); Serial.print("; ");
      if (a == b){
//        Serial.print("AA");
         X = X + cos(THETA)*VR*duration;
         Y = Y + sin(THETA)*VR*duration;
      }
      else
      { 
//        Serial.print("BB");
        double RSin = R*sin(THETA);
        double RCos = R*cos(THETA);
//
//        Serial.print("::::");
//
//        double aa = cos(omega)*(RSin);
//        double bb = sin(omega)*(-RCos);
//        double cc = sin(omega)*(RSin);
//        double dd = cos(omega)*(-RCos);
//
//        Serial.print("; aa = "); Serial.print(aa);
//        Serial.print("; bb = "); Serial.print(bb);
//        Serial.print("; cc = "); Serial.print(cc);
//        Serial.print("; dd = "); Serial.print(dd);
//        Serial.print("; X = "); Serial.print(X);
//        Serial.print("; Y = "); Serial.println(Y);
//        
        
        X = cos(omega)*(RSin) - sin(omega)*(-RCos) + (X-RSin);
        Y = sin(omega)*(RSin) + cos(omega)*(-RCos) + (Y+RCos);
      }

      THETA = THETA + omega;

//      if(THETA = -0.05){
//        THETA = 0; }

//      Serial.print(millis()); 
//      Serial.print("; a = "); Serial.print(a); Serial.print("; b = "); Serial.print(b);
 //     Serial.print(" VR = "); Serial.print(VR); Serial.print("; VL = "); Serial.print(VL);
//      Serial.print("; R = "); Serial.print(R); Serial.print("; omega = "); Serial.print(omega); 
      double x = X;
      double y = Y*-1;
      Serial.print("; X = "); Serial.print(x); Serial.print("; Y = "); Serial.print(y); Serial.print("; THETA = "); Serial.println(THETA); 


dtarget = sqrt(((xt-x0)*(xt-x0))+((yt-y0)*(yt-y0))); //Distance to target
dsekarang = sqrt(((x-x0)*(x-x0))+((y-y0)*(y-y0))); //Distance Traveled

     if (dsekarang < dtarget)
     {double coba = (1.34*yt)/(dtarget*dtarget);
      deltatheta = atan(coba)*1260/22; //Steering angle from Rad to Degree

      double purepursuit = 90-deltatheta;
      
      Serial.print("D Sekarang = "); Serial.println (dsekarang);
      Serial.print("xt="); Serial.println (xt);
      Serial.print("yt="); Serial.println (yt);
      Serial.print("D Target= "); Serial.println (dtarget);
      Serial.print("Delta Theta="); Serial.println (deltatheta);
      Serial.print("Coba="); Serial.println (coba);
        myservo.write(purepursuit);  // tell servo to go to pre-determined position
        Serial.print("Pure Pursuit="); Serial.println (purepursuit);
        Serial.print("Delta Theta="); Serial.println (deltatheta);
     }   
     else 
     {  double xt = 0; // Target; X2 Coordinate
        double yt = -3; // Target; Y2 Coordinate

        dtarget = sqrt(((xt-x0)*(xt-x0))+((yt-y0)*(yt-y0))); //Distance to target
        dsekarang = sqrt(((0-x0)*(0-x0))+((0-y0)*(0-y0))); //Distance Traveled
        
        if (dsekarang < dtarget)
     {double coba = (1.34*yt)/(dtarget*dtarget);
      deltatheta = atan(coba)*1260/22; //Steering angle from Rad to Degree

      double purepursuit = 90-deltatheta;
      
      Serial.print("D Sekarang2 = "); Serial.println (dsekarang);
      Serial.print("xt="); Serial.println (xt);
      Serial.print("yt="); Serial.println (yt);
      Serial.print("D Target= "); Serial.println (dtarget);
      Serial.print("Delta Theta="); Serial.println (deltatheta);
      Serial.print("Coba="); Serial.println (coba);
        myservo.write(purepursuit);  // tell servo to go to pre-determined position
        Serial.print("Pure Pursuit="); Serial.println (purepursuit);
        Serial.print("Delta Theta="); Serial.println (deltatheta);
     }   
     else 
      {  double xt = 5; // Target; X3 Coordinate
        double yt = 0; // Target; Y3 Coordinate

        dtarget = sqrt(((xt-x0)*(xt-x0))+((yt-y0)*(yt-y0))); //Distance to target
        dsekarang = sqrt(((0-x0)*(0-x0))+((0-y0)*(0-y0))); //Distance Traveled
        
        if (dsekarang < dtarget)
     {double coba = (1.34*yt)/(dtarget*dtarget);
      deltatheta = atan(coba)*1260/22; //Steering angle from Rad to Degree

      double purepursuit = 90-deltatheta;
      
      Serial.print("D Sekarang3 = "); Serial.println (dsekarang);
      Serial.print("xt="); Serial.println (xt);
      Serial.print("yt="); Serial.println (yt);
      Serial.print("D Target= "); Serial.println (dtarget);
      Serial.print("Delta Theta="); Serial.println (deltatheta);
      Serial.print("Coba="); Serial.println (coba);
        myservo.write(purepursuit);  // tell servo to go to pre-determined position
        Serial.print("Pure Pursuit="); Serial.println (purepursuit);
        Serial.print("Delta Theta="); Serial.println (deltatheta);
     }   
    else
     {  double xt = 0; // Target; X3 Coordinate
        double yt = -3; // Target; Y3 Coordinate

        dtarget = sqrt(((xt-x0)*(xt-x0))+((yt-y0)*(yt-y0))); //Distance to target
        dsekarang = sqrt(((0-x0)*(0-x0))+((0-y0)*(0-y0))); //Distance Traveled
        
        if (dsekarang < dtarget)
     {double coba = (1.34*yt)/(dtarget*dtarget);
      deltatheta = atan(coba)*1260/22; //Steering angle from Rad to Degree

      double purepursuit = 90-deltatheta;
      
      Serial.print("D Sekarang4 = "); Serial.println (dsekarang);
      Serial.print("xt="); Serial.println (xt);
      Serial.print("yt="); Serial.println (yt);
      Serial.print("D Target= "); Serial.println (dtarget);
      Serial.print("Delta Theta="); Serial.println (deltatheta);
      Serial.print("Coba="); Serial.println (coba);
        myservo.write(purepursuit);  // tell servo to go to pre-determined position
        Serial.print("Pure Pursuit="); Serial.println (purepursuit);
        Serial.print("Delta Theta="); Serial.println (deltatheta);
     } 
     else
     {  
  Wire.beginTransmission(44); // transmit to device #44 (0x2c)00, device address is specified in datasheet
  Wire.write(byte(0x00)); // instruction byte
  Wire.write(Stop);        // sends potentiometer value byte
  Serial.println("End1");
  Wire.endTransmission(); // stop transmitting

  Wire.beginTransmission(45); // transmit to device #44 (0x2c)00, device address is specified in datasheet
  Wire.write(byte(0x00)); // instruction byte
  Wire.write(Stop);        // sends potentiometer value byte
  Serial.println("End2");
  Wire.endTransmission(); // stop transmitting */
      Serial.print("D Sekarang = "); Serial.println (dsekarang);
      Serial.print("yt="); Serial.println (yt);
      Serial.print("D Target= "); Serial.println (dtarget);
      Serial.print("Delta Theta="); Serial.println (deltatheta);
      
        myservo.write(Lurus);
        Serial.println("Stop");
      }    
     }
    }
   }
  }
 }
}
