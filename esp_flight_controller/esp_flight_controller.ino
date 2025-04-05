#include <HardwareSerial.h>
#include <stdint.h>
#include <ESP32Servo.h>
#include <Wire.h>
volatile float RatePitch, RateRoll, RateYaw;
volatile float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int ESCfreq = 500;
const int drop_servo_pin = 26;
int RateCalibrationNumber;
Servo dropServo;
Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;
const int mot1_pin = 13;
const int mot2_pin = 12;
const int mot3_pin = 14;
const int mot4_pin = 27;
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
volatile int ReceiverValue[4]; // Increase the array size to 6 for Channel 1 to Channel 6
volatile int32_t drop_value = 0;
// float voltage;
float voltage;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};
float PRateRoll = 0.78; //For outdoor flights, keep this gain to 0.75 and for indoor flights keep the gain to be 0.6
float IRateRoll = 0.0625;
float DRateRoll = 0.022;
float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;
float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 4;
float IRateYaw = 2.8;
float DRateYaw = 0;

uint32_t LoopTimer;
float t=0.004;      //time cycle
volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
//Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll = 2; float PAnglePitch = PAngleRoll;
float IAngleRoll = 0; float IAnglePitch = IAngleRoll;
float DAngleRoll = 0.007; float DAnglePitch = DAngleRoll;
// UART configuration for CC2530 module
const int RXpin = 16; // GPIO19 (RX)
const int TXpin = 17; // GPIO20 (TX)
HardwareSerial vietnam(2); // UART2

void neutralPositionAdjustment()
{
  int min = 1490;
  int max = 1510;
  if (ReceiverValue[0] < max && ReceiverValue[0] > min)
  {
    ReceiverValue[0]= 1500;
  } 
  if (ReceiverValue[1] < max && ReceiverValue[1] > min)
  {
    ReceiverValue[1]= 1500;
  } 
  if (ReceiverValue[3] < max && ReceiverValue[3] > min)
  {
    ReceiverValue[3]= 1500;
  } 
  if(ReceiverValue[0]==ReceiverValue[1] && ReceiverValue[1]==ReceiverValue[3] && ReceiverValue[3]==ReceiverValue[0] )
  {
    ReceiverValue[0]= 1500;
    ReceiverValue[1]= 1500;
    ReceiverValue[3]= 1500;
  }



}


struct __attribute__((packed)) DataPacketReceive {
  uint8_t startByte;
  int32_t throttle_r;
  int32_t pitch_r;
  int32_t roll_r;
  int32_t yaw_r;
  int32_t drop_val;
  uint8_t endByte;
};


struct __attribute__((packed)) DataPacketSend {
  uint8_t startByte;
  float pitch_recev;
  float roll_recev;
  float voltage_recev; 
  uint8_t endByte;
};
void receiveData() {
  static DataPacketReceive packet;
  static uint8_t *ptr = (uint8_t*)&packet;
  static size_t index = 0;
  static bool receiving = false;

  while (vietnam.available()) {
    uint8_t byteReceived = vietnam.read();

    if (!receiving) {
      if (byteReceived == 0xAA) {
        ptr[0] = byteReceived;
        index = 1;
        receiving = true;
      }
    } else {
      ptr[index++] = byteReceived;

      if (index == sizeof(DataPacketReceive)) {
        receiving = false;
        index = 0;

        if (packet.endByte == 0x55) {

          ReceiverValue[0] = packet.roll_r;
          ReceiverValue[1] = packet.pitch_r;
          ReceiverValue[2] = packet.throttle_r;
          ReceiverValue[3] = packet.yaw_r;
          drop_value = packet.drop_val; 

        } else {
          Serial.println("Invalid end byte");
        }
      }
    }
  }
}

void sendDataBack() { 
  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastSendTime >= 50) { 
    lastSendTime = currentTime;

    float pitch_recev = complementaryAnglePitch; 
    float roll_recev = complementaryAngleRoll;   

    float voltage_recev = voltage;

    // Tạo gói dữ liệu
    DataPacketSend packet;
    packet.startByte = 0xAB; 
    packet.pitch_recev = pitch_recev;
    packet.roll_recev = roll_recev;
    packet.voltage_recev = voltage_recev; 
    packet.endByte = 0xBA; 

    vietnam.write((uint8_t*)&packet, sizeof(packet));

  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize UART for CC2530 communication
  analogReadResolution(12);
  vietnam.begin(115200, SERIAL_8N1, RXpin, TXpin);
  
  Serial.println("Receiver started");
  receiveData();
  // Initialize random number generator
  randomSeed(analogRead(0));
  int led_time=100;
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  //Đánh thức cảm biến dậy để đọc dữ liệu cảm biến, mặc định mpu6050 sẽ ở chế độ ngủ khi được khởi động 
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // thiết lập gửi địa chỉ thanh ghi cấu hình 0x1A và ghi 0x05 để thiết lập bộ lọc thông thấp của gia tốc kế
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  //gửi tiếp địa chỉ thanh ghi 0x1C cấu hình gia tốc và thiết lập phạm vi +- 8g (0x10)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  //Gửi địa chỉ thanh ghi cấu hình con quay hồi chuyển, ghi giá trị 0x8 để thiết lập phạm vi đo
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();
 
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  delay(1000);
  mot1.attach(mot1_pin,1000,2000);
  delay(1000);
  mot1.setPeriodHertz(ESCfreq);
  delay(100);
  mot2.attach(mot2_pin,1000,2000);
  delay(1000);
  mot2.setPeriodHertz(ESCfreq);
  delay(100);
  mot3.attach(mot3_pin,1000,2000);
  delay(1000);
  mot3.setPeriodHertz(ESCfreq);
  delay(100);
  mot4.attach(mot4_pin,1000,2000);
  delay(1000);
  mot4.setPeriodHertz(ESCfreq);
  delay(100);

  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
   delay(500);
  digitalWrite(15, LOW);
  digitalWrite(15, HIGH);
  delay(500);
  digitalWrite(15, LOW);
  delay(500);
  dropServo.attach(drop_servo_pin);
  dropServo.write(0); // Vị trí ban đầu

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
  {
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);
    int16_t GyroX=Wire.read()<<8 | Wire.read();
    int16_t GyroY=Wire.read()<<8 | Wire.read();
    int16_t GyroZ=Wire.read()<<8 | Wire.read();
    // Chuyển đổi từ giá trị thô sang giá trị tốc độ góc thực tế (chia cho 65.5 khi cài đặt phạm vi +- 500 độ/ giây)
    RateRoll=(float)GyroX/65.5;
    RatePitch=(float)GyroY/65.5;
    RateYaw=(float)GyroZ/65.5;
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  digitalWrite(15, HIGH);
  delay(1000);
  digitalWrite(15, LOW);
  delay(1000);
  digitalWrite(15, HIGH);
  delay(1000);
  digitalWrite(15, LOW);
  delay(1000);
  LoopTimer = micros();
}

void loop() {
  receiveData();
  if (drop_value == 1) {
    dropServo.write(90); // Nghiêng servo 90 độ để thả
  } else {
    dropServo.write(0); // Đưa servo về vị trí ban đầu
  }
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  //Gửi địa chỉ thanh ghi bắt đầu chưa dữ liệu con quay hồi chuyển 0x43(trục X), yêu cầu 6 byte dữ liệu                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  // Chuyển đổi từ giá trị thô sang giá trị tốc độ góc thực tế (chia cho 65.5 khi cài đặt phạm vi +- 500 độ/ giây)
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  //Chuyển đổi từ giá trị thô sang giá trị gia tốc thực tế (chia cho 4096 là tỷ lệ chuyển đổi cho phạm vi +-8g)
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AccX = AccX - 0.015;
  AccY = AccY + 0.007;
  AccZ = AccZ - 0.078; 
  // sử dụng công thức hàm arctan để chuyển từ giá trị gia tốc sang giá trị góc nghiên thực tế của drone và chuyển từ radian sang độ
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29;
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;
   RateRoll -= RateCalibrationRoll;
   RatePitch -= RateCalibrationPitch;
   RateYaw -= RateCalibrationYaw;
  complementaryAngleRoll=0.991*(complementaryAngleRoll+RateRoll*t) + 0.009*AngleRoll;
  complementaryAnglePitch=0.991*(complementaryAnglePitch+RatePitch*t) + 0.009*AnglePitch;
  // Clamping complementary filter roll angle to ±20 degrees
  complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
  complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);


  neutralPositionAdjustment();

  DesiredAngleRoll = 0.1*(ReceiverValue[0]-1500);
  DesiredAnglePitch = 0.1*(ReceiverValue[1]-1500);
  InputThrottle = ReceiverValue[2];
  DesiredRateYaw = 0.15*(ReceiverValue[3]-1500);
 

  voltage = ((float)analogRead(36)/4096)*12.44*(35.9/36) - 0.09158;
  sendDataBack();
  ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
PtermRoll = PAngleRoll * ErrorAngleRoll;
ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
DesiredRateRoll = PIDOutputRoll;
PrevErrorAngleRoll = ErrorAngleRoll;
PrevItermAngleRoll = ItermRoll;

ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
PtermPitch = PAnglePitch * ErrorAnglePitch;
ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
DesiredRatePitch = PIDOutputPitch;
PrevErrorAnglePitch = ErrorAnglePitch;
PrevItermAnglePitch = ItermPitch;

// Compute errors
ErrorRateRoll = DesiredRateRoll - RateRoll;
ErrorRatePitch = DesiredRatePitch - RatePitch;
ErrorRateYaw = DesiredRateYaw - RateYaw;

// Roll Axis PID
PtermRoll = PRateRoll * ErrorRateRoll;
ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);


InputRoll = PIDOutputRoll;
PrevErrorRateRoll = ErrorRateRoll;
PrevItermRateRoll = ItermRoll;

// Pitch Axis PID
PtermPitch = PRatePitch * ErrorRatePitch;
ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);


InputPitch = PIDOutputPitch;
PrevErrorRatePitch = ErrorRatePitch;
PrevItermRatePitch = ItermPitch;

// Yaw Axis PID
PtermYaw = PRateYaw * ErrorRateYaw;
ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);  
DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);  


InputYaw = PIDOutputYaw;
PrevErrorRateYaw = ErrorRateYaw;
PrevItermRateYaw = ItermYaw;


  if (InputThrottle > 1800)
  {
    InputThrottle = 1800;
  }

  
  MotorInput1 =  (InputThrottle - InputRoll - InputPitch - InputYaw); // front right - counter clockwise
  MotorInput2 =  (InputThrottle - InputRoll + InputPitch + InputYaw); // rear right - clockwise
  MotorInput3 =  (InputThrottle + InputRoll + InputPitch - InputYaw); // rear left  - counter clockwise
  MotorInput4 =  (InputThrottle + InputRoll - InputPitch + InputYaw); //front left - clockwise


  if (MotorInput1 > 2000)
  {
    MotorInput1 = 1999;
  }

  if (MotorInput2 > 2000)
  {
    MotorInput2 = 1999;
  }

  if (MotorInput3 > 2000)
  {
    MotorInput3 = 1999;
  }

  if (MotorInput4 > 2000)
  {
    MotorInput4 = 1999;
  }


  int ThrottleIdle = 1150;
  if (MotorInput1 < ThrottleIdle)
  {
    MotorInput1 = ThrottleIdle;
  }
  if (MotorInput2 < ThrottleIdle)
  {
    MotorInput2 = ThrottleIdle;
  }
  if (MotorInput3 < ThrottleIdle)
  {
    MotorInput3 = ThrottleIdle;
  }
  if (MotorInput4 < ThrottleIdle)
  {
    MotorInput4 = ThrottleIdle;
  }

  int ThrottleCutOff = 1000;
  if (ReceiverValue[2] < 1030)
  {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
    PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
    PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
    PrevItermAngleRoll=0; PrevItermAnglePitch=0;
  }
  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);
  Serial.print("MotorInput1: "); Serial.println(ReceiverValue[0]);
  Serial.print("MotorInput2: "); Serial.println(ReceiverValue[1]);
  Serial.print("MotorInput3: "); Serial.println(ReceiverValue[2]);
  Serial.print("MotorInput4: "); Serial.println(ReceiverValue[3]);

// voltage= (analogRead(36)/4096)*12.46*(35.9/36);
// if(voltage<11.1)
// {
  while (micros() - LoopTimer < (t*1000000));
  {
     LoopTimer = micros();

  }

}

