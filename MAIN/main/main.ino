#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include <SoftwareSerial.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


//MPU
MPU6050 mpu;
SoftwareSerial mySerial(3, 4); // RX, TX
// kiểm soát MPU tình trạng có
bool dmpReady = false;  // đặt true nếu init DMP thành công
uint8_t mpuIntStatus;   // giữ byte trạng thái ngắt thực tế từ MPU
uint8_t devStatus;      // trạng thái trả về sau mỗi thao tác thiết bị (0 = thành công,! 0 = lỗi)
uint16_t packetSize;    // kích thước gói DMP dự kiến (mặc định là 42 byte)
uint16_t fifoCount;     // count của tất cả các byte hiện tại trong FIFO
uint8_t fifoBuffer[64]; // Bộ đệm lưu trữ FIFO

// định hướng / chuyển động
Quaternion q;           // [w, x, y, z]         vùng chứa thứ ba
VectorFloat gravity;    // [x, y, z]            vector trọng lực
float ypr[3];           // [yaw, pitch, roll] yaw / pitch / roll container và vector trọng lực

//PID

double originalSetpoint = 176.5;
double setpoint = originalSetpoint;
//double movingAngleOffset = 0.3;
double input, output;
int moveState=0; //0 = balance; 1 = back; 2 = forth

double Kp = 10.5;//45;   30
double Kd = 0.5;//1.7    2
double Ki = 60;//140    150
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

char data;
int spd=255;
//MOTOR CONTROLLER


int ENA = 11;
int IN1 = 5;
int IN2 = 12;
int IN3 = 9;
int IN4 = 8;
int ENB = 10;


volatile bool mpuInterrupt = false;     //  // cho biết liệu pin ngắt MPU có cao không
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup()
{

    pinMode (IN1, OUTPUT);
    pinMode (IN2, OUTPUT);
    pinMode (IN3, OUTPUT);
    pinMode (IN4, OUTPUT);
    pinMode (ENA, OUTPUT);
    pinMode (ENB, OUTPUT);
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mySerial.begin(9600);// giao tieps blutooth baut=9600
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
      
  // khởi tạo thiết bị
    Serial.println(F("Khởi tạo thiết bị I2C ..."));
    mpu.initialize();// xác minh kết nối

    Serial.println(F("Kiểm tra kết nối thiết bị ..."));
    Serial.println (mpu.testConnection ()? F ("Kết nối thành công MPU6050"): F ("Kết nối MPU6050 không thành công"));
    // load and configure the DMP
    Serial.println(F("Khởi tạo DMP..."));
    devStatus = mpu.dmpInitialize();

    // cung cấp bù lệch con quay hồi chuyển chỉnh tỷ lệ cho độ nhạy tối thiểu
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

 // đảm bảo nó hoạt động (trả về 0 nếu có)
    if (devStatus == 0)
    {
        // bật DMP, bây giờ nó đã sẵn sàng
        Serial.println(F ("Kích hoạt DMP ..."));
        mpu.setDMPEnabled(true);
        // bật phát hiện ngắt Arduino
        
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // đặt cờ DMP Ready để chức năng vòng lặp chính ()
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

         // nhận kích thước gói DMP dự kiến để so sánh sau
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID
        
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(5);
        pid.SetOutputLimits(-255, 255);  
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
       Serial.print (F ("Khởi tạo DMP không thành công mã"));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


}

void loop()
{
      //  if (BluetoothSerial.available() )
      //       {
      //       data=BluetoothSerial.read();
      //       //BluetoothSerial.println(data);
            if (mySerial.available()) {
    data=mySerial.read();
  }
            
            
            else setpoint= 176.5;//176.8//cho can bang tai cho
            if(data=='F')
            {
              setpoint= 177.7;/////178.7;
               //Serial.println(setpoint);
                    //mySerial.println(data);
              }
            else if(data=='B')
              {
              setpoint= 175.7;
              //Serial.println(setpoint);
              //mySerial.println(data);    
            }
            else if(data=='L')//////Rẽ trai
            {
              //Serial.println(output);
            
              digitalWrite(IN1, HIGH);
              digitalWrite(IN2, LOW);
              digitalWrite(IN3, LOW);
              digitalWrite(IN4, HIGH);
              analogWrite(ENA,spd-15);
              analogWrite(ENB,spd-15);
            

            }
            else if(data=='R')//////Rẽ phải
            { 
              //Serial.println(output);
              //mySerial.println(data);
              digitalWrite(IN1, LOW);
              digitalWrite(IN2, HIGH);
              digitalWrite(IN3, HIGH);
              digitalWrite(IN4, LOW);
              analogWrite(ENA,spd-15);
              analogWrite(ENB,spd-15);
              
            }
            else if(data=='O')
            {
             // Serial.println(data);
              setpoint= 176.8; ///cho xe can bang 
    //           digitalWrite(IN1, LOW);
    //  digitalWrite(IN2, LOW);
    //  digitalWrite(IN3, LOW);
    //  digitalWrite(IN4, LOW);
     //analogWrite(ENA,0);
     //analogWrite(ENB,0);
            }

  
    // nếu lập trình không thành công, đừng cố làm gì cả 
    if (!dmpReady) return;

    {
   // chờ đợi cho MPU ngắt hoặc gói thêm (s) có sẵn
    while (!mpuInterrupt && fifoCount < packetSize)
    {
       // không có dữ liệu mpu - thực hiện các phép tính PID và đầu ra cho các động cơ
        
        pid.Compute();
        //motorController.move(output, MIN_ABS_SPEED);
        if (input>150 && input<220) 
        {
          setpoint= 176.5;//176.8
          
          // Nếu robot đang nghiêng
        if (output>0) //Nghiêng về phía trước
        Forward(); //Xoay bánh xe về phía trước
        else if (output<0) //Nghiêng về phía sau
        Reverse(); //Xoay bánh xe về phía sau
        }
        else //Nếu robot không nghiêng
        Stop();//Giữ bánh xe
        //dọng cơ khong hoạt dong
    
    }
         
    }

     // đặt lại cờ ngắt và nhận byte INT_STATUS
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    // nhận số FIFO hiện tại
    fifoCount = mpu.getFIFOCount();
    // kiểm tra tràn 
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // đặt lại để chúng tôi có thể tiếp tục sạch
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
         // nếu không, hãy kiểm tra dữ liệu DMP sẵn sàng gián đoạn (điều này sẽ xảy ra thường xuyên)
    }
    else if (mpuIntStatus & 0x02)
    {
         // chờ cho độ dài dữ liệu có sẵn chính xác, phải chờ đợi rất ngắn
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // đọc một gói từ FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
       // theo dõi FIFO đếm ở đây trong trường hợp có sẵn> 1 gói
        // (điều này cho phép chúng tôi ngay lập tức đọc thêm mà không phải chờ gián đoạn)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);// lấy giá trị cho q
        mpu.dmpGetGravity(&gravity, &q);// lấy giá trị cho lực hấp dẫn
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);// nhận giá trị cho ypr
        
        input = ypr[1] * 180/M_PI + 180;
   }
    
   //bluetoothSerial.print("Input:");
   Serial.print("Setpoint:");
   Serial.print(setpoint);
   Serial.print("\t");
   Serial.print(",");
   Serial.print("Input:");
    Serial.println(input);
   //Serial.print("Output:");
   //Serial.println(output);

}
void Forward()  //Code xoay bánh xe về phía trước
{
    //analogWrite(4,output*1.25);
   // analogWrite(5,0);
   // analogWrite(6,output*1.25);
    //analogWrite(7,0);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA,output*1.5);
    analogWrite(ENB,output*1.5);


}
void Reverse() //Code xoay bánh xe lùi
{
   // analogWrite(4,0);
    //analogWrite(5,output*-1.5);
    //analogWrite(6,0);
    //analogWrite(7,output*-1.5); 
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, HIGH);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, HIGH);
     analogWrite(ENA,output*-1.5);
     analogWrite(ENB,output*-1.5);

}
void Stop() //Code dừng cả hai bánh xe
{
    digitalWrite(IN1, LOW);
     digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, LOW);
     analogWrite(ENA,0);
     analogWrite(ENB,0);
}
