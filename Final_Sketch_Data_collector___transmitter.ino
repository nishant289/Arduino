// TODO: implement a clearing mechanism for SIM Memory of Messages. when your sim storage is full,
// your GET will not work.

// TODO: Change the _SS_MAX_RX_BUFF to 256 to increase RX buffer.
// in SoftwareSerial.h
// Connect GPS Tx to 0
// VCC to 3.3v
// GND to GND
// Connect GSM Rx to 10
// Tx to 9
// GND to GND
// Before uploading, disconnect the TX of GPS
#include <SoftwareSerial.h>
#include <Wire.h>


#include <SoftwareSerial.h>

// Initialize software serial to communicate with GSM module
SoftwareSerial mySerial(9, 10);


//////// Accelerometer 
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ ,Vx,Vy,Vz,gX,gY,gZ;
unsigned long  t_i,t_f;

//////// Kalman filter for GPS + accelerometer
float E_ESTx;
 int latt = 1;
 int lon =1;
 float E_ESTy;
float ESTt_1;
float ESTty_1;

float E_ESTxax;
 int axx = 1;
float ESTt_1ax;

float E_ESTxay;
 int ayy = 1;
float ESTt_1ay;

float E_ESTxaz;
 int azz = 1;
float ESTt_1az;
////////

double t,t1,dt;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
/////////

/////// GPS
SoftwareSerial GPSModule(10, 11); // RX, TX
int updates;
int failedUpdates;
int pos;
int stringplace = 0;


float x_degree;
float y_degree;
String timeUp;
String nmea[15];
String labels[12] {"Time: ", "Status: ", "Latitude: ", "Hemisphere: ", "Longitude: ", "Hemisphere: ", "Speed: ", "Track Angle: ", "Date: "};
////// 


/*
//Initialize GPS to communicate with GPS module
TinyGPS gps;
*/
// Variable to hold latitude data to be sent to server.
String latitude;

// Variable to hold longitude data to be sent to server.
String longitude;

// Configure your Device ID here, This will be root of this Device info
int DEVICEID = 0;

// Prototype for gpsdump method
//void gpsdump(TinyGPS &gps);

// Prototype for printFloat method
String printFloat(double f, int digits = 2);

/* Function name: setup

Parameter name: None
P.Datatype: Not Applicable
P.Description: None

F.Description: Executes only once. Initialization is done here in this method.

* */

void setup() {

mySerial.begin(9600); // Setting the baud rate of GSM Module
Serial.begin(9600); // Setting the baud rate of Serial Monitor (Arduino)
pinMode(3, OUTPUT);

GPSModule.begin(9600);
  Wire.begin();
  setupMPU();

delay(10000);

// Establish HTTP connection in GSM module.
setupHttpInit();

}

/* Function name: loop

Parameter name: none
P.Datatype: Not applicable
P.Description: None

F.Description: Main loop of the software

* */

void loop() {

  t_i = millis()/1000;
 Serial.flush();
  GPSModule.flush();
  while (GPSModule.available() > 0)
  {
    GPSModule.read();

  }
  if (GPSModule.find("$GPRMC,")) {
    String tempMsg = GPSModule.readStringUntil('\n');
    for (int i = 0; i < tempMsg.length(); i++) {
      if (tempMsg.substring(i, i + 1) == ",") {
        nmea[pos] = tempMsg.substring(stringplace, i);
        stringplace = i + 1;
        pos++;
      }
      if (i == tempMsg.length() - 1) {
        nmea[pos] = tempMsg.substring(stringplace, i);
      }
    }
    updates++;
    nmea[2] = ConvertLat();   // Latitude value 
    
         /////////////// * for Latitude Kalman Filter
      float ESTx =  nmea[2].toFloat();
    
    if (latt == 1){
      float ESTt_1 = ESTx;
      
      E_ESTx = 0.2;
      latt++;
    }
    
  float k = E_ESTx/(E_ESTx+ 0.000072);
  float current_Estimate = ESTt_1 + k*(ESTx-ESTt_1);
  float E_EST_t = (1-k)*E_ESTx;

 ESTt_1 = current_Estimate;
  E_ESTx = E_EST_t;
 // nmea[2] = String(current_Estimate);       not needed                   
        /////////////// * 
        
    nmea[4] = ConvertLng();  //  Longitude value 

     /////////////// * for Longitude Kalman Filter
   
float ESTy =  nmea[4].toFloat();
    
    if (lon == 1){
      float ESTty_1 = ESTy;
      
      E_ESTy = 0.2;
      lon++;
    }
    
  float ky = E_ESTy/(E_ESTy+ 0.000023);
  float current_Estimate_y = ESTty_1 + ky*(ESTy-ESTty_1);
  float E_EST_ty = (1-ky)*E_ESTy;
  
 ESTty_1 = current_Estimate_y;
  E_ESTy = E_EST_ty;
 // nmea[4] = String(current_Estimate_y);   not needed
 
    /////////////// *
 
/////////////////// Code for printing 


  recordAccelRegisters();
  recordGyroRegisters();


  velocity();
  
  printData(); // printing 
 
 
  gX=gForceX;
  gY=gForceY;
  gZ=gForceZ;
  t_f = t_i;


//////////////////

   
    for (int i = 0; i < 9; i++) {
     /* Serial.print(labels[i]);
      Serial.print(nmea[i]);
      Serial.println("");
      */
    }
    Serial.print('\n');

  }
  else {

    failedUpdates++;

  }
  stringplace = 0;
  pos = 0;

latitude = String(t_f)+","+String(rotX)+","+String(rotY)+","+String(rotZ)+","+String(gForceX)+","+String(gForceY)+","+String(gForceZ)+","+String(Vx)+","+String(Vy)+","+String(nmea[2])+","+String(nmea[4]);// Data for sending to the server 

longitude = String(t_f)+","+String(rotX)+","+String(rotY)+","+String(rotZ)+","+String(gForceX)+","+String(gForceY)+","+String(gForceZ)+","+String(Vx)+","+String(Vy)+","+String(nmea[2])+","+String(nmea[4]);// Data for sending to the server 


// Send to Server
sendDataToServer(DEVICEID);
delay(500);
}



/* Function name: waitUntilResponse

Parameter name: delayMs
P.Datatype: int
P.Description: Delay until next event. Should be specified in milliseconds.

F.Description: Hold Execution until the GSM Module sends reponse.

* */

void waitUntilReponse(int delayMs)
{
// Delay in ms
while (mySerial.available() < 0) { delay(delayMs); } while (mySerial.available() > 0) {
Serial.write(mySerial.read());
}

}

/* Function name: setupHttpInit

Parameter name: None
P.Datatype : Not Applicable
P.Description: None

F.Description: Setting up HTTP and Bearer configuration for sending HTTP GET request.

* */

void setupHttpInit() {

//Serial.println(“Waiting for GSM to get ready”);
//wait till gsm responds ok
waitUntilReponse(500);

//Serial.println(“GSM is ready proceed to send commands”);

// Sending APN Settings

mySerial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n");
delay(1000);
waitUntilReponse(500);

mySerial.println("AT+SAPBR=3,1,\"APN\",\"Your_APN_NAME\"\r\n");
delay(1000);
waitUntilReponse(500);

mySerial.println("AT+SAPBR=1,1\r\n");
delay(3000);
waitUntilReponse(500);

mySerial.println("AT+HTTPINIT\r\n");
delay(300);
waitUntilReponse(500);

}

/* Function name: sendDataToServer

Parameter name: deviceId
P.Datatype: int
P.Description: Device ID to be sent to server for whose GPS Coordinates is being Uploaded.

F.Description: Setting up HTTP and Bearer configuration for sending HTTP GET request.

* */

void sendDataToServer(int deviceId) {

mySerial.println("AT+HTTPINIT=?\r\n");
delay(300);
waitUntilReponse(500);

mySerial.println("AT+HTTPPARA=\"CID\",1\r\n");
delay(300);
waitUntilReponse(500);

mySerial.write("AT+HTTPPARA=\"URL\",\"https://scootdatatransfer.000webhostapp.com/;");
mySerial.print(deviceId);
mySerial.write("&lat=");
mySerial.print(latitude);
mySerial.write("&lon=");
mySerial.print(longitude);
mySerial.write(":80\"\r\n");
delay(500);
waitUntilReponse(500);

mySerial.write("AT+HTTPACTION=0\r\n");
delay(3000);
waitUntilReponse(500);

//mySerial.write(“AT+SAPBR =0,1\r\n”);
// delay(500);
//waitUntilReponse(500);

mySerial.write("AT+HTTPREAD=0,10000\r\n");
delay(500);
waitUntilReponse(500);

}

/* Function name: gpsdump

Parameter name: &gps
P.Datatype : TinyGPS
P.Description: Pointer to the gps global variable that contains the GPS data.

F.Description: writes GPS data to latitude(Global variable), longitude(Global variable).

* */
//////////////////////////////////////////////////////////////////////////////////////////////////////////


String ConvertLat() {
  String posneg = "";
  if (nmea[3] == "S") {
    posneg = "-";
  }
  String latfirst;
  float latsecond;
  for (int i = 0; i < nmea[2].length(); i++) {
    if (nmea[2].substring(i, i + 1) == ".") {
      latfirst = nmea[2].substring(0, i - 2);
      latsecond = nmea[2].substring(i - 2).toFloat();
    }
  }
  latsecond = latsecond / 60;
  String CalcLat = "";

  char charVal[9];
  dtostrf(latsecond, 4, 6, charVal);
  for (int i = 0; i < sizeof(charVal); i++)
  {
    CalcLat += charVal[i];
  }
  latfirst += CalcLat.substring(1);
  latfirst = posneg += latfirst;
  /*
  x_degree = latfirst.toFloat();
  float x= x_degree*110.574*1000;
  latfirst = String(x);
  */
  return latfirst;
}

String ConvertLng() {
  String posneg = "";
  if (nmea[5] == "W") {
    posneg = "-";
  }

  String lngfirst;
  float lngsecond;
  for (int i = 0; i < nmea[4].length(); i++) {
    if (nmea[4].substring(i, i + 1) == ".") {
      lngfirst = nmea[4].substring(0, i - 2);
      //Serial.println(lngfirst);
      lngsecond = nmea[4].substring(i - 2).toFloat();
      //Serial.println(lngsecond);

    }
  }
  lngsecond = lngsecond / 60;
  String CalcLng = "";
  char charVal[9];
  dtostrf(lngsecond, 4, 6, charVal);
  for (int i = 0; i < sizeof(charVal); i++)
  {
    CalcLng += charVal[i];
  }
  lngfirst += CalcLng.substring(1);
  lngfirst = posneg += lngfirst;
/*
 y_degree = lngfirst.toFloat();
 
float y= y_degree*111.320*1000*cos(x_degree/3.414);
  lngfirst = String(y);
  */
  return lngfirst;
}

////// Accelerometer

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00001000); //Setting the accel to +/- 4g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  gForceX = ((accelX / 8192.0)*9.8)+0.5;
  gForceY = (accelY / 8192.0)*9.8; 
  gForceZ = (accelZ / 8192.0)*9.8;
  
 float ESTxax =  gForceX;
    
    if (axx == 1){
      float ESTt_1ax = ESTxax;
      
      E_ESTxax = 0.01721703;
      axx++;
    }
    
  float kax = E_ESTxax/(E_ESTxax+ 0.0327);
  float current_Estimateax = ESTt_1ax + kax*(ESTxax-ESTt_1ax);
  float E_EST_tax = (1-kax)*E_ESTxax;
  
 
 ESTt_1ax = current_Estimateax;
  E_ESTxax = E_EST_tax;
  gForceX = current_Estimateax-0.41;

////////////////////////////////////////////////////////////

float ESTxay =  gForceY;
    
    if (ayy  == 1){
      float ESTt_1ay = ESTxay;
      
      E_ESTxay = 0.02140187;
      ayy++;
    }
    
  float kay = E_ESTxay/(E_ESTxay+ 0.029461);
  float current_Estimateay = ESTt_1ay + kay*(ESTxay-ESTt_1ay);
  float E_EST_tay = (1-kay)*E_ESTxay;
  
 
 ESTt_1ay = current_Estimateay;
  E_ESTxay = E_EST_tay;
  gForceY = current_Estimateay;

///////////////////////////////////////////////////////////

float ESTxaz =  gForceZ;
    
    if (azz == 1){
      float ESTt_1az = ESTxaz;
      
      E_ESTxaz = 0.2;
      azz++;
    }
    
  float kaz = E_ESTxaz/(E_ESTxaz+ 0.000072);
  float current_Estimateaz = ESTt_1az + kaz*(ESTxaz-ESTt_1az);
  float E_EST_taz = (1-kaz)*E_ESTxaz;
  
 
 ESTt_1az = current_Estimateaz;
  E_ESTxaz = E_EST_taz;
  gForceZ = current_Estimateaz;

  
///////////////////////////////////////////////////////////
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX ml
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

void printData() {
   Serial.print(t_f);
   Serial.print(",");
  
  
  Serial.print(rotX);

   Serial.print(",");
  Serial.print(rotY);
  
   Serial.print(",");
  Serial.print(rotZ);
   
   Serial.print(",");
  Serial.print(gForceX);
  
   Serial.print(",");
  Serial.print(gForceY);

   Serial.print(",");
  Serial.print(gForceZ);
  
   Serial.print(",");
  Serial.print(Vx);
   Serial.print(",");

  Serial.print(Vy);
Serial.print(",");

Serial.print( nmea[2]);
Serial.print(",");

Serial.print( nmea[4]);

 
}

void velocity(){

  
  Vx = 0.5*(gForceX+gX)*(t_i-t_f);
  Vy = 0.5*(gForceY+gY)*(t_i-t_f);
 

}
//////
