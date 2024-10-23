//MQ2
const int buzzer = 10;                      //when the calibration start , buzzer pin 10 will be high , and low when finish calibrating
const int MQ_PIN=A0;                                //define which analog input channel you are going to use
int RL_VALUE=5;                                     //define the load resistance on the board, in kilo ohms
float RO_CLEAN_AIR_FACTOR=9.83;                     //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
int lpg_threshold = 100;                                                  //which is derived from the chart in datasheet
 
int CALIBARAION_SAMPLE_TIMES=50;                    //define how many samples you are going to take in the calibration phase
int CALIBRATION_SAMPLE_INTERVAL=500;                //define the time interal(in milisecond) between each samples in the
                                                    //cablibration phase
int READ_SAMPLE_INTERVAL=50;                        //define how many samples you are going to take in normal operation
int READ_SAMPLE_TIMES=5;                            //define the time interal(in milisecond) between each samples in 
                                                    //normal operation
                                                    
#define         GAS_LPG             0   
#define         GAS_CO              1   
#define         GAS_SMOKE           2    
 
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms


//DHT11
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN 2     // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT11     // DHT 11
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;


//GPS GSM
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
float latitude,longitude; // create variable for latitude and longitude object 
SoftwareSerial gpsSerial(3,4);//tx,rx
SoftwareSerial gsmm(7,8);//tx,rx
TinyGPSPlus gps;// create gps object

void setup()
{ 
  
 Serial.begin(9600);
 pinMode(buzzer,OUTPUT);
 Serial.print("Calibrating...");                       

  
 Ro = MQCalibration(MQ_PIN);                         //Calibrating the sensor. Please make sure the sensor is in clean air         
           
  
 Serial.print("done!");                                 //LCD display

 Serial.print("Ro= ");
 Serial.print(Ro);
 Serial.print("kohm");
 delay(1000);

 dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}
 
void loop()
{  
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  } 
  
  int iPPM_LPG = 0;
  int iPPM_CO = 0;
  int iPPM_Smoke = 0;

  iPPM_LPG = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
  iPPM_CO = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
  iPPM_Smoke = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);
  
   Serial.println("Concentration of gas ");
   
   Serial.print("LPG: ");
   Serial.print(iPPM_LPG);
   Serial.println(" ppm");   
   
   Serial.print("CO: ");
   Serial.print(iPPM_CO);
   Serial.println(" ppm");    

   Serial.print("Smoke: ");
   Serial.print(iPPM_Smoke);
   Serial.println(" ppm");  

   Serial.print("Ro: ");
   Serial.print(iPPM_Smoke);
   Serial.println("kohm");  

   delay(200);

  // Checks if it has reached the threshold value
  if (iPPM_LPG > 100)
  {
   digitalWrite(buzzer,HIGH);
   gpsSerial.begin(9600);
   gpsSerial.listen();
   textgps();
   gsmm.begin(9600);
   gsmm.print("\r");
   delay(1000);                  
   gsmm.print("AT+CMGF=1\r");    
   delay(1000);
   gsmm.print("AT+CMGS=\"+919439231211\"\r");    
   delay(1000);
   gsmm.println("LPG LEAKAGE DETECTED!!!!");
   gsmm.print("https://www.google.com/maps/place/");
   gsmm.print(latitude, 6);
   gsmm.print(",");
   gsmm.print(longitude, 6);
   gsmm.print("\n");
   gsmm.print("LPG: ");
   gsmm.print(iPPM_LPG);
   gsmm.print("\n");
   gsmm.print("CO: ");
   gsmm.print(iPPM_CO);
    gsmm.print("\n");
   gsmm.print("SMOKE: ");
   gsmm.print(iPPM_Smoke);
    gsmm.print("\n");
   gsmm.print("TEMPERATURE : ");
   gsmm.print(event.temperature);
    gsmm.print("\n");
   gsmm.print("HUMIDITY : ");
   gsmm.print(event.relative_humidity);
  delay(1000);
  gsmm.write(0x1A);
  delay(1000); 
  }
  else
  {
    digitalWrite(buzzer,LOW);
  }
  delay(100);
}

void textgps()
{

    while(1)
  {
   while (gpsSerial.available() > 0)
   { gps.encode(gpsSerial.read()); }

      if (gps.location.isUpdated())
      {
       Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
       Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
       latitude=gps.location.lat();
       longitude=gps.location.lng();
      break;
      }
}

 Serial.print("LATTITUDE="); Serial.println(latitude,6);
 Serial.print("LONGITUDE="); Serial.println(longitude,6);
 delay(1000);
}
 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro                                        
  return val;                                                      //according to the chart in the datasheet 

}
 

float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 

long MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
 
  return 0;
}
 

long  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

 
