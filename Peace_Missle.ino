// run the HMC5883L compass calibration test and enter results into code below under void Setup() compass.setOffset(0,0);      


#include "Wire.h"                                                
#include "HMC5883L.h"                                             // https://github.com/jarzebski/Arduino-HMC5883L
#include <TinyGPS++.h>                                            // http://arduiniana.org/libraries/tinygpsplus/
#define address 0x1E
                                                                  // TinyGPS++ uses serial 2
                                                                  // 9600-baud gps on pins 16(tx) and 17(rx).                                                                  

//******************************************************************************************************                                                                  
// GPS
int GPS_Course;                                                    // gps's determined course to destination
int Number_of_SATS;                                                // number of satellites acquired
TinyGPSPlus gps;                                                   
                                                                   // pin 17 rx on arduino is connected to the TX on the GPS
                                                                   // pin 16 tx on arduino is connected to the RX on the GPS
                                                                   // vcc on gps to 5v on arduino
                                                                   // gnd on gps to gnd on arduino                                    
//******************************************************************************************************
//Motors

int turn_Speed = 175;                                              // motor speed compass to turn left and right
int mtr_Spd = 250;                                                 // motor speed moving forward and reverse

//******************************************************************************************************
// Compass

HMC5883L compass;                                                  
int16_t mx, my, mz;                                                // x,y,z axis from compass 
int desired_heading;                                               // value for desired heading
int compass_heading;                                               // value calculated from compass readings
int compass_dev = 5;                                               // the amount of deviation that is allowed in the compass heading - Adjust as Needed
                                                                   // setting this variable too low will cause the robot to continuously pivot left and right
                                                                   // setting this variable too high will cause the robot to veer off course
                                                                   //vcc to plus on motor board
                                                                   //gnd to minus on motor board
                                                                   //scl 21 arduino to scl compass
                                                                   //sda 20 arduino to sda compass

//******************************************************************************************************
//Possible Radio variables

//*****************************************************************************************************

// GPS Locations

unsigned long Distance_To_Home;                                    // distance to destination
double MaxHeight =-1;
bool MaxHeightAchieved=false;
double CurrentHeight=0;
int ac =0;                                                         // GPS array counter
double Home_LATarray[50];                                          // destination Latitude - 5 waypoints
double Home_LONarray[50];                                          // destination Longitude - 5 waypoints

int increment = 0;

void setup() 
{  
  Serial.begin(115200);                                            // Serial 0 is for communication with the computer
  Serial1.begin(9600);                                             // Serial 1 is for GPS communication 
  
  // // Compass
   Wire.begin();  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
  //  compass.begin();                                                 // initialize the compass (HMC5883L)
  // compass.setRange(HMC5883L_RANGE_1_3GA);                          // Set measurement range  
  // compass.setMeasurementMode(HMC5883L_CONTINOUS);                  // Set measurement mode  
  // compass.setDataRate(HMC5883L_DATARATE_30HZ);                     // Set data rate  
  // compass.setSamples(HMC5883L_SAMPLES_8);                          // Set number of samples averaged  
  // compass.setOffset(0,0);                                          // Set calibration offset 

  // Startup();                                                       // Run the Startup procedure on power-up one time
}

//********************************************************************************************************
// Main Loop

void loop()
{
// { 
//   getGPS();                                                        // Update the GPS location
//   getCompass();                                                    // Update the Compass Heading
//   CurrentHeight = gps.altitude.meters();
//   if ( MaxHeight < CurrentHeight && MaxHeightAchieved ==false)
//   {
//     MaxHeight = CurrentHeight;
//   }
//   else if(MaxHeight > CurrentHeight && MaxHeightAchieved ==false){
//     MaxHeightAchieved = true;      
//     goWaypoint();
//   }
//   else{

//   }
// getGPS();
  // gpsInfo();
   getCompass();

}
void Startup()
{           
  // for (int i=5; i >= 1; i--)                       // Count down for X seconds
  //     {         
  //       Serial.print("Wait for Startup... "); 
  //       Serial.println(i);
  //       delay(1000);                                   // Delay for X seconds
  //     }    
  // Serial.println("\nSearching for Satellites "); 
      
  // while (Number_of_SATS <= 5)                         // Wait until x number of satellites are acquired before starting main loop
  // {                                  
  //   getGPS();                                         // Update gps data
  //   Number_of_SATS = (int)(gps.satellites.value());   // Query Tiny GPS for the number of Satellites Acquired       
  // }    
  // setWaypoint();                                      // set intial waypoint to current location
  // ac = 0;                                             // zero array counter
  
  // getGPS();
}    

void getGPS()                                                 // Get Latest GPS coordinates
{
    if (Serial1.available() >0 ){
    gps.encode(Serial1.read());
    }
    

} 

void getCompass()                                               // get latest compass value
 {  

  // Vector norm = compass.readNormalize();

  // // Calculate heading
  // float heading = atan2(norm.YAxis, norm.XAxis);
 
  // if(heading < 0)
  //    heading += 2 * M_PI;      
  // compass_heading = (int)(heading * 180/M_PI);                   // assign compass calculation to variable (compass_heading) and convert to integer to remove decimal places                                                              
  // Serial.println(compass_heading);
   int x,y,z; //triple axis data
  int xmin,xmax,ymin,ymax,zmin,zmax;
  xmin=0; xmax=0; ymax=0; ymin = 0; zmin=0;zmax=0;
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  
  //Print out values of each axis
  Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  z: ");
  Serial.println(z);  

 }

void setWaypoint()                                            // Set up to 5 GPS waypoints
{
    Serial.println("GPS Waypoint Set");
    getGPS();                                                 // get the latest GPS coordinates
    getCompass();                                             // update latest compass heading     
                                               
    Home_LATarray[ac] = gps.location.lat();                   // store waypoint in an array   
    Home_LONarray[ac] = gps.location.lng();                   // store waypoint in an array   
                                                              
    Serial.print("Waypoint: ");
    Serial.print(Home_LATarray[0],6);
    Serial.print(" ");
    Serial.println(Home_LONarray[0],6);
    ac++;                                                       // increment array counter        
}

void clearWaypoints()
{
   memset(Home_LATarray, 0, sizeof(Home_LATarray));             // clear the array
   memset(Home_LONarray, 0, sizeof(Home_LONarray));             // clear the array
   ac = 0;   
   Serial1.print("GPS Waypoint Cleared");                      // display waypoints cleared
  
}


void gpsInfo()                                                  // displays Satellite data to user
{
   Number_of_SATS = (int)(gps.satellites.value());         //Query Tiny GPS for the number of Satellites Acquired 
   Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination    
   Serial.print("Lat:");
   Serial.print(gps.location.lat(),6);
   Serial.print(" Lon:");
   Serial.print(gps.location.lng(),6);
   Serial.print(" ");
   Serial.print(Number_of_SATS); 
   Serial.print(" SATs Acquired");
   Serial.print(Distance_To_Home);
   Serial.print("m"); 
   Serial.print("Distance to Home ");
   Serial.println(Distance_To_Home);
}
 

void goWaypoint()
{   
  Serial.println("Go to Waypoint");
  while (true)  
  {                                                                // Start of Go_Home procedure 
   getCompass();                                                    // Update Compass heading                                          
   getGPS();                                                        // Tiny GPS function that retrieves GPS data - update GPS location// delay time changed from 100 to 10
   if (millis() > 5000 && gps.charsProcessed() < 10){                // If no Data from GPS within 5 seconds then send error
    Serial.println(F("No GPS data: check wiring"));
   }     
 
  Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination
  GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),Home_LATarray[ac],Home_LONarray[ac]);                               //Query Tiny GPS for Course to Destination   

    if (Distance_To_Home == 0 || gps.altitude.meters()<1.5)                                   // If the Vehicle has reached it's Destination, then Stop
        {
        // StopGliding();                                               // Stop the robot after each waypoint is reached
        clearWaypoints();
        Serial.println("You have arrived!");                    // Print to Bluetooth device - "You have arrived"          
        break;                                                   // Break from Go_Home procedure and send control back to the Void Loop         
        }   
    if ( abs(GPS_Course - compass_heading) <= 15)                  // If GPS Course and the Compass Heading are within x degrees of each other then go Forward                                                                  
                                                                  // otherwise find the shortest turn radius and turn left or right  
      {
        // Forward();                                               // Go Forward
      } 
      else 
      {                                                       
         int x = (GPS_Course - 360);                           // x = the GPS desired heading - 360
         int y = (compass_heading - (x));                      // y = the Compass heading - x
         int z = (y - 360);                                    // z = y - 360
         
        //  if ((z <= 180) && (z >= 0))                           // if z is less than 180 and not a negative value then turn left otherwise turn right
        //  { SlowLeftTurn();  }
        //  else
        //  { SlowRightTurn(); }               
       } 
    

   }                                                              // End of While Loop

}

// void StopGliding()
// {
//   motor1.run(RELEASE);                                                         
//   motor2.run(RELEASE);   
// }

// void Forward()
// {
//   motor1.setSpeed(mtr_Spd);                                                   
//   motor2.setSpeed(mtr_Spd);                            
//   motor1.run(FORWARD);                                                         // go forward all wheels 
//   motor2.run(FORWARD);
// }

// void SlowLeftTurn()
// {
   
//   motor1.setSpeed(turn_Speed);                                                
//   motor2.setSpeed(turn_Speed);                      
//   motor1.run(BACKWARD);                         
//   motor2.run(FORWARD);
  
// }

// void SlowRightTurn()
// {
   
//   motor1.setSpeed(turn_Speed);                                                  
//   motor2.setSpeed(turn_Speed);                      
//   motor1.run(FORWARD);                                                           
//   motor2.run(BACKWARD);
// }
