#include <WiFi.h>
#include "ESPDateTime.h"    // Thanks to : https://github.com/mcxiaoke/ESPDateTime

RTC_DATA_ATTR int bootCount; 
RTC_DATA_ATTR int sun_azimuth, sun_elevation, panPosition, tiltPosition, sunriseAzimuth, sunriseSaved; pan_μS_CW; pan_μS_CCW; tilt_μS_CW; tilt_μS_CCW;
RTC_DATA_ATTR int sleepMinutes = 1;      // Adjust sleep time in minutes if needed. 15-30 minutes are ideal sleep time to conserve battery and at the same time relatively frequent movement of tracker.

const char* ssid =     "YourSSID";
const char* password = "YourPassword";

int Timezone = -4;      // Adjust according to your time zone.

float Lon = -72.71 * DEG_TO_RAD,
      Lat = 43.40 * DEG_TO_RAD;
     
// --------- End of configuration section ---------------

void setup() {
  Serial.begin(115200); delay(500);
  WiFi.mode(WIFI_STA);WiFi.begin(ssid, password);Serial.println("\nConnecting to WiFi"); while (WiFi.status() != WL_CONNECTED) {delay(500);Serial.print(".");} Serial.println("\nConnected to network");
  long upTime = bootCount * sleepMinutes;
  Serial.print("Up time "); Serial.print(upTime); Serial.println(" minutes.");
  ++bootCount;
  
  Serial.print("Last pan position was:   "); Serial.println(panPosition);
  Serial.print("Last tilt position was: ");  Serial.println(tiltPosition);
  //ToDo using https://github.com/ruenahcmohr/EGMK :
  //Serial.print("Now we are going to move to new pan postion target using : ");  Serial.print(pan_μs_CW); Serial.print("pan micro seconds forward and "); Serial.print(pan__μs_CW); Serial.println("pan micro seconds reverse."); 
  //Serial.print("Now we are going to move to new tilt postion target using : ");  Serial.print(tilt_μs_CW); Serial.print("tilt micro seconds forward and "); Serial.print(tilt_μs_CCW); Serial.println("tilt micro seconds reverse."); 

      
}

void loop() {

  DateTime.setTimeZone(Timezone);   // Eastern USA Time Zone (-5).
  DateTime.setServer("us.pool.ntp.org");
  DateTime.begin();  
  DateTimeParts p = DateTime.getParts();
  
  int lastPanPosition = panPosition; int lastTiltPosition = tiltPosition;
  Calculate_Sun_Position(p.getHours(), p.getMinutes(), 0, p.getMonthDay(), (p.getMonth() + 1), p.getYear());  // parameters are HH:MM:SS DD:MM:YYYY start from midnight and work out all 24 hour positions.
  
  if (sunriseSaved == 0 && sun_elevation > 0) {sunriseAzimuth = sun_azimuth; sunriseSaved = 1;}               // Save azimuth at sunrise in RTC memory once a day after sunrise to bring tracker back at this pan position for the next day's start point.
  if (sunriseSaved == 1 && sun_elevation < 0) {sunriseSaved = 0; /*Bring tracker back to sunrise azimuth position in the evening to lock it up during the night to save it from heavy winds*/} 
  if (sun_azimuth > sunriseAzimuth)  {panPosition =  sun_azimuth - sunriseAzimuth;}                           // Set pan position in RTC memory.
  if (sun_elevation > 0) {tiltPosition = sun_elevation;}                                                      // Set tilt position in RTC memory.
  int panMove = panPosition - lastPanPosition; int tiltMove = tiltPosition - lastTiltPosition;                // Find out the movement of both motors in degrees.
   
  Serial.print("Current Date is: "); Serial.printf("%d/%02d/%02d \n", (p.getMonth() + 1), p.getMonthDay(), p.getYear());
  Serial.print("Current time is: "); Serial.printf("%d:%02d:%02d \n", p.getHours(), p.getMinutes(), p.getSeconds());
  Serial.print("Longitude: "); Serial.println(String(Lon / DEG_TO_RAD, 3)); Serial.print("Latitude:   ");Serial.println(String(Lat / DEG_TO_RAD, 3));
  Serial.print("Sun Azimuth:   "); Serial.println(sun_azimuth); Serial.print("Sun Elevation: "); Serial.println(sun_elevation); 
  Serial.print("Sunrise Azimuth: "); Serial.println(sunriseAzimuth);        // Set sunrise as 0 (start position).
  Serial.print("Pan position:  "); Serial.println(panPosition);                     // Value ranges between (0 - 300).
  Serial.print("Tilt position: "); Serial.println(tiltPosition); Serial.println();  // When panel is vertical this value is 0.value ranges between (0 - 90).
  Serial.print("Moved pan position by degrees:   "); Serial.println(panMove);       // Value ranges between (0 - 300).
  Serial.print("Moved tilt position by degrees:  "); Serial.println(tiltMove);      // When panel is vertical this value is 0.value ranges between (0 - 90).
  
  esp_sleep_enable_timer_wakeup(sleepMinutes * 60000000); // 60000000 for 1 minute.
  esp_deep_sleep_start();  
}

/*
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
  float T, JD_frac, L0, M, e, C, L_true, f, R, GrHrAngle, Obl, RA, Decl, HrAngle;
  long JDate, JDx;
  int zone = 0;
  JDate      = JulianDate(year, month, day);
  JD_frac = (hour - (24+Timezone) + minute / 60.0 + second / 3600.0) / 24.0 - 0.5;
  T          = JDate - 2451545; T = (T + JD_frac) / 36525.0;
  L0         = DEG_TO_RAD * fmod(280.46645 + 36000.76983 * T, 360);
  M          = DEG_TO_RAD * fmod(357.5291 + 35999.0503 * T, 360);
  e          = 0.016708617 - 0.000042037 * T;
  C          = DEG_TO_RAD * ((1.9146 - 0.004847 * T) * sin(M) + (0.019993 - 0.000101 * T) * sin(2 * M) + 0.00029 * sin(3 * M));
  f          = M + C;
  Obl        = DEG_TO_RAD * (23 + 26 / 60.0 + 21.448 / 3600. - 46.815 / 3600 * T);
  JDx        = JDate - 2451545;
  GrHrAngle  = 280.46061837 + (360 * JDx) % 360 + 0.98564736629 * JDx + 360.98564736629 * JD_frac;
  GrHrAngle  = fmod(GrHrAngle, 360.0);
  L_true     = fmod(C + L0, 2 * PI);
  R          = 1.000001018 * (1 - e * e) / (1 + e * cos(f));
  RA         = atan2(sin(L_true) * cos(Obl), cos(L_true));
  Decl       = asin(sin(Obl) * sin(L_true));
  HrAngle    = DEG_TO_RAD * GrHrAngle + Lon - RA;
  float elevation  = asin(sin(Lat) * sin(Decl) + cos(Lat) * (cos(Decl) * cos(HrAngle)));
  float azimuth    = PI + atan2(sin(HrAngle), cos(HrAngle) * sin(Lat) - tan(Decl) * cos(Lat)); // Azimuth measured east from north, so 0 degrees is North
  sun_azimuth   = (azimuth   / DEG_TO_RAD);
  sun_elevation = elevation / DEG_TO_RAD;
  
}
*/

// Constants for improved precision
const double J2000 = 2451545.0;
const double PI2 = 2.0 * PI;
const double ARCSEC_TO_RAD = PI / (180.0 * 3600.0);
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;

// Auxiliary functions for better numerical stability
double normalize_angle(double angle, double min, double max) {
    double width = max - min;
    double offset = angle - min;
    return min + fmod(offset - floor(offset / width) * width, width);
}

double normalize_zero_2pi(double angle) {
    return normalize_angle(angle, 0.0, PI2);
}

void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    // Compute Julian Date with improved precision
    double JD = JulianDate(year, month, day);
    double UT = hour + minute / 60.0 + second / 3600.0;
    double JD_frac = (UT - (24.0 + Timezone)) / 24.0 - 0.5;
    
    // Time scales
    double T = (JD + JD_frac - J2000) / 36525.0;
    double T2 = T * T;
    double T3 = T2 * T;
    
    // Mean elements
    double L0 = normalize_zero_2pi(DEG_TO_RAD * (280.46645 + 36000.76983 * T + 0.0003032 * T2));
    double M = normalize_zero_2pi(DEG_TO_RAD * (357.52910 + 35999.05030 * T - 0.0001559 * T2 - 0.00000048 * T3));
    
    // Equation of center and eccentricity
    double e = 0.016708634 - 0.000042037 * T - 0.0000001267 * T2;
    double e2 = e * e;
    double e3 = e2 * e;
    
    // Calculate Sun's equation of center with higher order terms
    double C = DEG_TO_RAD * ((1.914602 - 0.004817 * T - 0.000014 * T2) * sin(M) +
                            (0.019993 - 0.000101 * T) * sin(2.0 * M) +
                            0.000289 * sin(3.0 * M));
    
    // True anomaly and Sun's true longitude
    double f = M + C;
    double L_true = normalize_zero_2pi(L0 + C);
    
    // Distance to Sun in AU
    double R = (1.000001018 * (1.0 - e2)) / (1.0 + e * cos(f));
    
    // Obliquity of ecliptic with improved accuracy
    double eps0 = 23.0 + 26.0/60.0 + 21.448/3600.0;
    double eps1 = -46.8150/3600.0;
    double eps2 = -0.00059/3600.0;
    double eps3 = 0.001813/3600.0;
    double Obl = DEG_TO_RAD * (eps0 + eps1 * T + eps2 * T2 + eps3 * T3);
    
    // Greenwich Hour Angle with higher precision
    double JDx = JD - J2000;
    double GrHrAngle = normalize_angle(280.46061837 + 360.98564736629 * JD_frac +
                                     (360.0 * JDx + 0.98564736629 * JDx),
                                     0.0, 360.0);
    
    // Calculate Right Ascension and Declination
    double sin_L = sin(L_true);
    double cos_L = cos(L_true);
    double sin_Obl = sin(Obl);
    double cos_Obl = cos(Obl);
    
    double RA = atan2(sin_L * cos_Obl, cos_L);
    double Decl = asin(sin_L * sin_Obl);
    
    // Hour angle with improved precision
    double HrAngle = normalize_zero_2pi(DEG_TO_RAD * GrHrAngle + Lon - RA);
    
    // Calculate elevation and azimuth with better numerical stability
    double sin_Lat = sin(Lat);
    double cos_Lat = cos(Lat);
    double sin_Decl = sin(Decl);
    double cos_Decl = cos(Decl);
    double cos_HrAngle = cos(HrAngle);
    
    double elevation = asin(sin_Lat * sin_Decl + 
                          cos_Lat * cos_Decl * cos_HrAngle);
    
    // Improved azimuth calculation
    double y = sin(HrAngle);
    double x = cos_HrAngle * sin_Lat - tan(Decl) * cos_Lat;
    double azimuth = normalize_zero_2pi(PI + atan2(y, x));
    
    // Apply atmospheric refraction correction for elevation
    double actual_elevation = elevation;
    if (elevation < DEG_TO_RAD * 85.0) {
        double r = 1.02 / tan(elevation + DEG_TO_RAD * 10.3 / (elevation * RAD_TO_DEG + 5.11));
        actual_elevation += r * ARCSEC_TO_RAD;
    }
    
    // Store results
    sun_azimuth = azimuth * RAD_TO_DEG;
    sun_elevation = actual_elevation * RAD_TO_DEG;
}


long JulianDate(int year, int month, int day) {
  long JDate;
  int A, B;
  if (month <= 2) {year--; month += 12;}
  A = year / 100; B = 2 - A + A / 4;
  JDate = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524;
  return JDate;
}
