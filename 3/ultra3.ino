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

/*
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

*/


/*
// Constants for improved precision
const double J2000 = 2451545.0;
const double PI2 = 2.0 * PI;
const double ARCSEC_TO_RAD = PI / (180.0 * 3600.0);
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;
const double AU_TO_KM = 149597870.7; // Astronomical Unit in kilometers
const double C = 299792.458; // Speed of light in km/s

// Auxiliary functions
double normalize_angle(double angle, double min, double max) {
    double width = max - min;
    double offset = angle - min;
    return min + fmod(offset - floor(offset / width) * width, width);
}

double normalize_zero_2pi(double angle) {
    return normalize_angle(angle, 0.0, PI2);
}

// Nutation calculation structure
struct Nutation {
    double dpsi; // Nutation in longitude
    double deps; // Nutation in obliquity
};

Nutation calculate_nutation(double T) {
    // Mean elongation of the Moon from the Sun
    double D = DEG_TO_RAD * (297.85036 + 445267.111480 * T - 0.0019142 * T * T + T * T * T / 189474.0);
    
    // Mean anomaly of the Sun (Earth)
    double M = DEG_TO_RAD * (357.52772 + 35999.050340 * T - 0.0001603 * T * T - T * T * T / 300000.0);
    
    // Mean anomaly of the Moon
    double Mprime = DEG_TO_RAD * (134.96298 + 477198.867398 * T + 0.0086972 * T * T + T * T * T / 56250.0);
    
    // Moon's argument of latitude
    double F = DEG_TO_RAD * (93.27191 + 483202.017538 * T - 0.0036825 * T * T + T * T * T / 327270.0);
    
    // Longitude of ascending node of Moon's mean orbit
    double omega = DEG_TO_RAD * (125.04452 - 1934.136261 * T + 0.0020708 * T * T + T * T * T / 450000.0);
    
    // Calculate nutation using the IAU 1980 theory
    // We'll implement the largest terms for precision while maintaining performance
    Nutation nut;
    
    // Principal term for nutation in longitude
    nut.dpsi = (-171996.0 - 174.2 * T) * sin(omega);
    nut.dpsi += (-13187.0 - 1.6 * T) * sin(-2.0 * D + 2.0 * F + 2.0 * omega);
    nut.dpsi += (-2274.0 - 0.2 * T) * sin(2.0 * F + 2.0 * omega);
    nut.dpsi += (2062.0 + 0.2 * T) * sin(2.0 * omega);
    nut.dpsi += (1426.0 - 3.4 * T) * sin(M);
    nut.dpsi += (712.0 + 0.1 * T) * sin(Mprime);
    
    // Principal term for nutation in obliquity
    nut.deps = (92025.0 + 8.9 * T) * cos(omega);
    nut.deps += (5736.0 - 3.1 * T) * cos(-2.0 * D + 2.0 * F + 2.0 * omega);
    nut.deps += (977.0 - 0.5 * T) * cos(2.0 * F + 2.0 * omega);
    nut.deps += (-895.0 + 0.5 * T) * cos(2.0 * omega);
    nut.deps += (54.0 - 0.1 * T) * cos(M);
    nut.deps += (-7.0) * cos(Mprime);
    
    // Convert from 0.0001 arcseconds to radians
    nut.dpsi *= ARCSEC_TO_RAD * 0.0001;
    nut.deps *= ARCSEC_TO_RAD * 0.0001;
    
    return nut;
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
    
    // Calculate nutation
    Nutation nut = calculate_nutation(T);
    
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
    
    // Calculate Earth's orbital velocity for aberration
    double v_orbit = 29.7859; // km/s
    double aberration_const = v_orbit / C;
    
    // Mean obliquity of the ecliptic with improved accuracy
    double eps0 = 23.0 + 26.0/60.0 + 21.448/3600.0;
    double eps1 = -46.8150/3600.0;
    double eps2 = -0.00059/3600.0;
    double eps3 = 0.001813/3600.0;
    double eps = DEG_TO_RAD * (eps0 + eps1 * T + eps2 * T2 + eps3 * T3);
    
    // True obliquity (mean obliquity + nutation in obliquity)
    double Obl = eps + nut.deps;
    
    // Apply nutation in longitude to true longitude
    L_true += nut.dpsi;
    
    // Greenwich Hour Angle with nutation correction
    double JDx = JD - J2000;
    double GrHrAngle = normalize_angle(280.46061837 + 360.98564736629 * JD_frac +
                                     (360.0 * JDx + 0.98564736629 * JDx),
                                     0.0, 360.0);
    GrHrAngle += nut.dpsi * cos(Obl) * RAD_TO_DEG;
    
    // Calculate Right Ascension and Declination
    double sin_L = sin(L_true);
    double cos_L = cos(L_true);
    double sin_Obl = sin(Obl);
    double cos_Obl = cos(Obl);
    
    // Calculate geometric coordinates
    double RA = atan2(sin_L * cos_Obl, cos_L);
    double Decl = asin(sin_L * sin_Obl);
    
    // Apply aberration corrections
    double dRA_aberr = -aberration_const * (cos(RA) * cos(L_true) * cos_Obl +
                                          sin(RA) * sin(L_true)) / cos(Decl);
    double dDecl_aberr = -aberration_const * (cos(L_true) * cos_Obl * 
                                            (tan(Decl) * cos_Obl - sin(L_true) * sin_Obl));
    
    // Apply corrections
    RA += dRA_aberr;
    Decl += dDecl_aberr;
    
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

*/


// Constants for improved precision
const double J2000 = 2451545.0;
const double PI2 = 2.0 * PI;
const double ARCSEC_TO_RAD = PI / (180.0 * 3600.0);
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;
const double AU_TO_KM = 149597870.7;
const double C = 299792.458;

// Planetary orbital elements at J2000
struct PlanetElements {
    double L; // Mean longitude
    double a; // Semi-major axis
    double e; // Eccentricity
    double i; // Inclination
    double omega; // Longitude of ascending node
    double pi; // Longitude of perihelion
};

// Planetary perturbation coefficients
struct PerturbationTerm {
    double A;    // Amplitude
    int iV;      // Multiplier for Venus mean longitude
    int iE;      // Multiplier for Earth mean longitude
    int iM;      // Multiplier for Mars mean longitude
    int iJ;      // Multiplier for Jupiter mean longitude
    int iS;      // Multiplier for Saturn mean longitude
};

// Initialize planetary elements
PlanetElements getPlanetaryElements(double T) {
    PlanetElements venus = {
        normalize_zero_2pi(DEG_TO_RAD * (181.979801 + 58519.2130302 * T)), // L
        0.723332, // a
        0.006773 - 0.000049 * T, // e
        DEG_TO_RAD * (3.394662 + 0.0010037 * T), // i
        DEG_TO_RAD * (76.679920 + 0.9011206 * T), // omega
        DEG_TO_RAD * (131.563703 + 0.0048746 * T)  // pi
    };
    
    PlanetElements mars = {
        normalize_zero_2pi(DEG_TO_RAD * (355.433275 + 19140.2993313 * T)), // L
        1.523679, // a
        0.093405 + 0.000092 * T, // e
        DEG_TO_RAD * (1.849726 - 0.0006010 * T), // i
        DEG_TO_RAD * (49.558093 + 0.7720959 * T), // omega
        DEG_TO_RAD * (336.060234 + 0.0001847 * T)  // pi
    };
    
    PlanetElements jupiter = {
        normalize_zero_2pi(DEG_TO_RAD * (34.351484 + 3034.9056746 * T)), // L
        5.202561, // a
        0.048498 + 0.000163 * T, // e
        DEG_TO_RAD * (1.303270 - 0.0054966 * T), // i
        DEG_TO_RAD * (100.464441 + 1.0209550 * T), // omega
        DEG_TO_RAD * (14.331309 + 0.0164943 * T)  // pi
    };
    
    PlanetElements saturn = {
        normalize_zero_2pi(DEG_TO_RAD * (50.077471 + 1222.1137943 * T)), // L
        9.554747, // a
        0.055546 - 0.000346 * T, // e
        DEG_TO_RAD * (2.488878 - 0.0037363 * T), // i
        DEG_TO_RAD * (113.665524 + 0.8770979 * T), // omega
        DEG_TO_RAD * (93.056787 + 0.0565314 * T)  // pi
    };
    
    return venus; // Return Venus as primary perturber
}

// Calculate planetary perturbations
struct PlanetaryPerturbations {
    double deltaL; // Perturbation in longitude
    double deltaR; // Perturbation in radius vector
    double deltaB; // Perturbation in latitude
};

PlanetaryPerturbations calculatePlanetaryPerturbations(double T) {
    PlanetElements venus = getPlanetaryElements(T);
    
    // Mean longitudes of planets
    double L_Venus = venus.L;
    double L_Earth = normalize_zero_2pi(DEG_TO_RAD * (100.466449 + 35999.3728519 * T));
    double L_Mars = normalize_zero_2pi(DEG_TO_RAD * (355.433275 + 19140.2993313 * T));
    double L_Jupiter = normalize_zero_2pi(DEG_TO_RAD * (34.351484 + 3034.9056746 * T));
    double L_Saturn = normalize_zero_2pi(DEG_TO_RAD * (50.077471 + 1222.1137943 * T));
    
    PlanetaryPerturbations pert = {0.0, 0.0, 0.0};
    
    // Venus perturbations
    pert.deltaL += 0.00313 * sin(2 * (L_Venus - L_Earth));
    pert.deltaL += 0.00198 * sin((2 * L_Venus - 3 * L_Earth + 213.5 * DEG_TO_RAD));
    pert.deltaL += 0.00136 * sin((2 * L_Venus - 2 * L_Earth));
    
    // Mars perturbations
    pert.deltaL += 0.00114 * sin(L_Mars - 2 * L_Earth);
    pert.deltaL += 0.00069 * sin((2 * L_Mars - 3 * L_Earth));
    
    // Jupiter perturbations
    pert.deltaL += 0.00071 * sin((L_Jupiter - L_Earth));
    pert.deltaL += 0.00057 * sin((2 * L_Jupiter - 2 * L_Earth));
    
    // Saturn perturbations
    pert.deltaL += 0.00021 * sin((2 * L_Saturn - 2 * L_Earth));
    
    // Radius vector perturbations (in AU)
    pert.deltaR += 0.00000 * cos(2 * (L_Venus - L_Earth));
    pert.deltaR += -0.00014 * cos((2 * L_Venus - 3 * L_Earth + 213.5 * DEG_TO_RAD));
    pert.deltaR += 0.00010 * cos((2 * L_Venus - 2 * L_Earth));
    
    // Latitude perturbations
    pert.deltaB += 0.00027 * sin((2 * L_Venus - 3 * L_Earth + 213.5 * DEG_TO_RAD));
    pert.deltaB += 0.00016 * sin((L_Venus - L_Earth));
    
    return pert;
}

// Main calculation function
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    // Previous time calculations remain the same...
    double JD = JulianDate(year, month, day);
    double UT = hour + minute / 60.0 + second / 3600.0;
    double JD_frac = (UT - (24.0 + Timezone)) / 24.0 - 0.5;
    
    double T = (JD + JD_frac - J2000) / 36525.0;
    double T2 = T * T;
    double T3 = T2 * T;
    
    // Calculate nutation (previous implementation remains...)
    Nutation nut = calculate_nutation(T);
    
    // Calculate planetary perturbations
    PlanetaryPerturbations pert = calculatePlanetaryPerturbations(T);
    
    // Mean elements with improved accuracy
    double L0 = normalize_zero_2pi(DEG_TO_RAD * (280.46645 + 36000.76983 * T + 0.0003032 * T2));
    double M = normalize_zero_2pi(DEG_TO_RAD * (357.52910 + 35999.05030 * T - 0.0001559 * T2 - 0.00000048 * T3));
    
    // Equation of center and eccentricity
    double e = 0.016708634 - 0.000042037 * T - 0.0000001267 * T2;
    double e2 = e * e;
    
    double C = DEG_TO_RAD * ((1.914602 - 0.004817 * T - 0.000014 * T2) * sin(M) +
                            (0.019993 - 0.000101 * T) * sin(2.0 * M) +
                            0.000289 * sin(3.0 * M));
    
    // True anomaly and Sun's true longitude with planetary perturbations
    double f = M + C;
    double L_true = normalize_zero_2pi(L0 + C + pert.deltaL);
    
    // Distance to Sun in AU with planetary perturbations
    double R = ((1.000001018 * (1.0 - e2)) / (1.0 + e * cos(f))) + pert.deltaR;
    
    // Mean obliquity with improved accuracy
    double eps0 = 23.0 + 26.0/60.0 + 21.448/3600.0;
    double eps1 = -46.8150/3600.0;
    double eps2 = -0.00059/3600.0;
    double eps3 = 0.001813/3600.0;
    double eps = DEG_TO_RAD * (eps0 + eps1 * T + eps2 * T2 + eps3 * T3);
    
    // True obliquity (mean obliquity + nutation in obliquity)
    double Obl = eps + nut.deps;
    
    // Apply nutation in longitude and planetary perturbations to true longitude
    L_true += nut.dpsi;
    
    // Convert ecliptic coordinates to equatorial coordinates
    double sin_eps = sin(Obl);
    double cos_eps = cos(Obl);
    
    // Include planetary perturbations in latitude
    double B = pert.deltaB;  // Heliocentric latitude
    
    // Calculate right ascension and declination with all corrections
    double X = R * cos(L_true) * cos(B);
    double Y = R * (cos(eps) * sin(L_true) * cos(B) - sin(eps) * sin(B));
    double Z = R * (sin(eps) * sin(L_true) * cos(B) + cos(eps) * sin(B));
    
    double RA = atan2(Y, X);
    double Decl = atan2(Z, sqrt(X * X + Y * Y));
    
    // Calculate aberration corrections (previous implementation remains...)
    double aberration_const = 29.7859 / C;
    double dRA_aberr = -aberration_const * (cos(RA) * cos(L_true) * cos_eps +
                                          sin(RA) * sin(L_true)) / cos(Decl);
    double dDecl_aberr = -aberration_const * (cos(L_true) * cos_eps * 
                                            (tan(Decl) * cos_eps - sin(L_true) * sin_eps));
    
    // Apply all corrections
    RA += dRA_aberr;
    Decl += dDecl_aberr;
    
    // Calculate hour angle with all corrections
    double GrHrAngle = normalize_angle(280.46061837 + 360.98564736629 * JD_frac +
                                     (360.0 * (JD - J2000) + 0.98564736629 * (JD - J2000)),
                                     0.0, 360.0);
    GrHrAngle += nut.dpsi * cos(Obl) * RAD_TO_DEG;
    
    double HrAngle = normalize_zero_2pi(DEG_TO_RAD * GrHrAngle + Lon - RA);
    
    // Calculate final topocentric coordinates
    double sin_Lat = sin(Lat);
    double cos_Lat = cos(Lat);
    double sin_Decl = sin(Decl);
    double cos_Decl = cos(Decl);
    double cos_HrAngle = cos(HrAngle);
    
    double elevation = asin(sin_Lat * sin_Decl + 
                          cos_Lat * cos_Decl * cos_HrAngle);
    
    double y = sin(HrAngle);
    double x = cos_HrAngle * sin_Lat - tan(Decl) * cos_Lat;
    double azimuth = normalize_zero_2pi(PI + atan2(y, x));
    
    // Apply atmospheric refraction
    double actual_elevation = elevation;
    if (elevation < DEG_TO_RAD * 85.0) {
        double r = 1.02 / tan(elevation + DEG_TO_RAD * 10.3 / (elevation * RAD_TO_DEG + 5.11));
        actual_elevation += r * ARCSEC_TO_RAD;
    }
    
    // Store final results
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
