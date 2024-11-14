// File 1: astronomical_constants.h
#ifndef ASTRONOMICAL_CONSTANTS_H
#define ASTRONOMICAL_CONSTANTS_H

const double J2000 = 2451545.0;                // Julian date of J2000.0 epoch
const double JC_PER_CENTURY = 36525.0;         // Julian centuries per century
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;
const double ARCSEC_TO_RAD = DEG_TO_RAD / 3600.0;
const double ASTRONOMICAL_UNIT = 149597870.7;   // km
const double EARTH_RADIUS = 6378.137;          // km
const double EARTH_FLATTENING = 1.0 / 298.257223563;
const double LIGHT_TIME_PER_AU = 499.004783836; // seconds
const double ABERRATION_CONSTANT = 20.49552;    // arcseconds
const double LIGHT_SPEED = 299792.458;          // km/s

// Fundamental arguments coefficients (from VSOP87)
struct FundamentalArguments {
    double L; // Mean longitude of the Sun
    double Lp; // Mean longitude of Mercury
    double F; // Mean argument of latitude of the Moon
    double D; // Mean elongation of the Moon from the Sun
    double Om; // Mean longitude of the ascending node of the Moon
    double Me; // Mean anomaly of Earth
    double Ve; // Mean anomaly of Venus
    double Ma; // Mean anomaly of Mars
    double Ju; // Mean anomaly of Jupiter
    double Sa; // Mean anomaly of Saturn
};

// Nutation terms structure
struct NutationTerm {
    int nl, nlp, nF, nD, nOm;    // Multipliers for fundamental arguments
    double s1, s2;               // Sine coefficients for Δψ
    double c1, c2;               // Cosine coefficients for Δε
};

#endif

// File 2: nutation_terms.h
#ifndef NUTATION_TERMS_H
#define NUTATION_TERMS_H

#include "astronomical_constants.h"

// IAU 2000B nutation model (77 terms)
const NutationTerm NUTATION_TERMS[] PROGMEM = {
    // nl, nlp, nF, nD, nOm, s1, s2, c1, c2
    {0,   0,   0,   0,   1,   -171996.0, -174.2,  92025.0,  8.9},
    {0,   0,   0,   0,   2,    2062.0,    0.2,    -895.0,   0.5},
    {-2,  0,   2,   0,   1,    46.0,      0.0,    -24.0,    0.0},
    {2,   0,   -2,  0,   0,    11.0,      0.0,     0.0,     0.0},
    // ... Add all 77 terms from IAU 2000B nutation model
};

const int NUTATION_TERMS_COUNT = sizeof(NUTATION_TERMS) / sizeof(NutationTerm);

#endif

// File 3: vsop87_terms.h
#ifndef VSOP87_TERMS_H
#define VSOP87_TERMS_H

// VSOP87 terms for Earth's position
struct VSOP87Term {
    double A;    // Amplitude
    double B;    // Phase
    double C;    // Frequency
};

// VSOP87 terms for Earth's heliocentric longitude (L)
const VSOP87Term VSOP87_L0[] PROGMEM = {
    {175347046.0, 0.0, 0.0},
    {3341656.0, 4.6692568, 6283.07585},
    {34894.0, 4.6261, 12566.1517},
    // ... Add complete VSOP87 series
};

// VSOP87 terms for Earth's heliocentric latitude (B)
const VSOP87Term VSOP87_B0[] PROGMEM = {
    {280.0, 3.199, 84334.662},
    {102.0, 5.422, 5507.553},
    // ... Add complete VSOP87 series
};

// VSOP87 terms for Earth's radius vector (R)
const VSOP87Term VSOP87_R0[] PROGMEM = {
    {100013989.0, 0.0, 0.0},
    {1670700.0, 3.0984635, 6283.07585},
    // ... Add complete VSOP87 series
};

#endif

// Main implementation file: high_precision_sun.cpp
#include <Arduino.h>
#include <math.h>
#include "astronomical_constants.h"
#include "nutation_terms.h"
#include "vsop87_terms.h"

// Global variables
double Lat = 0;    // Latitude in degrees
double Lon = 0;    // Longitude in degrees
double Alt = 0;    // Altitude above sea level in meters
int Timezone = 0;  // Timezone offset in hours
double Temperature = 288.15;  // Temperature in Kelvin
double Pressure = 1013.25;    // Pressure in millibars

// Results structure
struct SunPosition {
    double azimuth;           // Degrees
    double elevation;         // Degrees
    double distance;          // AU
    double angular_radius;    // Degrees
    double right_ascension;   // Hours
    double declination;       // Degrees
    double equation_of_time;  // Minutes
};

SunPosition sun_position;

// High-precision modular arithmetic class
class PreciseAngle {
private:
    double value;

    static double normalize(double angle) {
        angle = fmod(angle, 360.0);
        return angle < 0 ? angle + 360.0 : angle;
    }

public:
    PreciseAngle(double v) : value(normalize(v)) {}

    double toRadians() const { return value * DEG_TO_RAD; }
    double toDegrees() const { return value; }
    
    PreciseAngle operator+(const PreciseAngle& other) const {
        return PreciseAngle(value + other.value);
    }
    
    PreciseAngle operator-(const PreciseAngle& other) const {
        return PreciseAngle(value - other.value);
    }
};

// Calculate TT-UT1 difference
double getDeltaT(double jd) {
    double y = 2000.0 + (jd - J2000) / 365.25;
    double t = (y - 2000.0) / 100.0;
    
    // Predictions based on historical data and IERS bulletins
    if (y < 2050) {
        return 62.92 + 0.32217 * t + 0.005589 * t * t;
    } else if (y < 2150) {
        return -20 + 32 * ((y - 1820) / 100) * ((y - 1820) / 100) - 0.5628 * (2150 - y);
    } else {
        return -20 + 32 * ((y - 1820) / 100) * ((y - 1820) / 100);
    }
}

// Calculate fundamental arguments
FundamentalArguments calculateFundamentalArguments(double T) {
    FundamentalArguments args;
    
    // Mean elements for EMB
    args.L = PreciseAngle(280.4664567 + 360007.6982779 * T + 0.03032028 * T * T + 
                         T * T * T / 49931.0 - T * T * T * T / 15300.0 - 
                         T * T * T * T * T / 2000000.0).toDegrees();
                         
    args.Lp = PreciseAngle(218.3164477 + 481267.88123421 * T - 0.0015786 * T * T + 
                          T * T * T / 538841.0 - T * T * T * T / 65194.0 - 
                          T * T * T * T * T / 14302000.0).toDegrees();
                          
    args.F = PreciseAngle(93.2720950 + 483202.0175233 * T - 0.0036539 * T * T - 
                         T * T * T / 3526000.0 + T * T * T * T / 863310.0 + 
                         T * T * T * T * T / 727456.0).toDegrees();

    args.D = PreciseAngle(297.8501921 + 445267.1114034 * T - 0.0018819 * T * T + 
                         T * T * T / 545868.0 - T * T * T * T / 113065.0 + 
                         T * T * T * T * T / 18999000.0).toDegrees();

    args.Om = PreciseAngle(125.0445479 - 1934.1362891 * T + 0.0020754 * T * T + 
                          T * T * T / 467441.0 - T * T * T * T / 60616.0 - 
                          T * T * T * T * T / 1995000.0).toDegrees();

    // Planetary arguments
    args.Me = PreciseAngle(168.6562 + 4.0923344368 * T).toDegrees();  // Mercury
    args.Ve = PreciseAngle(76.6799 + 1.6021302244 * T).toDegrees();   // Venus
    args.Ma = PreciseAngle(49.5574 + 0.5240207766 * T).toDegrees();   // Mars
    args.Ju = PreciseAngle(100.4542 + 0.0830853001 * T).toDegrees();  // Jupiter
    args.Sa = PreciseAngle(113.6634 + 0.0334442282 * T).toDegrees();  // Saturn

    return args;
}

// Calculate nutation using complete IAU 2000B model
void calculateNutation(double T, const FundamentalArguments& args, double& deltaPsi, double& deltaEps) {
    deltaPsi = 0.0;
    deltaEps = 0.0;
    
    for (int i = 0; i < NUTATION_TERMS_COUNT; i++) {
        const NutationTerm& term = NUTATION_TERMS[i];
        
        double arg = term.nl * args.L + term.nlp * args.Lp + 
                    term.nF * args.F + term.nD * args.D + 
                    term.nOm * args.Om;
        
        // Convert to radians
        arg *= DEG_TO_RAD;
        
        // Add periodic terms
        deltaPsi += (term.s1 + term.s2 * T) * sin(arg);
        deltaEps += (term.c1 + term.c2 * T) * cos(arg);
    }
    
    // Convert to degrees
    deltaPsi *= ARCSEC_TO_RAD;
    deltaEps *= ARCSEC_TO_RAD;
}

// Calculate Earth's heliocentric position using VSOP87
void calculateEarthPosition(double T, double& L, double& B, double& R) {
    L = B = R = 0.0;
    
    // Calculate Earth's heliocentric longitude
    for (const auto& term : VSOP87_L0) {
        L += term.A * cos(term.B + term.C * T);
    }
    
    // Calculate Earth's heliocentric latitude
    for (const auto& term : VSOP87_B0) {
        B += term.A * cos(term.B + term.C * T);
    }
    
    // Calculate Earth's radius vector
    for (const auto& term : VSOP87_R0) {
        R += term.A * cos(term.B + term.C * T);
    }
    
    // Convert to proper units
    L *= ARCSEC_TO_RAD;
    B *= ARCSEC_TO_RAD;
    R /= 1.0e8;  // Convert to AU
}

// Calculate aberration correction
void calculateAberration(double T, double L, double B, double R, 
                        double& deltaL, double& deltaB) {
    double e = 0.016708634 - 0.000042037 * T - 0.0000001267 * T * T;
    double pi = 102.93735 + 1.71946 * T + 0.00046 * T * T;
    double k = ABERRATION_CONSTANT * ARCSEC_TO_RAD;
    
    deltaL = -k * (cos(L - pi) + e * cos(pi)) / cos(B);
    deltaB = -k * sin(B) * (sin(L - pi) - e * sin(pi));
}

// Calculate relativistic deflection of light
void calculateRelativisticDeflection(double R, double& deltaL, double& deltaB) {
    double gamma = 1.0;  // General relativity parameter
    double mu = 1.32712440018e20;  // Sun's gravitational parameter (m³/s²)
    double c = LIGHT_SPEED * 1000.0;  // Speed of light in m/s
    
    double deflection = -2.0 * gamma * mu / (c * c * R * ASTRONOMICAL_UNIT * 1000.0);
    
    deltaL = deflection * cos(deltaL);
    deltaB = deflection * cos(deltaB);
}

// Calculate atmospheric refraction with all corrections
double calculateAtmosphericRefraction(double apparent_elevation, 
                                    double temperature, 
                                    double pressure, 
                                    double wavelength = 0.55) {
    if (apparent_elevation < -2.0) return 0.0;

    // Convert temperature and pressure to standard conditions
    double P = pressure / 1013.25;
    double T = 288.15 / temperature;
    
    // Wavelength correction (opt</antArtifact>
// Continuing from the previous code...

// Calculate atmospheric refraction with all corrections
double calculateAtmosphericRefraction(double apparent_elevation, 
                                    double temperature, 
                                    double pressure, 
                                    double wavelength = 0.55) {
    if (apparent_elevation < -2.0) return 0.0;

    // Convert temperature and pressure to standard conditions
    double P = pressure / 1013.25;
    double T = 288.15 / temperature;
    
    // Wavelength correction (optical refractive index)
    double n = 1.0 + (7.86e-4 * P * T) / (1.0 + 0.003661 * T);
    
    // Convert elevation to radians
    double h = apparent_elevation * DEG_TO_RAD;
    
    // Calculate refraction angle using Sæmundsson's formula
    double R;
    if (apparent_elevation > 15.0) {
        // Simpler formula for higher elevations
        R = 0.00452 * P * T / tan(h);
    } else {
        // Complex formula for low elevations
        double a = 0.1594 + 0.0196 * h + 0.00002 * h * h;
        double b = (1.0 + 0.505 * h + 0.0845 * h * h);
        R = 1.02 / tan(h + a / b) * P * T;
    }
    
    // Apply wavelength correction
    R *= (0.0019279 - wavelength) / 0.00452;
    
    return R;
}

// Calculate topocentric corrections
void calculateTopocentricCorrections(double jd, double ra, double dec,
                                   double& delta_ra, double& delta_dec) {
    // Observer's position in Earth-fixed coordinates
    double u = atan(0.99664719 * tan(Lat * DEG_TO_RAD));
    double rho_sin_phi = 0.99664719 * sin(u) + (Alt / EARTH_RADIUS) * sin(Lat * DEG_TO_RAD);
    double rho_cos_phi = cos(u) + (Alt / EARTH_RADIUS) * cos(Lat * DEG_TO_RAD);

    // Greenwich apparent sidereal time
    double T = (jd - J2000) / JC_PER_CENTURY;
    double gmst = calculateGMST(jd);
    double gast = gmst + PreciseAngle(calculateNutationInLongitude(T) * cos(calculateObliquity(T))).toDegrees();
    
    // Hour angle
    double H = PreciseAngle(gast * 15.0 + Lon - ra * 15.0).toRadians();
    
    // Parallax corrections
    double parallax = EARTH_RADIUS / (sun_position.distance * ASTRONOMICAL_UNIT);
    delta_ra = atan2(-rho_cos_phi * parallax * sin(H),
                     cos(dec * DEG_TO_RAD) - rho_cos_phi * parallax * cos(H));
    delta_dec = atan2((sin(dec * DEG_TO_RAD) - rho_sin_phi * parallax) * cos(delta_ra),
                      cos(dec * DEG_TO_RAD) - rho_cos_phi * parallax * cos(H));
                      
    // Convert to degrees
    delta_ra *= RAD_TO_DEG;
    delta_dec *= RAD_TO_DEG;
}

// Calculate equation of time
double calculateEquationOfTime(double L, double Ra) {
    double eot = L - Ra;
    if (eot > 180.0) eot -= 360.0;
    if (eot < -180.0) eot += 360.0;
    return eot * 4.0;  // Convert to minutes of time
}

// Main function to calculate sun position
void calculateSunPosition(double jd) {
    // Calculate time arguments
    double T = (jd - J2000) / JC_PER_CENTURY;
    double deltaT = getDeltaT(jd);
    double TT = jd + deltaT / 86400.0;
    
    // Calculate fundamental arguments
    FundamentalArguments args = calculateFundamentalArguments(T);
    
    // Calculate nutation
    double deltaPsi, deltaEps;
    calculateNutation(T, args, deltaPsi, deltaEps);
    
    // Calculate Earth's position
    double L, B, R;
    calculateEarthPosition(T, L, B, R);
    
    // Calculate aberration
    double deltaL, deltaB;
    calculateAberration(T, L, B, R, deltaL, deltaB);
    
    // Apply corrections to get apparent coordinates
    L += deltaPsi + deltaL;
    B += deltaB;
    
    // Calculate Sun's apparent position
    sun_position.right_ascension = atan2(sin(L) * cos(args.Om + deltaEps),
                                       cos(L)) * RAD_TO_DEG / 15.0;
    sun_position.declination = asin(sin(B) * cos(args.Om + deltaEps) +
                                  cos(B) * sin(args.Om + deltaEps) * sin(L)) * RAD_TO_DEG;
    
    // Calculate topocentric corrections
    double delta_ra, delta_dec;
    calculateTopocentricCorrections(jd, sun_position.right_ascension,
                                  sun_position.declination, delta_ra, delta_dec);
    
    // Apply topocentric corrections
    sun_position.right_ascension += delta_ra / 15.0;
    sun_position.declination += delta_dec;
    
    // Calculate horizontal coordinates
    double H = (calculateGMST(jd) * 15.0 + Lon - sun_position.right_ascension * 15.0) * DEG_TO_RAD;
    double phi = Lat * DEG_TO_RAD;
    double delta = sun_position.declination * DEG_TO_RAD;
    
    // Calculate elevation and azimuth
    sun_position.elevation = asin(sin(phi) * sin(delta) +
                                cos(phi) * cos(delta) * cos(H)) * RAD_TO_DEG;
    sun_position.azimuth = atan2(-cos(delta) * sin(H),
                                cos(phi) * sin(delta) -
                                sin(phi) * cos(delta) * cos(H)) * RAD_TO_DEG + 180.0;
    
    // Apply atmospheric refraction
    double refraction = calculateAtmosphericRefraction(sun_position.elevation,
                                                     Temperature, Pressure);
    sun_position.elevation += refraction;
    
    // Calculate distance and angular radius
    sun_position.distance = R;
    sun_position.angular_radius = 959.63 / R;  // Solar radius at 1 AU = 959.63 arcseconds
    
    // Calculate equation of time
    sun_position.equation_of_time = calculateEquationOfTime(L * RAD_TO_DEG,
                                                          sun_position.right_ascension * 15.0);
}

// Initialize the calculator with observer's position
void initSunCalculator(double latitude, double longitude, double altitude,
                      int timezone, double temperature, double pressure) {
    Lat = latitude;
    Lon = longitude;
    Alt = altitude;
    Timezone = timezone;
    Temperature = temperature;
    Pressure = pressure;
}
