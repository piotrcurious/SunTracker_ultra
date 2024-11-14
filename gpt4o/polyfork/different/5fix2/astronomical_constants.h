// File 1: astronomical_constants.h
#ifndef ASTRONOMICAL_CONSTANTS_H
#define ASTRONOMICAL_CONSTANTS_H

#include <math.h>

const double PI = 3.14159265358979323846;
const double J2000 = 2451545.0;                // Julian date of J2000.0 epoch
const double JC_PER_CENTURY = 36525.0;         // Julian centuries per century
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;
const double ARCSEC_TO_RAD = DEG_TO_RAD / 3600.0;
const double RAD_TO_ARCSEC = RAD_TO_DEG * 3600.0;
const double ASTRONOMICAL_UNIT = 149597870.7;   // km
const double EARTH_RADIUS = 6378.137;          // km
const double EARTH_FLATTENING = 1.0 / 298.257223563;
const double LIGHT_TIME_PER_AU = 499.004783836; // seconds
const double ABERRATION_CONSTANT = 20.49552;    // arcseconds
const double LIGHT_SPEED = 299792.458;          // km/s

// General precession constants
const double PRECESSION_P = 0.02438175;       // General precession in longitude per Julian century
const double PRECESSION_N = 0.00000538691;    // Rate of change of general precession in longitude

// Obliquity of ecliptic constants
const double MEAN_OBLIQUITY_J2000 = 23.43929111; // degrees
const double OBLIQUITY_RATE = -0.013004166667;   // degrees per century

// Solar constants
const double SOLAR_RADIUS = 696000.0;         // km
const double SOLAR_MASS = 1.98847e30;         // kg
const double SOLAR_LUMINOSITY = 3.828e26;     // watts
const double SOLAR_TEMPERATURE = 5778.0;      // Kelvin
const double SOLAR_IRRADIANCE = 1361.0;       // W/m^2 at 1 AU

// Fundamental arguments coefficients (from VSOP87)
struct FundamentalArguments {
    double L;   // Mean longitude of the Sun
    double Lp;  // Mean longitude of Mercury
    double F;   // Mean argument of latitude of the Moon
    double D;   // Mean elongation of the Moon from the Sun
    double Om;  // Mean longitude of the ascending node of the Moon
    double Me;  // Mean anomaly of Earth
    double Ve;  // Mean anomaly of Venus
    double Ma;  // Mean anomaly of Mars
    double Ju;  // Mean anomaly of Jupiter
    double Sa;  // Mean anomaly of Saturn
};

// Nutation terms structure
struct NutationTerm {
    int nl, nlp, nF, nD, nOm;    // Multipliers for fundamental arguments
    double s1, s2;               // Sine coefficients for Δψ
    double c1, c2;               // Cosine coefficients for Δε
};

#endif
