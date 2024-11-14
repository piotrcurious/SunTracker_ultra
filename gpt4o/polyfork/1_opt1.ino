#include <Arduino.h>
#include <math.h>

// Optimized constants using fixed-point arithmetic where possible
// Multiplied by 1000 to maintain 3 decimal precision without floating point
const int32_t SCALE_FACTOR = 1000;
const int32_t PI_SCALED = 3142; // PI * 1000
const int32_t RAD_TO_DEG_SCALED = 57296; // (180/PI) * 1000
const int32_t DEG_TO_RAD_SCALED = 17; // (PI/180) * 1000

// Precomputed constants for frequent calculations
const int32_t GF360_SCALE = 360000; // 360 * SCALE_FACTOR
const int16_t POLYNOMIAL_MAX_TERMS = 4; // Maximum number of terms in polynomials

// Optimized fixed-point polynomial class
class FixedPolynomial {
private:
    int32_t coeffs[POLYNOMIAL_MAX_TERMS];
    uint8_t size;

public:
    FixedPolynomial() : size(0) {
        memset(coeffs, 0, sizeof(coeffs));
    }

    // Initialize with scaled integer coefficients
    FixedPolynomial(const int32_t* init_coeffs, uint8_t init_size) {
        size = min(init_size, POLYNOMIAL_MAX_TERMS);
        memcpy(coeffs, init_coeffs, size * sizeof(int32_t));
    }

    // Optimized evaluation using Horner's method and fixed-point arithmetic
    int32_t evaluate(int32_t T) const {
        int32_t result = coeffs[size - 1];
        for (int8_t i = size - 2; i >= 0; --i) {
            result = result * T / SCALE_FACTOR + coeffs[i];
        }
        return result;
    }

    // Optimized addition
    FixedPolynomial operator+(const FixedPolynomial& other) const {
        FixedPolynomial result;
        result.size = max(size, other.size);
        for (uint8_t i = 0; i < result.size; ++i) {
            result.coeffs[i] = (i < size ? coeffs[i] : 0) + 
                              (i < other.size ? other.coeffs[i] : 0);
        }
        return result;
    }

    // Scalar multiplication
    FixedPolynomial operator*(int32_t scalar) const {
        FixedPolynomial result;
        result.size = size;
        for (uint8_t i = 0; i < size; ++i) {
            result.coeffs[i] = (coeffs[i] * scalar) / SCALE_FACTOR;
        }
        return result;
    }
};

// Optimized GF360 class using fixed-point arithmetic
class GF360 {
private:
    FixedPolynomial value;

    // Optimized modulo 360 operation
    static int32_t mod360(int32_t x) {
        x = x % GF360_SCALE;
        if (x < 0) x += GF360_SCALE;
        return x;
    }

public:
    explicit GF360(const FixedPolynomial& p) : value(p) {}

    // Optimized addition in GF360
    GF360 operator+(const GF360& other) const {
        return GF360(value + other.value);
    }

    // Convert to radians with fixed-point arithmetic
    int32_t toRadians(int32_t T) const {
        int32_t degrees = mod360(value.evaluate(T));
        return (degrees * PI_SCALED) / (180 * SCALE_FACTOR);
    }

    // Convert to degrees
    int32_t toDegrees(int32_t T) const {
        return mod360(value.evaluate(T));
    }
};

// Optimized astronomical constants
const int32_t MEAN_LONGITUDE_COEFFS[] = {280466, 36000770, 303};
const int32_t MEAN_ANOMALY_COEFFS[] = {357529, 35999050, -154};
const int32_t ECCENTRICITY_COEFFS[] = {16709, -42, 0};
const int32_t OBLIQUITY_COEFFS[] = {23439, -13004, 0, 1};

// Helper function to create polynomials from coefficient arrays
FixedPolynomial createPolynomial(const int32_t* coeffs, uint8_t size) {
    return FixedPolynomial(coeffs, size);
}

// Optimized Julian Date calculation
int32_t calculateJulianDate(int16_t year, int8_t month, int8_t day) {
    if (month <= 2) {
        year--;
        month += 12;
    }

    int32_t A = year / 100;
    int32_t B = 2 - A + (A / 4);

    return (365.25 * (year + 4716)) +
           (30.6001 * (month + 1)) +
           day + B - 1524.5;
}

// Main calculation function with optimized fixed-point arithmetic
struct SunPosition {
    int32_t azimuth;
    int32_t elevation;
};

SunPosition calculateSunPosition(int8_t hour, int8_t minute, int8_t second,
                               int8_t day, int8_t month, int16_t year,
                               int8_t timezone, int32_t longitude, int32_t latitude) {
    // Input validation
    if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || 
        second < 0 || second > 59) {
        return {0, 0};
    }

    // Calculate Julian Date with fixed-point arithmetic
    int32_t JDate = calculateJulianDate(year, month, day);
    int32_t JD_frac = ((hour * 3600 + minute * 60 + second) * SCALE_FACTOR) / 86400;
    int32_t JD = JDate + JD_frac - (timezone * SCALE_FACTOR) / 24;
    int32_t T = ((JD - 2451545) * SCALE_FACTOR) / 36525;

    // Create optimized polynomials
    GF360 L0(createPolynomial(MEAN_LONGITUDE_COEFFS, 3));
    GF360 M(createPolynomial(MEAN_ANOMALY_COEFFS, 3));
    GF360 e(createPolynomial(ECCENTRICITY_COEFFS, 3));
    GF360 Obl(createPolynomial(OBLIQUITY_COEFFS, 4));

    // Calculate equation of center using optimized sine function
    int32_t M_rad = M.toRadians(T);
    int32_t sin_M = (sin(M_rad) * SCALE_FACTOR) / SCALE_FACTOR;
    int32_t sin_2M = (sin(2 * M_rad) * SCALE_FACTOR) / SCALE_FACTOR;
    int32_t sin_3M = (sin(3 * M_rad) * SCALE_FACTOR) / SCALE_FACTOR;

    // Equation of center coefficients (scaled)
    const int32_t C_COEFFS[] = {
        (1915 * sin_M) / SCALE_FACTOR,
        (20 * sin_2M) / SCALE_FACTOR,
        sin_3M / SCALE_FACTOR
    };
    GF360 C(createPolynomial(C_COEFFS, 3));

    // Calculate true longitude and anomaly
    GF360 L_true = L0 + C;
    GF360 f = M + C;

    // Calculate radius vector (scaled)
    int32_t e_val = e.toDegrees(T);
    int32_t f_rad = f.toRadians(T);
    int32_t radius = (SCALE_FACTOR * (SCALE_FACTOR - e_val * e_val)) /
                     (SCALE_FACTOR + e_val * cos(f_rad));

    // Calculate apparent longitude
    const int32_t NUTATION_COEFFS[] = {-569, 478};
    GF360 nutation(createPolynomial(NUTATION_COEFFS, 2));
    GF360 L_apparent = L_true + nutation;

    // Calculate right ascension and declination
    int32_t L_rad = L_apparent.toRadians(T);
    int32_t Obl_rad = Obl.toRadians(T);
    
    int32_t sin_L = sin(L_rad);
    int32_t cos_L = cos(L_rad);
    int32_t sin_Obl = sin(Obl_rad);
    int32_t cos_Obl = cos(Obl_rad);

    int32_t RA = atan2(cos_Obl * sin_L, cos_L);
    int32_t Decl = asin(sin_Obl * sin_L);

    // Calculate hour angle
    int32_t GHA = mod360((280461837 + 360985647 * JD_frac) / SCALE_FACTOR);
    int32_t LHA = mod360(GHA + longitude - (RA * RAD_TO_DEG_SCALED) / SCALE_FACTOR);

    // Calculate final position
    int32_t lat_rad = (latitude * PI_SCALED) / (180 * SCALE_FACTOR);
    int32_t sin_lat = sin(lat_rad);
    int32_t cos_lat = cos(lat_rad);
    int32_t sin_LHA = sin(LHA * DEG_TO_RAD_SCALED / SCALE_FACTOR);
    int32_t cos_LHA = cos(LHA * DEG_TO_RAD_SCALED / SCALE_FACTOR);

    // Calculate elevation
    int32_t elevation = asin(sin_lat * sin(Decl) + 
                            cos_lat * cos(Decl) * cos_LHA);

    // Calculate azimuth
    int32_t azimuth = atan2(sin_LHA,
                           cos_LHA * sin_lat - tan(Decl) * cos_lat);

    // Convert to degrees and normalize
    return {
        mod360((azimuth * RAD_TO_DEG_SCALED / SCALE_FACTOR + 180000) / SCALE_FACTOR),
        (elevation * RAD_TO_DEG_SCALED) / SCALE_FACTOR
    };
}

// Example usage function
void setup() {
    Serial.begin(9600);
    
    // Example calculation for New York City
    // Date: June 21, 2024, 12:00 PM
    // Location: 40.7128°N, 74.0060°W
    SunPosition pos = calculateSunPosition(12, 0, 0,    // Time
                                        21, 6, 2024,    // Date
                                        -4,             // Eastern Time
                                        -74006,         // Longitude (scaled)
                                        40713);         // Latitude (scaled)
    
    Serial.print(F("Sun Azimuth: "));
    Serial.print(pos.azimuth / 1000.0);
    Serial.print(F("°, Elevation: "));
    Serial.print(pos.elevation / 1000.0);
    Serial.println(F("°"));
}

void loop() {
    // Empty loop
}
