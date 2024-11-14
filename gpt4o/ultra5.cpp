#include <cmath>

// GaloisField360 class to handle all angle operations within GF(360)
class GaloisField360 {
public:
    float value;

    // Constructor to automatically apply modulus
    GaloisField360(float v = 0.0) : value(fmod(fmod(v, 360.0) + 360.0, 360.0)) {}

    // Addition within the field
    GaloisField360 operator+(const GaloisField360& other) const {
        return GaloisField360(value + other.value);
    }

    // Subtraction within the field
    GaloisField360 operator-(const GaloisField360& other) const {
        return GaloisField360(value - other.value);
    }

    // Multiplication within the field
    GaloisField360 operator*(float scalar) const {
        return GaloisField360(value * scalar);
    }

    // Conversion to radians for trigonometric functions
    float toRadians() const {
        return value * PI / 180.0;
    }

    // Conversion to degrees
    float toDegrees() const {
        return value;
    }
};

// Helper functions using GaloisField360 for angles
GaloisField360 mean_longitude(float T) {
    return GaloisField360(280.46646 + 36000.76983 * T + 0.0003032 * T * T);
}

GaloisField360 mean_anomaly(float T) {
    return GaloisField360(357.52911 + 35999.05029 * T - 0.0001537 * T * T);
}

GaloisField360 equation_of_center(const GaloisField360& M, float T) {
    float sin_M = sin(M.toRadians());
    float sin_2M = sin(2 * M.toRadians());
    float sin_3M = sin(3 * M.toRadians());
    return GaloisField360(
        (1.914602 - 0.004817 * T - 0.000014 * T * T) * sin_M +
        (0.019993 - 0.000101 * T) * sin_2M + 0.000289 * sin_3M
    );
}

GaloisField360 obliquity_of_ecliptic(float T) {
    return GaloisField360(23.439291 - 0.013004167 * T - 0.000000164 * T * T + 0.0000005036 * T * T * T);
}

void compute_ra_decl(const GaloisField360& L_apparent, const GaloisField360& Obl, float &RA, float &Decl) {
    float sin_L = sin(L_apparent.toRadians());
    float cos_L = cos(L_apparent.toRadians());
    RA = atan2(cos(Obl.toRadians()) * sin_L, cos_L);
    Decl = asin(sin(Obl.toRadians()) * sin_L);
}

// Main function to calculate the Sun's position
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    long JDate = JulianDate(year, month, day);
    float JD_frac = ((hour + (minute / 60.0) + (second / 3600.0)) / 24.0) - 0.5;
    float JD = JDate + JD_frac - Timezone / 24.0;
    float T = (JD - 2451545.0) / 36525.0;

    // Mean longitude, anomaly, and center equation using GaloisField360
    GaloisField360 L0 = mean_longitude(T);
    GaloisField360 M = mean_anomaly(T);
    GaloisField360 C = equation_of_center(M, T);

    // True longitude, anomaly, and obliquity within GaloisField360
    GaloisField360 L_true = L0 + C;
    GaloisField360 f = M + C;
    GaloisField360 Obl = obliquity_of_ecliptic(T);

    // Corrected radius vector
    float e = 0.016708634 - 0.000042037 * T - 0.0000001267 * T * T;
    float R = 1.000001018 * (1 - e * e) / (1 + e * cos(f.toRadians()));

    // Apparent longitude, including aberration correction
    GaloisField360 L_apparent = L_true - GaloisField360(0.00569 + 0.00478 * sin(GaloisField360(125.04 - 1934.136 * T).toRadians()));

    // Calculate Right Ascension and Declination
    float RA, Decl;
    compute_ra_decl(L_apparent, Obl, RA, Decl);

    // Greenwich and local hour angles
    float JDx = JDate - 2451545;
    GaloisField360 GrHrAngle = GaloisField360(280.46061837 + 360.98564736629 * JDx + 0.98564736629 * JD_frac);
    GaloisField360 HrAngle = GrHrAngle + GaloisField360(Lon) - GaloisField360(RA * RAD_TO_DEG);

    // Solar elevation and azimuth
    float elevation = asin(sin(Lat * PI / 180.0) * sin(Decl) + cos(Lat * PI / 180.0) * cos(Decl) * cos(HrAngle.toRadians()));
    GaloisField360 azimuth = GaloisField360(PI + atan2(sin(HrAngle.toRadians()), cos(HrAngle.toRadians()) * sin(Lat * PI / 180.0) - tan(Decl) * cos(Lat * PI / 180.0)));

    // Convert to degrees for final output
    sun_azimuth = azimuth.toDegrees();
    sun_elevation = elevation * RAD_TO_DEG;
}
