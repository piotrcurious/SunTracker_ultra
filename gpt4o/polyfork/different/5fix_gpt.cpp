#include <cmath>

// Constants for fixed-point scaling and conversion
constexpr int32_t SCALE = 100000;                       // Fixed-point scaling factor
constexpr int32_t DEG_TO_RAD_SCALED = 1745329;          // DEG_TO_RAD * SCALE
constexpr int32_t RAD_TO_DEG_SCALED = 5729578;          // RAD_TO_DEG * SCALE

// Location-specific constants (scaled degrees)
constexpr int32_t Lat_deg = 523333;  // Latitude in scaled degrees (52.3333°)
constexpr int32_t Lon_deg = 132000;  // Longitude in scaled degrees (132.0°)
constexpr int Timezone = 0;          // Timezone offset in hours from UTC

// Outputs
int32_t sun_azimuth;      // Scaled azimuth angle (degrees)
int32_t sun_elevation;    // Scaled elevation angle (degrees)

// Utility Functions
int32_t normalize_angle(int32_t angle) {
    while (angle < 0) angle += 360 * SCALE;
    while (angle >= 360 * SCALE) angle -= 360 * SCALE;
    return angle;
}

// Taylor series for sine approximation (fixed-point)
int32_t taylor_sin(int32_t x) {
    int32_t x2 = (x * x) / SCALE;
    int32_t result = x - (x * x2 / 6) + (x2 * x2 * x / 120);
    return result >= SCALE ? SCALE : result;  // Clamp to max range
}

// Taylor series for cosine approximation (fixed-point)
int32_t taylor_cos(int32_t x) {
    int32_t x2 = (x * x) / SCALE;
    int32_t result = SCALE - (x2 / 2) + (x2 * x2 / 24);
    return result >= SCALE ? SCALE : result;  // Clamp to max range
}

// Julian Date calculation (integer math)
int32_t JulianDate(int year, int month, int day) {
    if (month <= 2) {
        year -= 1;
        month += 12;
    }
    int32_t A = year / 100;
    int32_t B = 2 - A + (A / 4);
    return (36525 * (year + 4716) / 100) + (306001 * (month + 1) / 10000) + day - 1524;
}

// Solar position calculation
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    // Step 1: Julian Date and fractional day
    int32_t JDate = JulianDate(year, month, day) * SCALE;
    int32_t JD_frac = ((hour * 3600 + minute * 60 + second) * SCALE) / 86400;
    int32_t JD = JDate + JD_frac - Timezone * SCALE / 24;
    int32_t T = (JD - 2451545 * SCALE) / (36525 * SCALE);  // Julian centuries since J2000.0

    // Step 2: Mean longitude and anomaly
    int32_t L0 = (28046645 + (3600076983 * T / SCALE)) % (360 * SCALE); // Scaled mean longitude
    int32_t M = (35752910 + (359990503 * T / SCALE)) % (360 * SCALE);  // Scaled mean anomaly

    // Step 3: Eccentricity of Earth's orbit
    int32_t e = (16708617 - 42 * T) / 1000;

    // Step 4: Sun's true anomaly and distance
    int32_t C = taylor_sin(M) * (19146 - 481 * T) / SCALE +
                taylor_sin(2 * M) * (1999 - 10 * T) / SCALE +
                taylor_sin(3 * M) * 29 / SCALE;
    int32_t L_true = normalize_angle(L0 + C);  // Scaled true longitude
    int32_t R = SCALE * (1 - e * e / SCALE) / (1 + (e * taylor_cos(C)) / SCALE);

    // Step 5: Sun's Right Ascension and Declination
    int32_t sin_L_true = taylor_sin(L_true);
    int32_t cos_L_true = taylor_cos(L_true);
    int32_t Obl = (234942 - 46 * T) / 1000;  // Obliquity of the ecliptic
    int32_t RA = atan2(sin_L_true * taylor_cos(Obl), cos_L_true);
    int32_t Decl = asin(sin_L_true * taylor_sin(Obl));

    // Step 6: Local Hour Angle
    int32_t GrHrAngle = normalize_angle(280460 * SCALE + 3609856 * JD_frac / SCALE);
    int32_t HrAngle = normalize_angle(GrHrAngle + Lon_deg - RA);

    // Step 7: Solar elevation and azimuth
    int32_t sin_Lat = taylor_sin(Lat_deg * DEG_TO_RAD_SCALED / SCALE);
    int32_t cos_Lat = taylor_cos(Lat_deg * DEG_TO_RAD_SCALED / SCALE);
    int32_t elevation = asin(sin_Lat * taylor_sin(Decl) + cos_Lat * taylor_cos(Decl) * taylor_cos(HrAngle));
    int32_t azimuth = normalize_angle(atan2(taylor_sin(HrAngle), taylor_cos(HrAngle) * sin_Lat - taylor_cos(Decl) * cos_Lat));

    // Scale and output results
    sun_azimuth = (azimuth * RAD_TO_DEG_SCALED) / SCALE;
    sun_elevation = (elevation * RAD_TO_DEG_SCALED) / SCALE;
}
