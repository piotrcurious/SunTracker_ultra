#include <cmath>

// Constants and scale factors
constexpr int32_t DEG_TO_RAD_SCALED = 314159 / 180000;  // Fixed-point scale for DEG_TO_RAD
constexpr int32_t RAD_TO_DEG_SCALED = 180000 / 314159;  // Fixed-point scale for RAD_TO_DEG
constexpr int32_t SCALE = 100000;                       // Scaling factor for fixed-point arithmetic

// Assume these constants are set to the location's latitude and longitude in degrees
constexpr int32_t Lat_deg = 523333;  // Latitude in scaled degrees
constexpr int32_t Lon_deg = 132000;  // Longitude in scaled degrees
constexpr int Timezone = 0;          // Timezone offset in hours from UTC

// Outputs
int32_t sun_azimuth;      // Scaled output for azimuth in degrees
int32_t sun_elevation;    // Scaled output for elevation in degrees

// Function to normalize angles within a [0, 360) degree range
int32_t normalize_angle(int32_t angle) {
    while (angle < 0) angle += 360 * SCALE;
    while (angle >= 360 * SCALE) angle -= 360 * SCALE;
    return angle;
}

// Julian Date calculation in integer arithmetic with overflow management
int32_t JulianDate(int year, int month, int day) {
    if (month <= 2) {
        year -= 1;
        month += 12;
    }
    int32_t A = year / 100;
    int32_t B = 2 - A + (A / 4);
    return (36525 * (year + 4716) / 100) + (306001 * (month + 1) / 10000) + day - 1524;
}

// Modular arithmetic in fixed-point Galois Field for integers
class ModularRing {
private:
    int32_t value;
    int32_t modulus;

public:
    ModularRing(int32_t v, int32_t m) : modulus(m) {
        value = v % modulus;
        if (value < 0) value += modulus;
    }

    ModularRing operator+(const ModularRing& other) const {
        return ModularRing((value + other.value) % modulus, modulus);
    }

    ModularRing operator-(const ModularRing& other) const {
        return ModularRing((value - other.value + modulus) % modulus, modulus);
    }

    ModularRing operator*(int32_t scalar) const {
        return ModularRing((value * scalar) % modulus, modulus);
    }

    ModularRing divide_by_const(int32_t divisor) const {
        return ModularRing(value / divisor, modulus);
    }

    int32_t sin() const { return taylor_sin(value); }
    int32_t cos() const { return taylor_cos(value); }

    int32_t toDegrees() const { return (value * RAD_TO_DEG_SCALED) / SCALE; }
};

// Fixed-point Taylor series approximation for sine
int32_t taylor_sin(int32_t x) {
    int32_t x2 = (x * x) / SCALE;
    return x - (x * x2 / 6) + (x2 * x2 * x / 120);
}

// Fixed-point Taylor series approximation for cosine
int32_t taylor_cos(int32_t x) {
    int32_t x2 = (x * x) / SCALE;
    return SCALE - (x2 / 2) + (x2 * x2 / 24);
}

// Convert latitude and longitude to radians
constexpr int32_t Lat = (Lat_deg * DEG_TO_RAD_SCALED) / SCALE;
constexpr int32_t Lon = (Lon_deg * DEG_TO_RAD_SCALED) / SCALE;

// Calculate high-precision mean longitude using integer scaled values
int32_t mean_longitude(int32_t T) {
    return (28046645 + (3600076983 * T / SCALE)) % (360 * SCALE);
}

// Solar position calculation using modular and rational arithmetic in fixed-point
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    int32_t JDate = JulianDate(year, month, day) * SCALE;
    int32_t JD_frac = ((hour * 3600 + minute * 60 + second) * SCALE) / 86400;
    int32_t JD = JDate + JD_frac - Timezone * SCALE / 24;
    int32_t T = (JD - 2451545 * SCALE) / (36525 * SCALE);

    ModularRing L0(mean_longitude(T), 360 * SCALE);
    ModularRing M = L0;  // Mean anomaly

    ModularRing sinM(M.sin(), 360 * SCALE);
    ModularRing sin2M((M * 2).sin(), 360 * SCALE);
    ModularRing sin3M((M * 3).sin(), 360 * SCALE);

    ModularRing C = sinM * (19146 - 481 * T) + sin2M * (1999 - 10 * T) + sin3M * 29;
    ModularRing L_true = L0 + C;
    ModularRing f = M + C;

    int32_t RA = atan2(L_true.sin(), L_true.cos());  // Right Ascension in radians scaled
    int32_t Decl = asin(L_true.sin());               // Declination in radians scaled

    int32_t GrHrAngle = 280460 * SCALE + 3609856 * JD_frac / SCALE;
    ModularRing HrAngle(normalize_angle(GrHrAngle + (Lon - RA) * RAD_TO_DEG_SCALED), 360 * SCALE);

    int32_t elevation = asin((Lat * DEG_TO_RAD_SCALED) * Decl + taylor_cos(Lat * DEG_TO_RAD_SCALED) * taylor_cos(Decl) * taylor_cos(HrAngle.toDegrees()));
    int32_t azimuth = normalize_angle(atan2(taylor_sin(HrAngle.toDegrees()), taylor_cos(HrAngle.toDegrees())) + SCALE * 180);

    sun_azimuth = azimuth / SCALE;
    sun_elevation = elevation / SCALE;
}
