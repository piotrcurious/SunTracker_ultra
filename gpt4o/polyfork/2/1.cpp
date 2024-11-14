#include <cmath>
#include <vector>

// Constants for precision control
constexpr float DEG_TO_RAD = M_PI / 180.0;
constexpr float RAD_TO_DEG = 180.0 / M_PI;

// Class for managing modular ring operations in angle calculations
class ModularRing {
private:
    float value;
    float modulus;

public:
    ModularRing(float v, float m) : value(fmod(v, m)), modulus(m) {
        if (value < 0) value += modulus;
    }

    ModularRing operator+(const ModularRing& other) const {
        return ModularRing(fmod(value + other.value, modulus), modulus);
    }

    ModularRing operator-(const ModularRing& other) const {
        return ModularRing(fmod(value - other.value + modulus, modulus), modulus);
    }

    ModularRing operator*(float scalar) const {
        return ModularRing(fmod(value * scalar, modulus), modulus);
    }

    float toRadians() const { return value * DEG_TO_RAD; }
    float toDegrees() const { return value; }
};

// Helper function for computing polynomials with modular ring properties
ModularRing mean_longitude(float T) {
    return ModularRing(280.46646 + 36000.76983 * T + 0.0003032 * T * T, 360.0);
}

ModularRing mean_anomaly(float T) {
    return ModularRing(357.52911 + 35999.05029 * T - 0.0001537 * T * T, 360.0);
}

float eccentricity(float T) {
    return 0.016708634 - 0.000042037 * T - 0.0000001267 * T * T;
}

ModularRing obliquity_of_ecliptic(float T) {
    return ModularRing(23.439291 - 0.013004167 * T - 0.000000164 * T * T + 0.0000005036 * T * T * T, 360.0);
}

// Trigonometric cache optimized with lazy evaluations for specific angle values
class LazyTrigCache {
private:
    mutable float sin_val, cos_val;
    mutable bool cached;
    ModularRing angle;

public:
    LazyTrigCache(ModularRing a) : angle(a), cached(false) {}

    float sin() const {
        if (!cached) { sin_val = std::sin(angle.toRadians()); cos_val = std::cos(angle.toRadians()); cached = true; }
        return sin_val;
    }

    float cos() const {
        if (!cached) { sin_val = std::sin(angle.toRadians()); cos_val = std::cos(angle.toRadians()); cached = true; }
        return cos_val;
    }
};

// Refactored function for RA and Declination to reduce redundant computations
void compute_ra_decl(const LazyTrigCache& L_app, const LazyTrigCache& Obl, float& RA, float& Decl) {
    RA = atan2(Obl.cos() * L_app.sin(), L_app.cos());
    Decl = asin(Obl.sin() * L_app.sin());
}

// Enhanced solar position calculation with ring-aligned modular arithmetic
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    long JDate = JulianDate(year, month, day);
    float JD_frac = ((hour + (minute / 60.0) + (second / 3600.0)) / 24.0) - 0.5;
    float JD = JDate + JD_frac - Timezone / 24.0;
    float T = (JD - 2451545.0) / 36525.0;

    // Modular ring-based trigonometric calculations for core values
    ModularRing L0 = mean_longitude(T);
    ModularRing M = mean_anomaly(T);
    float e = eccentricity(T);
    ModularRing Obl = obliquity_of_ecliptic(T);

    // Use LazyTrigCache for optimized trigonometric evaluations
    LazyTrigCache sinM(M);
    LazyTrigCache sin2M(M * 2.0f);
    LazyTrigCache sin3M(M * 3.0f);
    ModularRing C = ModularRing((1.914602 - 0.004817 * T - 0.000014 * T * T) * sinM.sin() +
                                (0.019993 - 0.000101 * T) * sin2M.sin() +
                                0.000289 * sin3M.sin(), 360.0);

    // Compute L_true and f, ensuring modularity within the ring
    ModularRing L_true = L0 + C;
    ModularRing f = M + C;

    // Radius vector computed with rational optimization and modular consistency
    float radius = 1.000001018 * (1 - e * e) / (1 + e * cos(f.toRadians()));

    // Apparent longitude calculation within modular ring constraints
    ModularRing L_apparent = L_true - ModularRing(0.00569 + 0.00478 * sin(ModularRing(125.04 - 1934.136 * T, 360.0).toRadians()), 360.0);

    // RA and Declination computed with modular field handling
    LazyTrigCache L_app_cache(L_apparent);
    LazyTrigCache Obl_cache(Obl);
    float RA, Decl;
    compute_ra_decl(L_app_cache, Obl_cache, RA, Decl);

    // Greenwich Hour Angle within the modular ring for precision
    ModularRing GrHrAngle = ModularRing(280.46061837 + 360.98564736629 * JD_frac, 360.0);
    ModularRing HrAngle = GrHrAngle + ModularRing(Lon - RA * RAD_TO_DEG, 360.0);

    // Final elevation and azimuth using modular and cached trigonometry
    float elevation = asin(sin(Lat * DEG_TO_RAD) * sin(Decl) + cos(Lat * DEG_TO_RAD) * cos(Decl) * cos(HrAngle.toRadians()));
    LazyTrigCache HrAngle_cache(HrAngle);
    ModularRing azimuth = ModularRing(M_PI + atan2(HrAngle_cache.sin(), HrAngle_cache.cos() * sin(Lat * DEG_TO_RAD) - tan(Decl) * cos(Lat * DEG_TO_RAD)), 360.0);

    // Output results in degrees
    sun_azimuth = azimuth.toDegrees();
    sun_elevation = elevation * RAD_TO_DEG;
}
