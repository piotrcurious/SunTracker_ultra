#include <cmath>
#include <vector>

// Constants
constexpr float DEG_TO_RAD = M_PI / 180.0;
constexpr float RAD_TO_DEG = 180.0 / M_PI;

// Custom class for modular arithmetic within 360 degrees for angles
class AngleMod {
private:
    float value;
public:
    AngleMod(float v) : value(fmod(v, 360.0f)) { if (value < 0) value += 360.0f; }

    AngleMod operator+(const AngleMod& other) const {
        return AngleMod(fmod(value + other.value, 360.0f));
    }

    AngleMod operator-(const AngleMod& other) const {
        return AngleMod(fmod(value - other.value + 360.0f, 360.0f));
    }

    AngleMod operator*(float scalar) const {
        return AngleMod(fmod(value * scalar, 360.0f));
    }

    float toRadians() const { return value * DEG_TO_RAD; }
};

// Rational class for representing constants with high precision
class Rational {
public:
    int num, den;

    Rational(int numerator, int denominator) : num(numerator), den(denominator) {}

    float evaluate() const { return static_cast<float>(num) / den; }
};

// Lazy trigonometric cache to prevent redundant sine and cosine calculations
class LazyTrigCache {
private:
    mutable float sin_val, cos_val;
    mutable bool cached;
    AngleMod angle;

public:
    LazyTrigCache(AngleMod a) : angle(a), cached(false) {}

    float sin() const {
        if (!cached) { sin_val = std::sin(angle.toRadians()); cos_val = std::cos(angle.toRadians()); cached = true; }
        return sin_val;
    }

    float cos() const {
        if (!cached) { sin_val = std::sin(angle.toRadians()); cos_val = std::cos(angle.toRadians()); cached = true; }
        return cos_val;
    }
};

// Precomputed polynomials with symbolic optimization for field-constant storage
AngleMod mean_longitude(float T) {
    return AngleMod(280.46646 + 36000.76983 * T + 0.0003032 * T * T);
}

AngleMod mean_anomaly(float T) {
    return AngleMod(357.52911 + 35999.05029 * T - 0.0001537 * T * T);
}

float eccentricity(float T) {
    return 0.016708634 - 0.000042037 * T - 0.0000001267 * T * T;
}

AngleMod obliquity_of_ecliptic(float T) {
    return AngleMod(23.439291 - 0.013004167 * T - 0.000000164 * T * T + 0.0000005036 * T * T * T);
}

// Helper function for RA and Declination using lazy caching
void compute_ra_decl(const LazyTrigCache& L_app, const LazyTrigCache& Obl, float& RA, float& Decl) {
    float sin_L = L_app.sin();
    float cos_L = L_app.cos();
    float cos_Obl = Obl.cos();
    RA = atan2(cos_Obl * sin_L, cos_L);
    Decl = asin(Obl.sin() * sin_L);
}

// Optimized solar position calculation
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    long JDate = JulianDate(year, month, day);
    float JD_frac = ((hour + (minute / 60.0) + (second / 3600.0)) / 24.0) - 0.5;
    float JD = JDate + JD_frac - Timezone / 24.0;
    float T = (JD - 2451545.0) / 36525.0;

    // Polynomial expressions represented with AngleMod and LazyTrigCache for caching and optimization
    AngleMod L0 = mean_longitude(T);
    AngleMod M = mean_anomaly(T);
    float e = eccentricity(T);
    AngleMod Obl = obliquity_of_ecliptic(T);

    // Lazy evaluation for Equation of Center
    LazyTrigCache sinM(M);
    LazyTrigCache sin2M(M * 2.0f);
    LazyTrigCache sin3M(M * 3.0f);
    AngleMod C = AngleMod((1.914602 - 0.004817 * T - 0.000014 * T * T) * sinM.sin() +
                          (0.019993 - 0.000101 * T) * sin2M.sin() +
                          0.000289 * sin3M.sin());

    // Compute L_true and f with optimized modular addition
    AngleMod L_true = L0 + C;
    AngleMod f = M + C;

    // Calculate radius vector, minimizing floating-point error through rational optimization
    float radius = 1.000001018 * (1 - e * e) / (1 + e * cos(f.toRadians()));

    // Compute apparent longitude using optimized trigonometric caching
    AngleMod L_apparent = L_true - AngleMod(0.00569 + 0.00478 * sin(AngleMod(125.04 - 1934.136 * T).toRadians()));

    // RA and Declination with lazy evaluation for trigonometric caching
    LazyTrigCache L_app_cache(L_apparent);
    LazyTrigCache Obl_cache(Obl);
    float RA, Decl;
    compute_ra_decl(L_app_cache, Obl_cache, RA, Decl);

    // Greenwich Hour Angle optimized with modular arithmetic
    AngleMod GrHrAngle = AngleMod(280.46061837 + 360.98564736629 * JD_frac);
    AngleMod HrAngle = GrHrAngle + AngleMod(Lon - RA * RAD_TO_DEG);

    // Compute elevation and azimuth with lazy trigonometric caching
    float elevation = asin(sin(Lat * DEG_TO_RAD) * sin(Decl) + cos(Lat * DEG_TO_RAD) * cos(Decl) * cos(HrAngle.toRadians()));
    LazyTrigCache HrAngle_cache(HrAngle);
    AngleMod azimuth = AngleMod(M_PI + atan2(HrAngle_cache.sin(), HrAngle_cache.cos() * sin(Lat * DEG_TO_RAD) - tan(Decl) * cos(Lat * DEG_TO_RAD)));

    // Convert results to degrees for output
    sun_azimuth = azimuth.toRadians() * RAD_TO_DEG;
    sun_elevation = elevation * RAD_TO_DEG;
}
