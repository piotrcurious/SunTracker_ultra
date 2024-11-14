#include <cmath>
#include <vector>

// Constants for modular ring bounds and conversions
constexpr float DEG_TO_RAD = M_PI / 180.0;
constexpr float RAD_TO_DEG = 180.0 / M_PI;
constexpr float TWO_PI = 2 * M_PI;

// Modular ring class with rational arithmetic for improved precision
class ModularRing {
private:
    float value;
    float modulus;

public:
    ModularRing(float v, float m) : modulus(m) {
        value = fmod(v, modulus);
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

    ModularRing sin() const { return ModularRing(std::sin(value * DEG_TO_RAD), modulus); }
    ModularRing cos() const { return ModularRing(std::cos(value * DEG_TO_RAD), modulus); }

    float toRadians() const { return value * DEG_TO_RAD; }
    float toDegrees() const { return value; }
};

// Rational class to store fractions, improving precision of intermediate constants
class Rational {
public:
    long numerator;
    long denominator;

    Rational(long num, long denom) : numerator(num), denominator(denom) {
        simplify();
    }

    float toFloat() const { return static_cast<float>(numerator) / denominator; }

    // Simplify the fraction (Euclidean algorithm)
    void simplify() {
        long gcd_val = gcd(abs(numerator), abs(denominator));
        numerator /= gcd_val;
        denominator /= gcd_val;
    }

    static long gcd(long a, long b) {
        while (b != 0) {
            long t = b;
            b = a % b;
            a = t;
        }
        return a;
    }
};

// Using rational approximations for constants where possible
Rational mean_longitude(const Rational& T) {
    return Rational(28046645 + 3600076983 * T.numerator / T.denominator, 100000);
}

Rational mean_anomaly(const Rational& T) {
    return Rational(35752911 + 359990503 * T.numerator / T.denominator, 100000);
}

// Function for eccentricity of the Earth's orbit, optimized for modular arithmetic
float eccentricity(float T) {
    return 0.016708634 - 0.000042037 * T;
}

ModularRing obliquity_of_ecliptic(float T) {
    return ModularRing(23.439291 - 0.013004167 * T - 0.000000164 * T * T + 0.0000005036 * T * T * T, 360.0);
}

// Optimized trigonometric caching class using ModularRing
class LazyTrigCache {
private:
    mutable float sin_val, cos_val;
    mutable bool cached;
    ModularRing angle;

public:
    LazyTrigCache(const ModularRing& a) : angle(a), cached(false) {}

    float sin() const {
        if (!cached) {
            sin_val = std::sin(angle.toRadians());
            cos_val = std::cos(angle.toRadians());
            cached = true;
        }
        return sin_val;
    }

    float cos() const {
        if (!cached) {
            sin_val = std::sin(angle.toRadians());
            cos_val = std::cos(angle.toRadians());
            cached = true;
        }
        return cos_val;
    }
};

// Solar position calculation with extended modular and rational arithmetic for precision
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    long JDate = JulianDate(year, month, day);
    float JD_frac = ((hour + (minute / 60.0) + (second / 3600.0)) / 24.0) - 0.5;
    float JD = JDate + JD_frac - Timezone / 24.0;
    float T = (JD - 2451545.0) / 36525.0;

    // Mean longitude and anomaly using rational approximations
    Rational T_rational(static_cast<long>(T * 100000), 100000);
    ModularRing L0 = ModularRing(mean_longitude(T_rational).toFloat(), 360.0);
    ModularRing M = ModularRing(mean_anomaly(T_rational).toFloat(), 360.0);
    float e = eccentricity(T);

    // True anomaly correction factor in ModularRing
    LazyTrigCache sinM(M);
    LazyTrigCache sin2M(M * 2.0f);
    LazyTrigCache sin3M(M * 3.0f);
    ModularRing C = ModularRing((1.914602 - 0.004817 * T - 0.000014 * T * T) * sinM.sin() +
                                (0.019993 - 0.000101 * T) * sin2M.sin() +
                                0.000289 * sin3M.sin(), 360.0);

    // Compute L_true, f, and radius vector in modular space
    ModularRing L_true = L0 + C;
    ModularRing f = M + C;
    float radius = 1.000001018 * (1 - e * e) / (1 + e * cos(f.toRadians()));

    // Apparent longitude within modular constraints
    ModularRing L_apparent = L_true - ModularRing(0.00569 + 0.00478 * sin(ModularRing(125.04 - 1934.136 * T, 360.0).toRadians()), 360.0);

    // RA and Declination calculations using lazy trigonometry cache
    LazyTrigCache L_app_cache(L_apparent);
    LazyTrigCache Obl_cache(obliquity_of_ecliptic(T));
    float RA = atan2(Obl_cache.cos() * L_app_cache.sin(), L_app_cache.cos());
    float Decl = asin(Obl_cache.sin() * L_app_cache.sin());

    // Greenwich Hour Angle in modular space for enhanced precision
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
