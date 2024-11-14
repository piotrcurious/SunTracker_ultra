#include <cmath>

// Constants
constexpr int32_t DEG_TO_RAD_SCALED = 314159 / 180000;  // Fixed-point scale for DEG_TO_RAD
constexpr int32_t RAD_TO_DEG_SCALED = 180000 / 314159;  // Fixed-point scale for RAD_TO_DEG
constexpr int32_t SCALE = 100000;                       // Scaling factor for fixed-point arithmetic

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

    int32_t sin() const { return (int32_t)(value * DEG_TO_RAD_SCALED); }
    int32_t cos() const { return (int32_t)(value * DEG_TO_RAD_SCALED); }

    int32_t toDegrees() const { return value / SCALE; }
};

// Rational class for precise, scaled integer arithmetic
class Rational {
public:
    int32_t numerator;
    int32_t denominator;

    Rational(int32_t num, int32_t denom) : numerator(num), denominator(denom) {
        simplify();
    }

    int32_t toInt() const { return (numerator * SCALE) / denominator; }

    Rational operator*(const Rational& other) const {
        return Rational((numerator * other.numerator) / SCALE, (denominator * other.denominator) / SCALE);
    }

    Rational operator+(const Rational& other) const {
        int32_t new_num = (numerator * other.denominator + other.numerator * denominator) / SCALE;
        int32_t new_denom = denominator * other.denominator;
        return Rational(new_num, new_denom);
    }

    void simplify() {
        int32_t gcd_val = gcd(abs(numerator), abs(denominator));
        numerator /= gcd_val;
        denominator /= gcd_val;
    }

    static int32_t gcd(int32_t a, int32_t b) {
        while (b != 0) {
            int32_t t = b;
            b = a % b;
            a = t;
        }
        return a;
    }
};

// Fixed-point Taylor series approximation for sine
int32_t taylor_sin(int32_t x) {
    int32_t x2 = (x * x) / SCALE;
    return x - (x * x2 / 6) + (x2 * x2 / 120);
}

// Fixed-point Taylor series approximation for cosine
int32_t taylor_cos(int32_t x) {
    int32_t x2 = (x * x) / SCALE;
    return SCALE - (x2 / 2) + (x2 * x2 / 24);
}

// Lazy trigonometric evaluation cache with integer-based Taylor series
class LazyTrigCache {
private:
    mutable int32_t sin_val, cos_val;
    mutable bool cached;
    ModularRing angle;

public:
    LazyTrigCache(const ModularRing& a) : angle(a), cached(false) {}

    int32_t sin() const {
        if (!cached) {
            int32_t radians = angle.toDegrees() * DEG_TO_RAD_SCALED;
            sin_val = taylor_sin(radians);
            cos_val = taylor_cos(radians);
            cached = true;
        }
        return sin_val;
    }

    int32_t cos() const {
        if (!cached) {
            int32_t radians = angle.toDegrees() * DEG_TO_RAD_SCALED;
            sin_val = taylor_sin(radians);
            cos_val = taylor_cos(radians);
            cached = true;
        }
        return cos_val;
    }
};

// Calculate high-precision mean longitude using integer scaled values
Rational mean_longitude(const Rational& T) {
    return Rational(28046645 + 3600076983 * T.numerator / T.denominator, SCALE);
}

// Solar position calculation using modular and rational arithmetic in fixed-point
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    int32_t JDate = JulianDate(year, month, day);
    int32_t JD_frac = ((hour * 3600 + minute * 60 + second) * SCALE) / 86400;
    int32_t JD = JDate * SCALE + JD_frac - Timezone * SCALE / 24;
    int32_t T = (JD - 2451545 * SCALE) / (36525 * SCALE);

    Rational T_rational(T, SCALE);
    ModularRing L0 = ModularRing(mean_longitude(T_rational).toInt(), 360 * SCALE);
    ModularRing M = ModularRing(mean_longitude(T_rational).toInt(), 360 * SCALE);

    LazyTrigCache sinM(M);
    LazyTrigCache sin2M(M * 2);
    LazyTrigCache sin3M(M * 3);

    ModularRing C = ModularRing((1914602 - 4817 * T - 14 * T * T) * sinM.sin() +
                                (19993 - 101 * T) * sin2M.sin() +
                                289 * sin3M.sin(), 360 * SCALE);

    ModularRing L_true = L0 + C;
    ModularRing f = M + C;

    // Trigonometric terms for apparent longitude
    LazyTrigCache L_app_cache(L_true);
    int32_t RA = atan2(taylor_cos(L_app_cache.sin()), taylor_cos(L_app_cache.cos()));
    int32_t Decl = asin(taylor_cos(L_app_cache.sin()));

    ModularRing GrHrAngle = ModularRing(280460 + 3609856 * JD_frac / SCALE, 360 * SCALE);
    ModularRing HrAngle = GrHrAngle + ModularRing(Lon - RA * RAD_TO_DEG_SCALED, 360 * SCALE);

    int32_t elevation = asin((sin(Lat * DEG_TO_RAD_SCALED) * sin(Decl) + cos(Lat * DEG_TO_RAD_SCALED) * cos(Decl) * taylor_cos(HrAngle.toDegrees())));
    int32_t azimuth = ModularRing(M_PI * SCALE + atan2(taylor_sin(HrAngle.toDegrees()), taylor_cos(HrAngle.toDegrees())), 360 * SCALE).toDegrees();

    sun_azimuth = azimuth / SCALE;
    sun_elevation = elevation / SCALE;
}
