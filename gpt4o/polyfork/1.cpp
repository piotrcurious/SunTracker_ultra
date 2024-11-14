#include <cmath>
#include <vector>

// Polynomial class for symbolic manipulation in terms of T (time)
class Polynomial {
public:
    std::vector<float> coefficients;  // Coefficients for terms [c0, c1*T, c2*T^2, ...]

    Polynomial(std::initializer_list<float> coeffs) : coefficients(coeffs) {}

    // Polynomial addition
    Polynomial operator+(const Polynomial& other) const {
        std::vector<float> result_coeffs(std::max(coefficients.size(), other.coefficients.size()), 0.0);
        for (size_t i = 0; i < coefficients.size(); ++i) result_coeffs[i] += coefficients[i];
        for (size_t i = 0; i < other.coefficients.size(); ++i) result_coeffs[i] += other.coefficients[i];
        return Polynomial(result_coeffs);
    }

    // Polynomial multiplication by scalar
    Polynomial operator*(float scalar) const {
        std::vector<float> result_coeffs(coefficients.size());
        for (size_t i = 0; i < coefficients.size(); ++i) result_coeffs[i] = coefficients[i] * scalar;
        return Polynomial(result_coeffs);
    }

    // Evaluate the polynomial at a specific value of T
    float evaluate(float T) const {
        float result = 0.0;
        float term = 1.0;
        for (float coeff : coefficients) {
            result += coeff * term;
            term *= T;
        }
        return result;
    }
};

// Polynomial Field class to handle modular arithmetic in GF(360)
class GaloisField360 {
public:
    Polynomial value;

    GaloisField360(const Polynomial& v) : value(v) {}

    // Addition within the field, returning modulo 360 polynomial
    GaloisField360 operator+(const GaloisField360& other) const {
        Polynomial sum = value + other.value;
        return GaloisField360(reduceMod360(sum));
    }

    // Conversion of polynomial to radians
    float toRadians(float T) const {
        return reduceMod360(value).evaluate(T) * PI / 180.0;
    }

    // Polynomial modulus reduction to 360 degrees
    static Polynomial reduceMod360(const Polynomial& poly) {
        std::vector<float> mod_coeffs = poly.coefficients;
        for (float& coeff : mod_coeffs) coeff = fmod(coeff, 360.0);
        return Polynomial(mod_coeffs);
    }
};

// Helper functions return Polynomial objects for each parameter
Polynomial mean_longitude() {
    return Polynomial({280.46646, 36000.76983, 0.0003032});
}

Polynomial mean_anomaly() {
    return Polynomial({357.52911, 35999.05029, -0.0001537});
}

Polynomial eccentricity() {
    return Polynomial({0.016708634, -0.000042037, -0.0000001267});
}

Polynomial obliquity_of_ecliptic() {
    return Polynomial({23.439291, -0.013004167, -0.000000164, 0.0000005036});
}

// Trigonometric helper to compute Right Ascension and Declination
void compute_ra_decl(const GaloisField360& L_apparent, const GaloisField360& Obl, float T, float &RA, float &Decl) {
    float sin_L = sin(L_apparent.toRadians(T));
    float cos_L = cos(L_apparent.toRadians(T));
    RA = atan2(cos(Obl.toRadians(T)) * sin_L, cos_L);
    Decl = asin(sin(Obl.toRadians(T)) * sin_L);
}

// Main function to calculate the Sun's position using polynomial forms
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    long JDate = JulianDate(year, month, day);
    float JD_frac = ((hour + (minute / 60.0) + (second / 3600.0)) / 24.0) - 0.5;
    float JD = JDate + JD_frac - Timezone / 24.0;
    float T = (JD - 2451545.0) / 36525.0;

    // GaloisField360 applied for angular parameters with polynomial types
    GaloisField360 L0 = mean_longitude();
    GaloisField360 M = mean_anomaly();
    GaloisField360 e = eccentricity();
    GaloisField360 Obl = obliquity_of_ecliptic();

    // Equation of center using GaloisField360 polynomial representation
    float sin_M = sin(M.toRadians(T));
    float sin_2M = sin((M * 2.0f).toRadians(T));
    float sin_3M = sin((M * 3.0f).toRadians(T));
    GaloisField360 C(Polynomial({1.914602, -0.004817, -0.000014}) * sin_M +
                     Polynomial({0.019993, -0.000101}) * sin_2M +
                     Polynomial({0.000289}) * sin_3M);

    GaloisField360 L_true = L0 + C;
    GaloisField360 f = M + C;

    // Corrected radius vector calculation with polynomial eccentricity
    float radius = 1.000001018 * (1 - e.evaluate(T) * e.evaluate(T)) / (1 + e.evaluate(T) * cos(f.toRadians(T)));

    // Apparent longitude (modulo adjusted) and RA/Declination
    GaloisField360 L_apparent = L_true - GaloisField360(Polynomial({0.00569, 0.00478 * sin(GaloisField360(Polynomial({125.04, -1934.136}).evaluate(T)).toRadians(T))}));
    float RA, Decl;
    compute_ra_decl(L_apparent, Obl, T, RA, Decl);

    // Greenwich Hour Angle and Local Hour Angle
    GaloisField360 GrHrAngle(Polynomial({280.46061837, 360.98564736629}) * JD_frac);
    GaloisField360 HrAngle = GrHrAngle + GaloisField360(Lon) - GaloisField360(Polynomial({RA * RAD_TO_DEG}));

    // Elevation and Azimuth calculations
    float elevation = asin(sin(Lat * PI / 180.0) * sin(Decl) + cos(Lat * PI / 180.0) * cos(Decl) * cos(HrAngle.toRadians(T)));
    GaloisField360 azimuth(PI + atan2(sin(HrAngle.toRadians(T)), cos(HrAngle.toRadians(T)) * sin(Lat * PI / 180.0) - tan(Decl) * cos(Lat * PI / 180.0)));

    // Output sun azimuth and elevation in degrees
    sun_azimuth = azimuth.toDegrees();
    sun_elevation = elevation * RAD_TO_DEG;
}
