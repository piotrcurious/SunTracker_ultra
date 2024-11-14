#include <cmath>
#include <vector>

class Polynomial {
public:
    std::vector<float> coefficients;

    Polynomial(std::initializer_list<float> coeffs) : coefficients(coeffs) {}

    Polynomial operator+(const Polynomial& other) const {
        size_t max_size = std::max(coefficients.size(), other.coefficients.size());
        std::vector<float> result(max_size, 0.0f);
        for (size_t i = 0; i < coefficients.size(); ++i) result[i] += coefficients[i];
        for (size_t i = 0; i < other.coefficients.size(); ++i) result[i] += other.coefficients[i];
        return Polynomial(result);
    }

    Polynomial operator*(float scalar) const {
        std::vector<float> result(coefficients.size());
        for (size_t i = 0; i < coefficients.size(); ++i) result[i] = coefficients[i] * scalar;
        return Polynomial(result);
    }

    Polynomial mod360() const {
        std::vector<float> mod_coeffs = coefficients;
        for (float& coeff : mod_coeffs) coeff = fmod(coeff, 360.0f);
        return Polynomial(mod_coeffs);
    }

    float evaluate(float T) const {
        float result = 0.0f;
        float term = 1.0f;
        for (float coeff : coefficients) {
            result += coeff * term;
            term *= T;
        }
        return result;
    }
};

class GaloisFieldPoly {
public:
    Polynomial poly;

    GaloisFieldPoly(const Polynomial& p) : poly(p) {}

    GaloisFieldPoly operator+(const GaloisFieldPoly& other) const {
        return GaloisFieldPoly((poly + other.poly).mod360());
    }

    GaloisFieldPoly operator*(float scalar) const {
        return GaloisFieldPoly((poly * scalar).mod360());
    }

    float toRadians(float T) const {
        return poly.mod360().evaluate(T) * M_PI / 180.0;
    }
};

// Define optimized polynomial helper functions
Polynomial mean_longitude() { return Polynomial({280.46646, 36000.76983, 0.0003032}); }
Polynomial mean_anomaly() { return Polynomial({357.52911, 35999.05029, -0.0001537}); }
Polynomial eccentricity() { return Polynomial({0.016708634, -0.000042037, -0.0000001267}); }
Polynomial obliquity_of_ecliptic() { return Polynomial({23.439291, -0.013004167, -0.000000164, 0.0000005036}); }

// Optimized Right Ascension and Declination calculation
void compute_ra_decl(const GaloisFieldPoly& L_app, const GaloisFieldPoly& Obl, float T, float& RA, float& Decl) {
    float sin_L = sin(L_app.toRadians(T));
    float cos_L = cos(L_app.toRadians(T));
    RA = atan2(cos(Obl.toRadians(T)) * sin_L, cos_L);
    Decl = asin(sin(Obl.toRadians(T)) * sin_L);
}

// Main function to calculate Sun position
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    long JDate = JulianDate(year, month, day);
    float JD_frac = ((hour + (minute / 60.0) + (second / 3600.0)) / 24.0) - 0.5;
    float JD = JDate + JD_frac - Timezone / 24.0;
    float T = (JD - 2451545.0) / 36525.0;

    // Initialize fields using polynomials
    GaloisFieldPoly L0 = mean_longitude();
    GaloisFieldPoly M = mean_anomaly();
    GaloisFieldPoly e = eccentricity();
    GaloisFieldPoly Obl = obliquity_of_ecliptic();

    // Equation of center with reduced trig calculations
    float sin_M = sin(M.toRadians(T));
    float sin_2M = sin((M * 2.0f).toRadians(T));
    float sin_3M = sin((M * 3.0f).toRadians(T));
    GaloisFieldPoly C = GaloisFieldPoly(Polynomial({1.914602, -0.004817, -0.000014}) * sin_M +
                                        Polynomial({0.019993, -0.000101}) * sin_2M +
                                        Polynomial({0.000289}) * sin_3M);

    GaloisFieldPoly L_true = L0 + C;
    GaloisFieldPoly f = M + C;

    // Radius vector with direct polynomial evaluation
    float radius = 1.000001018 * (1 - e.evaluate(T) * e.evaluate(T)) / (1 + e.evaluate(T) * cos(f.toRadians(T)));

    // Apparent longitude and RA/Declination calculation
    GaloisFieldPoly L_apparent = L_true - GaloisFieldPoly(Polynomial({0.00569, 0.00478 * sin(GaloisFieldPoly(Polynomial({125.04, -1934.136}).evaluate(T)).toRadians(T))}));
    float RA, Decl;
    compute_ra_decl(L_apparent, Obl, T, RA, Decl);

    // Hour Angle calculations with harmonized field
    GaloisFieldPoly GrHrAngle = GaloisFieldPoly(Polynomial({280.46061837, 360.98564736629}) * JD_frac);
    GaloisFieldPoly HrAngle = GrHrAngle + GaloisFieldPoly(Polynomial({Lon - RA * RAD_TO_DEG}));

    // Calculate elevation and azimuth
    float elevation = asin(sin(Lat * M_PI / 180.0) * sin(Decl) + cos(Lat * M_PI / 180.0) * cos(Decl) * cos(HrAngle.toRadians(T)));
    GaloisFieldPoly azimuth = GaloisFieldPoly(M_PI + atan2(sin(HrAngle.toRadians(T)), cos(HrAngle.toRadians(T)) * sin(Lat * M_PI / 180.0) - tan(Decl) * cos(Lat * M_PI / 180.0)));

    // Output in degrees
    sun_azimuth = azimuth.toRadians(T) * RAD_TO_DEG;
    sun_elevation = elevation * RAD_TO_DEG;
}
