// Constants for field operations and angle conversions
constexpr float GF_MODULUS = 360.0;      // Modulus for Galois Field operations, representing 360 degrees
constexpr float PI_OVER_180 = PI / 180.0;
constexpr float RAD_TO_DEG = 180.0 / PI;

// Function for modular reduction within the Galois Field (GF(360))
float gf_mod(float value, float modulus = GF_MODULUS) {
    // Brings the value within the range [0, modulus)
    return fmod(fmod(value, modulus) + modulus, modulus);
}

// Elliptic addition within the Galois Field
// Useful for controlled summing of angles, maintaining cyclic properties
float elliptic_add(float a, float b, float modulus = GF_MODULUS) {
    return gf_mod(a + b, modulus);
}

// Elliptic subtraction within the Galois Field
// Controls angle subtraction, wrapping to maintain range consistency
float elliptic_sub(float a, float b, float modulus = GF_MODULUS) {
    return gf_mod(a - b, modulus);
}

// Calculate Sun Position using customized Galois Field and elliptic curve methods
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    // Julian Date calculation with precise fractional time offset
    long JDate = JulianDate(year, month, day);
    float JD_frac = ((hour + (minute / 60.0) + (second / 3600.0)) / 24.0) - 0.5;
    float JD = JDate + JD_frac - Timezone / 24.0;

    // Calculate centuries since J2000.0 (T)
    float T = (JD - 2451545.0) / 36525.0;

    // Mean longitude (L0) and mean anomaly (M) using modular reduction for higher precision
    float L0 = gf_mod(280.46646 + 36000.76983 * T + 0.0003032 * T * T);
    float M = gf_mod(357.52911 + 35999.05029 * T - 0.0001537 * T * T);

    // Earth's orbit eccentricity (e)
    float e = 0.016708634 - 0.000042037 * T - 0.0000001267 * T * T;

    // Equation of center (C), computed using elliptic addition for controlled cyclic summation
    float C = elliptic_add(
        (1.914602 - 0.004817 * T - 0.000014 * T * T) * sin(M * PI_OVER_180),
        elliptic_add(
            (0.019993 - 0.000101 * T) * sin(2 * M * PI_OVER_180),
            0.000289 * sin(3 * M * PI_OVER_180)
        )
    );

    // True longitude (L_true) and true anomaly (f) in the Galois Field
    float L_true = gf_mod(L0 + C);
    float f = gf_mod(M + C);

    // Obliquity of the ecliptic (Obl) within the Galois Field
    float Obl = gf_mod(23.439291 - 0.013004167 * T - 0.000000164 * T * T + 0.0000005036 * T * T * T);

    // Radius vector (R) using the corrected eccentricity
    float R = 1.000001018 * (1 - e * e) / (1 + e * cos(f * PI_OVER_180));

    // Apparent longitude (L_apparent) including aberration correction, reduced modulo 360
    float L_apparent = gf_mod(L_true - 0.00569 - 0.00478 * sin((125.04 - 1934.136 * T) * PI_OVER_180));

    // Right Ascension (RA) and Declination (Decl) using modular trigonometric functions
    float RA = atan2(cos(Obl * PI_OVER_180) * sin(L_apparent * PI_OVER_180), cos(L_apparent * PI_OVER_180));
    float Decl = asin(sin(Obl * PI_OVER_180) * sin(L_apparent * PI_OVER_180));

    // Greenwich Hour Angle (GrHrAngle) modular reduction
    float JDx = JDate - 2451545;
    float GrHrAngle = gf_mod(280.46061837 + 360.98564736629 * JDx + 0.98564736629 * JD_frac);

    // Local Hour Angle (HrAngle), factoring in Galois Field adjustments
    float HrAngle = gf_mod(GrHrAngle + Lon - RA * RAD_TO_DEG);

    // Solar elevation (elevation) and azimuth (azimuth) calculations with modular wrapping
    float elevation = asin(sin(Lat * PI_OVER_180) * sin(Decl) + cos(Lat * PI_OVER_180) * cos(Decl) * cos(HrAngle * PI_OVER_180));
    float azimuth = gf_mod(PI + atan2(sin(HrAngle * PI_OVER_180), cos(HrAngle * PI_OVER_180) * sin(Lat * PI_OVER_180) - tan(Decl) * cos(Lat * PI_OVER_180)));

    // Convert elevation and azimuth back to degrees for readability
    sun_azimuth = gf_mod(azimuth * RAD_TO_DEG);
    sun_elevation = elevation * RAD_TO_DEG;
}
