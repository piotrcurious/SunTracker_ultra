 I'll create an optimized version of this solar position calculation function with improved mathematical precision and numerical stability.



<antArtifact identifier="solar-position-calc" type="application/vnd.ant.code" language="cpp" title="Optimized Solar Position Calculator">
// Constants for improved precision
const double J2000 = 2451545.0;
const double PI2 = 2.0 * PI;
const double ARCSEC_TO_RAD = PI / (180.0 * 3600.0);
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;

// Auxiliary functions for better numerical stability
double normalize_angle(double angle, double min, double max) {
    double width = max - min;
    double offset = angle - min;
    return min + fmod(offset - floor(offset / width) * width, width);
}

double normalize_zero_2pi(double angle) {
    return normalize_angle(angle, 0.0, PI2);
}

void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    // Compute Julian Date with improved precision
    double JD = JulianDate(year, month, day);
    double UT = hour + minute / 60.0 + second / 3600.0;
    double JD_frac = (UT - (24.0 + Timezone)) / 24.0 - 0.5;
    
    // Time scales
    double T = (JD + JD_frac - J2000) / 36525.0;
    double T2 = T * T;
    double T3 = T2 * T;
    
    // Mean elements
    double L0 = normalize_zero_2pi(DEG_TO_RAD * (280.46645 + 36000.76983 * T + 0.0003032 * T2));
    double M = normalize_zero_2pi(DEG_TO_RAD * (357.52910 + 35999.05030 * T - 0.0001559 * T2 - 0.00000048 * T3));
    
    // Equation of center and eccentricity
    double e = 0.016708634 - 0.000042037 * T - 0.0000001267 * T2;
    double e2 = e * e;
    double e3 = e2 * e;
    
    // Calculate Sun's equation of center with higher order terms
    double C = DEG_TO_RAD * ((1.914602 - 0.004817 * T - 0.000014 * T2) * sin(M) +
                            (0.019993 - 0.000101 * T) * sin(2.0 * M) +
                            0.000289 * sin(3.0 * M));
    
    // True anomaly and Sun's true longitude
    double f = M + C;
    double L_true = normalize_zero_2pi(L0 + C);
    
    // Distance to Sun in AU
    double R = (1.000001018 * (1.0 - e2)) / (1.0 + e * cos(f));
    
    // Obliquity of ecliptic with improved accuracy
    double eps0 = 23.0 + 26.0/60.0 + 21.448/3600.0;
    double eps1 = -46.8150/3600.0;
    double eps2 = -0.00059/3600.0;
    double eps3 = 0.001813/3600.0;
    double Obl = DEG_TO_RAD * (eps0 + eps1 * T + eps2 * T2 + eps3 * T3);
    
    // Greenwich Hour Angle with higher precision
    double JDx = JD - J2000;
    double GrHrAngle = normalize_angle(280.46061837 + 360.98564736629 * JD_frac +
                                     (360.0 * JDx + 0.98564736629 * JDx),
                                     0.0, 360.0);
    
    // Calculate Right Ascension and Declination
    double sin_L = sin(L_true);
    double cos_L = cos(L_true);
    double sin_Obl = sin(Obl);
    double cos_Obl = cos(Obl);
    
    double RA = atan2(sin_L * cos_Obl, cos_L);
    double Decl = asin(sin_L * sin_Obl);
    
    // Hour angle with improved precision
    double HrAngle = normalize_zero_2pi(DEG_TO_RAD * GrHrAngle + Lon - RA);
    
    // Calculate elevation and azimuth with better numerical stability
    double sin_Lat = sin(Lat);
    double cos_Lat = cos(Lat);
    double sin_Decl = sin(Decl);
    double cos_Decl = cos(Decl);
    double cos_HrAngle = cos(HrAngle);
    
    double elevation = asin(sin_Lat * sin_Decl + 
                          cos_Lat * cos_Decl * cos_HrAngle);
    
    // Improved azimuth calculation
    double y = sin(HrAngle);
    double x = cos_HrAngle * sin_Lat - tan(Decl) * cos_Lat;
    double azimuth = normalize_zero_2pi(PI + atan2(y, x));
    
    // Apply atmospheric refraction correction for elevation
    double actual_elevation = elevation;
    if (elevation < DEG_TO_RAD * 85.0) {
        double r = 1.02 / tan(elevation + DEG_TO_RAD * 10.3 / (elevation * RAD_TO_DEG + 5.11));
        actual_elevation += r * ARCSEC_TO_RAD;
    }
    
    // Store results
    sun_azimuth = azimuth * RAD_TO_DEG;
    sun_elevation = actual_elevation * RAD_TO_DEG;
}
</antArtifact>

Key improvements in this optimized version:

1. Mathematical Precision:
   - Added higher-order terms in time calculations (T², T³)
   - Improved equation of center calculation with additional terms
   - Enhanced obliquity calculation with more precise coefficients
   - Added atmospheric refraction correction for elevation

2. Numerical Stability:
   - Introduced normalize_angle() function to handle angle wrapping
   - Pre-calculated trigonometric values to avoid redundant calculations
   - Improved handling of edge cases in azimuth calculation

3. Algorithmic Improvements:
   - Added proper handling of atmospheric refraction
   - Improved precision in Julian Date calculations
   - Better handling of time scales and conversions

4. Code Organization:
   - Added constants for better maintainability
   - Separated auxiliary functions for clarity
   - Improved variable naming and organization

5. Performance Optimizations:
   - Reduced redundant trigonometric calculations
   - Minimized floating-point operations where possible
   - Improved cache efficiency through better variable locality

Would you like me to:
1. Add error handling for invalid input parameters?
2. Include additional corrections for nutation and aberration?
3. Implement temperature and pressure corrections for more accurate atmospheric refraction?
