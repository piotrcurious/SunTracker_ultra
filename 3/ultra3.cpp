// Constants for improved precision
const double J2000 = 2451545.0;
const double PI2 = 2.0 * PI;
const double ARCSEC_TO_RAD = PI / (180.0 * 3600.0);
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;
const double AU_TO_KM = 149597870.7;
const double C = 299792.458;

// Planetary orbital elements at J2000
struct PlanetElements {
    double L; // Mean longitude
    double a; // Semi-major axis
    double e; // Eccentricity
    double i; // Inclination
    double omega; // Longitude of ascending node
    double pi; // Longitude of perihelion
};

// Planetary perturbation coefficients
struct PerturbationTerm {
    double A;    // Amplitude
    int iV;      // Multiplier for Venus mean longitude
    int iE;      // Multiplier for Earth mean longitude
    int iM;      // Multiplier for Mars mean longitude
    int iJ;      // Multiplier for Jupiter mean longitude
    int iS;      // Multiplier for Saturn mean longitude
};

// Initialize planetary elements
PlanetElements getPlanetaryElements(double T) {
    PlanetElements venus = {
        normalize_zero_2pi(DEG_TO_RAD * (181.979801 + 58519.2130302 * T)), // L
        0.723332, // a
        0.006773 - 0.000049 * T, // e
        DEG_TO_RAD * (3.394662 + 0.0010037 * T), // i
        DEG_TO_RAD * (76.679920 + 0.9011206 * T), // omega
        DEG_TO_RAD * (131.563703 + 0.0048746 * T)  // pi
    };
    
    PlanetElements mars = {
        normalize_zero_2pi(DEG_TO_RAD * (355.433275 + 19140.2993313 * T)), // L
        1.523679, // a
        0.093405 + 0.000092 * T, // e
        DEG_TO_RAD * (1.849726 - 0.0006010 * T), // i
        DEG_TO_RAD * (49.558093 + 0.7720959 * T), // omega
        DEG_TO_RAD * (336.060234 + 0.0001847 * T)  // pi
    };
    
    PlanetElements jupiter = {
        normalize_zero_2pi(DEG_TO_RAD * (34.351484 + 3034.9056746 * T)), // L
        5.202561, // a
        0.048498 + 0.000163 * T, // e
        DEG_TO_RAD * (1.303270 - 0.0054966 * T), // i
        DEG_TO_RAD * (100.464441 + 1.0209550 * T), // omega
        DEG_TO_RAD * (14.331309 + 0.0164943 * T)  // pi
    };
    
    PlanetElements saturn = {
        normalize_zero_2pi(DEG_TO_RAD * (50.077471 + 1222.1137943 * T)), // L
        9.554747, // a
        0.055546 - 0.000346 * T, // e
        DEG_TO_RAD * (2.488878 - 0.0037363 * T), // i
        DEG_TO_RAD * (113.665524 + 0.8770979 * T), // omega
        DEG_TO_RAD * (93.056787 + 0.0565314 * T)  // pi
    };
    
    return venus; // Return Venus as primary perturber
}

// Calculate planetary perturbations
struct PlanetaryPerturbations {
    double deltaL; // Perturbation in longitude
    double deltaR; // Perturbation in radius vector
    double deltaB; // Perturbation in latitude
};

PlanetaryPerturbations calculatePlanetaryPerturbations(double T) {
    PlanetElements venus = getPlanetaryElements(T);
    
    // Mean longitudes of planets
    double L_Venus = venus.L;
    double L_Earth = normalize_zero_2pi(DEG_TO_RAD * (100.466449 + 35999.3728519 * T));
    double L_Mars = normalize_zero_2pi(DEG_TO_RAD * (355.433275 + 19140.2993313 * T));
    double L_Jupiter = normalize_zero_2pi(DEG_TO_RAD * (34.351484 + 3034.9056746 * T));
    double L_Saturn = normalize_zero_2pi(DEG_TO_RAD * (50.077471 + 1222.1137943 * T));
    
    PlanetaryPerturbations pert = {0.0, 0.0, 0.0};
    
    // Venus perturbations
    pert.deltaL += 0.00313 * sin(2 * (L_Venus - L_Earth));
    pert.deltaL += 0.00198 * sin((2 * L_Venus - 3 * L_Earth + 213.5 * DEG_TO_RAD));
    pert.deltaL += 0.00136 * sin((2 * L_Venus - 2 * L_Earth));
    
    // Mars perturbations
    pert.deltaL += 0.00114 * sin(L_Mars - 2 * L_Earth);
    pert.deltaL += 0.00069 * sin((2 * L_Mars - 3 * L_Earth));
    
    // Jupiter perturbations
    pert.deltaL += 0.00071 * sin((L_Jupiter - L_Earth));
    pert.deltaL += 0.00057 * sin((2 * L_Jupiter - 2 * L_Earth));
    
    // Saturn perturbations
    pert.deltaL += 0.00021 * sin((2 * L_Saturn - 2 * L_Earth));
    
    // Radius vector perturbations (in AU)
    pert.deltaR += 0.00000 * cos(2 * (L_Venus - L_Earth));
    pert.deltaR += -0.00014 * cos((2 * L_Venus - 3 * L_Earth + 213.5 * DEG_TO_RAD));
    pert.deltaR += 0.00010 * cos((2 * L_Venus - 2 * L_Earth));
    
    // Latitude perturbations
    pert.deltaB += 0.00027 * sin((2 * L_Venus - 3 * L_Earth + 213.5 * DEG_TO_RAD));
    pert.deltaB += 0.00016 * sin((L_Venus - L_Earth));
    
    return pert;
}

// Main calculation function
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    // Previous time calculations remain the same...
    double JD = JulianDate(year, month, day);
    double UT = hour + minute / 60.0 + second / 3600.0;
    double JD_frac = (UT - (24.0 + Timezone)) / 24.0 - 0.5;
    
    double T = (JD + JD_frac - J2000) / 36525.0;
    double T2 = T * T;
    double T3 = T2 * T;
    
    // Calculate nutation (previous implementation remains...)
    Nutation nut = calculate_nutation(T);
    
    // Calculate planetary perturbations
    PlanetaryPerturbations pert = calculatePlanetaryPerturbations(T);
    
    // Mean elements with improved accuracy
    double L0 = normalize_zero_2pi(DEG_TO_RAD * (280.46645 + 36000.76983 * T + 0.0003032 * T2));
    double M = normalize_zero_2pi(DEG_TO_RAD * (357.52910 + 35999.05030 * T - 0.0001559 * T2 - 0.00000048 * T3));
    
    // Equation of center and eccentricity
    double e = 0.016708634 - 0.000042037 * T - 0.0000001267 * T2;
    double e2 = e * e;
    
    double C = DEG_TO_RAD * ((1.914602 - 0.004817 * T - 0.000014 * T2) * sin(M) +
                            (0.019993 - 0.000101 * T) * sin(2.0 * M) +
                            0.000289 * sin(3.0 * M));
    
    // True anomaly and Sun's true longitude with planetary perturbations
    double f = M + C;
    double L_true = normalize_zero_2pi(L0 + C + pert.deltaL);
    
    // Distance to Sun in AU with planetary perturbations
    double R = ((1.000001018 * (1.0 - e2)) / (1.0 + e * cos(f))) + pert.deltaR;
    
    // Mean obliquity with improved accuracy
    double eps0 = 23.0 + 26.0/60.0 + 21.448/3600.0;
    double eps1 = -46.8150/3600.0;
    double eps2 = -0.00059/3600.0;
    double eps3 = 0.001813/3600.0;
    double eps = DEG_TO_RAD * (eps0 + eps1 * T + eps2 * T2 + eps3 * T3);
    
    // True obliquity (mean obliquity + nutation in obliquity)
    double Obl = eps + nut.deps;
    
    // Apply nutation in longitude and planetary perturbations to true longitude
    L_true += nut.dpsi;
    
    // Convert ecliptic coordinates to equatorial coordinates
    double sin_eps = sin(Obl);
    double cos_eps = cos(Obl);
    
    // Include planetary perturbations in latitude
    double B = pert.deltaB;  // Heliocentric latitude
    
    // Calculate right ascension and declination with all corrections
    double X = R * cos(L_true) * cos(B);
    double Y = R * (cos(eps) * sin(L_true) * cos(B) - sin(eps) * sin(B));
    double Z = R * (sin(eps) * sin(L_true) * cos(B) + cos(eps) * sin(B));
    
    double RA = atan2(Y, X);
    double Decl = atan2(Z, sqrt(X * X + Y * Y));
    
    // Calculate aberration corrections (previous implementation remains...)
    double aberration_const = 29.7859 / C;
    double dRA_aberr = -aberration_const * (cos(RA) * cos(L_true) * cos_eps +
                                          sin(RA) * sin(L_true)) / cos(Decl);
    double dDecl_aberr = -aberration_const * (cos(L_true) * cos_eps * 
                                            (tan(Decl) * cos_eps - sin(L_true) * sin_eps));
    
    // Apply all corrections
    RA += dRA_aberr;
    Decl += dDecl_aberr;
    
    // Calculate hour angle with all corrections
    double GrHrAngle = normalize_angle(280.46061837 + 360.98564736629 * JD_frac +
                                     (360.0 * (JD - J2000) + 0.98564736629 * (JD - J2000)),
                                     0.0, 360.0);
    GrHrAngle += nut.dpsi * cos(Obl) * RAD_TO_DEG;
    
    double HrAngle = normalize_zero_2pi(DEG_TO_RAD * GrHrAngle + Lon - RA);
    
    // Calculate final topocentric coordinates
    double sin_Lat = sin(Lat);
    double cos_Lat = cos(Lat);
    double sin_Decl = sin(Decl);
    double cos_Decl = cos(Decl);
    double cos_HrAngle = cos(HrAngle);
    
    double elevation = asin(sin_Lat * sin_Decl + 
                          cos_Lat * cos_Decl * cos_HrAngle);
    
    double y = sin(HrAngle);
    double x = cos_HrAngle * sin_Lat - tan(Decl) * cos_Lat;
    double azimuth = normalize_zero_2pi(PI + atan2(y, x));
    
    // Apply atmospheric refraction
    double actual_elevation = elevation;
    if (elevation < DEG_TO_RAD * 85.0) {
        double r = 1.02 / tan(elevation + DEG_TO_RAD * 10.3 / (elevation * RAD_TO_DEG + 5.11));
        actual_elevation += r * ARCSEC_TO_RAD;
    }
    
    // Store final results
    sun_azimuth = azimuth * RAD_TO_DEG;
    sun_elevation = actual_elevation * RAD_TO_DEG;
}
