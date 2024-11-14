// Previous constants remain...
// Additional constants for Earth orientation and parallax
const double EARTH_RADIUS = 6378.137;              // Earth's equatorial radius in km
const double EARTH_FLATTENING = 1.0 / 298.257223563; // Earth's flattening factor
const double IERS_MJD_2000 = 51544.5;              // Modified Julian Date for J2000.0

// Structure for Earth Orientation Parameters
struct EarthOrientation {
    double dUT1;    // UT1-UTC in seconds
    double xp;      // Polar motion coefficient x in arcseconds
    double yp;      // Polar motion coefficient y in arcseconds
    double dX;      // Celestial pole offset in X in arcseconds
    double dY;      // Celestial pole offset in Y in arcseconds
    double LOD;     // Length of day offset in milliseconds
};

// Structure for observer's location
struct GeographicLocation {
    double latitude;    // Geodetic latitude in radians
    double longitude;   // Geodetic longitude in radians
    double height;      // Height above ellipsoid in meters
    double temperature; // Ambient temperature in Celsius
    double pressure;    // Atmospheric pressure in millibars
};

// Convert geodetic coordinates to geocentric XYZ
struct GeoCentricCoord {
    double X;
    double Y;
    double Z;
};

GeoCentricCoord geodetic_to_geocentric(const GeographicLocation& loc) {
    double sin_lat = sin(loc.latitude);
    double cos_lat = cos(loc.latitude);
    double sin_lon = sin(loc.longitude);
    double cos_lon = cos(loc.longitude);
    
    // Calculate Earth radius at observer's latitude
    double N = EARTH_RADIUS / sqrt(1.0 - EARTH_FLATTENING * (2.0 - EARTH_FLATTENING) * 
                                 sin_lat * sin_lat);
    
    GeoCentricCoord gc;
    gc.X = (N + loc.height * 0.001) * cos_lat * cos_lon;
    gc.Y = (N + loc.height * 0.001) * cos_lat * sin_lon;
    gc.Z = (N * (1.0 - EARTH_FLATTENING) * (2.0 - EARTH_FLATTENING) + 
            loc.height * 0.001) * sin_lat;
    
    return gc;
}

// Get Earth Orientation Parameters for given date
EarthOrientation getEOP(double JD) {
    // In practice, these values would come from IERS Bulletin A/B
    // Here we'll implement a simplified model for demonstration
    double T = (JD - J2000) / 36525.0;
    
    EarthOrientation eop;
    // Approximate values based on recent trends
    eop.dUT1 = 0.2 * sin(2 * PI * T);  // Approximate UT1-UTC variation
    eop.xp = 0.3 * sin(2 * PI * T);    // Approximate polar motion
    eop.yp = 0.3 * cos(2 * PI * T);
    eop.dX = 0.1 * sin(2 * PI * T);    // Approximate celestial pole offsets
    eop.dY = 0.1 * cos(2 * PI * T);
    eop.LOD = 1.0 + 0.1 * sin(2 * PI * T); // Approximate LOD variation
    
    return eop;
}

// Calculate diurnal aberration
struct DiurnalAberration {
    double dRA;
    double dDec;
};

DiurnalAberration calculate_diurnal_aberration(const GeoCentricCoord& gc, 
                                             double lst, double ra, double dec) {
    double omega = 7.292115855e-5; // Earth's angular velocity in rad/s
    DiurnalAberration da;
    
    double H = lst - ra;
    double cos_dec = cos(dec);
    double sin_dec = sin(dec);
    double cos_H = cos(H);
    double sin_H = sin(H);
    
    // Calculate diurnal aberration
    double v = omega * sqrt(gc.X * gc.X + gc.Y * gc.Y) / C;
    
    da.dRA = -v * cos_dec * sin_H / cos_dec;
    da.dDec = -v * sin_dec * cos_H;
    
    return da;
}

// Calculate topocentric corrections
struct TopocentricCorrection {
    double dRA;
    double dDec;
    double dDist;
};

TopocentricCorrection calculate_topocentric_correction(
    const GeoCentricCoord& gc, double lst, double ra, double dec, double dist) {
    
    TopocentricCorrection tc;
    double H = lst - ra;
    double sin_H = sin(H);
    double cos_H = cos(H);
    double sin_dec = sin(dec);
    double cos_dec = cos(dec);
    
    // Convert distance to kilometers
    double dist_km = dist * AU_TO_KM;
    
    // Calculate parallax corrections
    double parallax = EARTH_RADIUS / dist_km;
    
    // Project observer's location
    double rho_sin_phi_prime = gc.Z / EARTH_RADIUS;
    double rho_cos_phi_prime = sqrt(gc.X * gc.X + gc.Y * gc.Y) / EARTH_RADIUS;
    
    tc.dRA = atan2(-rho_cos_phi_prime * sin_H * parallax,
                   cos_dec - rho_cos_phi_prime * cos_H * parallax);
    
    tc.dDec = atan2((sin_dec - rho_sin_phi_prime * parallax) * cos(tc.dRA),
                    cos_dec - rho_cos_phi_prime * cos_H * parallax);
    
    tc.dDist = -parallax * (rho_cos_phi_prime * cos_H * cos_dec + 
                           rho_sin_phi_prime * sin_dec);
    
    return tc;
}

void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year,
                          const GeographicLocation& observer) {
    // Previous time calculations remain the same...
    double JD = JulianDate(year, month, day);
    double UT = hour + minute / 60.0 + second / 3600.0;
    
    // Get Earth Orientation Parameters
    EarthOrientation eop = getEOP(JD);
    
    // Apply UT1-UTC correction
    double UT1 = UT + eop.dUT1 / 3600.0;
    double JD_UT1 = JD + (UT1 - 12.0) / 24.0;
    
    // Calculate GMST with improved accuracy
    double T = (JD_UT1 - J2000) / 36525.0;
    double gmst = normalize_zero_2pi(DEG_TO_RAD * (280.46061837 + 
                                                  360.98564736629 * (JD_UT1 - J2000) +
                                                  0.000387933 * T * T -
                                                  T * T * T / 38710000.0));
    
    // Calculate equation of the equinoxes
    Nutation nut = calculate_nutation(T);
    double eps = calculate_mean_obliquity(T) + nut.deps;
    double eq_equinox = nut.dpsi * cos(eps);
    
    // Calculate apparent sidereal time
    double ast = gmst + eq_equinox;
    
    // Apply polar motion to observer's coordinates
    double xp_rad = eop.xp * ARCSEC_TO_RAD;
    double yp_rad = eop.yp * ARCSEC_TO_RAD;
    
    // Convert observer's position to geocentric coordinates
    GeoCentricCoord gc = geodetic_to_geocentric(observer);
    
    // Apply polar motion
    double x_cor = gc.X - xp_rad * gc.Z;
    double y_cor = gc.Y - yp_rad * gc.Z;
    double z_cor = gc.Z + xp_rad * gc.X + yp_rad * gc.Y;
    
    gc.X = x_cor;
    gc.Y = y_cor;
    gc.Z = z_cor;
    
    // Calculate planetary perturbations (previous implementation)
    PlanetaryPerturbations pert = calculatePlanetaryPerturbations(T);
    
    // Calculate solar position (previous calculations through RA/Dec)
    // ... (previous calculations until we have RA, Dec, and distance)
    
    // Apply diurnal aberration
    DiurnalAberration da = calculate_diurnal_aberration(gc, ast, RA, Decl);
    RA += da.dRA;
    Decl += da.dDec;
    
    // Calculate topocentric corrections
    TopocentricCorrection tc = calculate_topocentric_correction(gc, ast, RA, Decl, R);
    
    // Apply topocentric corrections
    RA += tc.dRA;
    Decl += tc.dDec;
    R += tc.dDist;
    
    // Calculate local hour angle
    double local_ast = ast + observer.longitude;
    double H = normalize_zero_2pi(local_ast - RA);
    
    // Calculate topocentric horizontal coordinates
    double sin_lat = sin(observer.latitude);
    double cos_lat = cos(observer.latitude);
    double sin_dec = sin(Decl);
    double cos_dec = cos(Decl);
    double cos_H = cos(H);
    
    // Calculate elevation with improved precision
    double elevation = asin(sin_lat * sin_dec + cos_lat * cos_dec * cos_H);
    
    // Calculate azimuth
    double y = sin(H);
    double x = cos_H * sin_lat - tan(Decl) * cos_lat;
    double azimuth = normalize_zero_2pi(PI + atan2(y, x));
    
    // Apply atmospheric refraction with weather conditions
    double actual_elevation = elevation;
    if (elevation < DEG_TO_RAD * 85.0) {
        // More accurate refraction model using temperature and pressure
        double P = observer.pressure;
        double T = observer.temperature;
        double R = 1.02 / tan(elevation + DEG_TO_RAD * 10.3 / 
                             (elevation * RAD_TO_DEG + 5.11));
        R *= P / 1010.0 * 283.0 / (273.0 + T);
        actual_elevation += R * ARCSEC_TO_RAD;
    }
    
    // Store final results
    sun_azimuth = azimuth * RAD_TO_DEG;
    sun_elevation = actual_elevation * RAD_TO_DEG;
}
