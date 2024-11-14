void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    const float DEG_TO_RAD = PI / 180.0;
    const float RAD_TO_DEG = 180.0 / PI;

    // Calculate Julian Date with time component
    long JDate = JulianDate(year, month, day);
    float JD_frac = ((hour + (minute / 60.0) + (second / 3600.0)) / 24.0) - 0.5;
    float JD = JDate + JD_frac - Timezone / 24.0;

    // Calculate centuries since J2000.0 (T)
    float T = (JD - 2451545.0) / 36525.0;

    // Mean longitude (L0), corrected for higher precision
    float L0 = 280.46646 + 36000.76983 * T + 0.0003032 * T * T;
    L0 = fmod(L0, 360.0);

    // Mean anomaly (M) and eccentricity of Earth's orbit (e)
    float M = 357.52911 + 35999.05029 * T - 0.0001537 * T * T;
    M = fmod(M, 360.0);
    float e = 0.016708634 - 0.000042037 * T - 0.0000001267 * T * T;

    // Equation of center (C), with additional terms for accuracy
    float C = (1.914602 - 0.004817 * T - 0.000014 * T * T) * sin(DEG_TO_RAD * M) 
            + (0.019993 - 0.000101 * T) * sin(2 * DEG_TO_RAD * M) 
            + 0.000289 * sin(3 * DEG_TO_RAD * M);

    // True longitude (L_true) and true anomaly (f)
    float L_true = L0 + C;
    float f = M + C;

    // Obliquity of the ecliptic (Obl)
    float Obl = 23.439291 - 0.013004167 * T - 0.000000164 * T * T + 0.0000005036 * T * T * T;

    // Radius vector (R)
    float R = 1.000001018 * (1 - e * e) / (1 + e * cos(DEG_TO_RAD * f));

    // Apparent longitude (L_apparent), including aberration correction
    float L_apparent = L_true - 0.00569 - 0.00478 * sin(DEG_TO_RAD * (125.04 - 1934.136 * T));

    // Right Ascension (RA) and Declination (Decl)
    float RA = atan2(cos(DEG_TO_RAD * Obl) * sin(DEG_TO_RAD * L_apparent), cos(DEG_TO_RAD * L_apparent));
    float Decl = asin(sin(DEG_TO_RAD * Obl) * sin(DEG_TO_RAD * L_apparent));

    // Calculate Greenwich Hour Angle
    float JDx = JDate - 2451545;
    float GrHrAngle = 280.46061837 + 360.98564736629 * JDx + 0.98564736629 * JD_frac;
    GrHrAngle = fmod(GrHrAngle, 360.0);

    // Local Hour Angle (HrAngle)
    float HrAngle = DEG_TO_RAD * (GrHrAngle + Lon) - RA;

    // Solar elevation (elevation) and azimuth (azimuth)
    float elevation = asin(sin(DEG_TO_RAD * Lat) * sin(Decl) + cos(DEG_TO_RAD * Lat) * cos(Decl) * cos(HrAngle));
    float azimuth = PI + atan2(sin(HrAngle), cos(HrAngle) * sin(DEG_TO_RAD * Lat) - tan(Decl) * cos(DEG_TO_RAD * Lat));

    // Convert to degrees for output
    sun_azimuth = fmod(azimuth * RAD_TO_DEG + 360.0, 360.0);
    sun_elevation = elevation * RAD_TO_DEG;
}
