#include <Arduino.h>
#include <math.h>

// Constants
const float DEG_TO_RAD = PI / 180.0;
const float RAD_TO_DEG = 180.0 / PI;

// Global variables (add these at the top)
float Lat = 0;  // Latitude in degrees
float Lon = 0;  // Longitude in degrees
int Timezone = 0;  // Timezone offset in hours
float sun_azimuth = 0;
float sun_elevation = 0;

// Custom class for modular arithmetic within 360 degrees for angles
class AngleMod {
private:
    float value;
public:
    AngleMod(float v) : value(fmod(v, 360.0f)) { 
        if (value < 0) value += 360.0f; 
    }

    AngleMod operator+(const AngleMod& other) const {
        return AngleMod(value + other.value);
    }

    AngleMod operator-(const AngleMod& other) const {
        return AngleMod(value - other.value);
    }

    AngleMod operator*(float scalar) const {
        return AngleMod(value * scalar);
    }

    float toRadians() const { return value * DEG_TO_RAD; }
};

// Lazy trigonometric cache
class LazyTrigCache {
private:
    mutable float sin_val, cos_val;
    mutable bool cached;
    AngleMod angle;

public:
    LazyTrigCache(AngleMod a) : angle(a), cached(false) {}

    float sin() const {
        if (!cached) { 
            sin_val = ::sin(angle.toRadians()); 
            cos_val = ::cos(angle.toRadians()); 
            cached = true; 
        }
        return sin_val;
    }

    float cos() const {
        if (!cached) { 
            sin_val = ::sin(angle.toRadians()); 
            cos_val = ::cos(angle.toRadians()); 
            cached = true; 
        }
        return cos_val;
    }
};

// Julian Date calculation function (was missing in original)
long JulianDate(int year, int month, int day) {
    if (month <= 2) {
        year -= 1;
        month += 12;
    }
    long A = year / 100;
    long B = 2 - A + (A / 4);
    
    return (long)(365.25 * (year + 4716)) + 
           (long)(30.6001 * (month + 1)) + 
           day + B - 1524.5;
}

// Solar calculation functions
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

void compute_ra_decl(const LazyTrigCache& L_app, const LazyTrigCache& Obl, float& RA, float& Decl) {
    float sin_L = L_app.sin();
    float cos_L = L_app.cos();
    float cos_Obl = Obl.cos();
    RA = atan2(cos_Obl * sin_L, cos_L);
    Decl = asin(Obl.sin() * sin_L);
}

void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
    long JDate = JulianDate(year, month, day);
    float JD_frac = ((hour + (minute / 60.0) + (second / 3600.0)) / 24.0) - 0.5;
    float JD = JDate + JD_frac - Timezone / 24.0;
    float T = (JD - 2451545.0) / 36525.0;

    AngleMod L0 = mean_longitude(T);
    AngleMod M = mean_anomaly(T);
    float e = eccentricity(T);
    AngleMod Obl = obliquity_of_ecliptic(T);

    LazyTrigCache sinM(M);
    LazyTrigCache sin2M(AngleMod(M.toRadians() * 2.0f));
    LazyTrigCache sin3M(AngleMod(M.toRadians() * 3.0f));
    
    AngleMod C = AngleMod((1.914602 - 0.004817 * T - 0.000014 * T * T) * sinM.sin() +
                          (0.019993 - 0.000101 * T) * sin2M.sin() +
                          0.000289 * sin3M.sin());

    AngleMod L_true = L0 + C;
    AngleMod f = M + C;

    float radius = 1.000001018 * (1 - e * e) / (1 + e * cos(f.toRadians()));

    AngleMod L_apparent = L_true - AngleMod(0.00569 + 0.00478 * sin(AngleMod(125.04 - 1934.136 * T).toRadians()));

    LazyTrigCache L_app_cache(L_apparent);
    LazyTrigCache Obl_cache(Obl);
    float RA, Decl;
    compute_ra_decl(L_app_cache, Obl_cache, RA, Decl);

    AngleMod GrHrAngle = AngleMod(280.46061837 + 360.98564736629 * (JD - 2451545.0));
    AngleMod HrAngle = GrHrAngle + AngleMod(Lon - RA * RAD_TO_DEG);

    float elevation = asin(sin(Lat * DEG_TO_RAD) * sin(Decl) + 
                         cos(Lat * DEG_TO_RAD) * cos(Decl) * cos(HrAngle.toRadians()));
    
    LazyTrigCache HrAngle_cache(HrAngle);
    float azimuth = PI + atan2(HrAngle_cache.sin(),
                              HrAngle_cache.cos() * sin(Lat * DEG_TO_RAD) - 
                              tan(Decl) * cos(Lat * DEG_TO_RAD));

    sun_azimuth = azimuth * RAD_TO_DEG;
    sun_elevation = elevation * RAD_TO_DEG;
}
