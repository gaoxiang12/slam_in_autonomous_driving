/***************************************************************************/
/* RSC IDENTIFIER: TRANSVERSE MERCATOR
 *
 * ABSTRACT
 *
 *    This component provides conversions between Geodetic coordinates
 *    (latitude and longitude) and Transverse Mercator projection coordinates
 *    (easting and northing).
 *
 * ERROR HANDLING
 *
 *    This component checks parameters for valid values.  If an invalid value
 *    is found the error code is combined with the current error code using
 *    the bitwise or.  This combining allows multiple error codes to be
 *    returned. The possible error codes are:
 *
 *       TRANMERC_NO_ERROR           : No errors occurred in function
 *       TRANMERC_LAT_ERROR          : Latitude outside of valid range
 *                                      (-90 to 90 degrees)
 *       TRANMERC_LON_ERROR          : Longitude outside of valid range
 *                                      (-180 to 360 degrees, and within
 *                                        +/-90 of Central Meridian)
 *       TRANMERC_EASTING_ERROR      : Easting outside of valid range
 *                                      (depending on ellipsoid and
 *                                       projection parameters)
 *       TRANMERC_NORTHING_ERROR     : Northing outside of valid range
 *                                      (depending on ellipsoid and
 *                                       projection parameters)
 *       TRANMERC_ORIGIN_LAT_ERROR   : Origin latitude outside of valid range
 *                                      (-90 to 90 degrees)
 *       TRANMERC_CENT_MER_ERROR     : Central meridian outside of valid range
 *                                      (-180 to 360 degrees)
 *       TRANMERC_A_ERROR            : Semi-major axis less than or equal to zero
 *       TRANMERC_INV_F_ERROR        : Inverse flattening outside of valid range
 *								  	                  (250 to 350)
 *       TRANMERC_SCALE_FACTOR_ERROR : Scale factor outside of valid
 *                                     range (0.3 to 3.0)
 *		 TM_LON_WARNING              : Distortion will result if longitude is more
 *                                       than 9 degrees from the Central Meridian
 *
 * REUSE NOTES
 *
 *    TRANSVERSE MERCATOR is intended for reuse by any application that
 *    performs a Transverse Mercator projection or its inverse.
 *
 * REFERENCES
 *
 *    Further information on TRANSVERSE MERCATOR can be found in the
 *    Reuse Manual.
 *
 *    TRANSVERSE MERCATOR originated from :
 *                      U.S. Army Topographic Engineering Center
 *                      Geospatial Information Division
 *                      7701 Telegraph Road
 *                      Alexandria, VA  22310-3864
 *
 * LICENSES
 *
 *    None apply to this component.
 *
 * RESTRICTIONS
 *
 *    TRANSVERSE MERCATOR has no restrictions.
 *
 * ENVIRONMENT
 *
 *    TRANSVERSE MERCATOR was tested and certified in the following
 *    environments:
 *
 *    1. Solaris 2.5 with GCC, version 2.8.1
 *    2. Windows 95 with MS Visual C++, version 6
 *
 * MODIFICATIONS
 *
 *    Date              Description
 *    ----              -----------
 *    10-02-97          Original Code
 *    03-02-97          Re-engineered Code
 *
 */

/***************************************************************************/
/*
 *                               INCLUDES
 */

#include "tranmerc.h"
#include <math.h>

/*
 *    math.h      - Standard C math library
 *    tranmerc.h  - Is for prototype error checking
 */

/***************************************************************************/
/*                               DEFINES
 *
 */

#define PI 3.14159265358979323e0           /* PI     */
#define PI_OVER_2 (PI / 2.0e0)             /* PI over 2 */
#define MAX_LAT ((PI * 89.99) / 180.0)     /* 89.99 degrees in radians */
#define MAX_DELTA_LONG ((PI * 90) / 180.0) /* 90 degrees in radians */
#define MIN_SCALE_FACTOR 0.3
#define MAX_SCALE_FACTOR 3.0

#define SPHTMD(Latitude)                                                                                         \
    ((double)(TranMerc_ap * Latitude - TranMerc_bp * sin(2.e0 * Latitude) + TranMerc_cp * sin(4.e0 * Latitude) - \
              TranMerc_dp * sin(6.e0 * Latitude) + TranMerc_ep * sin(8.e0 * Latitude)))

#define SPHSN(Latitude) ((double)(TranMerc_a / sqrt(1.e0 - TranMerc_es * pow(sin(Latitude), 2))))

#define SPHSR(Latitude) ((double)(TranMerc_a * (1.e0 - TranMerc_es) / pow(DENOM(Latitude), 3)))

#define DENOM(Latitude) ((double)(sqrt(1.e0 - TranMerc_es * pow(sin(Latitude), 2))))

/**************************************************************************/
/*                               GLOBAL DECLARATIONS
 *
 */

/* Ellipsoid Parameters, default to WGS 84  */
static double TranMerc_a = 6378137.0;              /* Semi-major axis of ellipsoid in meters */
static double TranMerc_f = 1 / 298.257223563;      /* Flattening of ellipsoid  */
static double TranMerc_es = 0.0066943799901413800; /* Eccentricity (0.08181919084262188000) squared */
static double TranMerc_ebs = 0.0067394967565869;   /* Second Eccentricity squared */

/* Transverse_Mercator projection Parameters */
static double TranMerc_Origin_Lat = 0.0;     /* Latitude of origin in radians */
static double TranMerc_Origin_Long = 0.0;    /* Longitude of origin in radians */
static double TranMerc_False_Northing = 0.0; /* False northing in meters */
static double TranMerc_False_Easting = 0.0;  /* False easting in meters */
static double TranMerc_Scale_Factor = 1.0;   /* Scale factor  */

/* Isometeric to geodetic latitude parameters, default to WGS 84 */
static double TranMerc_ap = 6367449.1458008;
static double TranMerc_bp = 16038.508696861;
static double TranMerc_cp = 16.832613334334;
static double TranMerc_dp = 0.021984404273757;
static double TranMerc_ep = 3.1148371319283e-005;

/* Maximum variance for easting and northing values for WGS 84. */
static double TranMerc_Delta_Easting = 40000000.0;
static double TranMerc_Delta_Northing = 40000000.0;

/* These state variables are for optimization purposes. The only function
 * that should modify them is Set_Tranverse_Mercator_Parameters.         */

/************************************************************************/
/*                              FUNCTIONS
 *
 */

long Set_Transverse_Mercator_Parameters(double a, double f, double Origin_Latitude, double Central_Meridian,
                                        double False_Easting, double False_Northing, double Scale_Factor)

{ /* BEGIN Set_Tranverse_Mercator_Parameters */
    /*
     * The function Set_Tranverse_Mercator_Parameters receives the ellipsoid
     * parameters and Tranverse Mercator projection parameters as inputs, and
     * sets the corresponding state variables. If any errors occur, the error
     * code(s) are returned by the function, otherwise TRANMERC_NO_ERROR is
     * returned.
     *
     *    a                 : Semi-major axis of ellipsoid, in meters    (input)
     *    f                 : Flattening of ellipsoid						         (input)
     *    Origin_Latitude   : Latitude in radians at the origin of the   (input)
     *                         projection
     *    Central_Meridian  : Longitude in radians at the center of the  (input)
     *                         projection
     *    False_Easting     : Easting/X at the center of the projection  (input)
     *    False_Northing    : Northing/Y at the center of the projection (input)
     *    Scale_Factor      : Projection scale factor                    (input)
     */

    double tn; /* True Meridianal distance constant  */
    double tn2;
    double tn3;
    double tn4;
    double tn5;
    double dummy_northing;
    double TranMerc_b; /* Semi-minor axis of ellipsoid, in meters */
    double inv_f = 1 / f;
    long Error_Code = TRANMERC_NO_ERROR;

    if (a <= 0.0) { /* Semi-major axis must be greater than zero */
        Error_Code |= TRANMERC_A_ERROR;
    }
    if ((inv_f < 250) || (inv_f > 350)) { /* Inverse flattening must be between 250 and 350 */
        Error_Code |= TRANMERC_INV_F_ERROR;
    }
    if ((Origin_Latitude < -PI_OVER_2) || (Origin_Latitude > PI_OVER_2)) { /* origin latitude out of range */
        Error_Code |= TRANMERC_ORIGIN_LAT_ERROR;
    }
    if ((Central_Meridian < -PI) || (Central_Meridian > (2 * PI))) { /* origin longitude out of range */
        Error_Code |= TRANMERC_CENT_MER_ERROR;
    }
    if ((Scale_Factor < MIN_SCALE_FACTOR) || (Scale_Factor > MAX_SCALE_FACTOR)) {
        Error_Code |= TRANMERC_SCALE_FACTOR_ERROR;
    }
    if (!Error_Code) { /* no errors */
        TranMerc_a = a;
        TranMerc_f = f;
        TranMerc_Origin_Lat = Origin_Latitude;
        if (Central_Meridian > PI) Central_Meridian -= (2 * PI);
        TranMerc_Origin_Long = Central_Meridian;
        TranMerc_False_Northing = False_Northing;
        TranMerc_False_Easting = False_Easting;
        TranMerc_Scale_Factor = Scale_Factor;

        /* Eccentricity Squared */
        TranMerc_es = 2 * TranMerc_f - TranMerc_f * TranMerc_f;
        /* Second Eccentricity Squared */
        TranMerc_ebs = (1 / (1 - TranMerc_es)) - 1;

        TranMerc_b = TranMerc_a * (1 - TranMerc_f);
        /*True meridianal constants  */
        tn = (TranMerc_a - TranMerc_b) / (TranMerc_a + TranMerc_b);
        tn2 = tn * tn;
        tn3 = tn2 * tn;
        tn4 = tn3 * tn;
        tn5 = tn4 * tn;

        TranMerc_ap = TranMerc_a * (1.e0 - tn + 5.e0 * (tn2 - tn3) / 4.e0 + 81.e0 * (tn4 - tn5) / 64.e0);
        TranMerc_bp = 3.e0 * TranMerc_a * (tn - tn2 + 7.e0 * (tn3 - tn4) / 8.e0 + 55.e0 * tn5 / 64.e0) / 2.e0;
        TranMerc_cp = 15.e0 * TranMerc_a * (tn2 - tn3 + 3.e0 * (tn4 - tn5) / 4.e0) / 16.0;
        TranMerc_dp = 35.e0 * TranMerc_a * (tn3 - tn4 + 11.e0 * tn5 / 16.e0) / 48.e0;
        TranMerc_ep = 315.e0 * TranMerc_a * (tn4 - tn5) / 512.e0;
        Convert_Geodetic_To_Transverse_Mercator(MAX_LAT, MAX_DELTA_LONG + Central_Meridian, &TranMerc_Delta_Easting,
                                                &TranMerc_Delta_Northing);
        Convert_Geodetic_To_Transverse_Mercator(0, MAX_DELTA_LONG + Central_Meridian, &TranMerc_Delta_Easting,
                                                &dummy_northing);
        TranMerc_Delta_Northing++;
        TranMerc_Delta_Easting++;

    } /* END OF if(!Error_Code) */
    return (Error_Code);
} /* END of Set_Transverse_Mercator_Parameters  */

void Get_Transverse_Mercator_Parameters(double *a, double *f, double *Origin_Latitude, double *Central_Meridian,
                                        double *False_Easting, double *False_Northing, double *Scale_Factor)

{ /* BEGIN Get_Tranverse_Mercator_Parameters  */
    /*
     * The function Get_Transverse_Mercator_Parameters returns the current
     * ellipsoid and Transverse Mercator projection parameters.
     *
     *    a                 : Semi-major axis of ellipsoid, in meters    (output)
     *    f                 : Flattening of ellipsoid						         (output)
     *    Origin_Latitude   : Latitude in radians at the origin of the   (output)
     *                         projection
     *    Central_Meridian  : Longitude in radians at the center of the  (output)
     *                         projection
     *    False_Easting     : Easting/X at the center of the projection  (output)
     *    False_Northing    : Northing/Y at the center of the projection (output)
     *    Scale_Factor      : Projection scale factor                    (output)
     */

    *a = TranMerc_a;
    *f = TranMerc_f;
    *Origin_Latitude = TranMerc_Origin_Lat;
    *Central_Meridian = TranMerc_Origin_Long;
    *False_Easting = TranMerc_False_Easting;
    *False_Northing = TranMerc_False_Northing;
    *Scale_Factor = TranMerc_Scale_Factor;
    return;
} /* END OF Get_Tranverse_Mercator_Parameters */

long Convert_Geodetic_To_Transverse_Mercator(double Latitude, double Longitude, double *Easting, double *Northing)

{ /* BEGIN Convert_Geodetic_To_Transverse_Mercator */

    /*
     * The function Convert_Geodetic_To_Transverse_Mercator converts geodetic
     * (latitude and longitude) coordinates to Transverse Mercator projection
     * (easting and northing) coordinates, according to the current ellipsoid
     * and Transverse Mercator projection coordinates.  If any errors occur, the
     * error code(s) are returned by the function, otherwise TRANMERC_NO_ERROR is
     * returned.
     *
     *    Latitude      : Latitude in radians                         (input)
     *    Longitude     : Longitude in radians                        (input)
     *    Easting       : Easting/X in meters                         (output)
     *    Northing      : Northing/Y in meters                        (output)
     */

    double c; /* Cosine of latitude                          */
    double c2;
    double c3;
    double c5;
    double c7;
    double dlam; /* Delta longitude - Difference in Longitude       */
    double eta;  /* constant - TranMerc_ebs *c *c                   */
    double eta2;
    double eta3;
    double eta4;
    double s;  /* Sine of latitude                        */
    double sn; /* Radius of curvature in the prime vertical       */
    double t;  /* Tangent of latitude                             */
    double tan2;
    double tan3;
    double tan4;
    double tan5;
    double tan6;
    double t1;   /* Term in coordinate conversion formula - GP to Y */
    double t2;   /* Term in coordinate conversion formula - GP to Y */
    double t3;   /* Term in coordinate conversion formula - GP to Y */
    double t4;   /* Term in coordinate conversion formula - GP to Y */
    double t5;   /* Term in coordinate conversion formula - GP to Y */
    double t6;   /* Term in coordinate conversion formula - GP to Y */
    double t7;   /* Term in coordinate conversion formula - GP to Y */
    double t8;   /* Term in coordinate conversion formula - GP to Y */
    double t9;   /* Term in coordinate conversion formula - GP to Y */
    double tmd;  /* True Meridional distance                        */
    double tmdo; /* True Meridional distance for latitude of origin */
    long Error_Code = TRANMERC_NO_ERROR;
    double temp_Origin;
    double temp_Long;

    if ((Latitude < -MAX_LAT) || (Latitude > MAX_LAT)) { /* Latitude out of range */
        Error_Code |= TRANMERC_LAT_ERROR;
    }
    if (Longitude > PI) Longitude -= (2 * PI);
    if ((Longitude < (TranMerc_Origin_Long - MAX_DELTA_LONG)) ||
        (Longitude > (TranMerc_Origin_Long + MAX_DELTA_LONG))) {
        if (Longitude < 0) temp_Long = Longitude + 2 * PI;
        else
            temp_Long = Longitude;
        if (TranMerc_Origin_Long < 0) temp_Origin = TranMerc_Origin_Long + 2 * PI;
        else
            temp_Origin = TranMerc_Origin_Long;
        if ((temp_Long < (temp_Origin - MAX_DELTA_LONG)) || (temp_Long > (temp_Origin + MAX_DELTA_LONG)))
            Error_Code |= TRANMERC_LON_ERROR;
    }
    if (!Error_Code) { /* no errors */

        /*
         *  Delta Longitude
         */
        dlam = Longitude - TranMerc_Origin_Long;

        if (fabs(dlam) >
            (9.0 * PI /
             180)) { /* Distortion will result if Longitude is more than 9 degrees from the Central Meridian */
            Error_Code |= TRANMERC_LON_WARNING;
        }

        if (dlam > PI) dlam -= (2 * PI);
        if (dlam < -PI) dlam += (2 * PI);
        if (fabs(dlam) < 2.e-10) dlam = 0.0;

        s = sin(Latitude);
        c = cos(Latitude);
        c2 = c * c;
        c3 = c2 * c;
        c5 = c3 * c2;
        c7 = c5 * c2;
        t = tan(Latitude);
        tan2 = t * t;
        tan3 = tan2 * t;
        tan4 = tan3 * t;
        tan5 = tan4 * t;
        tan6 = tan5 * t;
        eta = TranMerc_ebs * c2;
        eta2 = eta * eta;
        eta3 = eta2 * eta;
        eta4 = eta3 * eta;

        /* radius of curvature in prime vertical */
        sn = SPHSN(Latitude);

        /* True Meridianal Distances */
        tmd = SPHTMD(Latitude);

        /*  Origin  */
        tmdo = SPHTMD(TranMerc_Origin_Lat);

        /* northing */
        t1 = (tmd - tmdo) * TranMerc_Scale_Factor;
        t2 = sn * s * c * TranMerc_Scale_Factor / 2.e0;
        t3 = sn * s * c3 * TranMerc_Scale_Factor * (5.e0 - tan2 + 9.e0 * eta + 4.e0 * eta2) / 24.e0;

        t4 = sn * s * c5 * TranMerc_Scale_Factor *
             (61.e0 - 58.e0 * tan2 + tan4 + 270.e0 * eta - 330.e0 * tan2 * eta + 445.e0 * eta2 + 324.e0 * eta3 -
              680.e0 * tan2 * eta2 + 88.e0 * eta4 - 600.e0 * tan2 * eta3 - 192.e0 * tan2 * eta4) /
             720.e0;

        t5 = sn * s * c7 * TranMerc_Scale_Factor * (1385.e0 - 3111.e0 * tan2 + 543.e0 * tan4 - tan6) / 40320.e0;

        *Northing = TranMerc_False_Northing + t1 + pow(dlam, 2.e0) * t2 + pow(dlam, 4.e0) * t3 + pow(dlam, 6.e0) * t4 +
                    pow(dlam, 8.e0) * t5;

        /* Easting */
        t6 = sn * c * TranMerc_Scale_Factor;
        t7 = sn * c3 * TranMerc_Scale_Factor * (1.e0 - tan2 + eta) / 6.e0;
        t8 = sn * c5 * TranMerc_Scale_Factor *
             (5.e0 - 18.e0 * tan2 + tan4 + 14.e0 * eta - 58.e0 * tan2 * eta + 13.e0 * eta2 + 4.e0 * eta3 -
              64.e0 * tan2 * eta2 - 24.e0 * tan2 * eta3) /
             120.e0;
        t9 = sn * c7 * TranMerc_Scale_Factor * (61.e0 - 479.e0 * tan2 + 179.e0 * tan4 - tan6) / 5040.e0;

        *Easting =
            TranMerc_False_Easting + dlam * t6 + pow(dlam, 3.e0) * t7 + pow(dlam, 5.e0) * t8 + pow(dlam, 7.e0) * t9;
    }
    return (Error_Code);
} /* END OF Convert_Geodetic_To_Transverse_Mercator */

long Convert_Transverse_Mercator_To_Geodetic(double Easting, double Northing, double *Latitude,
                                             double *Longitude) { /* BEGIN Convert_Transverse_Mercator_To_Geodetic */

    /*
     * The function Convert_Transverse_Mercator_To_Geodetic converts Transverse
     * Mercator projection (easting and northing) coordinates to geodetic
     * (latitude and longitude) coordinates, according to the current ellipsoid
     * and Transverse Mercator projection parameters.  If any errors occur, the
     * error code(s) are returned by the function, otherwise TRANMERC_NO_ERROR is
     * returned.
     *
     *    Easting       : Easting/X in meters                         (input)
     *    Northing      : Northing/Y in meters                        (input)
     *    Latitude      : Latitude in radians                         (output)
     *    Longitude     : Longitude in radians                        (output)
     */

    double c;    /* Cosine of latitude                          */
    double de;   /* Delta easting - Difference in Easting (Easting-Fe)    */
    double dlam; /* Delta longitude - Difference in Longitude       */
    double eta;  /* constant - TranMerc_ebs *c *c                   */
    double eta2;
    double eta3;
    double eta4;
    double ftphi; /* Footpoint latitude                              */
    int i;        /* Loop iterator                   */
    double s;     /* Sine of latitude                        */
    double sn;    /* Radius of curvature in the prime vertical       */
    double sr;    /* Radius of curvature in the meridian             */
    double t;     /* Tangent of latitude                             */
    double tan2;
    double tan4;
    double t10;  /* Term in coordinate conversion formula - GP to Y */
    double t11;  /* Term in coordinate conversion formula - GP to Y */
    double t12;  /* Term in coordinate conversion formula - GP to Y */
    double t13;  /* Term in coordinate conversion formula - GP to Y */
    double t14;  /* Term in coordinate conversion formula - GP to Y */
    double t15;  /* Term in coordinate conversion formula - GP to Y */
    double t16;  /* Term in coordinate conversion formula - GP to Y */
    double t17;  /* Term in coordinate conversion formula - GP to Y */
    double tmd;  /* True Meridional distance                        */
    double tmdo; /* True Meridional distance for latitude of origin */
    long Error_Code = TRANMERC_NO_ERROR;

    if ((Easting < (TranMerc_False_Easting - TranMerc_Delta_Easting)) ||
        (Easting > (TranMerc_False_Easting + TranMerc_Delta_Easting))) { /* Easting out of range  */
        Error_Code |= TRANMERC_EASTING_ERROR;
    }
    if ((Northing < (TranMerc_False_Northing - TranMerc_Delta_Northing)) ||
        (Northing > (TranMerc_False_Northing + TranMerc_Delta_Northing))) { /* Northing out of range */
        Error_Code |= TRANMERC_NORTHING_ERROR;
    }

    if (!Error_Code) {
        /* True Meridional Distances for latitude of origin */
        tmdo = SPHTMD(TranMerc_Origin_Lat);

        /*  Origin  */
        tmd = tmdo + (Northing - TranMerc_False_Northing) / TranMerc_Scale_Factor;

        /* First Estimate */
        sr = SPHSR(0.e0);
        ftphi = tmd / sr;

        for (i = 0; i < 5; i++) {
            t10 = SPHTMD(ftphi);
            sr = SPHSR(ftphi);
            ftphi = ftphi + (tmd - t10) / sr;
        }

        /* Radius of Curvature in the meridian */
        sr = SPHSR(ftphi);

        /* Radius of Curvature in the meridian */
        sn = SPHSN(ftphi);

        /* Sine Cosine terms */
        s = sin(ftphi);
        c = cos(ftphi);

        /* Tangent Value  */
        t = tan(ftphi);
        tan2 = t * t;
        tan4 = tan2 * tan2;
        eta = TranMerc_ebs * pow(c, 2);
        eta2 = eta * eta;
        eta3 = eta2 * eta;
        eta4 = eta3 * eta;
        de = Easting - TranMerc_False_Easting;
        if (fabs(de) < 0.0001) de = 0.0;

        /* Latitude */
        t10 = t / (2.e0 * sr * sn * pow(TranMerc_Scale_Factor, 2));
        t11 = t * (5.e0 + 3.e0 * tan2 + eta - 4.e0 * pow(eta, 2) - 9.e0 * tan2 * eta) /
              (24.e0 * sr * pow(sn, 3) * pow(TranMerc_Scale_Factor, 4));
        t12 = t *
              (61.e0 + 90.e0 * tan2 + 46.e0 * eta + 45.E0 * tan4 - 252.e0 * tan2 * eta - 3.e0 * eta2 + 100.e0 * eta3 -
               66.e0 * tan2 * eta2 - 90.e0 * tan4 * eta + 88.e0 * eta4 + 225.e0 * tan4 * eta2 + 84.e0 * tan2 * eta3 -
               192.e0 * tan2 * eta4) /
              (720.e0 * sr * pow(sn, 5) * pow(TranMerc_Scale_Factor, 6));
        t13 = t * (1385.e0 + 3633.e0 * tan2 + 4095.e0 * tan4 + 1575.e0 * pow(t, 6)) /
              (40320.e0 * sr * pow(sn, 7) * pow(TranMerc_Scale_Factor, 8));
        *Latitude = ftphi - pow(de, 2) * t10 + pow(de, 4) * t11 - pow(de, 6) * t12 + pow(de, 8) * t13;

        t14 = 1.e0 / (sn * c * TranMerc_Scale_Factor);

        t15 = (1.e0 + 2.e0 * tan2 + eta) / (6.e0 * pow(sn, 3) * c * pow(TranMerc_Scale_Factor, 3));

        t16 = (5.e0 + 6.e0 * eta + 28.e0 * tan2 - 3.e0 * eta2 + 8.e0 * tan2 * eta + 24.e0 * tan4 - 4.e0 * eta3 +
               4.e0 * tan2 * eta2 + 24.e0 * tan2 * eta3) /
              (120.e0 * pow(sn, 5) * c * pow(TranMerc_Scale_Factor, 5));

        t17 = (61.e0 + 662.e0 * tan2 + 1320.e0 * tan4 + 720.e0 * pow(t, 6)) /
              (5040.e0 * pow(sn, 7) * c * pow(TranMerc_Scale_Factor, 7));

        /* Difference in Longitude */
        dlam = de * t14 - pow(de, 3) * t15 + pow(de, 5) * t16 - pow(de, 7) * t17;

        /* Longitude */
        (*Longitude) = TranMerc_Origin_Long + dlam;

        if ((fabs)(*Latitude) > (90.0 * PI / 180.0)) Error_Code |= TRANMERC_NORTHING_ERROR;

        if ((*Longitude) > (PI)) {
            *Longitude -= (2 * PI);
            if ((fabs)(*Longitude) > PI) Error_Code |= TRANMERC_EASTING_ERROR;
        } else if ((*Longitude) < (-PI)) {
            *Longitude += (2 * PI);
            if ((fabs)(*Longitude) > PI) Error_Code |= TRANMERC_EASTING_ERROR;
        }

        if (fabs(dlam) > (9.0 * PI / 180) * cos(*Latitude)) { /* Distortion will result if Longitude is more than 9
                                                                 degrees from the Central Meridian at the equator */
            /* and decreases to 0 degrees at the poles */
            /* As you move towards the poles, distortion will become more significant */
            Error_Code |= TRANMERC_LON_WARNING;
        }
    }
    return (Error_Code);
} /* END OF Convert_Transverse_Mercator_To_Geodetic */
