#ifndef UTM_H
#define UTM_H

/***************************************************************************/
/* RSC IDENTIFIER: UTM
 *
 * ABSTRACT
 *
 *    This component provides conversions between geodetic coordinates
 *    (latitude and longitudes) and Universal Transverse Mercator (UTM)
 *    projection (zone, hemisphere, easting, and northing) coordinates.
 *
 * ERROR HANDLING
 *
 *    This component checks parameters for valid values.  If an invalid value
 *    is found, the error code is combined with the current error code using
 *    the bitwise or.  This combining allows multiple error codes to be
 *    returned. The possible error codes are:
 *
 *          UTM_NO_ERROR           : No errors occurred in function
 *          UTM_LAT_ERROR          : Latitude outside of valid range
 *                                    (-80.5 to 84.5 degrees)
 *          UTM_LON_ERROR          : Longitude outside of valid range
 *                                    (-180 to 360 degrees)
 *          UTM_EASTING_ERROR      : Easting outside of valid range
 *                                    (100,000 to 900,000 meters)
 *          UTM_NORTHING_ERROR     : Northing outside of valid range
 *                                    (0 to 10,000,000 meters)
 *          UTM_ZONE_ERROR         : Zone outside of valid range (1 to 60)
 *          UTM_HEMISPHERE_ERROR   : Invalid hemisphere ('N' or 'S')
 *          UTM_ZONE_OVERRIDE_ERROR: Zone outside of valid range
 *                                    (1 to 60) and within 1 of 'natural' zone
 *          UTM_A_ERROR            : Semi-major axis less than or equal to zero
 *          UTM_INV_F_ERROR        : Inverse flattening outside of valid range
 *								  	                (250 to 350)
 *
 * REUSE NOTES
 *
 *    UTM is intended for reuse by any application that performs a Universal
 *    Transverse Mercator (UTM) projection or its inverse.
 *
 * REFERENCES
 *
 *    Further information on UTM can be found in the Reuse Manual.
 *
 *    UTM originated from :  U.S. Army Topographic Engineering Center
 *                           Geospatial Information Division
 *                           7701 Telegraph Road
 *                           Alexandria, VA  22310-3864
 *
 * LICENSES
 *
 *    None apply to this component.
 *
 * RESTRICTIONS
 *
 *    UTM has no restrictions.
 *
 * ENVIRONMENT
 *
 *    UTM was tested and certified in the following environments:
 *
 *    1. Solaris 2.5 with GCC, version 2.8.1
 *    2. MSDOS with MS Visual C++, version 6
 *
 * MODIFICATIONS
 *
 *    Date              Description
 *    ----              -----------
 *    10-02-97          Original Code
 *
 */

/***************************************************************************/
/*
 *                              DEFINES
 */

#define UTM_NO_ERROR 0x0000
#define UTM_LAT_ERROR 0x0001
#define UTM_LON_ERROR 0x0002
#define UTM_EASTING_ERROR 0x0004
#define UTM_NORTHING_ERROR 0x0008
#define UTM_ZONE_ERROR 0x0010
#define UTM_HEMISPHERE_ERROR 0x0020
#define UTM_ZONE_OVERRIDE_ERROR 0x0040
#define UTM_A_ERROR 0x0080
#define UTM_INV_F_ERROR 0x0100

/***************************************************************************/
/*
 *                              FUNCTION PROTOTYPES
 *                                for UTM.C
 */

/* ensure proper linkage to c++ programs */
//  #ifdef __cplusplus
// extern "C" {
//  #endif

long Set_UTM_Parameters(double a, double f, long override);
/*
 * The function Set_UTM_Parameters receives the ellipsoid parameters and
 * UTM zone override parameter as inputs, and sets the corresponding state
 * variables.  If any errors occur, the error code(s) are returned by the
 * function, otherwise UTM_NO_ERROR is returned.
 *
 *    a                 : Semi-major axis of ellipsoid, in meters       (input)
 *    f                 : Flattening of ellipsoid                       (input)
 *    override          : UTM override zone, zero indicates no override (input)
 */

void Get_UTM_Parameters(double *a, double *f, long *override);
/*
 * The function Get_UTM_Parameters returns the current ellipsoid
 * parameters and UTM zone override parameter.
 *
 *    a                 : Semi-major axis of ellipsoid, in meters       (output)
 *    f                 : Flattening of ellipsoid                       (output)
 *    override          : UTM override zone, zero indicates no override (output)
 */

long Convert_Geodetic_To_UTM(double Latitude, double Longitude, long *Zone, char *Hemisphere, double *Easting,
                             double *Northing);
/*
 * The function Convert_Geodetic_To_UTM converts geodetic (latitude and
 * longitude) coordinates to UTM projection (zone, hemisphere, easting and
 * northing) coordinates according to the current ellipsoid and UTM zone
 * override parameters.  If any errors occur, the error code(s) are returned
 * by the function, otherwise UTM_NO_ERROR is returned.
 *
 *    Latitude          : Latitude in radians                 (input)
 *    Longitude         : Longitude in radians                (input)
 *    Zone              : UTM zone                            (output)
 *    Hemisphere        : North or South hemisphere           (output)
 *    Easting           : Easting (X) in meters               (output)
 *    Northing          : Northing (Y) in meters              (output)
 */

long Convert_UTM_To_Geodetic(long Zone, char Hemisphere, double Easting, double Northing, double *Latitude,
                             double *Longitude);
/*
 * The function Convert_UTM_To_Geodetic converts UTM projection (zone,
 * hemisphere, easting and northing) coordinates to geodetic(latitude
 * and  longitude) coordinates, according to the current ellipsoid
 * parameters.  If any errors occur, the error code(s) are returned
 * by the function, otherwise UTM_NO_ERROR is returned.
 *
 *    Zone              : UTM zone                               (input)
 *    Hemisphere        : North or South hemisphere              (input)
 *    Easting           : Easting (X) in meters                  (input)
 *    Northing          : Northing (Y) in meters                 (input)
 *    Latitude          : Latitude in radians                    (output)
 *    Longitude         : Longitude in radians                   (output)
 */

//  #ifdef __cplusplus
//}
//  #endif

#endif /* UTM_H */
