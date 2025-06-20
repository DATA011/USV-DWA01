#ifndef CUSVCOMMON_H
#define CUSVCOMMON_H
namespace CUSV
{
typedef enum
{
    LSL = 0,
    LSR = 1,
    RSL = 2,
    RSR = 3,
    RLR = 4,
    LRL = 5
} DubinsPathType;

typedef struct
{
    /* the initial configuration */
    double qi[3];
    /* the lengths of the three segments */
    double param[3];
    /* model forward velocity / model angular velocity */
    double rho;
    /* the path type described */
    DubinsPathType type;
} DubinsPath;

struct PID_param  // 目前仅为航向转变使用，航速控制采用其他策略
{
    /* data 角度制*/
    float KP = 1;
    float KI = 0.01;
    float KD = 0.0;
    float I_max = 2.0;
    float I_min = -2.0;
    float Out_max = 10.0; //最大角速度
    float Out_min = -10.0;
    float error = 0.0;
    float last_error = 0.0;

    float p_output =0.0;
    float i_output = 0.0;
    float d_output = 0.0;
};

double get_radius(float v, int type);

void xy2lonlat(double lonA, double latA, double* dbpos,double XA, double YA, double XB, double YB);
double* lonlat2xy(double dbx, double dby, double latA, double lonA, double latB, double lonB);
double mod2pi(double theta);
double mod360deg(double theta);
float StateTransMethod(float state_desire, float state_last, float t, float fCurrTIme, float deltat);



#define EDUBOK        (0)   /* No error */
#define EDUBCOCONFIGS (1)   /* Colocated configurations */
#define EDUBPARAM     (2)   /* Path parameterisitation error */
#define EDUBBADRHO    (3)   /* the rho value is invalid */
#define EDUBNOPATH    (4)   /* no connection between configurations with this word */

/**
 * Callback function for path sampling
 *
 * @note the q parameter is a configuration
 * @note the t parameter is the distance along the path
 * @note the user_data parameter is forwarded from the caller
 * @note return non-zero to denote sampling should be stopped
 */
typedef int (*DubinsPathSamplingCallback)(double q[3], double t, void* user_data);

/**
 * Generate a path from an initial configuration to
 * a target configuration, with a specified maximum turning
 * radii
 *
 * A configuration is (x, y, theta), where theta is in radians, with zero
 * along the line x = 0, and counter-clockwise is positive
 *
 * @param path  - the resultant path
 * @param q0    - a configuration specified as an array of x, y, theta
 * @param q1    - a configuration specified as an array of x, y, theta
 * @param rho   - turning radius of the vehicle (forward velocity divided by maximum angular velocity)
 * @return      - non-zero on error
 */
int dubins_shortest_path(DubinsPath* path, double q0[3], double q1[3], double rho);

/**
 * Generate a path with a specified word from an initial configuration to
 * a target configuration, with a specified turning radius
 *
 * @param path     - the resultant path
 * @param q0       - a configuration specified as an array of x, y, theta
 * @param q1       - a configuration specified as an array of x, y, theta
 * @param rho      - turning radius of the vehicle (forward velocity divided by maximum angular velocity)
 * @param pathType - the specific path type to use
 * @return         - non-zero on error
 */
int dubins_path(DubinsPath* path, double q0[3], double q1[3], double rho, DubinsPathType pathType);

/**
 * Calculate the length of an initialised path
 *
 * @param path - the path to find the length of
 */
double dubins_path_length(DubinsPath* path);

/**
 * Return the length of a specific segment in an initialized path
 *
 * @param path - the path to find the length of
 * @param i    - the segment you to get the length of (0-2)
 */
double dubins_segment_length(DubinsPath* path, int i);

/**
 * Return the normalized length of a specific segment in an initialized path
 *
 * @param path - the path to find the length of
 * @param i    - the segment you to get the length of (0-2)
 */
double dubins_segment_length_normalized( DubinsPath* path, int i );

/**
 * Extract an integer that represents which path type was used
 *
 * @param path    - an initialised path
 * @return        - one of LSL, LSR, RSL, RSR, RLR or LRL
 */
DubinsPathType dubins_path_type(DubinsPath* path);

/**
 * Calculate the configuration along the path, using the parameter t
 *
 * @param path - an initialised path
 * @param t    - a length measure, where 0 <= t < dubins_path_length(path)
 * @param q    - the configuration result
 * @returns    - non-zero if 't' is not in the correct range
 */
int dubins_path_sample(DubinsPath* path, double t, double q[3]);

/**
 * Walk along the path at a fixed sampling interval, calling the
 * callback function at each interval
 *
 * The sampling process continues until the whole path is sampled, or the callback returns a non-zero value
 *
 * @param path      - the path to sample
 * @param stepSize  - the distance along the path for subsequent samples
 * @param cb        - the callback function to call for each sample
 * @param user_data - optional information to pass on to the callback
 *
 * @returns - zero on successful completion, or the result of the callback
 */
int dubins_path_sample_many(DubinsPath* path,
                            double stepSize,
                            DubinsPathSamplingCallback cb,
                            void* user_data);

/**
 * Convenience function to identify the endpoint of a path
 *
 * @param path - an initialised path
 * @param q    - the configuration result
 */
int dubins_path_endpoint(DubinsPath* path, double q[3]);

/**
 * Convenience function to extract a subset of a path
 *
 * @param path    - an initialised path
 * @param t       - a length measure, where 0 < t < dubins_path_length(path)
 * @param newpath - the resultant path
 */
int dubins_extract_subpath(DubinsPath* path, double t, DubinsPath* newpath);
}
#endif // CUSVMODEL_H

