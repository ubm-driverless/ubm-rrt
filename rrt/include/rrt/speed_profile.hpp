/**
 * @file speed_profile.hpp
 * @brief This header file contains the function implementations for calculating the speed profile of an open trajectory.
 * (i.e., start and end points are different)
 */

#pragma once

#include <armadillo>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <list>
#include <sstream>
#include <fstream>
#include <map>

#define ARMA_USE_BLAS
#define ARMA_BLAS_OPENBLAS
#define ARMA_USE_LAPACK

using namespace std;


/**
 * @brief Read a csv file and create a matrix (vector<vector<double>>) with its contents.
 * 
 * @param filename the path of the csv file.
 * @return vector<vector<double>>
 */
vector<vector<double>> csv_to_mat(const string& filename);

/**
 * @brief Extract a single column for a csv file and return it as a vector<doule>.
 * 
 * @param filename the path of the csv file.
 * @param column_name the name of the column to extract.
 * @return vector<double> 
 */
vector<double> extract_column(const string& filename, const string& column_name);

/**
 * @brief Parse the raceline.ini file and return its parameters as a map.
 * 
 * @param filename the path of the raceline.ini file.
 * @return map<string, string> 
 */
map<string, string> parse_raceline_ini(const string& filename);


/**
 * @brief Calculate the speed profile as a vector<float>
 * 
 *  @param ax_max_machines: longitudinal acceleration limits by the electrical motors: [vx, ax_max_machines]. Velocity
 *                          in m/s, accelerations in m/s2. They should be handed in without considering drag resistance,
 *                          i.e. simply by calculating F_x_drivetrain / m_veh
 *  @param kappa:           curvature profile of given trajectory in rad/m (always unclosed).
 *  @param el_lengths:      element lengths (distances between coordinates) of given trajectory.
 *  @param closed:          flag to set if the velocity profile must be calculated for a closed or unclosed trajectory.
 *  @param drag_coeff:      drag coefficient including all constants: drag_coeff = 0.5 * c_w * A_front * rho_air
 *  @param m_veh:           vehicle mass in kg.
 *  @param ggv:             ggv-diagram to be applied: [vx, ax_max, ay_max]. Velocity in m/s, accelerations in m/s2.
 *  @param v_max:           Maximum longitudinal speed in m/s (optional if ggv is supplied, taking the minimum of the
 *                          fastest velocities covered by the ggv and ax_max_machines arrays then).
 *  @param dyn_model_exp:   exponent used in the vehicle dynamics model (usual range [1.0,2.0]).
 *  @param mu:              friction coefficients (always unclosed).
 *  @param v_start:         start velocity in m/s (used in unclosed case only).
 *  @param v_end:           end velocity in m/s (used in unclosed case only).
 *  @param filt_window:     filter window size for moving average filter (must be odd).
 */
vector<float> calc_vel_profile(const vector<vector<double>>& ax_max_machines,
                                    const vector<double>& kappa,
                                    const vector<double>& el_lengths,
                                    double drag_coeff,
                                    double m_veh,
                                    const vector<vector<double>>& ggv = vector<vector<double>>(),
                                    double v_max = numeric_limits<double>::quiet_NaN(),
                                    double dyn_model_exp = 1.0,
                                    const vector<double>& mu = vector<double>(),
                                    double v_start = numeric_limits<double>::quiet_NaN(),
                                    double v_end = numeric_limits<double>::quiet_NaN(),
                                    int filt_window = 0);

/**
 * @brief Not to be called outside of the speed profile source code.
 *        Calculate the possible acceleration for a point in the raceline.
 * 
 * @param vx_start          current velocity of the point
 * @param radius            current curvature of the point in radii
 * @param ggv               ggv profile
 * @param mu                friction coefficients
 * @param dyn_model_exp     exponent used in the vehicle dynamics model
 * @param drag_coeff        drag coefficient
 * @param m_veh             vehicle mass in kg
 * @param ax_max_machines   longitudinal acceleration limits by the electrical motors: [vx, ax_max_machines]. Velocity
 *                          in m/s, accelerations in m/s2.
 * @param mode              either "accel_forw" for forward calculation or "decel_backw" for backwards calculation
 * @return double           acceleration
 */
double calc_ax_poss(double vx_start,
                    double radius,
                    const arma::mat& ggv,
                    double mu,
                    double dyn_model_exp,
                    double drag_coeff,
                    double m_veh,
                    const arma::mat& ax_max_machines,
                    const string& mode);

/**
 * @brief Not to be called outside of the speed profile source code.
 *        Calculate the speed profile of an unclosed section of the raceline.
 * 
 * @param p_ggv             ggv-diagram to be applied: [vx, ax_max, ay_max]. Velocity in m/s, accelerations in m/s2. Copied for each point of the raceline.
 * @param ax_max_machines   longitudinal acceleration limits by the electrical motors: [vx, ax_max_machines]. Velocity
 *                          in m/s, accelerations in m/s2.
 * @param v_max             maximum velocity in m/s.
 * @param radii             curvature of every point in the raceline in radii.
 * @param el_lengths        lenghts of every segment of the raceline.
 * @param v_start           the velocity of the first point in the raceline.
 * @param drag_coeff        drag coefficients.
 * @param m_veh             vehicle mass in kg.
 * @param op_mode           should always be "ggv".
 * @param mu                friction coefficients.
 * @param v_end             velocity of the last point in the raceline.
 * @param dyn_model_exp     exponent used in the vehicle dynamics model.
 * @return arma::vec        the speed profile.
 */
arma::vec __solver_fb_unclosed(const arma::cube& p_ggv,
                               const arma::mat& ax_max_machines,
                               double v_max,
                               const arma::vec& radii,
                               const arma::vec& el_lengths,
                               double v_start,
                               double drag_coeff,
                               double m_veh,
                               const string& op_mode,
                               const arma::vec& mu = arma::vec(),
                               double v_end = numeric_limits<double>::quiet_NaN(),
                               double dyn_model_exp = 1.0);

/**
 * @brief Not to be called outside of the speed profile source code.
 *        Calculate the speed profile of the raceline either by forward acceleration or backwards deceleration.
 * 
 * @param p_ggv             ggv-diagram to be applied: [vx, ax_max, ay_max]. Velocity in m/s, accelerations in m/s2. Copied for each point of the raceline.
 * @param ax_max_machines   longitudinal acceleration limits by the electrical motors: [vx, ax_max_machines]. Velocity
 *                          in m/s, accelerations in m/s2.
 * @param v_max             maximum velocity in m/s.
 * @param radii             curvature of every point in the raceline in radii.
 * @param el_lengths        lenghts of every segment of the raceline.
 * @param mu                friction coefficients.
 * @param vx_profile        current velocity profile.
 * @param drag_coeff        drag coefficient including all constants: drag_coeff = 0.5 * c_w * A_front * rho_air
 * @param m_veh             vehicle mass in kg.
 * @param dyn_model_exp     exponent used in the vehicle dynamics model.
 * @param backwards         true for backwards deceleration, false otherwise.
 * @return arma::vec        the speed profile.
 */
arma::vec __solver_fb_acc_profile(const arma::cube& p_ggv,
                                  const arma::mat& ax_max_machines,
                                  double v_max,
                                  const arma::vec& radii,
                                  const arma::vec& el_lengths,
                                  const arma::vec& mu,
                                  arma::vec& vx_profile,
                                  double drag_coeff,
                                  double m_veh,
                                  double dyn_model_exp = 1.0,
                                  bool backwards = false);
