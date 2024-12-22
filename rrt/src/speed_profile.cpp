#include <rrt/speed_profile.hpp>


vector<vector<double>> csv_to_mat(const string& filename) {
    ifstream file(filename);
    string line;
    vector<vector<double>> values;
    size_t rows = 0, cols = 0;

    if (file.is_open()) {
        // Skip heading line
        getline(file, line);
        while (getline(file, line)) {
            stringstream ss(line);
            string cell;
            vector<double> row_values;
            while (getline(ss, cell, ',')) {
                row_values.push_back(stod(cell));
            }
            if (cols == 0) {
                cols = row_values.size();
            }
            values.push_back(row_values);
            rows++;
        }
        file.close();
    } else {
        throw runtime_error("Could not open file");
    }

    return values;
}


vector<double> extract_column(const string& filename, const string& column_name) {
    ifstream file(filename);
    string line;
    vector<double> column_values;
    bool column_found = false;
    int column_index = -1;

    if (file.is_open()) {
        // Header line is the third line in the file
        // getline(file, line);
        // getline(file, line);
        // Read the header line
        if (getline(file, line)) {
            stringstream ss(line);
            string header_cell;
            int index = 0;

            // Find the index of the target column
            while (getline(ss, header_cell, ';')) {
                if (header_cell == column_name) {
                    column_index = index;
                    column_found = true;
                    break;
                }
                ++index;
            }
        }

        // If the column is found, extract its values
        if (column_found) {
            while (getline(file, line)) {
                stringstream ss(line);
                string cell;
                int current_index = 0;

                // Navigate to the target column
                while (getline(ss, cell, ';')) {
                    if (current_index == column_index) {
                        column_values.push_back(stod(cell));
                        break;
                    }
                    ++current_index;
                }
            }
        } else {
            throw runtime_error("Column not found");
        }

        file.close();
    } else {
        throw runtime_error("Could not open file");
    }

    return column_values;
}


map<string, string> parse_raceline_ini(const string& filename) {
    map<string, string> parameters;
    ifstream file(filename);

    if (!file.is_open()) {
        throw runtime_error("Unable to open file");
    }

    string line;
    while (getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Split the line into parameter name and value
        stringstream ss(line);
        string param_name, param_value;
        getline(ss, param_name, '=');
        getline(ss, param_value);

        // Remove leading and trailing whitespace from parameter name and value
        param_name.erase(0, param_name.find_first_not_of(" \t"));
        param_name.erase(param_name.find_last_not_of(" \t") + 1);
        param_value.erase(0, param_value.find_first_not_of(" \t"));
        param_value.erase(param_value.find_last_not_of(" \t") + 1);

        // Store the parameter name and value
        parameters[param_name] = param_value;
    }

    file.close();
    return parameters;
}




vector<float> calc_vel_profile(const vector<vector<double>>& _ax_max_machines,
                                    const vector<double>& _kappa,
                                    const vector<double>& _el_lengths,
                                    double drag_coeff,
                                    double m_veh,
                                    const vector<vector<double>>& _ggv,
                                    // const vector<vector<double>>& loc_gg, // we don't need it
                                    double v_max,
                                    double dyn_model_exp,
                                    const vector<double>& _mu,
                                    double v_start,
                                    double v_end,
                                    int filt_window) {

    // funcion to turn std matrices into arma matrices
    auto stdtoArmaMatrix = [](const vector<vector<double>>& inputMatrix) -> arma::mat {
        size_t rows = inputMatrix.size();
        size_t cols = inputMatrix[0].size();
        
        arma::mat outputMatrix(rows, cols);
        
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                outputMatrix(i, j) = inputMatrix[i][j];
            }
        }
        
        return outputMatrix;
    };
    
    // Turning std matrices to arma matrices:
    const arma::mat ggv = stdtoArmaMatrix(_ggv);
    const arma::mat ax_max_machines = stdtoArmaMatrix(_ax_max_machines);

    // Turning the std vectors into armadillo ones:
    const arma::vec kappa(_kappa);
    const arma::vec tmp(_el_lengths);
    const arma::vec el_lengths = arma::diff(tmp);
    const arma::vec mu(_mu);


    // INPUT CHECKS
    // check if either ggv (and optionally mu) or loc_gg are handed in
    if (ggv.is_empty()) {
        throw runtime_error("ggvmust be supplied");
    }

    // check shape of ggv
    if (!ggv.is_empty() && ggv.n_cols != 3) {
        throw runtime_error("ggv diagram must consist of the three columns [vx, ax_max, ay_max]!");
    }

    // check size of mu
    if (!mu.is_empty() && kappa.n_elem != mu.n_elem) {
        throw runtime_error("kappa and mu must have the same length!");
    }

    if (!isnan(v_start) && v_start < 0.0) {
        v_start = 0.0;
        cout << "WARNING: Input v_start was < 0.0. Using v_start = 0.0 instead!" << endl;
    }

    if (!isnan(v_end) && v_end < 0.0) {
        v_end = 0.0;
        cout << "WARNING: Input v_end was < 0.0. Using v_end = 0.0 instead!" << endl;
    }

    // check dyn_model_exp
    if (dyn_model_exp < 1.0 || dyn_model_exp > 2.0) {
        cout << "WARNING: Exponent for the vehicle dynamics model should be in the range [1.0, 2.0]!" << endl;
    }

    // check shape of ax_max_machines
    if (ax_max_machines.n_cols != 2) {
        throw runtime_error("ax_max_machines must consist of the two columns [vx, ax_max_machines]!");
    }

    // check v_max
    if (isnan(v_max)) {
        if (ggv.is_empty()) {
            throw runtime_error("v_max must be supplied if ggv is None!");
        } else {
            v_max = min(ggv(ggv.n_rows - 1, 0), ax_max_machines(ax_max_machines.n_rows - 1, 0));
        }
    } else {
        // check if ggv covers velocity until v_max
        if (!ggv.is_empty() && ggv(ggv.n_rows - 1, 0) < v_max) {
            throw runtime_error("ggv has to cover the entire velocity range of the car (i.e. >= v_max)!");
        }

        // check if ax_max_machines covers velocity until v_max
        if (ax_max_machines(ax_max_machines.n_rows - 1, 0) < v_max) {
            throw runtime_error("ax_max_machines has to cover the entire velocity range of the car (i.e. >= v_max)!");
        }
    }

    // BRINGING GGV OR LOC_GG INTO SHAPE FOR EQUAL HANDLING AFTERWARDS
    arma::cube p_ggv;

    // CASE 1: ggv supplied -> copy it for every waypoint
    if (!ggv.is_empty()) {
        p_ggv = arma::cube(ggv.n_rows, 3, kappa.n_elem); // kappa.n_elems is depth of the cube
        // fflush(stdout);
        // cout << "  Size of slice: (" << p_ggv.row(0).n_rows << ", " << p_ggv.row(0).n_cols <<  ")" << endl;
        arma::mat slice = p_ggv.slice(0);
        for (size_t i = 0; i < kappa.n_elem; ++i) {
            p_ggv.slice(i) = ggv;
        }
    }

    // SPEED PROFILE CALCULATION (FB)

    //arma::vec radii = arma::abs(1.0 / kappa);
    arma::vec radii(kappa.size());
    for (size_t i = 0; i < kappa.size(); ++i) {
        if (kappa[i] != 0.0) {
            radii[i] = 1.0 / abs(kappa[i]);
        } else {
            radii[i] = arma::datum::inf; // Set to infinity or any other suitable value
        }
    }

    arma::vec vx_profile;
    vx_profile = __solver_fb_unclosed(p_ggv,
                                    ax_max_machines,
                                    v_max,
                                    radii,
                                    el_lengths,
                                    v_start,
                                    drag_coeff,
                                    m_veh,
                                    "ggv",
                                    mu,
                                    v_end,
                                    dyn_model_exp);


    // POSTPROCESSING
    if (filt_window > 0) {
        vx_profile = arma::conv(vx_profile, arma::ones(filt_window) / filt_window, "same");
    }

    return vector<float>(vx_profile.begin(), vx_profile.end());
}


arma::vec __solver_fb_unclosed(const arma::cube& p_ggv,
                               const arma::mat& ax_max_machines,
                               double v_max,
                               const arma::vec& radii,
                               const arma::vec& el_lengths,
                               double v_start,
                               double drag_coeff,
                               double m_veh,
                               const string& op_mode,
                               const arma::vec& mu,
                               double v_end,
                               double dyn_model_exp) {
    // handle mu
    arma::vec mu_local = mu.is_empty() ? arma::ones(radii.n_elem) : mu;
    double mu_mean = arma::mean(mu_local);

    // initial velocity profile estimate
    arma::vec vx_profile;
    if (op_mode == "ggv") {
        double ay_max_global = mu_mean * p_ggv.slice(0).col(2).min();
        vx_profile = arma::sqrt(radii * ay_max_global);
        arma::vec interpRes;
        arma::vec yValues = p_ggv.slice(0).col(2);
        arma::interp1(p_ggv.slice(0).col(0),
                      p_ggv.slice(0).col(2),
                      vx_profile, interpRes,
                      "linear",
                      yValues(yValues.n_elem - 1));
        arma::vec ay_max_curr = mu_local % interpRes;
        vx_profile = arma::sqrt(ay_max_curr % radii);
    } else {
        //double ay_max_global = p_ggv.slice(2).col(0);
        vx_profile = arma::sqrt(radii % arma::mat(p_ggv.row(0)).col(2));
    }

    // Cut vx_profile to car's top speed
    vx_profile.elem(arma::find(vx_profile > v_max)).fill(v_max);

    // Consider v_start
    // if (vx_profile(0) > v_start) {
    vx_profile(0) = v_start;
    // }

    // forwards and backwards calculation of the acceleration profile
    vx_profile = __solver_fb_acc_profile(p_ggv,
                                        ax_max_machines,
                                        v_max,
                                        radii,
                                        el_lengths,
                                        mu_local,
                                        vx_profile,
                                        drag_coeff,
                                        m_veh,
                                        dyn_model_exp,
                                        false);


    if (!isnan(v_end) && *(vx_profile.end() - 1) > v_end) {
        vx_profile.at(vx_profile.n_elem - 1) = v_end;
    }


    vx_profile = __solver_fb_acc_profile(p_ggv,
                                        ax_max_machines,
                                        v_max,
                                        radii,
                                        el_lengths,
                                        mu_local,
                                        vx_profile,
                                        drag_coeff,
                                        m_veh,
                                        dyn_model_exp,
                                        true);


    return vx_profile;
}


arma::vec __solver_fb_acc_profile(const arma::cube& p_ggv,
                                  const arma::mat& ax_max_machines,
                                  double v_max,
                                  const arma::vec& radii,
                                  const arma::vec& el_lengths,
                                  const arma::vec& mu,
                                  arma::vec& vx_profile,
                                  double drag_coeff,
                                  double m_veh,
                                  double dyn_model_exp,
                                  bool backwards) {
    // int n_segments = el_lengths.n_elem;
    unsigned int no_points = vx_profile.n_elem;

    arma::vec radii_mod;
    arma::vec el_lengths_mod;
    arma::vec mu_mod;
    string mode;
    arma::vec vx_profile_mod;

    if (backwards) {
        // Flip vectors
        radii_mod = arma::flipud(radii);
        el_lengths_mod = arma::flipud(el_lengths);
        mu_mod = arma::flipud(mu);
        vx_profile_mod = arma::flipud(vx_profile);
        mode = "decel_backw";
    } else {
        // No modification needed
        radii_mod = radii;
        el_lengths_mod = el_lengths;
        mu_mod = mu;
        vx_profile_mod = vx_profile;
        mode = "accel_forw";
    }

    arma::vec vx_diffs = arma::diff(vx_profile_mod);
    arma::uvec acc_inds = arma::find(vx_diffs > 0.0); // indices of points with positive acceleration
    arma::uvec acc_inds_rel = arma::uvec();

    if (acc_inds.n_elem != 0 ) {
        // check index diffs -> we only need the first point of every acceleration phase
        // Compute the differences between consecutive indices
        arma::uvec acc_inds_diffs = arma::diff(acc_inds);

        // Insert 2 at the beginning of acc_inds_diffs
        acc_inds_diffs.insert_rows(0, 1);
        acc_inds_diffs[0] = 2;

        // Extract indices where acc_inds_diffs > 1
        arma::uvec indices = arma::find(acc_inds_diffs > 1);
        acc_inds_rel = acc_inds.elem(indices);
    } else {
        acc_inds_rel.reset();
    }


    // list<arma::u64>
    arma::u64* curr_ind = acc_inds_rel.begin(); // pointer to i in the python code
    while (curr_ind != acc_inds_rel.end()) {
        unsigned long i = *curr_ind;
        while (i < no_points - 1) {
            double ax_possible_cur = calc_ax_poss(vx_profile_mod(i), radii_mod(i), p_ggv.slice(i), mu_mod(i), dyn_model_exp, drag_coeff, m_veh, ax_max_machines, mode);
            double vx_possible_next = sqrt(pow(vx_profile_mod(i), 2) + 2 * ax_possible_cur * el_lengths_mod(i));
            if (backwards) {
                /*
                We have to loop the calculation if we are in the backwards iteration (currently just once). This is
                because we calculate the possible ax at a point i which does not necessarily fit for point i + 1
                (which is i - 1 in the real direction). At point i + 1 (or i - 1 in real direction) we have a different
                start velocity (vx_possible_next), radius and mu value while the absolute value of ax remains the same
                in both directions.
                */
                for(int j = 0; j <1; ++j){
                    double ax_possible_next = calc_ax_poss(vx_profile_mod(i+1), radii_mod(i+1), p_ggv.slice(i+1), mu_mod(i+1), dyn_model_exp, drag_coeff, m_veh, ax_max_machines, mode);
                    double vx_tmp = sqrt(pow(vx_profile_mod(i), 2) + 2 * ax_possible_next * el_lengths_mod(i));
                    if (vx_tmp < vx_possible_next) {
                        vx_possible_next = vx_tmp;
                    } else {
                        break;
                    }
                }
            }
            // save possible next velocity if it is smaller than the current value
            if (vx_possible_next < vx_profile_mod(i + 1)){
                vx_profile_mod(i + 1) = vx_possible_next;
            }

            i++;

            // break current acceleration phase if next speed would be higher than the maximum vehicle velocity or if we
            //are at the next acceleration phase start index
            if ((vx_possible_next > v_max)) {
                // Break if i >= of next value in acc_inds_rel
                if (!(curr_ind + 1) && i >= acc_inds_rel(*(curr_ind + 1))) {
                    break;
                }
            }
        }
        curr_ind++;
    }
    // ------------------------------------------------------------------------------------------------------------------
    // POSTPROCESSING ---------------------------------------------------------------------------------------------------
    // ------------------------------------------------------------------------------------------------------------------

    // flip output vel_profile if necessary
    if (backwards){
        vx_profile_mod = arma::flipud(vx_profile_mod);
    }

    return vx_profile_mod;
}


double calc_ax_poss(double vx_start,
                    double radius,
                    const arma::mat& ggv,
                    double mu,
                    double dyn_model_exp,
                    double drag_coeff,
                    double m_veh,
                    const arma::mat& ax_max_machines,
                    const string& mode) {
    // calculate possible and used accelerations (considering tires)
    arma::vec interpRes1;
    arma::vec interpRes2;
    arma::vec toInterp = arma::vec(1);
    toInterp(0) = vx_start;
    arma::interp1(ggv.col(0), ggv.col(1), toInterp, interpRes1);
    arma::interp1(ggv.col(0), ggv.col(2), toInterp, interpRes2);
    double ax_max_tires = mu * interpRes1(0);               //arma::interp1(vx_start, ggv[:, 0], ggv[:, 1])
    double ay_max_tires = mu * interpRes2(0);                           //arma::interp1(vx_start, ggv[:, 0], ggv[:, 2])
    double ay_used = pow(vx_start, 2) / radius;


    // during forward acceleration and backward deceleration ax_max_tires must be considered positive, during forward
    // deceleration it must be considered negative
    if ((mode == "accel_forw" || mode == "decel_backw") && ax_max_tires < 0.0) {
        cout << "WARNING: Inverting sign of ax_max_tires because it should be positive but was negative!" << endl;
        ax_max_tires *= -1.0;
    } else if (mode == "decel_forw" && ax_max_tires > 0.0) {
        cout << "WARNING: Inverting sign of ax_max_tires because it should be negative but was positve!" << endl;
        ax_max_tires *= -1.0;
    }


    double radicand = 1.0 - pow(ay_used / ay_max_tires, dyn_model_exp);

    double ax_avail_tires;
    if (radicand > 0.0) {
        ax_avail_tires = ax_max_tires * pow(radicand, 1.0 / dyn_model_exp);
    } else {
        ax_avail_tires = 0.0;
    }

    // ------------------------------------------------------------------------------------------------------------------
    // CONSIDER MACHINE LIMITATIONS -------------------------------------------------------------------------------------
    // ------------------------------------------------------------------------------------------------------------------

    // consider limitations imposed by electrical machines during forward acceleration
    double ax_avail_vehicle;
    if (mode == "accel_forw") {
        // interpolate machine acceleration to be able to consider varying gear ratios, efficiencies etc.
        arma::vec ax_max_machines_tmp;
        arma::interp1(ax_max_machines.col(0), ax_max_machines.col(1), toInterp, ax_max_machines_tmp);
        ax_avail_vehicle = ax_avail_tires < ax_max_machines_tmp(0) ? ax_avail_tires : ax_max_machines_tmp(0);
    } else {
        ax_avail_vehicle = ax_avail_tires;
    }

    // ------------------------------------------------------------------------------------------------------------------
    // CONSIDER DRAG ----------------------------------------------------------------------------------------------------
    // ------------------------------------------------------------------------------------------------------------------

    // # calculate equivalent longitudinal acceleration of drag force at the current speed
    double ax_drag = -pow(vx_start, 2) * drag_coeff / m_veh;
    double ax_final;
    // drag reduces the possible acceleration in the forward case and increases it in the backward case
    if (mode == "accel_forw" || mode == "decel_forw") {
        ax_final = ax_avail_vehicle + ax_drag;
        // attention: this value will now be negative in forward direction if tire is entirely used for cornering
    } else {
        ax_final = ax_avail_vehicle - ax_drag;
    }
    return ax_final;
}
