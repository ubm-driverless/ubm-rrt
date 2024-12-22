#include <rrt/speed_profile.hpp>


///////////////////////////////////////////////////////////////////////////////////////////////////
//// Importing our Data ///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


std::string ax_max_machines_path = "/home/ubm/repo/raceline/raceline/TUM/inputs/veh_dyn_info/ax_max_machines.csv"; // arma::mat& ax_max_machines
std::string ggv_path = "/home/ubm/repo/raceline/raceline/TUM/inputs/veh_dyn_info/ggv.csv";
//arma::vec& kappa,
//
std::string TUM_raceline = "/home/ubm/repo/raceline/csv/TUM_raceline/1mar.csv";

// we need these parameters from raceline_ini
// inside vehicle_params
// mu is optim_opts_mintime["mue"]      // arma::vec& mu = arma::vec()
// mu should be the same size as the kappa but it's a constant in raeline_ini
// mu, v_max, m_veh (mass), drag_coef (dragcoef)
std::string raceline_ini = "/home/ubm/repo/raceline/raceline/TUM/params/racecar.ini";



int main() {
    // Placeholder example usage
    const std::vector<double> lengths = extract_column(TUM_raceline, "# s_m");
    const std::vector<double> kappa = extract_column(TUM_raceline, " kappa_radpm");
    const vector<vector<double>> ggv = csv_to_mat(ggv_path);
    const vector<vector<double>> ax_max_machines = csv_to_mat(ax_max_machines_path);
    double drag_coeff = 0.200;
    double v_max = 11.000;
    double m_veh = 3.158;
    double dyn_model_exp = 1.0;

    vector<float> speed = calc_vel_profile(ax_max_machines,
                                           kappa,
                                           lengths,
                                           drag_coeff,
                                           m_veh,
                                           ggv,
                                           v_max,
                                           dyn_model_exp,
                                           std::vector<double>(),
                                           0,
                                           0);

    for (auto& value : speed) {
        cout << value << endl;
    }
    // arma::mat ax_max_machines = {{0.0, 3.0}, {10.0, 3.0}};
    // arma::vec kappa = {0.1, 0.2, 0.3};
    // arma::vec el_lengths = {50.0, 100.0, 150.0};
    // double drag_coeff = 0.3;
    // double m_veh = 1500.0;

    // arma::mat ggv = {{0.0, 3.0, 1.0}, {10.0, 3.0, 1.0}};

    // arma::vec vel_profile = calc_vel_profile(ax_max_machines, kappa, el_lengths, false, drag_coeff, m_veh, ggv);

    // vel_profile.print("Velocity Profile:");

    return 0;
}
