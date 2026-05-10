uint8 path_type
uint32 path_order
uint32 path_index
string path_id

bool is_valid

bool has_amplitude
float64 amplitude_abs

bool has_path_gain
float64 path_gain_linear

bool has_power_db
float64 power_db

bool has_tau
float64 tau_s
float64 tau_std_s

bool has_doppler
float64 doppler_hz
float64 doppler_std_hz

bool has_aoa
float64 aoa_az_rad
float64 aoa_el_rad
float64 aoa_az_std_rad
float64 aoa_el_std_rad

geometry_msgs/Point tx_position
geometry_msgs/Point rx_position
geometry_msgs/Point[] path_points
