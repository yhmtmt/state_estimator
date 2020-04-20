// Copyright(c) 2020 Yohei Matsumoto, All right reserved. 

// f_state_estimator.hpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_state_estimator.hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_state_estimator.hpp.  If not, see <http://www.gnu.org/licenses/>.
#ifndef F_STATE_ESTIMATOR_HPP
#define F_STATE_ESTIMATOR_HPP
#include "filter_base.hpp"

#include "ch_state.hpp"
#include "ch_nmea.hpp"

class f_state_estimator: public f_base
{
protected:
  ch_state * state;
  ch_nmea_data * nmea_data_in;
  ch_nmea_data * nmea_data_out;
  ch_time_sync * time_sync;

  Eigen::Vector3d x_gps_ant; // gps antenna position relative to the boat center
  Eigen::Matrix3d Q_position;// position transition error var/cov-matrix in enu coordinate
  Eigen::Matrix3d Q_velocity;// velocity transition error var/cov-matrix in enu coordinate
  Eigen::Matrix3d Q_attitude;// attitude transition error var/cov-matrix
  Eigen::Matrix3d R_position;// position observation error var/cov-matrix in enu coordinate
  Eigen::Matrix3d R_velocity;// velocity observation error var/cov-matrix in enu coordinate
  Eigen::Matrix3d R_attitude;// attitude observation error var/cov-matrix
  
  Eigen::Matrix3d P_position; // position estimation error var/cov-matrix
  Eigen::Matrix3d P_attitude; // attitude estimation error var/cov-matrix (currently is R_attitude)
  Eigen::Matrix3d P_velocity; // velocity estimation error var/cov-matrix (currently is R_attitude)
  
  unsigned char buffer[256];
  size_t data_len;
  
  long long tvtg;
  double sog /*speed in mps*/, cog /* Course in radian */;
  double angle_drift;
  double u, v; // velocity in x, y direction in body fixed coordinate
  double du_dt, dv_dt; // acceleration in x, y direction
  
  long long thpr;
  double roll, pitch, yaw; // attitude in radian
  double droll_dt, dpitch_dt, dyaw_dt;
  Eigen::Matrix3d Rbody;
  Eigen::Vector3d vrot;
  
  long long tgga;
  double latitude, longitude, altitude; // popsition in radian
  Eigen::Vector3d x_ecef;
  // kalman filter optimal estimation 
  double latitude_opt, longitude_opt, altitude_opt;
  Eigen::Vector3d x_ecef_opt;  
  double Renu[9];

  long long tdbt;
  double depth;  

  bool load_state();
  bool save_state();

  struct s_log_record_header{
    long long tsys; // system time 
    size_t size; // data size
    s_log_record_header(const long long _tsys, 
			const size_t _size):tsys(_tsys), size(_size)
    {};
  };
  
  ofstream log_file_stream;
  unsigned int current_log_size;
  unsigned int max_log_size;
  bool log_nmea_data();
  bool open_log_file();
  bool close_log_file();
public:
  f_state_estimator(const char * name);
  virtual ~f_state_estimator();
  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();
};

#endif

