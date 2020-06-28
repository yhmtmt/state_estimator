// Copyright(c) 2020 Yohei Matsumoto, All right reserved. 

// f_state_estimator.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_state_estimator.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_state_estimator.cpp.  If not, see <http://www.gnu.org/licenses/>.

#include "f_state_estimator.hpp"
#include "state_estimator.pb.h"
#include "aws_proto.hpp"


DEFINE_FILTER(f_state_estimator)

f_state_estimator::f_state_estimator(const char * name):f_base(name),
  state(nullptr), nmea_data_in(nullptr), nmea_data_out(nullptr),
  n2k_data_in(nullptr), n2k_data_out(nullptr),
  time_sync(nullptr), max_log_size(2<<30), log_nmea(true), replay(false)
{
  register_fpar("state", (ch_base**)&state, typeid(ch_state).name(),
		"State channel");
  register_fpar("eng_state", (ch_base**)&eng_state, typeid(ch_eng_state).name(),
		"Engine state channel");
 
  register_fpar("nmea_data_in", (ch_base**)&nmea_data_in,
		typeid(ch_nmea_data).name(), "Input NMEA data channel");
  register_fpar("nmea_data_out", (ch_base**)&nmea_data_out,
		typeid(ch_nmea_data).name(), "Output NMEA data channel");
  register_fpar("n2k_data_in", (ch_base**)&n2k_data_in,
		typeid(ch_n2k_data).name(), "Input NMEA2000 data channel");
  register_fpar("n2k_data_out", (ch_base**)&n2k_data_out,
		typeid(ch_n2k_data).name(), "Output NMEA2000 data channel");
  register_fpar("time_sync", (ch_base**)&time_sync,
		typeid(ch_time_sync).name(), "Time sync channel");
  register_fpar("max_log_size", &max_log_size, "Maximum size of log file.");

  register_fpar("log_nmea", &log_nmea, "NMEA logging flag.");
  register_fpar("replay", &replay, "Replay flag.");
  
  x_gps_ant << 0, 0, -1;
  
  Q_position = Q_attitude = Q_velocity =
    R_position = R_attitude = R_velocity = Eigen::Matrix3d::Identity();
  
}

f_state_estimator::~f_state_estimator()
{  
}
  
bool f_state_estimator::load_state()
{
  char filepath[2048];
  snprintf(filepath, 2048, "%s/%s.json",
	   f_base::get_data_path().c_str(), get_name());

  StateEstimator::State st;
  if(!load_proto_object(filepath, st)){
    spdlog::error("[{}] Failed to load {}.", get_name(), filepath);
    return false;
  }

  latitude = st.x_own().latitude();
  longitude = st.x_own().longitude();
  x_gps_ant << st.x_gps_ant().x(), st.x_gps_ant().y(), st.x_gps_ant().z();
  if(st.q_position_size() != 3 ||
     st.q_attitude_size() != 3 ||
     st.q_velocity_size() != 3 ||       
     st.r_position_size() != 3 ||
     st.r_attitude_size() != 3 ||
     st.r_velocity_size() != 3)      
    return false;
  
  for(int i = 0; i < 3; i++){
    if(st.q_position(i).v_size() != 3 ||
       st.q_attitude(i).v_size() != 3 ||
       st.q_velocity(i).v_size() != 3 ||       
       st.r_position(i).v_size() != 3 ||
       st.r_attitude(i).v_size() != 3 ||
       st.r_velocity(i).v_size() != 3)      
      return false;
    
    for (int j = 0; j < 3; j++){
      Q_position(i, j) = st.q_position(i).v(j);
      Q_attitude(i, j) = st.q_attitude(i).v(j);
      Q_velocity(i, j) = st.q_velocity(i).v(j);            
      R_position(i, j) = st.r_position(i).v(j);
      R_attitude(i, j) = st.r_attitude(i).v(j);
      R_velocity(i, j) = st.r_velocity(i).v(j);      
    }
  }

  return true;
}

bool f_state_estimator::save_state()
{
  char filepath[2048];
  snprintf(filepath, 2048, "%s/%s.json",
	   f_base::get_data_path().c_str(), get_name());

  StateEstimator::State st;
  StateEstimator::Location2D _x_own;
  _x_own.set_latitude(latitude);
  _x_own.set_longitude(longitude);
  st.set_allocated_x_own(&_x_own);
  
  StateEstimator::Vector3D _x_gps_ant;
  _x_gps_ant.set_x(x_gps_ant(0));
  _x_gps_ant.set_y(x_gps_ant(1));
  _x_gps_ant.set_z(x_gps_ant(2));  
  st.set_allocated_x_gps_ant(&_x_gps_ant);

  StateEstimator::Vector * _Q_position[3];
  StateEstimator::Vector * _Q_attitude[3];
  StateEstimator::Vector * _Q_velocity[3];  
  StateEstimator::Vector * _R_position[3];
  StateEstimator::Vector * _R_attitude[3];
  StateEstimator::Vector * _R_velocity[3];

  for(int i = 0; i < 3; i++){
    _Q_position[i] = st.add_q_position();
    _Q_attitude[i] = st.add_q_attitude();
    _Q_velocity[i] = st.add_q_velocity();    

    _R_position[i] = st.add_r_position();
    _R_attitude[i] = st.add_r_attitude();
    _R_velocity[i] = st.add_r_velocity();    
    
    for(int j = 0; j < 3; j++){
      _Q_position[i]->add_v(Q_position(i, j));
      _Q_attitude[i]->add_v(Q_attitude(i, j));
      _Q_velocity[i]->add_v(Q_velocity(i, j));      
      _R_position[i]->add_v(R_position(i, j));
      _R_attitude[i]->add_v(R_attitude(i, j));
      _R_velocity[i]->add_v(R_velocity(i, j));
    }
  }

  if(!save_proto_object(filepath, st)){
    spdlog::error("[{}] Failed to save {}.", get_name(), filepath);
    return false;
  }
  
  st.release_x_own();
  st.release_x_gps_ant();
  
  return true;
}

bool f_state_estimator::open_log_file()
{
  char file_prefix[2048];
  snprintf(file_prefix, 2048, "%s_nmea0183", get_name());
  if(!log_nmea0183.init(f_base::get_data_path(), file_prefix, replay, max_log_size)){
    spdlog::error("[{}] Failed to open nmea0183 log file at {}", get_name(), f_base::get_data_path());
    return false;
  }
  
  snprintf(file_prefix, 2048, "%s_n2k", get_name());  
  if(!log_n2k.init(f_base::get_data_path(), file_prefix, replay, max_log_size)){
    spdlog::error("[{}] Failed to open nmea2000 log file at {}", get_name(), f_base::get_data_path());
    return false;
  }
  
  return true;
}

void f_state_estimator::close_log_file()
{
  log_n2k.destroy();
  log_nmea0183.destroy();
}

bool f_state_estimator::get_nmea_data(bool n2k)
{
  if(replay){
    long long t;
    if(n2k){
      return log_n2k.read(t, buffer, data_len);
    }else{
      return log_nmea0183.read(t, buffer, data_len);
    }
  }else{
    if(n2k){
      n2k_data_in->pop(buffer, data_len);
      if(log_nmea && data_len)
	return log_n2k.write(get_time(), buffer, data_len);
    }else{
      nmea_data_in->pop(buffer, data_len);
      if(log_nmea && data_len)
	return log_nmea0183.write(get_time(), buffer, data_len);    
    }
  }
  return true;
}

bool f_state_estimator::init_run()
{
  if(!open_log_file()){
    return false;
  }
  
  if(!load_state()){
    spdlog::error("[{}] Failed to load state file.", get_name());
  }

  // time record of each nmea sentence is initialized as zero.
  tvtg = thpr = tgll = tdbt = 0;

  // accelerations are initialized as zero.
  du_dt = dv_dt = droll_dt = dpitch_dt = dyaw_dt = 0.0;

  P_position = R_position;
  P_velocity = R_velocity;
  P_attitude = R_attitude;
  return true;
}


void f_state_estimator::destroy_run()
{

  if(!save_state()){
    spdlog::error("[{}] Failed to save state file.", get_name());
  }

  close_log_file();
}

bool f_state_estimator::proc()
{
  // NMEA2000 data handled here
  // EngineParametersRapidUpdate
  // EngineParametersDynamic
  while(1){
    if(n2k_data_in == nullptr)
      break;
    
    if(!get_nmea_data(true)){
      spdlog::error("[{}] Failed to get n2k data.", get_name());
      return false;
    }
    
    if(data_len == 0)
      break;
    
    const NMEA2000::Data * data =  NMEA2000::GetData(buffer);
    long long tdata = data->t();
    switch(data->payload_type()){
    case NMEA2000::Payload_EngineParametersRapidUpdate:{
      const NMEA2000::EngineParametersRapidUpdate * pl
	= data->payload_as_EngineParametersRapidUpdate();
      eng_state->set_rapid(tdata, (float)((double)pl->engineSpeed() * 0.25),
			   pl->engineTrim());
    }
      if (n2k_data_out)
	n2k_data_out->push(buffer, data_len);
      break;
    case NMEA2000::Payload_EngineParametersDynamic:{
      const NMEA2000::EngineParametersDynamic * pl
	= data->payload_as_EngineParametersDynamic();
      eng_state->set_dynamic(tdata, (int) pl->oilPressure(),
			     (float)pl->oilTemperature(),
			     (float) pl->temperature(),
			     (float)((double)pl->alternatorPotential() * 0.01),
			     (float)((double)pl->fuelRate() * 0.1),
			     (unsigned int)pl->totalEngineHours(),
			     (int)pl->coolantPressure(),
			     (int)pl->fuelPressure(),
			     pl->discreteStatus1(), pl->discreteStatus2(),
			     (unsigned char) pl->percentEngineLoad(),
			     (unsigned char) pl->percentEngineTorque());
    }
      if (n2k_data_out)
	n2k_data_out->push(buffer, data_len);
           
      break;
    }
  }

  
  // NMEA0183 data handled here
  
  // GLL(position data)
  // VTG(velocity data)
  // ZDA(time data)
  // PSAT,HPR(v104's attitude data)
  // HEV(Heave data)
  // DBT(Depth data)
  // MDA(weather data)

  while(1){
    if(!get_nmea_data(false)){
      spdlog::error("[{}] Failed to get nmea0183 data.", get_name());
      return false;
    }
    if(data_len == 0)
      break;    

    const NMEA0183::Data * data = NMEA0183::GetData(buffer);
    long long tdata = data->t();
    switch(data->payload_type()){
    case NMEA0183::Payload_GLL:{
      const NMEA0183::GLL * gll = data->payload_as_GLL();
      latitude = gll->latitude() * (PI / 180.0);
      longitude = gll->longitude() * (PI / 180.0);
      state->set_position(tdata, gll->latitude(), gll->longitude());

      Eigen::Vector3d x_ecef_new;
      blhtoecef(latitude, longitude, altitude,
		x_ecef_new(0), x_ecef_new(1), x_ecef_new(2));
      if(tgll != 0){
	double tdelta = (double)(tdata - tgll) / (double)SEC;
	double c = cos(yaw), s = sin(yaw); // optimal yaw would be used in the future
	// subtracting velocity due to antena rotation, and rotatin the vector
	// by yaw to convert to world coordinate.
	double upred = u - vrot(0); 
	double vpred = v - vrot(1);
	upred = c * upred - s * vpred;
	vpred = s * upred + c * vpred;

	// Because we need to apply Kalman filter in enu coordinate,
	// the x-y axes of the velocity vector derived above should be swapped.
	Eigen::Vector3d x_enu_pred(vpred * tdelta, upred * tdelta, 0.0);
	Eigen::Vector3d x_enu_obs;	
	eceftowrld(Renu, x_ecef_opt(0), x_ecef_opt(1), x_ecef_opt(2),
		   x_ecef_new(0), x_ecef_new(1), x_ecef_new(2),
		   x_enu_obs(0), x_enu_obs(1), x_enu_obs(2));

	// Note that tdelta would be different in many reasons,
	// we assume Q_position represents the unit-time transition error,
	// and the random walk develops the var/cov-matrix by tdelta.
	// Additionally we assume no correlation between velocity and position
	// estimation, 
	// F = [ 1 0 0 dt 0  0 ] = [I diag3(dt)]
	//     [ 0 1 0 0 dt  0 ]
	//     [ 0 0 1 0  0 dt ]
	// P = [ Ppos   0 ]
	//     [  0   Pvel]
	// FPF^t = Ppos + diag3(dt) Pvel diag3(dt)^t
	
	Eigen::Matrix3d P_prediction =
	  P_position + tdelta * tdelta * P_velocity + tdelta * Q_position;	
	Eigen::Matrix3d S_obs_error = R_position + P_prediction;
	Eigen::Matrix3d K = P_prediction * S_obs_error.inverse();
	
	Eigen::Vector3d x_enu_error = x_enu_obs - x_enu_pred;	
	Eigen::Vector3d x_enu_opt = x_enu_pred + K * x_enu_error;
	wrldtoecef(Renu, x_ecef_opt(0), x_ecef_opt(1), x_ecef_opt(2),
		   x_enu_opt(0), x_enu_opt(1), x_enu_opt(2),
		   x_ecef_opt(0), x_ecef_opt(1), x_ecef_opt(2));
	eceftoblh(x_ecef_opt(0), x_ecef_opt(1), x_ecef_opt(2),
		  latitude_opt, longitude_opt, altitude_opt);
      }else{
	// in the initial observation, observed values are
	// set as optimal value, and the cov-matrix is set
	// as observation error cov-matrix
	x_ecef_opt = x_ecef_new;
      }
      x_ecef = x_ecef_new;
      getwrldrot(latitude_opt, longitude_opt, Renu);
      tgll = tdata;
    }break;
    case NMEA0183::Payload_VTG:{
      const NMEA0183::VTG * vtg = data->payload_as_VTG();
      state->set_velocity(tdata, vtg->cogTrue(), vtg->sogN());
      cog = vtg->cogTrue() * (PI / 180.0);
      sog = vtg->sogN() * KNOT;
      
      angle_drift = normalize_angle_rad(cog - yaw);
      double unew = sog * cos(angle_drift);
      double vnew = sog * sin(angle_drift);
      if(tvtg != 0){
	double inv_tdiff = (double)SEC / (double)(tdata - tvtg);
	du_dt = (unew - u) * inv_tdiff;
	dv_dt = (vnew - v) * inv_tdiff;
      }
      tvtg = tdata;
      u = unew;
      v = vnew;
    }break;
    case NMEA0183::Payload_ZDA:{
      const NMEA0183::ZDA * zda = data->payload_as_ZDA();
	
    }break;
    case NMEA0183::Payload_PSAT:{
      const NMEA0183::PSAT * psat = data->payload_as_PSAT();
      const NMEA0183::HPR * hpr = psat->payload_as_HPR();
      if(hpr){
	float y = hpr->heading();
	float p = hpr->pitch();
	float r = hpr->roll();
	double rollnew = r * (PI / 180.0);
	double pitchnew = p * (PI / 180.0);
	double yawnew = y * (PI / 180.0);
	  
	state->set_attitude(tdata, r, p, y);

	if(thpr != 0){
	  double inv_tdiff = (double)SEC / (double)(tdata - thpr);
	  droll_dt = normalize_angle_rad(rollnew - roll) * inv_tdiff;
	  dpitch_dt = normalize_angle_rad(pitchnew - pitch) * inv_tdiff;
	  dyaw_dt = normalize_angle_rad(yawnew - yaw) * inv_tdiff;
	  Rbody = rotation_matrix(rollnew, pitchnew, yawnew);
	  Eigen::Matrix3d rx = left_cross_product_matrix(droll_dt,
							 dpitch_dt, dyaw_dt);
	  vrot = rx * Rbody * x_gps_ant;
	}
	
	thpr = tdata;
	roll = rollnew;
	pitch = pitchnew;
	yaw = yawnew;	
      }
    }break;
    case NMEA0183::Payload_HEV:{
      const NMEA0183::HEV * hev = data->payload_as_HEV();
      altitude = hev->heave();
      state->set_alt(tdata, hev->heave());
    }break;
    case NMEA0183::Payload_DBT:{
      const NMEA0183::DBT * dbt = data->payload_as_DBT();
      tdbt = tdata;
      state->set_depth(tdata, dbt->depthM());
    }break;
    case NMEA0183::Payload_MDA:{
      const NMEA0183::MDA * mda = data->payload_as_MDA();
      state->set_weather(tdata, mda->barometricPressureBar(),
			 mda->airTemperature(),
			 mda->relativeHumidity(),
			 mda->dewPoint(),
			 mda->windDirectionTrue(),
			 mda->windSpeedMps());
    }break;
    default:
      if(nmea_data_out)     
	nmea_data_out->push(buffer, data_len);
    }
  }
  
  return true;
}
