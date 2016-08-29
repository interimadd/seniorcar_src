#include "risk_calculater_for_elevation_map.h"

RiskCalculater::RiskCalculater(float pos_x,float pos_y,int map_size_x,int map_size_y,float horizontal_resolution) : ElevationMap( pos_x ,pos_y ,map_size_x ,map_size_y ,horizontal_resolution)
{
	predicted_path.resize(DEG_CALCULTE_NUM);

	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		predicted_path[i].resize(PATH_POINT_NUM);
	}

	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		for(int j=0; j < PATH_POINT_NUM; j++){
			predicted_path[i][j].resize(3);
		}
	}

	// SetTireRadiusの部分
	// タイヤのパラメータからタイヤの大きさをグリッドに離散化
	tire_radius_in_grid =  SENIORCAR_WHEEL_RADIUS / HORIZONTAL_RESOLUTION;
	half_tire_width_in_grid = SENIORCAR_WHEEL_THICKNESS / (2.0 * HORIZONTAL_RESOLUTION);
	if(half_tire_width_in_grid==0){ half_tire_width_in_grid = 1; }
	if(tire_radius_in_grid==0){ tire_radius_in_grid = 1; }

	for(int i=0;i<=tire_radius_in_grid;i++){
		float calc_height = pow(SENIORCAR_WHEEL_RADIUS,2) - pow(HORIZONTAL_RESOLUTION*i,2);
		if( calc_height < 0 ) calc_height = 0;
		calc_height = sqrt(calc_height);
		tire_height.push_back(calc_height);
	}
	
};

void RiskCalculater::generatePath(geometry_msgs::Pose now_pos, ultimate_seniorcar::SeniorcarState now_state){

	int X = 0;
	int Y = 1;
	int TH = 2;

	// ヨー角度を取得
	tf::Quaternion q(now_pos.orientation.x,now_pos.orientation.y,now_pos.orientation.z,now_pos.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	//cout << roll << "," << pitch <<","<<yaw<<endl;

	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		predicted_path[i][0][X] = now_pos.position.x ;
		predicted_path[i][0][Y] = now_pos.position.y ;
		predicted_path[i][0][TH] = yaw;
	}

	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		float tmp_steer_angle = ( now_state.steer_angle - MAX_STEER_DEG_CHANGE + CALCULATE_STEER_DEG_STEP * float(i) ) * M_PI / 180.0;
		for(int j=1; j < PATH_POINT_NUM; j++){
			predicted_path[i][j][TH] = predicted_path[i][j-1][TH] + now_state.vehicle_velocity * tan(tmp_steer_angle) * CALCULATE_TIME_STEP / SENIORCAR_WHEEL_BASE_LENGTH;
			float tmp_th = ( predicted_path[i][j-1][TH] + predicted_path[i][j-1][TH] ) / 2.0;
			predicted_path[i][j][X] = predicted_path[i][j-1][X] + now_state.vehicle_velocity * cos(tmp_th) * CALCULATE_TIME_STEP; 
			predicted_path[i][j][Y] = predicted_path[i][j-1][Y] + now_state.vehicle_velocity * sin(tmp_th) * CALCULATE_TIME_STEP;
		}
	}

};


void RiskCalculater::returnTirePositionAtGivenPose(CalculatedVehicleState *return_tire_pos,vector<float> pose){

	int X = 0;
	int Y = 1;
	int Z = 2;
	int TH = 2;

	int FRONT_LEFT  = 0;
	int FRONT_RIGHT = 1;
	int BACK_LEFT	= 2;
	int BACK_RIGHT	= 3;

	return_tire_pos->vehicle_pose[X] = pose[X];
	return_tire_pos->vehicle_pose[Y] = pose[Y];
	return_tire_pos->vehicle_pose[TH] = pose[TH];

	return_tire_pos->tire_pos[FRONT_LEFT][X] = pose[X] + SENIORCAR_WHEEL_BASE_LENGTH * cos(pose[TH]) - SENIORCAR_HARF_TREAD_LENGTH * sin(pose[TH]);
	return_tire_pos->tire_pos[FRONT_LEFT][Y] = pose[Y] + SENIORCAR_WHEEL_BASE_LENGTH * sin(pose[TH]) + SENIORCAR_HARF_TREAD_LENGTH * cos(pose[TH]);

	return_tire_pos->tire_pos[FRONT_RIGHT][X] = pose[X] + SENIORCAR_WHEEL_BASE_LENGTH * cos(pose[TH]) + SENIORCAR_HARF_TREAD_LENGTH * sin(pose[TH]);
	return_tire_pos->tire_pos[FRONT_RIGHT][Y] = pose[Y] + SENIORCAR_WHEEL_BASE_LENGTH * sin(pose[TH]) - SENIORCAR_HARF_TREAD_LENGTH * cos(pose[TH]);

	return_tire_pos->tire_pos[BACK_LEFT][X] = pose[X] - SENIORCAR_HARF_TREAD_LENGTH * sin(pose[TH]);
	return_tire_pos->tire_pos[BACK_LEFT][Y] = pose[Y] + SENIORCAR_HARF_TREAD_LENGTH * cos(pose[TH]);

	return_tire_pos->tire_pos[BACK_RIGHT][X] = pose[X] + SENIORCAR_HARF_TREAD_LENGTH * sin(pose[TH]);
	return_tire_pos->tire_pos[BACK_RIGHT][Y] = pose[Y] - SENIORCAR_HARF_TREAD_LENGTH * cos(pose[TH]);

	int tmp_index[2];

	for(int i = 0; i < 4 ;i++){
		//cout << i << "," << return_tire_pos->tire_pos[i][0] << "," << return_tire_pos->tire_pos[i][1] << ",";
		TranslateRealCordinateToIndex(tmp_index,return_tire_pos->tire_pos[i]);
		//cout << tmp_index[0] << "," << tmp_index[1] << ",";
		return_tire_pos->tire_pos[i][Z] = returnTireHeightInGrid(tmp_index[0],tmp_index[1]);
		//cout << return_tire_pos->tire_pos[i][2] << endl;
	}

};


void RiskCalculater::calculateSlopeOfVehicle(CalculatedVehicleState *predicted_state){

	int FRONT_LEFT  = 0;
	int FRONT_RIGHT = 1;
	int BACK_LEFT	= 2;
	int BACK_RIGHT	= 3;

	int X = 0;
	int Y = 1;
	int Z = 2;
	int TH = 2;

	// どの二点で接地しているか判定
	if ( predicted_state->tire_pos[FRONT_LEFT][Z] + predicted_state->tire_pos[BACK_RIGHT][Z] > predicted_state->tire_pos[FRONT_RIGHT][Z] + predicted_state->tire_pos[BACK_LEFT][Z] ){

		// 左前と右後で接地している場合
		float vec_to_FL_from_BR[3] = {	predicted_state->tire_pos[FRONT_LEFT][X] - predicted_state->tire_pos[BACK_RIGHT][X] ,
										predicted_state->tire_pos[FRONT_LEFT][Y] - predicted_state->tire_pos[BACK_RIGHT][Y] ,
										predicted_state->tire_pos[FRONT_LEFT][Z] - predicted_state->tire_pos[BACK_RIGHT][Z] };
		float vec_to_FR_from_BR[3] = {	predicted_state->tire_pos[FRONT_RIGHT][X] - predicted_state->tire_pos[BACK_RIGHT][X] ,
										predicted_state->tire_pos[FRONT_RIGHT][Y] - predicted_state->tire_pos[BACK_RIGHT][Y] ,
										predicted_state->tire_pos[FRONT_RIGHT][Z] - predicted_state->tire_pos[BACK_RIGHT][Z] };
		float vec_to_BL_from_BR[3] = {	predicted_state->tire_pos[BACK_LEFT][X] - predicted_state->tire_pos[BACK_RIGHT][X] ,
										predicted_state->tire_pos[BACK_LEFT][Y] - predicted_state->tire_pos[BACK_RIGHT][Y] ,
										predicted_state->tire_pos[BACK_LEFT][Z] - predicted_state->tire_pos[BACK_RIGHT][Z] };

		predicted_state->calculated_roll_angle[0] = calculateRollAngleFrom2Vectors(vec_to_FL_from_BR,vec_to_FR_from_BR);
		predicted_state->calculated_roll_angle[1] = calculateRollAngleFrom2Vectors(vec_to_FL_from_BR,vec_to_BL_from_BR);
		predicted_state->calculated_pitch_angle[0] = calculatePitchAngleFrom2Vectors(vec_to_FL_from_BR,vec_to_FR_from_BR);
		predicted_state->calculated_pitch_angle[1] = calculatePitchAngleFrom2Vectors(vec_to_FL_from_BR,vec_to_BL_from_BR);

	}

	else{

		// 右前と左後で接地している場合
		float vec_to_FR_from_BL[3] = {	predicted_state->tire_pos[FRONT_RIGHT][X] - predicted_state->tire_pos[BACK_LEFT][X] ,
										predicted_state->tire_pos[FRONT_RIGHT][Y] - predicted_state->tire_pos[BACK_LEFT][Y] ,
										predicted_state->tire_pos[FRONT_RIGHT][Z] - predicted_state->tire_pos[BACK_LEFT][Z] };
		float vec_to_FL_from_BL[3] = {	predicted_state->tire_pos[FRONT_LEFT][X] - predicted_state->tire_pos[BACK_LEFT][X] ,
										predicted_state->tire_pos[FRONT_LEFT][Y] - predicted_state->tire_pos[BACK_LEFT][Y] ,
										predicted_state->tire_pos[FRONT_LEFT][Z] - predicted_state->tire_pos[BACK_LEFT][Z] };
		float vec_to_BR_from_BL[3] = {	predicted_state->tire_pos[BACK_RIGHT][X] - predicted_state->tire_pos[BACK_LEFT][X] ,
										predicted_state->tire_pos[BACK_RIGHT][Y] - predicted_state->tire_pos[BACK_LEFT][Y] ,
										predicted_state->tire_pos[BACK_RIGHT][Z] - predicted_state->tire_pos[BACK_LEFT][Z] };

		predicted_state->calculated_roll_angle[0] = calculateRollAngleFrom2Vectors(vec_to_FR_from_BL,vec_to_FL_from_BL);
		predicted_state->calculated_roll_angle[1] = calculateRollAngleFrom2Vectors(vec_to_FR_from_BL,vec_to_BR_from_BL);
		predicted_state->calculated_pitch_angle[0] = calculatePitchAngleFrom2Vectors(vec_to_FR_from_BL,vec_to_FL_from_BL);
		predicted_state->calculated_pitch_angle[1] = calculatePitchAngleFrom2Vectors(vec_to_FR_from_BL,vec_to_BR_from_BL);

	}

};


float RiskCalculater::calculateRollAngleFrom2Vectors(float vec1[3],float vec2[3]){

	float cross_vec[3] = {
		vec1[1]*vec2[2] - vec2[1]*vec1[2],
		vec1[2]*vec2[0] - vec2[2]*vec1[0],
		vec1[0]*vec2[1] - vec2[0]*vec1[1]
	};

	return atan(cross_vec[1]/cross_vec[2]);

};


float RiskCalculater::calculatePitchAngleFrom2Vectors(float vec1[3],float vec2[3]){

	float cross_vec[3] = {
		vec1[1]*vec2[2] - vec2[1]*vec1[2],
		vec1[2]*vec2[0] - vec2[2]*vec1[0],
		vec1[0]*vec2[1] - vec2[0]*vec1[1]
	};

	return atan(cross_vec[0]/cross_vec[2]);

};


float RiskCalculater::calculateRisk(geometry_msgs::Pose now_pos, ultimate_seniorcar::SeniorcarState now_state){

	CalculatedVehicleState tmp_calculated_state;
	cout << "calstart" ;
	//cout << endl << endl << "start  now_pos  x:" << now_pos.position.x << ", y:" << now_pos.position.y << ", th:" << now_pos.orientation.w << endl;

	//  速度0だと崖のすぐ側でもリスク0となってしまうので、一定値以上の速度を持つように調整
	if(now_state.vehicle_velocity < MIN_VHEICLE_VELOCITY){
		now_state.vehicle_velocity = MIN_VHEICLE_VELOCITY;
	}
	now_pos_z = now_pos.position.z + 0.2;

	// 予測経路の更新
	generatePath(now_pos,now_state);
	int j = 0;
	int max_j = PATH_POINT_NUM;

	vector < CalculatedVehicleState > reset_array;
	calculated_state_array = reset_array;

	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		//cout << "output  " << now_state.steer_angle - MAX_STEER_DEG_CHANGE + CALCULATE_STEER_DEG_STEP * float(i) << "deg" << endl;
		for(j=1; j < PATH_POINT_NUM; j++){
			returnTirePositionAtGivenPose(&tmp_calculated_state,predicted_path[i][j]);
			calculateSlopeOfVehicle(&tmp_calculated_state);
			
			/*
			cout << "pos x:" << predicted_path[i][j][0] << ", y:" << predicted_path[i][j][1] << ", th:" << predicted_path[i][j][2] * RAD_TO_DEG
			<< "0 pitch:"<< tmp_calculated_state.calculated_pitch_angle[0] * RAD_TO_DEG << " roll:" << tmp_calculated_state.calculated_roll_angle[0] * RAD_TO_DEG<< " zmp:" << calculateZMP(tmp_calculated_state.calculated_roll_angle[0] ,now_state.vehicle_velocity, now_state.steer_angle - MAX_STEER_DEG_CHANGE + CALCULATE_STEER_DEG_STEP * float(i))
			<< ",1 pitch:"<< tmp_calculated_state.calculated_pitch_angle[1] * RAD_TO_DEG<< " roll:" << tmp_calculated_state.calculated_roll_angle[1] *RAD_TO_DEG<< " zmp:" << calculateZMP(tmp_calculated_state.calculated_roll_angle[1] ,now_state.vehicle_velocity, now_state.steer_angle - MAX_STEER_DEG_CHANGE + CALCULATE_STEER_DEG_STEP * float(i)) << endl;
			*/

			/*
			cout << "tire FR:" << tmp_calculated_state.tire_pos[0][0] << "," << tmp_calculated_state.tire_pos[0][1] << "," << tmp_calculated_state.tire_pos[0][2] 
			<< " FL:" << tmp_calculated_state.tire_pos[1][0] << "," << tmp_calculated_state.tire_pos[1][1] << "," << tmp_calculated_state.tire_pos[1][2]
			<< " BR:" << tmp_calculated_state.tire_pos[2][0] << "," << tmp_calculated_state.tire_pos[2][1] << "," << tmp_calculated_state.tire_pos[2][2]
			<< " BL:" << tmp_calculated_state.tire_pos[3][0] << "," << tmp_calculated_state.tire_pos[3][1] << "," << tmp_calculated_state.tire_pos[3][2] <<endl;
			*/

			//cout << predicted_path[i][j][0] << "," << predicted_path[i][j][1] << "," << tmp_calculated_state.calculated_pitch_angle[0] * RAD_TO_DEG << "," << tmp_calculated_state.calculated_pitch_angle[1] * RAD_TO_DEG << "," << (tmp_calculated_state.calculated_pitch_angle[0] * RAD_TO_DEG + tmp_calculated_state.calculated_pitch_angle[1] * RAD_TO_DEG)/2.0 << endl;

			if( !canDrive(tmp_calculated_state.calculated_pitch_angle[0],tmp_calculated_state.calculated_roll_angle[0],calculateZMP(tmp_calculated_state.calculated_roll_angle[0] ,now_state.vehicle_velocity, now_state.steer_angle - MAX_STEER_DEG_CHANGE + CALCULATE_STEER_DEG_STEP * float(i)))){
				calculated_state_array.push_back(tmp_calculated_state);
				break;
			}
			if( !canDrive(tmp_calculated_state.calculated_pitch_angle[1],tmp_calculated_state.calculated_roll_angle[1],calculateZMP(tmp_calculated_state.calculated_roll_angle[1] ,now_state.vehicle_velocity, now_state.steer_angle - MAX_STEER_DEG_CHANGE + CALCULATE_STEER_DEG_STEP * float(i)))){
				calculated_state_array.push_back(tmp_calculated_state);
				break;
			}
			calculated_state_array.push_back(tmp_calculated_state);
		}
		if( max_j > j){
			max_j = j;
		}
	}

	float TTI = float(max_j) * CALCULATE_TIME_STEP;

	cout << "," << now_pos.position.x << "," << TTI << endl;

	return TTI;

};


float RiskCalculater::calculateZMP(float roll_angle,float vehicle_velocity,float steer_angle_deg){

	const float deg90 = M_PI / 2.0 - 0.01;
	const float CANNOT_CALICULATE = 100;

	float steer_angle = steer_angle_deg * M_PI / 180.0;

	// 横方向加速度。旋回半径を以下のサイトを参考に計算
	// http://k-ichikawa.blog.enjoy.jp/etc/HP/js/Car/car.html
	float acc_to_y_direction = vehicle_velocity * vehicle_velocity * sin( steer_angle ) / SENIORCAR_WHEEL_BASE_LENGTH ;
	float mixed_acc_vector_angle = roll_angle + atan( acc_to_y_direction / GRVITY_ACCELERATION ) ;

	if( mixed_acc_vector_angle > deg90){
		return CANNOT_CALICULATE;
	}
	else if( mixed_acc_vector_angle < -deg90){
		return -CANNOT_CALICULATE;
	}
	else{
		return SENIORCAR_COG_HEIGHT * tan( roll_angle + atan( acc_to_y_direction / GRVITY_ACCELERATION ) ) ;
	}

}


bool RiskCalculater::canDrive(float pitch_angle,float roll_angle,float y_zmp){

	if( abs(pitch_angle) > DENGER_ANGLE ){ return false; }
	if( abs(roll_angle)  > DENGER_ANGLE ){ return false; }
	if( abs(y_zmp) > DENGER_Y_ZMP){ return false; }

	return true;

};


void RiskCalculater::returnCalculatedVehicleState(geometry_msgs::PoseArray *out){

	geometry_msgs::Pose tmp_pose;
	tmp_pose.position.z = now_pos_z;

	for(int i=0; i<calculated_state_array.size(); i++){
		tmp_pose.position.x = calculated_state_array[i].vehicle_pose[0];
		tmp_pose.position.y = calculated_state_array[i].vehicle_pose[1];
		tmp_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(calculated_state_array[i].calculated_roll_angle[0],calculated_state_array[i].calculated_pitch_angle[0]*3.0,calculated_state_array[i].vehicle_pose[2]);
		out->poses.push_back(tmp_pose);
		tmp_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(calculated_state_array[i].calculated_roll_angle[1],calculated_state_array[i].calculated_pitch_angle[1]*3.0,calculated_state_array[i].vehicle_pose[2]);
		out->poses.push_back(tmp_pose);
	}

}


inline float RiskCalculater::returnTireHeightInGrid(int x_index,int y_index){

	const float START_HEIGHT = -10.0;
	float max_height = START_HEIGHT;
	int max_height_grid;
	
	for(int i=0;i<=tire_radius_in_grid;i++){
		for(int j=0;j<=half_tire_width_in_grid;j++){

			if( max_height < interpolated_height_map[x_index + i][y_index + j] + tire_height[i] ){
				max_height = interpolated_height_map[x_index + i][y_index + j] + tire_height[i];
				max_height_grid = i;
			}

			if( max_height < interpolated_height_map[x_index - i][y_index + j] + tire_height[i] ){
				max_height = interpolated_height_map[x_index - i][y_index + j] + tire_height[i];
				max_height_grid = i;
			}

			if( max_height < interpolated_height_map[x_index + i][y_index - j] + tire_height[i] ){
				max_height = interpolated_height_map[x_index + i][y_index - j] + tire_height[i];
				max_height_grid = i;
			}

			if( max_height < interpolated_height_map[x_index - i][y_index - j] + tire_height[i] ){
				max_height = interpolated_height_map[x_index - i][y_index - j] + tire_height[i];
				max_height_grid = i;
			}
		}
	}

	for(int i=0;i<=tire_radius_in_grid;i++){
		for(int j=0;j<=half_tire_width_in_grid;j++){

			if( max_height < interpolated_height_map[x_index + j][y_index + i] + tire_height[i] ){
				max_height = interpolated_height_map[x_index + j][y_index + i] + tire_height[i];
				max_height_grid = i;
			}

			if( max_height < interpolated_height_map[x_index - j][y_index + i] + tire_height[i] ){
				max_height = interpolated_height_map[x_index - j][y_index + i] + tire_height[i];
				max_height_grid = i;
			}

			if( max_height < interpolated_height_map[x_index + j][y_index - i] + tire_height[i] ){
				max_height = interpolated_height_map[x_index + j][y_index - i] + tire_height[i];
				max_height_grid = i;
			}

			if( max_height < interpolated_height_map[x_index - j][y_index - i] + tire_height[i] ){
				max_height = interpolated_height_map[x_index - j][y_index - i] + tire_height[i];
				max_height_grid = i;
			}
		}
	}

	return max_height;

}