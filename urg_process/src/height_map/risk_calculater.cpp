#include "risk_calculater.h"

RiskCalculater::RiskCalculater(float pos_x,float pos_y,int map_size_x,int map_size_y,float horizontal_resolution) : Height_Map( pos_x ,pos_y ,map_size_x ,map_size_y ,horizontal_resolution)
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
	//cout << endl << endl << "start  now_pos  x:" << now_pos.position.x << ", y:" << now_pos.position.y << ", th:" << now_pos.orientation.w << endl;

	//  速度0だと崖のすぐ側でもリスク0となってしまうので、一定値以上の速度を持つように調整
	if(now_state.vehicle_velocity < MIN_VHEICLE_VELOCITY){
		now_state.vehicle_velocity = MIN_VHEICLE_VELOCITY;
	}

	// 予測経路の更新
	generatePath(now_pos,now_state);
	int j = 0;
	int max_j = PATH_POINT_NUM;

	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		cout << "output  " << now_state.steer_angle - MAX_STEER_DEG_CHANGE + CALCULATE_STEER_DEG_STEP * float(i) << "deg" << endl;
		for(j=1; j < PATH_POINT_NUM; j++){
			returnTirePositionAtGivenPose(&tmp_calculated_state,predicted_path[i][j]);
			calculateSlopeOfVehicle(&tmp_calculated_state);
			
			cout << "pos x:" << predicted_path[i][j][0] << ", y:" << predicted_path[i][j][1] << ", th:" << predicted_path[i][j][2]   
			<< "0 pitch:"<< tmp_calculated_state.calculated_pitch_angle[0] * RAD_TO_DEG << " roll:" << tmp_calculated_state.calculated_roll_angle[0] * RAD_TO_DEG<< " zmp:" << calculateZMP(tmp_calculated_state.calculated_roll_angle[0] ,now_state.vehicle_velocity, now_state.steer_angle - MAX_STEER_DEG_CHANGE + CALCULATE_STEER_DEG_STEP * float(i))
			<< ",1 pitch:"<< tmp_calculated_state.calculated_pitch_angle[1] * RAD_TO_DEG<< " roll:" << tmp_calculated_state.calculated_roll_angle[1] *RAD_TO_DEG<< " zmp:" << calculateZMP(tmp_calculated_state.calculated_roll_angle[1] ,now_state.vehicle_velocity, now_state.steer_angle - MAX_STEER_DEG_CHANGE + CALCULATE_STEER_DEG_STEP * float(i)) << endl;
			
			if( !canDrive(tmp_calculated_state.calculated_pitch_angle[0],tmp_calculated_state.calculated_roll_angle[0],calculateZMP(tmp_calculated_state.calculated_roll_angle[0] ,now_state.vehicle_velocity, now_state.steer_angle - MAX_STEER_DEG_CHANGE + CALCULATE_STEER_DEG_STEP * float(i)))){
				break;
			}
			if( !canDrive(tmp_calculated_state.calculated_pitch_angle[1],tmp_calculated_state.calculated_roll_angle[1],calculateZMP(tmp_calculated_state.calculated_roll_angle[1] ,now_state.vehicle_velocity, now_state.steer_angle - MAX_STEER_DEG_CHANGE + CALCULATE_STEER_DEG_STEP * float(i)))){
				break;
			}
		}
		if( max_j > j){
			max_j = j;
		}
	}

	float TTI = float(max_j) * CALCULATE_TIME_STEP;

	cout << TTI << endl << endl;

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