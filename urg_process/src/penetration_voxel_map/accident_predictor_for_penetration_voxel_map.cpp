#include "accident_predictor.h"

AccidentPredictor::AccidentPredictor(float pos_x,float pos_y,int map_size_x,int map_size_y,float horizontal_resolution) : ElevationMap( pos_x ,pos_y ,map_size_x ,map_size_y ,horizontal_resolution)
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
	tire_radius_in_grid =  ( SENIORCAR_WHEEL_RADIUS / HORIZONTAL_RESOLUTION ) + 1;
	half_tire_width_in_grid = ( SENIORCAR_WHEEL_THICKNESS / (2.0 * HORIZONTAL_RESOLUTION) ) + 1;

	if(half_tire_width_in_grid==0){ half_tire_width_in_grid = 1; }
	if(tire_radius_in_grid==0){ tire_radius_in_grid = 1; }

	for(int i=0;i<=tire_radius_in_grid;i++){
		float calc_height = pow(SENIORCAR_WHEEL_RADIUS,2) - pow(HORIZONTAL_RESOLUTION*i,2);
		if( calc_height < 0 ) calc_height = 0;
		calc_height = sqrt(calc_height);
		tire_height.push_back(calc_height);
	}

	setCollisionIndex(); // SetCollisionindex

	// ObjectMapの初期化
	object_map.resize(map_size_x*2);
	for(int i=0; i < map_size_x*2 ;i++){
		object_map[i].resize(map_size_y*2);
	}
	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			object_map[i][j] = UNKNOWN;
		}
	}

	cout << "tire_radius_in_grid:" << tire_radius_in_grid << "  half_tire_width_in_grid:" << half_tire_width_in_grid << endl;

}


void AccidentPredictor::predictAccident(float pos_x,float pos_y,float pos_z,float roll,float pitch,float yaw)
{
	//cout << endl << endl << "start predict" << endl;
	generatePath(pos_x,pos_y,yaw); // 走行領域の更新

	InterpolateMap();	// 地図の補間
	FindObjectFromMap(); // 障害物地図の更新

	vector < CalculatedVehicleState > reset_array;
	calculated_state_array = reset_array;
	CalculatedVehicleState tmp_calculated_state;

	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		for(int j=1; j < PATH_POINT_NUM; j++){
			
			returnTirePositionAtGivenPose(&tmp_calculated_state,predicted_path[i][j]);	// 四輪の接地位置計算
			
			// 転落判定
			tmp_calculated_state.is_fall = isFall(tmp_calculated_state) ;
			
			// 転倒判定
			calculateSlopeOfVehicle(&tmp_calculated_state) ;
			float zmp_y_pos = calculateZMP( tmp_calculated_state, 1.5, CALCULATE_STEER_DEG_STEP * float(i) - MAX_STEER_DEG_CHANGE);
			tmp_calculated_state.is_rollover = abs(zmp_y_pos) > SENIORCAR_DANGER_ZMP_POS_Y;

			// 衝突判定
			tmp_calculated_state.is_collision = isCollision(tmp_calculated_state);
			
			calculated_state_array.push_back(tmp_calculated_state);
			//printCalculatedState(tmp_calculated_state);

		}
	}

	//printCalculatedState(calculated_state_array[17]);
	CalculatedVehicleState state = calculated_state_array[17];
	cout << state.vehicle_pose[0] << "," << state.vehicle_pose[1] << "," << state.vehicle_pose[2]
	<< "," << state.calculated_roll_angle[0] << "," << state.calculated_roll_angle[1]  
	<< "," << state.tire_pos[0][2] <<  "," <<  state.tire_pos[1][2] <<  "," <<  state.tire_pos[2][2] <<  "," <<  state.tire_pos[3][2] << endl;  
	
}


void AccidentPredictor::generatePath(float pos_x,float pos_y,float yaw)
{
	int X = 0;
	int Y = 1;
	int TH = 2;

	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		predicted_path[i][0][X] = pos_x ;
		predicted_path[i][0][Y] = pos_y ;
		predicted_path[i][0][TH] = yaw ;
	}

	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		float tmp_steer_angle = ( CALCULATE_STEER_DEG_STEP * float(i) - MAX_STEER_DEG_CHANGE ) * M_PI / 180.0;
		for(int j=1; j < PATH_POINT_NUM; j++){
			predicted_path[i][j][TH] = predicted_path[i][j-1][TH] + CALCULATE_DISTANCE_STEP * tan(tmp_steer_angle)  / SENIORCAR_WHEEL_BASE_LENGTH ;
			float tmp_th = ( predicted_path[i][j][TH] + predicted_path[i][j-1][TH] ) / 2.0 ;
			predicted_path[i][j][X] = predicted_path[i][j-1][X] + CALCULATE_DISTANCE_STEP * cos(tmp_th) ; 
			predicted_path[i][j][Y] = predicted_path[i][j-1][Y] + CALCULATE_DISTANCE_STEP * sin(tmp_th) ;
		}
	}
}


void AccidentPredictor::returnTirePositionAtGivenPose(CalculatedVehicleState *return_tire_pos,vector<float> pose)
{
	int X = 0; int Y = 1; int Z = 2; int TH = 2;

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
		TranslateRealCordinateToIndex(tmp_index,return_tire_pos->tire_pos[i]);
		return_tire_pos->tire_pos[i][Z] = returnTireHeightInGrid(tmp_index[0],tmp_index[1]);
	}

}


float AccidentPredictor::returnTireHeightInGrid(int x_index,int y_index)
{
	const float START_HEIGHT = -10.0;
	float max_height = START_HEIGHT;
	
	for(int i=0;i<=tire_radius_in_grid;i++){
		for(int j=0;j<=half_tire_width_in_grid;j++){

			if( max_height < interpolated_height_map[x_index + i][y_index + j] + tire_height[i] ){
				max_height = interpolated_height_map[x_index + i][y_index + j] + tire_height[i];
			}

			if( max_height < interpolated_height_map[x_index - i][y_index + j] + tire_height[i] ){
				max_height = interpolated_height_map[x_index - i][y_index + j] + tire_height[i];
			}

			if( max_height < interpolated_height_map[x_index + i][y_index - j] + tire_height[i] ){
				max_height = interpolated_height_map[x_index + i][y_index - j] + tire_height[i];
			}

			if( max_height < interpolated_height_map[x_index - i][y_index - j] + tire_height[i] ){
				max_height = interpolated_height_map[x_index - i][y_index - j] + tire_height[i];
			}
		}
	}

	for(int i=0;i<=tire_radius_in_grid;i++){
		for(int j=0;j<=half_tire_width_in_grid;j++){

			if( max_height < interpolated_height_map[x_index + j][y_index + i] + tire_height[i] ){
				max_height = interpolated_height_map[x_index + j][y_index + i] + tire_height[i];
			}

			if( max_height < interpolated_height_map[x_index - j][y_index + i] + tire_height[i] ){
				max_height = interpolated_height_map[x_index - j][y_index + i] + tire_height[i];
			}

			if( max_height < interpolated_height_map[x_index + j][y_index - i] + tire_height[i] ){
				max_height = interpolated_height_map[x_index + j][y_index - i] + tire_height[i];
			}

			if( max_height < interpolated_height_map[x_index - j][y_index - i] + tire_height[i] ){
				max_height = interpolated_height_map[x_index - j][y_index - i] + tire_height[i];
			}
		}
	}

	return max_height;
}


float AccidentPredictor::calculateZMP(CalculatedVehicleState vehicle_state,float vehicle_velocity,float steer_angle_deg)
{

	const float deg90 = M_PI / 2.0 - 0.01;
	const float CANNOT_CALICULATE = 100;

	float steer_angle = steer_angle_deg * M_PI / 180.0;

	// 横方向加速度。旋回半径を以下のサイトを参考に計算
	// http://k-ichikawa.blog.enjoy.jp/etc/HP/js/Car/car.html
	float acc_to_y_direction      = vehicle_velocity * vehicle_velocity * sin( steer_angle ) / SENIORCAR_WHEEL_BASE_LENGTH ;
	float mixed_acc_vector_angle0 = vehicle_state.calculated_roll_angle[0] + atan( acc_to_y_direction / GRVITY_ACCELERATION ) ;
	float mixed_acc_vector_angle1 = vehicle_state.calculated_roll_angle[1] + atan( acc_to_y_direction / GRVITY_ACCELERATION ) ; 

	float abs_mixed_acc_vector_angle = max( abs(mixed_acc_vector_angle0) , abs(mixed_acc_vector_angle1) );
	if( abs_mixed_acc_vector_angle > deg90 ){
		return CANNOT_CALICULATE;
	}
	else{
		return SENIORCAR_COG_HEIGHT * tan( abs_mixed_acc_vector_angle ) ;
	}

}


bool AccidentPredictor::isFall(CalculatedVehicleState vehicle_state)
{
	vector<float> wheel_height_list(4,0);
	wheel_height_list[0] = vehicle_state.tire_pos[0][2] ;
	wheel_height_list[1] = vehicle_state.tire_pos[1][2] ;
	wheel_height_list[2] = vehicle_state.tire_pos[2][2] ;
	wheel_height_list[3] = vehicle_state.tire_pos[3][2] ;

	sort(wheel_height_list.begin(), wheel_height_list.end());

	if( abs(wheel_height_list[3] - wheel_height_list[0]) > SENIORCAR_WHEEL_RADIUS ){
		return true;
	}
	else{
		return false;
	}
}


void AccidentPredictor::FindObjectFromMap(){

	for(int i=2 ; i < MAP_SIZE_X * 2 - 2 ; i++){
		for(int j=2 ; j < MAP_SIZE_Y * 2 - 2 ;j++){
			if( height_map[i][j] == NOT_DETECT){
				object_map[i][j] = UNKNOWN;
			}
			else{
				float tmp_gap = 0;
				for(int n=-1;n<=1;n++){
					for(int m=-1;m<=1;m++){
						if( height_map[i+n][j+m] != NOT_DETECT ){
							tmp_gap = max( height_map[i][j] - height_map[i+n][j+m] , tmp_gap );
						}
					}
				}
				if( tmp_gap > SENIORCAR_WHEEL_RADIUS * 0.7 ){
					object_map[i][j] = OBJECT;
				}
				else{
					object_map[i][j] = ROAD;
				}
			}
		}
	}

}

void AccidentPredictor::setCollisionIndex(){

	int radius_num_in_grid = ( SENIORCAR_FULL_WIDTH / 2.0 ) / HORIZONTAL_RESOLUTION  ;
	//円で判定するのはめんどいのでとりあえず正方形で！！
	for(int i = -radius_num_in_grid; i <= radius_num_in_grid ; i++ ){
		for(int j = -radius_num_in_grid; j <= radius_num_in_grid ; j++){
			vector<int> tmp_index(2,0);
			tmp_index[0] = i; tmp_index[1] = j;
			collision_index.push_back(tmp_index);
		}
	}

}

bool AccidentPredictor::isCollision(CalculatedVehicleState vehicle_state){
	float collision_center[2];
	int   collision_center_index[2];
	collision_center[0] = vehicle_state.vehicle_pose[0] + SENIORCAR_WHEEL_BASE_LENGTH * cos(vehicle_state.vehicle_pose[2]);
	collision_center[1] = vehicle_state.vehicle_pose[1] + SENIORCAR_WHEEL_BASE_LENGTH * sin(vehicle_state.vehicle_pose[2]);
	TranslateRealCordinateToIndex(collision_center_index , collision_center);

	for(int i=0;i < collision_index.size();i++){
		if( object_map[ collision_center_index[0] +	collision_index[i][0] ][ collision_center_index[1] + collision_index[i][1] ] == OBJECT ){
			return true;
		}
	}

	return false;
};


void AccidentPredictor::calculateSlopeOfVehicle(CalculatedVehicleState *predicted_state){

	int FRONT_LEFT  = 0;
	int FRONT_RIGHT = 1;
	int BACK_LEFT	= 2;
	int BACK_RIGHT	= 3;

	int X = 0;
	int Y = 1;
	int Z = 2;

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

float AccidentPredictor::calculateRollAngleFrom2Vectors(float vec1[3],float vec2[3]){

	float cross_vec[3] = {
		vec1[1]*vec2[2] - vec2[1]*vec1[2],
		vec1[2]*vec2[0] - vec2[2]*vec1[0],
		vec1[0]*vec2[1] - vec2[0]*vec1[1]
	};

	return -atan(cross_vec[1]/cross_vec[2]);

};


float AccidentPredictor::calculatePitchAngleFrom2Vectors(float vec1[3],float vec2[3]){

	float cross_vec[3] = {
		vec1[1]*vec2[2] - vec2[1]*vec1[2],
		vec1[2]*vec2[0] - vec2[2]*vec1[0],
		vec1[0]*vec2[1] - vec2[0]*vec1[1]
	};

	return atan(cross_vec[0]/cross_vec[2]);

};

void AccidentPredictor::returnObjectMapPointsMarker(visualization_msgs::Marker *points){

	//FindObjectFromMap();

	points->header.frame_id = "/odom";
	points->ns = "object_map";
	points->action = visualization_msgs::Marker::ADD;
	points->pose.orientation.w = 1.0;
	points->id = 0;
	points->type = visualization_msgs::Marker::POINTS;
	points->scale.x = points->scale.y = HORIZONTAL_RESOLUTION;

	geometry_msgs::Point p;
	std_msgs::ColorRGBA rgba;
	rgba.a = 0.2f;

	for(int i=2 ; i < MAP_SIZE_X * 2 - 2 ; i++){
		for(int j=2 ; j < MAP_SIZE_Y * 2 - 2 ;j++){
			if(object_map[i][j] != UNKNOWN){
				geometry_msgs::Point32 tmp_p32 = TranslateIndexToRealCordinate(i,j);
				p.x = tmp_p32.x ;  p.y = tmp_p32.y ;  p.z = tmp_p32.z ;
				if(object_map[i][j] == OBJECT){
					rgba.r = 1.0f;  rgba.g = 0.0f;  rgba.b = 0.0f;
				}
				else if(object_map[i][j] == ROAD){
					rgba.r = 0.0f;  rgba.g = 1.0f;  rgba.b = 0.0f;
				}
				points->points.push_back(p);
				points->colors.push_back(rgba);
			}
		}
	}

};


PredictResult AccidentPredictor::returnPredictResult(){

	PredictResult returnResult;

	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		returnResult.steer_angle[i] = ( CALCULATE_STEER_DEG_STEP * float(i) - MAX_STEER_DEG_CHANGE ) * M_PI / 180.0 ;
		returnResult.max_distance_to_go[i] = 0;
		for(int j=1; j < PATH_POINT_NUM; j++){
			CalculatedVehicleState tmp_state = calculated_state_array[ j - 1 + ( PATH_POINT_NUM - 1 ) * i ];
			//cout << "index:" << j - 1 + ( PATH_POINT_NUM - 1 ) * i << "  is:" << tmp_state.is_collision << tmp_state.is_fall << tmp_state.is_rollover << endl;
			if( !tmp_state.is_collision && !tmp_state.is_fall ){
				returnResult.max_distance_to_go[i] = float(j) * CALCULATE_DISTANCE_STEP;
			}
			else{
				break;
			}
		}
	}
	return returnResult;
}


void AccidentPredictor::returnCalculatedVehicleState(visualization_msgs::Marker *triangles){

	triangles->header.frame_id = "/odom";
	triangles->ns = "calculated_state";
	triangles->action = visualization_msgs::Marker::ADD;
	triangles->pose.orientation.w = 1.0;
	triangles->id = 0;
	triangles->type = visualization_msgs::Marker::TRIANGLE_LIST;
	triangles->scale.x = triangles->scale.y = triangles->scale.z = 1.0f;

	geometry_msgs::Point p[3],tmp_p[3];
	std_msgs::ColorRGBA rgba;

	float TRIANGLE_LENGTH = 0.1;
	tmp_p[0].x =  TRIANGLE_LENGTH; tmp_p[0].y =  0.0f;
	tmp_p[1].x = -TRIANGLE_LENGTH; tmp_p[1].y =  TRIANGLE_LENGTH/2.0f; 
	tmp_p[2].x = -TRIANGLE_LENGTH; tmp_p[2].y = -TRIANGLE_LENGTH/2.0f;

	for(int i=0; i < calculated_state_array.size(); i++){
		
		if( calculated_state_array[i].is_collision ){
			rgba.r = 1.0f; rgba.g = 0.0f; rgba.b = 0.0f; rgba.a = 1.0f;
		}
		else if( calculated_state_array[i].is_fall ){
			rgba.r = 1.0f; rgba.g = 0.0f; rgba.b = 0.0f; rgba.a = 1.0f;
		}
		else if( calculated_state_array[i].is_rollover ){
			rgba.r = 1.0f; rgba.g = 0.0f; rgba.b = 0.0f; rgba.a = 1.0f;
		}
		else{
			rgba.r = 0.0f; rgba.g = 1.0f; rgba.b = 0.0f; rgba.a = 1.0f;
		}

		for(int p_i=0;p_i<3;p_i++){
			float yaw = calculated_state_array[i].vehicle_pose[2];
			p[p_i].x = calculated_state_array[i].vehicle_pose[0] + cos(yaw) * tmp_p[p_i].x - sin(yaw) * tmp_p[p_i].y;
			p[p_i].y = calculated_state_array[i].vehicle_pose[1] + sin(yaw) * tmp_p[p_i].x + cos(yaw) * tmp_p[p_i].y;
			p[p_i].z = 0;
			triangles->points.push_back(p[p_i]);
			triangles->colors.push_back(rgba);
		}

	}

}

void AccidentPredictor::printCalculatedState(CalculatedVehicleState state){
/*
	typedef struct{
	float vehicle_pose[3];			// 車両原点位置位置(x,y,th)
	float tire_pos[4][3];			// 車輪位置座標((x,y,z)*4)
	float calculated_roll_angle[2];	// 車輪位置から計算された車両の傾き（基本三点接地なので2通りある）
	float calculated_pitch_angle[2];
	bool  is_fall;
	bool  is_collision;
	bool  is_rollover;
} CalculatedVehicleState;
*/
	cout << "pose:(" << state.vehicle_pose[0] << "," << state.vehicle_pose[1] << "," << state.vehicle_pose[2] << ")  " 
	<< "wheel_height:(" << state.tire_pos[0][2] <<  "," <<  state.tire_pos[1][2] <<  "," <<  state.tire_pos[2][2] <<  "," <<  state.tire_pos[3][2] <<  ")  " 
	<< "roll_angle:(" << state.calculated_roll_angle[0] << "," << state.calculated_roll_angle[1] << ")  " 
	<< "fall:" << state.is_fall << "  collision:" << state.is_collision << "  rollover:" << state.is_rollover << endl;  
}