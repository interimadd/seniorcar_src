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

	// タイヤの内どこで何を計算するかを計算しておく
	const double TIRE_X_SPLIT_NUM = 2;
	const double TIRE_Y_SPLIT_NUM = 1;
	const double TIRE_CALC_MARGINE = 0.01;
	tire_calculation_point.resize( ( TIRE_X_SPLIT_NUM * 2 + 1 ) * (TIRE_Y_SPLIT_NUM * 2 + 1));
	for(int i=0; i < ( TIRE_X_SPLIT_NUM * 2 + 1 ) * (TIRE_Y_SPLIT_NUM * 2 + 1) ;i++){
		tire_calculation_point[i].resize(3);
	}
	for(int i=0; i < TIRE_X_SPLIT_NUM * 2 + 1 ; i++){
		for(int j=0; j < TIRE_Y_SPLIT_NUM * 2 + 1 ; j++){
			int tire_calc_index = i*( TIRE_Y_SPLIT_NUM * 2 + 1 ) + j ;
			tire_calculation_point[tire_calc_index][0] = ( (SENIORCAR_WHEEL_RADIUS - TIRE_CALC_MARGINE) / TIRE_X_SPLIT_NUM ) * ( i - TIRE_X_SPLIT_NUM );
			tire_calculation_point[tire_calc_index][1] = ( SENIORCAR_WHEEL_THICKNESS / ( TIRE_Y_SPLIT_NUM * 2 ) ) * ( j - TIRE_Y_SPLIT_NUM );
			tire_calculation_point[tire_calc_index][2] = -sqrt( pow(SENIORCAR_WHEEL_RADIUS,2) - pow(tire_calculation_point[tire_calc_index][0],2));
			cout << "tire_index:" << tire_calc_index << " x:" << tire_calculation_point[tire_calc_index][0] << " y:" << tire_calculation_point[tire_calc_index][1] << " z:" << tire_calculation_point[tire_calc_index][2] << endl;
		}
	}

	// 定数計算
	C_f = SENIORCAR_FRONT_SPRING_CONSTANT * SENIORCAR_TREAD_LENGTH * 0.5 / ( 0.5 * ( SENIORCAR_FRONT_SPRING_CONSTANT + SENIORCAR_BACK_SPRING_CONSTANT ) * SENIORCAR_TREAD_LENGTH * SENIORCAR_TREAD_LENGTH - SENIORCAR_WEIGHT*9.8*SENIORCAR_COG_HEIGHT) ;
	C_b = SENIORCAR_BACK_SPRING_CONSTANT  * SENIORCAR_TREAD_LENGTH * 0.5 / ( 0.5 * ( SENIORCAR_FRONT_SPRING_CONSTANT + SENIORCAR_BACK_SPRING_CONSTANT ) * SENIORCAR_TREAD_LENGTH * SENIORCAR_TREAD_LENGTH - SENIORCAR_WEIGHT*9.8*SENIORCAR_COG_HEIGHT) ;
	C_a = SENIORCAR_WEIGHT  / ( 0.5 * ( SENIORCAR_FRONT_SPRING_CONSTANT + SENIORCAR_BACK_SPRING_CONSTANT ) * SENIORCAR_TREAD_LENGTH * SENIORCAR_TREAD_LENGTH - SENIORCAR_WEIGHT*9.8*SENIORCAR_COG_HEIGHT) ;

}


void AccidentPredictor::predictAccident(float pos_x,float pos_y,float pos_z,float roll,float pitch,float yaw,float vehicle_velocity)
{
	//cout << endl << endl << "start predict" << endl;
	generatePath(pos_x,pos_y,yaw,vehicle_velocity); // 走行領域の更新

	InterpolateMap();	// 地図の補間
	FindObjectFromMap(); // 障害物地図の更新

	vector < CalculatedVehicleState > reset_array;
	calculated_state_array = reset_array;
	CalculatedVehicleState tmp_calculated_state, tmp_calculated_state_old;

	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		returnTirePositionAtGivenPose(&tmp_calculated_state_old,predicted_path[i][0]);
		for(int j=1; j < PATH_POINT_NUM; j++){
			
			returnTirePositionAtGivenPose(&tmp_calculated_state,predicted_path[i][j]);	// 四輪の接地位置計算

			// 転倒判定
			calculateSlopeOfVehicle(&tmp_calculated_state) ;
			float zmp_y_pos = calculateZMP( tmp_calculated_state, 1.5, CALCULATE_STEER_DEG_STEP * float(i) - MAX_STEER_DEG_CHANGE);
			tmp_calculated_state.is_rollover = abs(zmp_y_pos) > SENIORCAR_DANGER_ZMP_POS_Y;
			
			// 転落判定
			tmp_calculated_state.is_fall = isFall(tmp_calculated_state) ;

			// 衝突判定
			//tmp_calculated_state.is_collision = isCollision(tmp_calculated_state);
			tmp_calculated_state.is_collision = isCollisionByTirePos(tmp_calculated_state,tmp_calculated_state_old);
			
			calculated_state_array.push_back(tmp_calculated_state);
			//printCalculatedState(tmp_calculated_state);

			tmp_calculated_state_old = tmp_calculated_state ;

		}
	}

	//printCalculatedState(calculated_state_array[17]);
	/*
	CalculatedVehicleState state = calculated_state_array[17];
	cout << state.vehicle_pose[0] << "," << state.vehicle_pose[1] << "," << state.vehicle_pose[2]
	<< "," << state.calculated_roll_angle[0] << "," << state.calculated_roll_angle[1]  
	<< "," << state.tire_pos[0][2] <<  "," <<  state.tire_pos[1][2] <<  "," <<  state.tire_pos[2][2] <<  "," <<  state.tire_pos[3][2] << endl;  
	*/
	printCalculatedState(calculated_state_array[PREDEICT_FRONT_INDEX]);
	//printPredictedFrontLeftTireHeight();
	//printPredictedVehicleAngle();
	
}


void AccidentPredictor::generatePath(float pos_x,float pos_y,float yaw,float vehicle_velocity)
{
	int X = 0;
	int Y = 1;
	int TH = 2;

	float calculate_distance_step;
	if(vehicle_velocity == NOT_SET){
		calculate_distance_step = CALCULATE_DISTANCE_STEP;
	}
	else{
		vehicle_velocity = max( vehicle_velocity , MIN_VHEICLE_VELOCITY );
		calculate_distance_step = vehicle_velocity * TIME_STEP_RESOLUTION;
	}

	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		predicted_path[i][0][X] = pos_x ;
		predicted_path[i][0][Y] = pos_y ;
		predicted_path[i][0][TH] = yaw ;
	}

	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		float tmp_steer_angle = ( CALCULATE_STEER_DEG_STEP * float(i) - MAX_STEER_DEG_CHANGE ) * M_PI / 180.0;
		for(int j=1; j < PATH_POINT_NUM; j++){
			predicted_path[i][j][TH] = predicted_path[i][j-1][TH] + calculate_distance_step * tan(tmp_steer_angle)  / SENIORCAR_WHEEL_BASE_LENGTH ;
			float tmp_th = ( predicted_path[i][j][TH] + predicted_path[i][j-1][TH] ) / 2.0 ;
			predicted_path[i][j][X] = predicted_path[i][j-1][X] + calculate_distance_step * cos(tmp_th) ; 
			predicted_path[i][j][Y] = predicted_path[i][j-1][Y] + calculate_distance_step * sin(tmp_th) ;
		}
	}
}


void AccidentPredictor::returnTirePositionAtGivenPose(CalculatedVehicleState *return_tire_pos,vector<float> pose)
{
	int X = 0; int Y = 1; int Z = 2; int TH = 2;

	return_tire_pos->vehicle_pose[X] = pose[X];
	return_tire_pos->vehicle_pose[Y] = pose[Y];
	return_tire_pos->vehicle_pose[TH] = pose[TH];

	return_tire_pos->tire_pos[FRONT_LEFT][X] = pose[X] + SENIORCAR_WHEEL_BASE_LENGTH * cos(pose[TH]) - SENIORCAR_HARF_TREAD_LENGTH * sin(pose[TH]);
	return_tire_pos->tire_pos[FRONT_LEFT][Y] = pose[Y] + SENIORCAR_WHEEL_BASE_LENGTH * sin(pose[TH]) + SENIORCAR_HARF_TREAD_LENGTH * cos(pose[TH]);
	return_tire_pos->tire_pos[FRONT_LEFT][3] = pose[TH];

	return_tire_pos->tire_pos[FRONT_RIGHT][X] = pose[X] + SENIORCAR_WHEEL_BASE_LENGTH * cos(pose[TH]) + SENIORCAR_HARF_TREAD_LENGTH * sin(pose[TH]);
	return_tire_pos->tire_pos[FRONT_RIGHT][Y] = pose[Y] + SENIORCAR_WHEEL_BASE_LENGTH * sin(pose[TH]) - SENIORCAR_HARF_TREAD_LENGTH * cos(pose[TH]);
	return_tire_pos->tire_pos[FRONT_RIGHT][3] = pose[TH];

	return_tire_pos->tire_pos[BACK_LEFT][X] = pose[X] - SENIORCAR_HARF_TREAD_LENGTH * sin(pose[TH]);
	return_tire_pos->tire_pos[BACK_LEFT][Y] = pose[Y] + SENIORCAR_HARF_TREAD_LENGTH * cos(pose[TH]);
	return_tire_pos->tire_pos[BACK_LEFT][3] = pose[TH];

	return_tire_pos->tire_pos[BACK_RIGHT][X] = pose[X] + SENIORCAR_HARF_TREAD_LENGTH * sin(pose[TH]);
	return_tire_pos->tire_pos[BACK_RIGHT][Y] = pose[Y] - SENIORCAR_HARF_TREAD_LENGTH * cos(pose[TH]);
	return_tire_pos->tire_pos[BACK_RIGHT][3] = pose[TH];

	//int tmp_index[2];

	for(int i = 0; i < 4 ;i++){
		//TranslateRealCordinateToIndex(tmp_index,return_tire_pos->tire_pos[i]);
		//return_tire_pos->tire_pos[i][Z] = returnTireHeightInGrid(tmp_index[0],tmp_index[1]);
		return_tire_pos->tire_pos[i][Z] = returnTireHeightAtGivenPositionAndPose(return_tire_pos->tire_pos[i][X],return_tire_pos->tire_pos[i][Y],return_tire_pos->tire_pos[i][3]);
	}

}


double AccidentPredictor::returnTireHeightAtGivenPositionAndPose(double x_pos,double y_pos,double tire_theta){

	const float START_HEIGHT = -10.0;
	float max_height = START_HEIGHT;
	float tire_pos[2],tire_pos_z;
	int    tire_pos_index[2];

	for(int i=0; i < tire_calculation_point.size() ;i++){
		tire_pos[0] = x_pos + tire_calculation_point[i][0] * cos(tire_theta) - tire_calculation_point[i][1] * sin(tire_theta);
		tire_pos[1] = y_pos + tire_calculation_point[i][0] * sin(tire_theta) + tire_calculation_point[i][1] * cos(tire_theta);
		TranslateRealCordinateToIndex(tire_pos_index, tire_pos);
		tire_pos_z = tire_calculation_point[i][2] + height_map[tire_pos_index[0]][tire_pos_index[1]] ;
		max_height = max(max_height,tire_pos_z);
	}

	return max_height;

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
	// 低い方の前輪の接地高さを幾何的な形状から予測し、路面形状から予測される接地高さからの差が車輪が上下に動く高さ程度に収まっていれば転落していない判定

	int higher_pos_tire_index,lower_pos_tire_index;
	
	if( vehicle_state.tire_pos[FRONT_LEFT][2] > vehicle_state.tire_pos[FRONT_RIGHT][2] ){
		higher_pos_tire_index = FRONT_LEFT;
		lower_pos_tire_index  = FRONT_RIGHT;
	}
	else{
		higher_pos_tire_index = FRONT_RIGHT;
		lower_pos_tire_index  = FRONT_LEFT;
	}

	// そもそも車両のピッチ角度が登坂の限界である10度を超えてたら転落するという判定
	if( abs(vehicle_state.calculated_pitch_angle[2]) > SENIORCAR_DRIVABLE_PITCH_ANGLE_THRESHOLD || abs(vehicle_state.calculated_roll_angle[2]) > SENIORCAR_DRIVABLE_ROLL_ANGLE_THRESHOLD ){
		return true;
	}

	//float expected_lower_tire_pos_z = vehicle_state.tire_pos[higher_pos_tire_index][2] - ( vehicle_state.tire_pos[higher_pos_tire_index+2][2] + SENIORCAR_TIRE_MOVABLE_LENGHT ) + vehicle_state.tire_pos[lower_pos_tire_index+2][2] ;
	float expected_lower_tire_pos_z = vehicle_state.tire_pos[higher_pos_tire_index][2] - abs(SENIORCAR_TREAD_LENGTH * vehicle_state.calculated_roll_angle[2]);

	if( expected_lower_tire_pos_z - vehicle_state.tire_pos[lower_pos_tire_index][2] < SENIORCAR_TIRE_MOVABLE_LENGHT){
		return false;
	}
	else{
		return true;
	}

	/*
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
	*/
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

void AccidentPredictor::setCollisionIndex(float yaw){

	/*
	int radius_num_in_grid = ( SENIORCAR_FULL_WIDTH / 2.0 ) / HORIZONTAL_RESOLUTION  ;
	//円で判定するのはめんどいのでとりあえず正方形で！！
	for(int i = -radius_num_in_grid; i <= radius_num_in_grid ; i++ ){
		for(int j = -radius_num_in_grid; j <= radius_num_in_grid ; j++){
			vector<int> tmp_index(2,0);
			tmp_index[0] = i; tmp_index[1] = j;
			collision_index.push_back(tmp_index);
		}
	}
	*/

	// 初期化
	collision_index.clear();
	vector<int> tmp_index(2,0);
	float tmp_vehicle_point[2], vehicle_point[2];

	//cout << "calc_colindex  yaw:" << yaw << endl;

	// タイヤの内どこで何を計算するかを計算しておく
	const double VHEICLE_X_SPLIT_NUM = int( (SENIORCAR_FULL_LENGTH) / HORIZONTAL_RESOLUTION + 1 );
	const double VHEICLE_Y_SPLIT_NUM = int( (SENIORCAR_FULL_WIDTH / 2.0) / HORIZONTAL_RESOLUTION + 1 );
	for(int i=0; i < VHEICLE_X_SPLIT_NUM ; i++){
		for(int j=0; j < VHEICLE_Y_SPLIT_NUM * 2 + 1 ; j++){
			tmp_vehicle_point[0] = HORIZONTAL_RESOLUTION * double(i) ;
			tmp_vehicle_point[1] = HORIZONTAL_RESOLUTION * double( j - VHEICLE_Y_SPLIT_NUM ) ;
			vehicle_point[0] = tmp_vehicle_point[0] * cos(yaw) - tmp_vehicle_point[1] * sin(yaw);
			vehicle_point[1] = tmp_vehicle_point[0] * sin(yaw) + tmp_vehicle_point[1] * cos(yaw);
			tmp_index[0] = ( vehicle_point[0] + HORIZONTAL_RESOLUTION/2.0 ) / HORIZONTAL_RESOLUTION;
			tmp_index[1] = ( vehicle_point[1] + HORIZONTAL_RESOLUTION/2.0 ) / HORIZONTAL_RESOLUTION;
			collision_index.push_back(tmp_index);
			//cout << "(" << tmp_vehicle_point[0] << "," << tmp_vehicle_point[1] << "," << tmp_index[0] << "," << tmp_index[1] << "," << vehicle_point[0] << "," << vehicle_point[1] << ") " ;
			//cout << tmp_index[0] << "," << tmp_index[1] << endl ;
		}
	}
	//cout << endl;

}

bool AccidentPredictor::isCollision(CalculatedVehicleState vehicle_state){

	setCollisionIndex(vehicle_state.vehicle_pose[2]);
	/*
	float collision_center[2];
	int   collision_center_index[2];
	collision_center[0] = vehicle_state.vehicle_pose[0] + SENIORCAR_WHEEL_BASE_LENGTH * cos(vehicle_state.vehicle_pose[2]);
	collision_center[1] = vehicle_state.vehicle_pose[1] + SENIORCAR_WHEEL_BASE_LENGTH * sin(vehicle_state.vehicle_pose[2]);
	TranslateRealCordinateToIndex(collision_center_index , collision_center);
	*/
	int vehicle_pos_index[2];
	float vehicle_pos[2] = {vehicle_state.vehicle_pose[0] ,vehicle_state.vehicle_pose[1] };
	TranslateRealCordinateToIndex(vehicle_pos_index,vehicle_pos);

	for(int i=0;i < collision_index.size();i++){
		if( object_map[ vehicle_pos_index[0] +	collision_index[i][0] ][ vehicle_pos_index[1] + collision_index[i][1] ] == OBJECT ){
			return true;
		}
	}

	return false;
};


bool AccidentPredictor::isCollisionByTirePos(CalculatedVehicleState state_now,CalculatedVehicleState state_old){
	return ( state_now.tire_pos[0][2] - state_old.tire_pos[0][2] > SENIORCAR_DRIVABLE_STEP_HEIGHT || state_now.tire_pos[1][2] - state_old.tire_pos[1][2] > SENIORCAR_DRIVABLE_STEP_HEIGHT );
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

	// 新しい実装
	predicted_state->calculated_roll_angle[2] = C_f * ( predicted_state->tire_pos[FRONT_RIGHT][Z] - predicted_state->tire_pos[FRONT_LEFT][Z] ) + C_b * ( predicted_state->tire_pos[BACK_RIGHT][Z] - predicted_state->tire_pos[BACK_LEFT][Z] ) ;
	predicted_state->calculated_pitch_angle[2] = -atan( 0.5 * ( predicted_state->tire_pos[FRONT_RIGHT][Z] + predicted_state->tire_pos[FRONT_LEFT][Z] - predicted_state->tire_pos[BACK_RIGHT][Z] - predicted_state->tire_pos[BACK_LEFT][Z] ) / SENIORCAR_WHEEL_BASE_LENGTH );

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

	
	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		for(int j=1; j < PATH_POINT_NUM; j++){
			CalculatedVehicleState tmp_state = calculated_state_array[ j - 1 + ( PATH_POINT_NUM - 1 ) * i ];
			if( !tmp_state.is_collision && !tmp_state.is_fall && !tmp_state.is_rollover ){
				rgba.r = 0.0f; rgba.g = 1.0f; rgba.b = 0.0f; rgba.a = 1.0f;
				for(int p_i=0;p_i<3;p_i++){
					float yaw = tmp_state.vehicle_pose[2];
					p[p_i].x = tmp_state.vehicle_pose[0] + cos(yaw) * tmp_p[p_i].x - sin(yaw) * tmp_p[p_i].y;
					p[p_i].y = tmp_state.vehicle_pose[1] + sin(yaw) * tmp_p[p_i].x + cos(yaw) * tmp_p[p_i].y;
					p[p_i].z = 0;
					triangles->points.push_back(p[p_i]);
					triangles->colors.push_back(rgba);
				}
				if( tmp_state.vehicle_pose[0] > 4){
					//cout << tmp_state.vehicle_pose[0] << "," << tmp_state.vehicle_pose[1] << "," << tmp_state.vehicle_pose[2] << endl;
				}
			}
			else{
				break;
			}
		}
	}
	

	/*
	for(int i=0; i < DEG_CALCULTE_NUM; i++){
		for(int j=1; j < PATH_POINT_NUM; j++){
		
		int index =  j - 1 + ( PATH_POINT_NUM - 1 ) * i ;
		if( calculated_state_array[index].is_collision ){
			rgba.r = 0.0f; rgba.g = 0.0f; rgba.b = 1.0f; rgba.a = 1.0f;
		}
		else if( calculated_state_array[index].is_fall ){
			rgba.r = 1.0f; rgba.g = 0.0f; rgba.b = 1.0f; rgba.a = 1.0f;
		}
		else if( calculated_state_array[index].is_rollover ){
			rgba.r = 1.0f; rgba.g = 0.0f; rgba.b = 0.0f; rgba.a = 1.0f;
		}
		else{
			rgba.r = 0.0f; rgba.g = 1.0f; rgba.b = 0.0f; rgba.a = 1.0f;
		}

		for(int p_i=0;p_i<3;p_i++){
			float yaw = calculated_state_array[index].vehicle_pose[2];
			p[p_i].x = calculated_state_array[index].vehicle_pose[0] + cos(yaw) * tmp_p[p_i].x - sin(yaw) * tmp_p[p_i].y;
			p[p_i].y = calculated_state_array[index].vehicle_pose[1] + sin(yaw) * tmp_p[p_i].x + cos(yaw) * tmp_p[p_i].y;
			p[p_i].z = 0;
			triangles->points.push_back(p[p_i]);
			triangles->colors.push_back(rgba);
		}

	}
	}
	*/
	
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
	cout << "pose:(," << state.vehicle_pose[0] << "," << state.vehicle_pose[1] << "," << state.vehicle_pose[2] << ",)  " 
	<< "wheel_height:(," << state.tire_pos[0][2] <<  "," <<  state.tire_pos[1][2] <<  "," <<  state.tire_pos[2][2] <<  "," <<  state.tire_pos[3][2] <<  ",)  " 
	<< "roll_angle:(," << state.calculated_roll_angle[0] << "," << state.calculated_roll_angle[1] << "," << state.calculated_roll_angle[2] << ",)  " 
	<< "pitch_angle:(," << state.calculated_pitch_angle[0] << "," << state.calculated_pitch_angle[1] << "," << state.calculated_pitch_angle[2] << ",)  " 
	<< "fall:," << state.is_fall << "  collision:," << state.is_collision << "  rollover:," << state.is_rollover << endl;  
}

void AccidentPredictor::printPredictedFrontLeftTireHeight(){
	CalculatedVehicleState state = calculated_state_array[PREDEICT_FRONT_INDEX];
	cout << state.tire_pos[0][0] << "," << state.tire_pos[0][1] << "," << state.tire_pos[0][2] + SENIORCAR_WHEEL_RADIUS << endl;
}

void AccidentPredictor::printPredictedVehicleAngle(){

	CalculatedVehicleState state = calculated_state_array[PREDEICT_FRONT_INDEX];
	cout << state.vehicle_pose[0] << "," << state.vehicle_pose[1] << ","  << state.vehicle_pose[2] << ","  << state.vehicle_pose[3] << ","  << state.calculated_pitch_angle[2] << ","  << state.calculated_roll_angle[2] << endl;
}