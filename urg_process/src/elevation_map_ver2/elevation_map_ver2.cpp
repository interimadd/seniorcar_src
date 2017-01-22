#include "elevation_map_ver2.h"

ElevationMap::ElevationMap(float pos_x,float pos_y,int map_size_x,int map_size_y,float horizontal_resolution){

	MAP_SIZE_X = map_size_x;
	MAP_SIZE_Y = map_size_y;

	center_x = pos_x;
    center_y = pos_y;

    last_calc_x = pos_x;
    last_calc_y = pos_y;

	HORIZONTAL_RESOLUTION = horizontal_resolution;

	height_map.resize(map_size_x*2);
	interpolated_height_map.resize(map_size_x*2);

	for(int i=0; i < map_size_x*2 ;i++){
		height_map[i].resize(map_size_y*2);
		interpolated_height_map[i].resize(map_size_y*2);
	}

	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			height_map[i][j] = NOT_DETECT;
			interpolated_height_map[i][j] = NOT_DETECT;
		}
	}

}

void ElevationMap::RecordSensorData(sensor_msgs::PointCloud laser_point_data){
	
	int num_x,num_y;
	int DATA_NUM = laser_point_data.points.size();

	for(int i=0;i<DATA_NUM;i++){
		num_x = int( float(MAP_SIZE_X) + ( laser_point_data.points[i].x - center_x ) / HORIZONTAL_RESOLUTION );
		if( 0 <= num_x && num_x < MAP_SIZE_X * 2 ){
			num_y = int( float(MAP_SIZE_Y) + ( laser_point_data.points[i].y - center_y ) / HORIZONTAL_RESOLUTION );
			if( 0 <= num_y && num_y < MAP_SIZE_Y * 2 ){
				if(RECOAR_DATA_UPDATE_STRATEGY==0){
					height_map[num_x][num_y] = laser_point_data.points[i].z;
				}
				else if(RECOAR_DATA_UPDATE_STRATEGY==1){
					if( height_map[num_x][num_y] > laser_point_data.points[i].z || height_map[num_x][num_y] == NOT_DETECT ){
						height_map[num_x][num_y] = laser_point_data.points[i].z;
					}
				}
			}
		}
	}

	InterpolateMap();

	return;
}


void ElevationMap::TranslateRealCordinateToIndex(int *return_index,float pose[2]){
	return_index[0] = int( float(MAP_SIZE_X) + ( pose[0] - center_x ) / HORIZONTAL_RESOLUTION );
	return_index[1] = int( float(MAP_SIZE_Y) + ( pose[1] - center_y ) / HORIZONTAL_RESOLUTION );
}


void ElevationMap::HeightMapToPointCloud(sensor_msgs::PointCloud *out){

	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			if( height_map[i][j] != NOT_DETECT){
				out->points.push_back(TranslateIndexToRealCordinate(i,j));
			}
		}
	}

}


geometry_msgs::Point32 ElevationMap::TranslateIndexToRealCordinate(int x_index,int y_index){
	geometry_msgs::Point32 point;
	point.x = float(x_index - MAP_SIZE_X) * HORIZONTAL_RESOLUTION + center_x;
	point.y = float(y_index - MAP_SIZE_Y) * HORIZONTAL_RESOLUTION + center_y;
	point.z = height_map[x_index][y_index];
	return point;
}


void ElevationMap::MoveHeightMapCenter(float pos_x,float pos_y){

	pos_x = floor(pos_x);
	pos_y = floor(pos_y);

	// 複製用のvector
	vector < vector <float> > tmp_map;
	vector < vector < vector <float> > > tmp_variance_map;

	tmp_map.resize( MAP_SIZE_X * 2);
	tmp_variance_map.resize( MAP_SIZE_X * 2);
	for(int i=0; i < MAP_SIZE_X * 2 ;i++){
		tmp_map[i].resize(MAP_SIZE_Y * 2 );
		tmp_variance_map[i].resize(MAP_SIZE_Y * 2);
	}

	// vectorのコピーともとのvectorの初期化
	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			
			tmp_map[i][j] = height_map[i][j];
			height_map[i][j] = NOT_DETECT;
			
		}
	}

	// 複製vectorを移動させる
	int new_x_index,new_y_index;
	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			if(tmp_map[i][j] != NOT_DETECT){
				new_x_index = i + (center_x - pos_x) / HORIZONTAL_RESOLUTION;
				if( 0 <= new_x_index && new_x_index < MAP_SIZE_X * 2 ){
					new_y_index = j + (center_y - pos_y) / HORIZONTAL_RESOLUTION;
					if( 0 <= new_y_index && new_y_index < MAP_SIZE_Y * 2 ){
						height_map[new_x_index][new_y_index] = tmp_map[i][j];
					}
				}
			}	
		}
	}

	center_x = pos_x;
	center_y = pos_y;

}


void ElevationMap::printElevationMapData(){

	cout << "print start" << endl;
	cout << endl;

}


void ElevationMap::InterpolateMap(){

	// 補間は甘え
	
	for(int i=2 ; i < MAP_SIZE_X * 2 - 2 ; i++){
		for(int j=2 ; j < MAP_SIZE_Y * 2 - 2 ;j++){

			interpolated_height_map[i][j] = height_map[i][j];

			/*
			float height_sum = 0;
			float sum_num = 0;
			interpolated_height_map[i][j] = NOT_DETECT;

			height_sum = 0 ;
			sum_num = 0 ;

			for(int n=-1;n<=1;n++){
				for(int m=-1;m<=1;m++){
					if(height_map[i+n][j+m] != NOT_DETECT){
						sum_num += 1;
						height_sum += height_map[i+n][j+m];
					}
				}
			}

			height_sum = height_sum / sum_num ;

			if(height_map[i][j] == NOT_DETECT){
				if(sum_num>3){
					interpolated_height_map[i][j] = height_sum;
				}
				else{
					interpolated_height_map[i][j] = NOT_DETECT;
				}
			}
			else{
				// まわりから見てトビのある点の場合は消す
				if( height_map[i][j] - height_sum > 0.2){
					height_map[i][j] = NOT_DETECT ;
					interpolated_height_map[i][j] = NOT_DETECT ;
				}
				else{
					interpolated_height_map[i][j] = height_map[i][j] ;
				}
			}
			*/

		}
	}

}