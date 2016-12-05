#include "penetration_voxel_map.h"

PenetrationVoxelMap::PenetrationVoxelMap(float pos_x,float pos_y,float pos_z){

	center_x = pos_x;
    center_y = pos_y;
    center_z = pos_z;

    last_calc_x = pos_x;
    last_calc_y = pos_y;

	voxel_map.resize(MAP_SIZE_X_Y*2);

	for(int i=0; i < MAP_SIZE_X_Y*2 ;i++){
		voxel_map[i].resize(MAP_SIZE_X_Y*2);
	}

	for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
			voxel_map[i][j].resize(MAP_SIZE_Z*2);
		}
	}

	for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
			for(int k=0 ; k < MAP_SIZE_Z * 2 ; k++){
				voxel_map[i][j][k].resize(2);
				voxel_map[i][j][k][0] = voxel_map[i][j][k][1] = 0;
			}
		}
	}

	voxel_map_for_copy.resize(MAP_SIZE_X_Y*2);

	for(int i=0; i < MAP_SIZE_X_Y*2 ;i++){
		voxel_map_for_copy[i].resize(MAP_SIZE_X_Y*2);
	}

	for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
			voxel_map_for_copy[i][j].resize(MAP_SIZE_Z*2);
		}
	}

	for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
			for(int k=0 ; k < MAP_SIZE_Z * 2 ; k++){
				voxel_map_for_copy[i][j][k].resize(2);
				voxel_map_for_copy[i][j][k][0] = voxel_map_for_copy[i][j][k][1] = 0;
			}
		}
	}

}

// ここがめっちゃむずそう
void PenetrationVoxelMap::RecordSensorData( geometry_msgs::Point LRFcoordinate , sensor_msgs::PointCloud laser_point_data ){
	
	int laser_reflect_index[3],lidar_cordinate_index[3];
	int DATA_NUM = laser_point_data.points.size();

	TranslateRealCordinateToIndex(lidar_cordinate_index , LRFcoordinate.x ,  LRFcoordinate.y ,  LRFcoordinate.z);
	if(!isInMap(lidar_cordinate_index[0],lidar_cordinate_index[1],lidar_cordinate_index[2])){ return; }


	int now_calc_index[3],last_calc_index[3];
	double separated_vector_to_reflected_index[3];
	int separated_num = 1;

	for(int i=0;i<DATA_NUM;i++){
		TranslateRealCordinateToIndex(laser_reflect_index ,laser_point_data.points[i].x,laser_point_data.points[i].y,laser_point_data.points[i].z);
		if( isInMap(laser_reflect_index[0],laser_reflect_index[1],laser_reflect_index[2]) ){
			// ray castを気合で実装
			// 基本的には起点から終点までのベクトルをいい感じに分割し、分割したベクトルが新しいボクセルを通過していたら記録
			separated_num = sqrt( (laser_reflect_index[0]-lidar_cordinate_index[0])*(laser_reflect_index[0]-lidar_cordinate_index[0]) + (laser_reflect_index[1]-lidar_cordinate_index[1])*(laser_reflect_index[1]-lidar_cordinate_index[1]) + (laser_reflect_index[2]-lidar_cordinate_index[2])*(laser_reflect_index[2]-lidar_cordinate_index[2]));
			separated_vector_to_reflected_index[0] = double( laser_reflect_index[0] - lidar_cordinate_index[0] ) / double(separated_num) ;
			separated_vector_to_reflected_index[1] = double( laser_reflect_index[1] - lidar_cordinate_index[1] ) / double(separated_num) ;
			separated_vector_to_reflected_index[2] = double( laser_reflect_index[2] - lidar_cordinate_index[2] ) / double(separated_num) ;
			last_calc_index[0] = lidar_cordinate_index[0] ; last_calc_index[1] = lidar_cordinate_index[1] ; last_calc_index[2] = lidar_cordinate_index[2] ; 
			//cout << separated_num << "," << separated_vector_to_reflected_index[0] << "," <<   separated_vector_to_reflected_index[1] << "," <<   separated_vector_to_reflected_index[2] << "," << endl; 
			
			for(int j=0; j < separated_num; j++){
				now_calc_index[0] = lidar_cordinate_index[0] + separated_vector_to_reflected_index[0] * double(j);
				now_calc_index[1] = lidar_cordinate_index[1] + separated_vector_to_reflected_index[1] * double(j);
				now_calc_index[2] = lidar_cordinate_index[2] + separated_vector_to_reflected_index[2] * double(j);
				if( !(now_calc_index[0] == last_calc_index[0] && now_calc_index[1] == last_calc_index[1] && now_calc_index[2] == last_calc_index[2]) ){
					if( !(now_calc_index[0] == laser_reflect_index[0] && now_calc_index[1] == laser_reflect_index[1] && now_calc_index[2] == laser_reflect_index[2]) ){
						voxel_map[now_calc_index[0]][now_calc_index[1]][now_calc_index[2]][PASS]++;
					}
					//cout << now_calc_index[0] << "," << now_calc_index[1] << "," << now_calc_index[2] << endl;
				}
			}
			

			voxel_map[laser_reflect_index[0]][laser_reflect_index[1]][laser_reflect_index[2]][REFLECT]++;
		}
	}

	return;
}

inline bool PenetrationVoxelMap::isInMap(int x_index,int y_index,int z_index){

	if( 0 <= x_index && x_index < MAP_SIZE_X_Y * 2 && 0 <= y_index && y_index < MAP_SIZE_X_Y * 2 && 0 <= z_index && z_index < MAP_SIZE_Z * 2 ){
		return true;
	}
	else{
		return false;
	}

}


inline void PenetrationVoxelMap::TranslateRealCordinateToIndex(int *return_index,float position_x,float position_y,float position_z){
	return_index[0] = int( float(MAP_SIZE_X_Y) + ( position_x - center_x ) / HORIZONTAL_RESOLUTION );
	return_index[1] = int( float(MAP_SIZE_X_Y) + ( position_y - center_y ) / HORIZONTAL_RESOLUTION );
	return_index[2] = int( float(MAP_SIZE_Z  ) + ( position_z - center_z ) / VERTICAL_RESOLUTION );
}


inline double PenetrationVoxelMap::returnPenetrationRate(int x_index,int y_index,int z_index){
	if(returnMeasuredTimes(x_index,y_index,z_index)==0){
		return -1.0;
	}
	return double(voxel_map[x_index][y_index][z_index][PASS]) / double( voxel_map[x_index][y_index][z_index][REFLECT] + voxel_map[x_index][y_index][z_index][PASS] );
}

inline double PenetrationVoxelMap::returnReflectionRate(int x_index,int y_index,int z_index){
	if(returnMeasuredTimes(x_index,y_index,z_index)==0){
		return -1.0;
	}
	return double(voxel_map[x_index][y_index][z_index][REFLECT]) / double( voxel_map[x_index][y_index][z_index][REFLECT] + voxel_map[x_index][y_index][z_index][PASS] );
}

inline int PenetrationVoxelMap::returnMeasuredTimes(int x_index,int y_index,int z_index){
	return voxel_map[x_index][y_index][z_index][REFLECT] + voxel_map[x_index][y_index][z_index][PASS];
}


void PenetrationVoxelMap::VoxelMapToPointCloud(sensor_msgs::PointCloud *out){

	for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
			for(int k=0 ; k < MAP_SIZE_Z * 2 ;k++){
				if( returnReflectionRate(i,j,k) > 0){
					out->points.push_back(TranslateIndexToRealCordinate(i,j,k));
				}
			}
		}
	}

}


geometry_msgs::Point32 PenetrationVoxelMap::TranslateIndexToRealCordinate(int x_index,int y_index,int z_index){
	geometry_msgs::Point32 point;
	point.x = float(x_index - MAP_SIZE_X_Y) * HORIZONTAL_RESOLUTION + center_x;
	point.y = float(y_index - MAP_SIZE_X_Y) * HORIZONTAL_RESOLUTION + center_y;
	point.z = float(z_index - MAP_SIZE_Z) * VERTICAL_RESOLUTION + center_z;
	return point;
}


void PenetrationVoxelMap::MoveVoxelMapCenter(float pos_x,float pos_y,float pos_z){

	// vectorのコピーともとのvectorの初期化
	for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
			for(int k=0 ; k < MAP_SIZE_Z * 2 ;k++){
				voxel_map_for_copy[i][j][k][0] = voxel_map[i][j][k][0] ;
				voxel_map_for_copy[i][j][k][1] = voxel_map[i][j][k][1] ;
				voxel_map[i][j][k][0] = 0 ;
				voxel_map[i][j][k][1] = 0 ;
			}
		}
	}

	// 複製vectorを移動させる
	int new_x_index,new_y_index,new_z_index;
	for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
			for(int k=0 ; k < MAP_SIZE_Z * 2 ;k++){
				new_x_index = i + (center_x - pos_x) / HORIZONTAL_RESOLUTION;
				new_y_index = j + (center_y - pos_y) / HORIZONTAL_RESOLUTION;
				new_z_index = k + (center_z - pos_z) / VERTICAL_RESOLUTION;
				if( 0 <= new_x_index && 0 <= new_y_index && 0 <= new_z_index && new_x_index < MAP_SIZE_X_Y * 2 && new_y_index < MAP_SIZE_X_Y*2 && new_z_index < MAP_SIZE_Z*2){
					voxel_map[new_x_index][new_y_index][new_z_index][0] = voxel_map_for_copy[i][j][k][0];
					voxel_map[new_x_index][new_y_index][new_z_index][1] = voxel_map_for_copy[i][j][k][1];
				}
			}
		}
	}

	center_x = pos_x;
	center_y = pos_y;
	center_z = pos_z;

}


void PenetrationVoxelMap::printVoxelMapData(){

	cout << "print start" << endl;
	cout << endl;

}