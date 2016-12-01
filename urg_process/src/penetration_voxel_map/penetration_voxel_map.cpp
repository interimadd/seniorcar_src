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
	
	int num_x,num_y,num_z;
	int DATA_NUM = laser_point_data.points.size();

	for(int i=0;i<DATA_NUM;i++){
		num_x = int( float(MAP_SIZE_X_Y) + ( laser_point_data.points[i].x - center_x ) / HORIZONTAL_RESOLUTION );
		if( 0 <= num_x && num_x < MAP_SIZE_X_Y * 2 ){
			num_y = int( float(MAP_SIZE_X_Y) + ( laser_point_data.points[i].y - center_y ) / HORIZONTAL_RESOLUTION );
			if( 0 <= num_y && num_y < MAP_SIZE_X_Y * 2 ){
				num_z = int( float(MAP_SIZE_Z) + ( laser_point_data.points[i].z - center_z ) / VERTICAL_RESOLUTION );
				if( 0 <= num_z && num_z < MAP_SIZE_Z * 2 ){
					voxel_map[num_x][num_y][num_z][REFLECT]++;
				}
			}
		}
	}

	return;
}


void PenetrationVoxelMap::TranslateRealCordinateToIndex(int *return_index,float pose[3]){
	return_index[0] = int( float(MAP_SIZE_X_Y) + ( pose[0] - center_x ) / HORIZONTAL_RESOLUTION );
	return_index[1] = int( float(MAP_SIZE_X_Y) + ( pose[1] - center_y ) / HORIZONTAL_RESOLUTION );
	return_index[2] = int( float(MAP_SIZE_Z  ) + ( pose[2] - center_z ) / VERTICAL_RESOLUTION );
}


void PenetrationVoxelMap::VoxelMapToPointCloud(sensor_msgs::PointCloud *out){

	for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
			for(int k=0 ; k < MAP_SIZE_Z * 2 ;k++){
				if( voxel_map[i][j][k][REFLECT] > 0){
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