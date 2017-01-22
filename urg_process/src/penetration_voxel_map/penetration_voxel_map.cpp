#include "penetration_voxel_map.h"

PenetrationVoxelMap::PenetrationVoxelMap(float pos_x,float pos_y,float pos_z){

	center_x = pos_x;
    center_y = pos_y;
    center_z = pos_z;


    // 透過数カウントボクセルマップの初期化
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

	// 地図中心移動時退避用ボクセルマップの初期化
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

	// 対数オッズボクセルマップの初期化
	l_0 = translatePtoLogOdds(P_R_DEFAULT);
	l_occ = translatePtoLogOdds(P_R_Z);
	l_free = translatePtoLogOdds(P_R_notZ);
	cout << "l_0:" << l_0 << ",l_occ:" << l_occ << ",l_free:" << l_free << endl;

	voxel_odds_map.resize(MAP_SIZE_X_Y*2);
	voxel_odds_map_for_copy.resize(MAP_SIZE_X_Y*2);
	for(int i=0; i < MAP_SIZE_X_Y*2 ;i++){
		voxel_odds_map[i].resize(MAP_SIZE_X_Y*2);
		voxel_odds_map_for_copy[i].resize(MAP_SIZE_X_Y*2);
	}
	for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
			voxel_odds_map[i][j].resize(MAP_SIZE_Z*2);
			voxel_odds_map_for_copy[i][j].resize(MAP_SIZE_Z*2);
		}
	}
	for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
			for(int k=0 ; k < MAP_SIZE_Z * 2 ; k++){
				voxel_odds_map[i][j][k] = l_0;
				voxel_odds_map_for_copy[i][j][k] = l_0;
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
	//int mergine_num   = 0.02 / VERTICAL_RESOLUTION + 2 ; //  レーザの計測誤差を考慮して通過カウントを少し前で切る
	int mergine_num = 0.02 / VERTICAL_RESOLUTION ;

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
			//cout << separated_num << "," << mergine_num << "," << separated_vector_to_reflected_index[0] << "," <<   separated_vector_to_reflected_index[1] << "," <<   separated_vector_to_reflected_index[2] << "," << endl; 
			
			for(int j=0; j < separated_num - mergine_num ; j++){
				now_calc_index[0] = lidar_cordinate_index[0] + separated_vector_to_reflected_index[0] * double(j);
				now_calc_index[1] = lidar_cordinate_index[1] + separated_vector_to_reflected_index[1] * double(j);
				now_calc_index[2] = lidar_cordinate_index[2] + separated_vector_to_reflected_index[2] * double(j);
				// ほとんどカウントが被ることがなかったので冗長かなと思って削除
				/*
				cout << now_calc_index[0] << "," << now_calc_index[1] << "," << now_calc_index[2] << endl;
				if( !(now_calc_index[0] == last_calc_index[0] && now_calc_index[1] == last_calc_index[1] && now_calc_index[2] == last_calc_index[2]) ){
					if( !(now_calc_index[0] == laser_reflect_index[0] && now_calc_index[1] == laser_reflect_index[1] && now_calc_index[2] == laser_reflect_index[2]) ){
						voxel_map[now_calc_index[0]][now_calc_index[1]][now_calc_index[2]][PASS]++;
						last_calc_index[0] = now_calc_index[0]; last_calc_index[1] = now_calc_index[1]; last_calc_index[2] = now_calc_index[2];
					}
				}
				else{
					cout << "kabutta" << endl;
				}
				*/
				voxel_map[now_calc_index[0]][now_calc_index[1]][now_calc_index[2]][PASS]++ ;
				voxel_odds_map[now_calc_index[0]][now_calc_index[1]][now_calc_index[2]] += l_free - l_0;
			}
			voxel_map[laser_reflect_index[0]][laser_reflect_index[1]][laser_reflect_index[2]][REFLECT]++;
			voxel_odds_map[laser_reflect_index[0]][laser_reflect_index[1]][laser_reflect_index[2]] += l_occ - l_0;
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

	sensor_msgs::ChannelFloat32 channel_tmp;
	channel_tmp.name = "intensity";

	float threshold = translatePtoLogOdds(0.1);

	for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
			for(int k=0 ; k < MAP_SIZE_Z * 2 ;k++){
		
				/*
				if( returnReflectionRate(i,j,k) > 0.0){
					out->points.push_back(TranslateIndexToRealCordinate(i,j,k));
					channel_tmp.values.push_back(returnReflectionRate(i,j,k));
				}
				*/

				
				if( voxel_odds_map[i][j][k] == l_0 ){
					//out->points.push_back(TranslateIndexToRealCordinate(i,j,k));
					//channel_tmp.values.push_back(2000);
				}
				else if( voxel_odds_map[i][j][k] < l_0 ){
					//out->points.push_back(TranslateIndexToRealCordinate(i,j,k));
					//channel_tmp.values.push_back(1000);
				}
				else if( voxel_odds_map[i][j][k] > threshold ){
					out->points.push_back(TranslateIndexToRealCordinate(i,j,k));
					channel_tmp.values.push_back(3000);
				}

				/*
				// 目印用のバーを可視化するためのもの
				if( returnReflectionRate(i,j,k) > 0.0 && TranslateIndexToRealCordinate(i,j,k).z > 0.35){
					out->points.push_back(TranslateIndexToRealCordinate(i,j,k));
					channel_tmp.values.push_back(returnReflectionRate(i,j,k));
				}
				*/
				
			}
		}
	}

	out->channels.push_back(channel_tmp);

}


geometry_msgs::Point32 PenetrationVoxelMap::TranslateIndexToRealCordinate(int x_index,int y_index,int z_index){
	geometry_msgs::Point32 point;
	point.x = float(x_index - MAP_SIZE_X_Y) * HORIZONTAL_RESOLUTION + center_x;
	point.y = float(y_index - MAP_SIZE_X_Y) * HORIZONTAL_RESOLUTION + center_y;
	point.z = float(z_index - MAP_SIZE_Z) * VERTICAL_RESOLUTION + center_z;
	return point;
}


void PenetrationVoxelMap::MoveVoxelMapCenter(float pos_x,float pos_y,float pos_z){

	pos_x = floor(pos_x);
	pos_y = floor(pos_y);
	pos_z = floor(pos_z);

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
	geometry_msgs::Point32 tmp_point;
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

	// 対数オッズボクセルマップの移動
	// vectorのコピーともとのvectorの初期化
	for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
			for(int k=0 ; k < MAP_SIZE_Z * 2 ;k++){
				voxel_odds_map_for_copy[i][j][k] = voxel_odds_map[i][j][k] ;
				voxel_odds_map[i][j][k] = l_0;
			}
		}
	}

	// 複製vectorを移動させる
	//int new_x_index,new_y_index,new_z_index;
	//geometry_msgs::Point32 tmp_point;
	for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
			for(int k=0 ; k < MAP_SIZE_Z * 2 ;k++){
				new_x_index = i + (center_x - pos_x) / HORIZONTAL_RESOLUTION;
				new_y_index = j + (center_y - pos_y) / HORIZONTAL_RESOLUTION;
				new_z_index = k + (center_z - pos_z) / VERTICAL_RESOLUTION;
				if( 0 <= new_x_index && 0 <= new_y_index && 0 <= new_z_index && new_x_index < MAP_SIZE_X_Y * 2 && new_y_index < MAP_SIZE_X_Y*2 && new_z_index < MAP_SIZE_Z*2){
					voxel_odds_map[new_x_index][new_y_index][new_z_index] = voxel_odds_map_for_copy[i][j][k];
				}
			}
		}
	}

	center_x = pos_x;
	center_y = pos_y;
	center_z = pos_z;

}


void PenetrationVoxelMap::printVoxelMapData(){

	geometry_msgs::Point32 tmp_p;

	cout << "print start" << endl;
	cout << "x,y,z,reflect_num,pass_num,reflectrate,penetrationrate,road_p" << endl;

	for(int k=0 ; k < MAP_SIZE_Z * 2 ;k++){
		for(int i=0 ; i < MAP_SIZE_X_Y * 2 ; i++){
			for(int j=0 ; j < MAP_SIZE_X_Y * 2 ;j++){
				//if( returnReflectionRate(i,j,k) > 0.0){
					tmp_p = TranslateIndexToRealCordinate(i,j,k);
					if(3.3 < tmp_p.x && tmp_p.x <= 3.6 && -0.4 < tmp_p.y && tmp_p.y < 0.4 && 0.29 < tmp_p.z && tmp_p.z < 0.34){
						cout << tmp_p.x << "," << tmp_p.y << "," << tmp_p.z << "," << int(voxel_map[i][j][k][REFLECT]) << "," << int(voxel_map[i][j][k][PASS]) << "," << returnReflectionRate(i,j,k) << "," << returnPenetrationRate(i,j,k) << "," << float(voxel_odds_map[i][j][k])  <<"," <<float(translateLogOddstoP(voxel_odds_map[i][j][k])) <<endl;
					}
				//}
			}
		}
	}

	cout << endl;

}

float PenetrationVoxelMap::translatePtoLogOdds(float p){
	return log( p / (1.0 - p) );
}

float PenetrationVoxelMap::translateLogOddstoP(float log_odds){
	return 1.0 / ( 1.0 + exp(-log_odds) );
}

double PenetrationVoxelMap::returnHighestVoxeclHeightInCordinate(float position_x,float position_y){

	int tmp_index[3];
	TranslateRealCordinateToIndex(tmp_index , position_x ,  position_y ,  center_z);

	int max_i = 0;
	for(int i=0 ; i < MAP_SIZE_Z * 2 ;i++){
		if( voxel_odds_map[tmp_index[0]][tmp_index[1]][i] > translatePtoLogOdds(0.9) ){
			max_i = i;
		}
	}

	return float(max_i - MAP_SIZE_Z) * VERTICAL_RESOLUTION + center_z;
}