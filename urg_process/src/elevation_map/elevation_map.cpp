#include "elevation_map.h"

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
	variance_map.resize(map_size_x*2);

	for(int i=0; i < map_size_x*2 ;i++){
		height_map[i].resize(map_size_y*2);
		interpolated_height_map[i].resize(map_size_y*2);
		variance_map[i].resize(map_size_y*2);
	}

	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			variance_map[i][j].resize(4,0.0);
		}
	}
	variance_map[1][1];

	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			height_map[i][j] = NOT_DETECT;
			interpolated_height_map[i][j] = NOT_DETECT;
		}
	}

	getElevationMapFilePos();

}


void ElevationMap::RecordSensorData(sensor_msgs::PointCloud laser_point_data){
	
	int num_x,num_y;
	int DATA_NUM = laser_point_data.points.size();

	for(int i=0;i<DATA_NUM;i++){
		num_x = int( float(MAP_SIZE_X) + ( laser_point_data.points[i].x - center_x ) / HORIZONTAL_RESOLUTION );
		if( 0 <= num_x && num_x < MAP_SIZE_X * 2 ){
			num_y = int( float(MAP_SIZE_Y) + ( laser_point_data.points[i].y - center_y ) / HORIZONTAL_RESOLUTION );
			if( 0 <= num_y && num_y < MAP_SIZE_Y * 2 ){
				if( -11.0 < laser_point_data.points[i].z && laser_point_data.points[i].z < 10.5){
					variance_map[num_x][num_y][1] += 1; //n
					variance_map[num_x][num_y][2] += pow( laser_point_data.points[i].z , 2) ;
					variance_map[num_x][num_y][3] += laser_point_data.points[i].z ;
					if( variance_map[num_x][num_y][1] > 1 ){
						variance_map[num_x][num_y][0] = (variance_map[num_x][num_y][2] - pow(variance_map[num_x][num_y][3],2)/variance_map[num_x][num_y][1] ) / (variance_map[num_x][num_y][1] - 1);
					}
				//height_map[num_x][num_y] = laser_point_data.points[i].z;
					if(height_map[num_x][num_y] == NOT_DETECT){
						height_map[num_x][num_y] = laser_point_data.points[i].z;
					}
					else if( height_map[num_x][num_y] > laser_point_data.points[i].z){
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
			if( interpolated_height_map[i][j] != NOT_DETECT){
				out->points.push_back(TranslateIndexToRealCordinate(i,j));
			}
		}
	}

}


void ElevationMap::VarianceMapToPointCloud(sensor_msgs::PointCloud *out){

	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			if( interpolated_height_map[i][j] != NOT_DETECT){
				geometry_msgs::Point32 push_point = TranslateIndexToRealCordinate(i,j);
				push_point.z = variance_map[i][j][0] * 1000;
				out->points.push_back(push_point);
			}
		}
	}

}

void ElevationMap::TypeMapToPointCloud(sensor_msgs::PointCloud *road,sensor_msgs::PointCloud *grass,sensor_msgs::PointCloud *others){

	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			if( interpolated_height_map[i][j] != NOT_DETECT){
				geometry_msgs::Point32 push_point = TranslateIndexToRealCordinate(i,j);
				//push_point.z = 0.5;
				if(variance_map[i][j][0] < VEGETATION_THRESHOLD_MIN){
					road->points.push_back(push_point);
				}
				else if( VEGETATION_THRESHOLD_MIN <= variance_map[i][j][0] && variance_map[i][j][0] < VEGETATION_THRESHOLD_MAX){
					//grass->points.push_back(push_point);
					road->points.push_back(push_point);
				}
				/*
				else{
					others->points.push_back(push_point);
				}
				*/
			}
			else{
				if( variance_map[i][j][1] > 0){
					geometry_msgs::Point32 push_point = TranslateIndexToRealCordinate(i,j);
					push_point.z = variance_map[i][j][3] / variance_map[i][j][1];
					others->points.push_back(push_point);
				}
			}
		}
	}

}

inline geometry_msgs::Point32 ElevationMap::TranslateIndexToRealCordinate(int x_index,int y_index){
	geometry_msgs::Point32 point;
	point.x = float(x_index - MAP_SIZE_X) * HORIZONTAL_RESOLUTION + center_x;
	point.y = float(y_index - MAP_SIZE_Y) * HORIZONTAL_RESOLUTION + center_y;
	point.z = interpolated_height_map[x_index][y_index];
	return point;
}


void ElevationMap::MoveHeightMapCenter(float pos_x,float pos_y){

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
			
			tmp_variance_map[i][j].resize(4,0.0);
			tmp_variance_map[i][j][0] = variance_map[i][j][0] ;
			tmp_variance_map[i][j][1] = variance_map[i][j][1] ;
			tmp_variance_map[i][j][2] = variance_map[i][j][2] ;
			tmp_variance_map[i][j][3] = variance_map[i][j][3] ;

			variance_map[i][j][0] = 0.0 ;
			variance_map[i][j][1] = 0.0 ;
			variance_map[i][j][2] = 0.0 ;
			variance_map[i][j][3] = 0.0 ;
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
						variance_map[new_x_index][new_y_index][0] = tmp_variance_map[i][j][0];
						variance_map[new_x_index][new_y_index][1] = tmp_variance_map[i][j][1];
						variance_map[new_x_index][new_y_index][2] = tmp_variance_map[i][j][2];
						variance_map[new_x_index][new_y_index][3] = tmp_variance_map[i][j][3];
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

	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			if(variance_map[i][j][1]>1){
				geometry_msgs::Point32 tmp_point = TranslateIndexToRealCordinate(i,j);
				cout << "cordinate:" << tmp_point.x << "," << tmp_point.y << "," << tmp_point.z << ",";
				cout << "variance_map:" << variance_map[i][j][0] << "," << variance_map[i][j][1] << "," << variance_map[i][j][3] / variance_map[i][j][1] << endl;
			}
		}
	}

	cout << endl;

}


void ElevationMap::InterpolateMap(){
	
	for(int i=2 ; i < MAP_SIZE_X * 2 - 2 ; i++){
		for(int j=2 ; j < MAP_SIZE_Y * 2 - 2 ;j++){

			float height_sum = 0;
			float sum_num = 0;
			interpolated_height_map[i][j] = NOT_DETECT;

			/*
			if(variance_map[i][j][1] > 2 && variance_map[i][j][0] < VEGETATION_THRESHOLD_MIN){
				// 草地でない判定がされた場合は計測値の平均値を路面高さとする
				interpolated_height_map[i][j] = variance_map[i][j][3] / variance_map[i][j][1];				
			}
			else{
				for(int n=-1;n<=1;n++){
					for(int m=-1;m<=1;m++){
						if(variance_map[i+n][j+m][1] > 2 && variance_map[i+n][j+m][0] < VEGETATION_THRESHOLD_MIN){
							sum_num += 1;
							height_sum += variance_map[i+n][j+m][3] / variance_map[i+n][j+m][1] ;
						}
					}
				}
				if(sum_num>3){
					interpolated_height_map[i][j] = height_sum / sum_num;
				}
			}
			*/

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
				if(sum_num>10){
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
			

			/*
			if(height_map[i][j] == NOT_DETECT){
				for(int n=-1;n<=1;n++){
					for(int m=-1;m<=1;m++){
						if(height_map[i+n][j+m] != NOT_DETECT){
							sum_num += 1;
							height_sum += height_map[i+n][j+m];
						}
					}
				}
				if(sum_num>14){
					interpolated_height_map[i][j] = height_sum / sum_num;
				}
				else{
					interpolated_height_map[i][j] = NOT_DETECT;
				}
				height_sum = 0;
				sum_num = 0;
			}
			else{
				interpolated_height_map[i][j] = height_map[i][j];
			}
			*/

		}
	}
}

			


void ElevationMap::outputElevationMapToTextFile(){

	stringstream filename;
	filename << MAP_TEXT_DIR << "x" << int(center_x) << "y" << int(center_y) << ".csv";
	ofstream output_file;
	output_file.open(filename.str().c_str(),ios::out);

	output_file << center_x << "," << center_y << "," << MAP_SIZE_X << "," << MAP_SIZE_Y << "," << HORIZONTAL_RESOLUTION << endl;

	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			output_file << variance_map[i][j][0] << "," << variance_map[i][j][1] << "," << variance_map[i][j][2] << "," << variance_map[i][j][3] << "," << height_map[i][j] << endl; 
		}
	}

	output_file.close();

	cout << "record map " << filename.str() << endl;

}


void ElevationMap::inputElevationMapFromTextFile(float pos_x,float pos_y){

	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			variance_map[i][j][0] = 0.0 ;
			variance_map[i][j][1] = 0.0 ;
			variance_map[i][j][2] = 0.0 ;
			variance_map[i][j][3] = 0.0 ;
		}
	}

	float min_pos = 10000;
	int min_distance_index = 0;
	for(int i = 0;i < map_pos_list.size() ;i+=2){
		float tmp_pos = pow( pos_x - map_pos_list[i] , 2) + pow( pos_y - map_pos_list[i+1] , 2);
		if( min_pos > tmp_pos){
			min_pos = tmp_pos;
			min_distance_index = i;
		}
	}

	stringstream filename;
	filename << MAP_TEXT_DIR << "x" << map_pos_list[min_distance_index] << "y" << map_pos_list[min_distance_index + 1] << ".csv";
	cout << "open " << filename.str() << endl;

	
	FILE *fp;
	fp = fopen(filename.str().c_str(), "r");

	double file_center_x , file_center_y , file_MAP_SIZE_X , file_MAP_SIZE_Y , file_HORIZONTAL_RESOLUTION;
	fscanf(fp, "%lf,%lf,%lf,%lf,%lf", &file_center_x , &file_center_y , &file_MAP_SIZE_X , &file_MAP_SIZE_Y , &file_HORIZONTAL_RESOLUTION);

	center_x = file_center_x;
	center_y = file_center_y;

	// ゲインファイル内の数字をGainForStand[番号][配列のサイズ]配列内に格納
	int index = 0;
	double v0,v1,v2,v3, elev_map;
	while (fscanf(fp, "%lf,%lf,%lf,%lf,%lf", &v0 , &v1 , &v2 ,&v3 ,&elev_map) != EOF){

		int i = index / int(file_MAP_SIZE_X*2);
		int j = index % int(file_MAP_SIZE_Y*2);

		variance_map[i][j][0] = v0;
		variance_map[i][j][1] = v1;
		variance_map[i][j][2] = v2;
		variance_map[i][j][3] = v3;

		//height_map[i][j] = v3 / v1;
		height_map[i][j] = elev_map;

		index++;
	}

	fclose(fp);

}

void ElevationMap::getElevationMapFilePos(){

	struct dirent* dent;
	DIR* dp = opendir(MAP_TEXT_DIR.c_str());

	if(dp!=NULL){
		do{
			dent = readdir(dp);
			if(dent!=NULL){
				string filename(dent->d_name);
				if(filename.find(".csv") != string::npos){
					cout << dent->d_name << endl;
					size_t pos_y = filename.find("y");
					size_t pos_dot = filename.find(".");
					if( pos_y != string::npos && pos_dot != string::npos){
						int x_value = atoi(filename.substr(1,pos_y-1).c_str());
						int y_value = atoi(filename.substr(pos_y+1, pos_dot - pos_y - 1).c_str());
						cout << x_value << "," << y_value << endl;
						map_pos_list.push_back(x_value);
						map_pos_list.push_back(y_value);
					}
				}
			}
		}while( dent != NULL );
	}
	closedir(dp);

}