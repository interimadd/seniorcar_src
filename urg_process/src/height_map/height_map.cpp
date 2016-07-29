#include "height_map.h"

Height_Map::Height_Map(float pos_x,float pos_y,int map_size_x,int map_size_y,float horizontal_resolution){

	MAP_SIZE_X = map_size_x;
	MAP_SIZE_Y = map_size_y;

	center_x = pos_x;
    center_y = pos_y;

    last_calc_x = pos_x;
    last_calc_y = pos_y;

	HORIZONTAL_RESOLUTION = horizontal_resolution;

	height_map.resize(map_size_x*2);
	interpolated_height_map.resize(map_size_x*2);
	rotate_enable_map.resize(map_size_x*2);

	for(int i=0; i < map_size_x*2 ;i++){
		height_map[i].resize(map_size_y*2);
		interpolated_height_map[i].resize(map_size_y*2);
		rotate_enable_map[i].resize(map_size_y*2);
	}

	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			height_map[i][j] = NOT_DETECT;
			interpolated_height_map[i][j] = NOT_DETECT;
			rotate_enable_map[i][j] = UNKNOWN;
		}
	}

	buff = new BufferPoint[map_size_x*2];

}


void Height_Map::RecordSensorData(sensor_msgs::PointCloud laser_point_data){
	
	int num_x,num_y;
	int DATA_NUM = laser_point_data.points.size();

	for(int i=0;i<DATA_NUM;i++){
		num_x = int( float(MAP_SIZE_X) + ( laser_point_data.points[i].x - center_x ) / HORIZONTAL_RESOLUTION );
		if( 0 <= num_x && num_x < MAP_SIZE_X * 2 ){
			num_y = int( float(MAP_SIZE_Y) + ( laser_point_data.points[i].y - center_y ) / HORIZONTAL_RESOLUTION );
			if( 0 <= num_y && num_y < MAP_SIZE_Y * 2 ){
				if(laser_point_data.points[i].z < 0.5){
					if( height_map[num_x][num_y] < laser_point_data.points[i].z && laser_point_data.points[i].z > NOT_DETECT){
						height_map[num_x][num_y] = laser_point_data.points[i].z;
					}
					//height_map[num_x][num_y] = laser_point_data.points[i].z;
				}
			}
		}
	}

	InterpolateMap();

	return;
}


void Height_Map::TranslateRealCordinateToIndex(int *return_index,float pose[2]){
	return_index[0] = int( float(MAP_SIZE_X) + ( pose[0] - center_x ) / HORIZONTAL_RESOLUTION );
	return_index[1] = int( float(MAP_SIZE_Y) + ( pose[1] - center_y ) / HORIZONTAL_RESOLUTION );
}


void Height_Map::HeightMapToPointCloud(sensor_msgs::PointCloud *out){

	for(int i=0 ; i < MAP_SIZE_X * 2 ; i++){
		for(int j=0 ; j < MAP_SIZE_Y * 2 ;j++){
			if( interpolated_height_map[i][j] != NOT_DETECT){
				out->points.push_back(TranslateIndexToRealCordinate(i,j));
			}
		}
	}

}


void Height_Map::TireHeightMapToPointCloud(sensor_msgs::PointCloud *out){

	int SPACE = tire_radius_in_grid + 3;

	for(int i=SPACE ; i < MAP_SIZE_X * 2 - SPACE ; i++){
		for(int j=SPACE ; j < MAP_SIZE_Y * 2 - SPACE ;j++){
			//if( interpolated_height_map[i][j] != NOT_DETECT){
				geometry_msgs::Point32 tmp_p = TranslateIndexToRealCordinate(i,j);
				tmp_p.z = returnTireHeightInGrid(i,j);
				if(tmp_p.z > -2.0 && tmp_p.z < 1.5){
					out->points.push_back(tmp_p);
				}
			//}
		}
	}

}


inline geometry_msgs::Point32 Height_Map::TranslateIndexToRealCordinate(int x_index,int y_index){
	geometry_msgs::Point32 point;
	point.x = float(x_index - MAP_SIZE_X) * HORIZONTAL_RESOLUTION + center_x;
	point.y = float(y_index - MAP_SIZE_Y) * HORIZONTAL_RESOLUTION + center_y;
	point.z = interpolated_height_map[x_index][y_index];
	return point;
}


void Height_Map::MoveHeightMapCenter(float pos_x,float pos_y){

	// 複製用のvector
	vector < vector <float> > tmp_map;
	tmp_map.resize( MAP_SIZE_X * 2);
	for(int i=0; i < MAP_SIZE_X * 2 ;i++){
		tmp_map[i].resize(MAP_SIZE_Y * 2 );
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


void Height_Map::SetTireRadius(float tire_radius,float tire_width){

	float sin45 = 0.3;

	enable_rolling_threshold = tire_radius * sin45 / HORIZONTAL_RESOLUTION;
	tire_radius_in_grid = tire_radius / HORIZONTAL_RESOLUTION;
	half_tire_width_in_grid = tire_width / (2.0 * HORIZONTAL_RESOLUTION);

	for(int i=0;i<tire_radius_in_grid;i++){
		float tmp_height = sqrt(pow(tire_radius,2) - pow(HORIZONTAL_RESOLUTION*i,2));
		tire_height.push_back(tmp_height);
	}

}


inline RotateEnable Height_Map::isEnableRolling(int x_index,int y_index){

	const float START_HEIGHT = -10.0;
	float max_height = START_HEIGHT;
	int max_height_grid;
	
	for(int i=0;i<tire_radius_in_grid;i++){
		for(int j=0;j<half_tire_width_in_grid;j++){

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

	if(max_height_grid > enable_rolling_threshold){
		return DISABLE;
	}

	if( max_height < START_HEIGHT + 1.0f ){
		return DISABLE;
	}

	max_height = START_HEIGHT;

	for(int i=0;i<tire_radius_in_grid;i++){
		for(int j=0;j<half_tire_width_in_grid;j++){

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

	if(max_height_grid > enable_rolling_threshold){
		return DISABLE;
	}

	if( max_height < START_HEIGHT + 1.0f){
		return DISABLE;
	}

	return ENABLE;

}


inline float Height_Map::returnTireHeightInGrid(int x_index,int y_index){

	const float START_HEIGHT = -10.0;
	float max_height = START_HEIGHT;
	int max_height_grid;
	
	for(int i=0;i<tire_radius_in_grid;i++){
		for(int j=0;j<half_tire_width_in_grid;j++){

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

	for(int i=0;i<tire_radius_in_grid;i++){
		for(int j=0;j<half_tire_width_in_grid;j++){

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



void  Height_Map::RotateEnableAreaToPointCloud(sensor_msgs::PointCloud *out){

	geometry_msgs::Point32 tmp_p;
	int SPACE = tire_radius_in_grid + 3;
	int count=0;

	for(int i=SPACE ; i < MAP_SIZE_X * 2 - SPACE ; i++){
		for(int j=SPACE ; j < MAP_SIZE_Y * 2 - SPACE ;j++){
			if(rotate_enable_map[i][j]==DRIVABLE){
			//if(rotate_enable_map[i][j]==DRIVABLE || rotate_enable_map[i][j] == ENABLE){
				tmp_p = TranslateIndexToRealCordinate(i,j);
				tmp_p.z = 0.1f;
				out->points.push_back(tmp_p);
				count++;
			}
		}
	}

	//printf("pub:%d\n",count);

}

void Height_Map::UpdateRotateEnableMap(){

	int SPACE = tire_radius_in_grid + 3;

	for(int i=SPACE ; i < MAP_SIZE_X * 2 - SPACE ; i++){
		for(int j=SPACE ; j < MAP_SIZE_Y * 2 - SPACE ;j++){
			rotate_enable_map[i][j] = isEnableRolling(i,j) ;
		}
	}

}


// スキャンシードフィルアルゴリズムを使用
// http://fussy.web.fc2.com/algo/algo3-1.htm

void Height_Map::DetectRotateEnableAreaFromPoint(float pos_x,float pos_y){

	vector<BufferPoint> seed_buffer;

	int left_y , right_y;
	int line_x;
	int i;
	int SPACE = tire_radius_in_grid + 2;
	int MAX_BUFF_X = MAP_SIZE_X*2-SPACE;
	int MAX_BUFF_Y = MAP_SIZE_Y*2-SPACE;
	int endless_loop_count = 0; // 無限ループしてそうなときにループを終了する

	BufferPoint tmp_point;
	tmp_point.sx = float(MAP_SIZE_X) + ( pos_x - center_x ) / HORIZONTAL_RESOLUTION;
	tmp_point.sy = float(MAP_SIZE_Y) + ( pos_y - center_y ) / HORIZONTAL_RESOLUTION;

	seed_buffer.push_back(tmp_point);

	if(rotate_enable_map[seed_buffer[0].sx ][seed_buffer[0].sy ] == DISABLE){
		//printf("disable_area:%d,%d\n",seed_buffer[0].sx,seed_buffer[0].sy);
		return;
	}
	else{
		//printf("able_area:%d,%d\n",seed_buffer[0].sx,seed_buffer[0].sy);
	}

	// FIFO的な処理
	while( !seed_buffer.empty() ){

		//printf("seed_buffer:%d,%d ",seed_buffer[0].sx,seed_buffer[0].sy);

		left_y = right_y = seed_buffer[0].sy;
		line_x = seed_buffer[0].sx;

		if(rotate_enable_map[seed_buffer[0].sx][seed_buffer[0].sy] == ENABLE){

			while( right_y < MAX_BUFF_Y ){
				if(rotate_enable_map[line_x][right_y] == DISABLE){
					right_y--;
					break;
				}
				else{
					right_y++;
				}
			}

			while( left_y > SPACE ){
				if(rotate_enable_map[line_x][left_y] == DISABLE){
					break;
					left_y++;
				}
				else{
					left_y--;
				}
			}

			for(i=left_y;i<=right_y;i++){
				rotate_enable_map[line_x][i] = DRIVABLE;
			}

			//printf("ly:%d,ry:%d\n",left_y,right_y);


			// ここから次のシードを探してバッファに追加する処理
			// まずは上側
			if(line_x + 1 < MAX_BUFF_X){

				tmp_point.sx = line_x + 1;
				tmp_point.sy = left_y;

				while(tmp_point.sy <= right_y){

					// 走行可能領域の左端の座標を探す
					for(;tmp_point.sy<=right_y;tmp_point.sy++){
						if( rotate_enable_map[tmp_point.sx][tmp_point.sy] == ENABLE){
							break;
						}
					}

					// 走行可能領域の右端の座標を探す
					for(;tmp_point.sy<=right_y;tmp_point.sy++){
						if( rotate_enable_map[tmp_point.sx][tmp_point.sy] != ENABLE){
							tmp_point.sy--;
							seed_buffer.push_back(tmp_point);
							tmp_point.sy++;
							break;
						}
					}

				}

				tmp_point.sy--;
				if(rotate_enable_map[tmp_point.sx][tmp_point.sy] == ENABLE){
					seed_buffer.push_back(tmp_point);
				}

			}

			// 下のバッファを探す

			if(line_x - 1 > SPACE){

				tmp_point.sx = line_x - 1;
				tmp_point.sy = left_y;

				while(tmp_point.sy <= right_y){

					// 走行可能領域の左端の座標を探す
					for(;tmp_point.sy<=right_y;tmp_point.sy++){
						if( rotate_enable_map[tmp_point.sx][tmp_point.sy] == ENABLE){
							break;
						}
					}

					// 走行可能領域の右端の座標を探す
					for(;tmp_point.sy<=right_y;tmp_point.sy++){
						if( rotate_enable_map[tmp_point.sx][tmp_point.sy] != ENABLE){
							tmp_point.sy--;
							seed_buffer.push_back(tmp_point);
							tmp_point.sy++;
							break;
						}
					}

				}

				tmp_point.sy--;
				if(rotate_enable_map[tmp_point.sx][tmp_point.sy] == ENABLE){
					seed_buffer.push_back(tmp_point);
				}

			}

		}

		// 処理したバッファを削除
		seed_buffer.erase(seed_buffer.begin());
		// 無限ループっぽい時に処理を終了
		if(++endless_loop_count > 1000){
			printf("it seems to be in endless loop\n");
			return;
		}

	}

}



/*
void Height_Map::DetectRotateEnableAreaFromPoint(float pos_x,float pos_y){

	int lx, rx; // 塗り潰す線分の両端のX座標
 	int ly;     // 塗り潰す線分のY座標
 	int i;
 	int SPACE = tire_radius_in_grid + 3;

 	sIdx = buff;
 	eIdx = buff + 1;
 	sIdx->sx = float(MAP_SIZE_X) + ( pos_x - center_x ) / HORIZONTAL_RESOLUTION;;
 	sIdx->sy = float(MAP_SIZE_Y) + ( pos_y - center_y ) / HORIZONTAL_RESOLUTION;

 	int count = 0;


 	if(!rotate_enable_map[sIdx->sx][sIdx->sy]){
 		return;
 	}

	do {
    	lx = rx = sIdx->sx;
    	ly = sIdx->sy;
    	if ( ++sIdx == &buff[MAP_SIZE_X*2-SPACE] ){
      		sIdx = buff;
      	}

    	// 処理済のシードなら無視
    	if ( !rotate_enable_map[lx][ly] ){
      		continue;
      	}

	    // 右方向の境界を探す
	    while ( rx < MAP_SIZE_X*2-SPACE ) {
 	    	if ( !rotate_enable_map[rx+1][ly] ) break;
	    	rx++;
 	   	}
 	   
 	   	// 左方向の境界を探す
 	   	while ( lx > SPACE ) {
 	    	if ( !rotate_enable_map[lx-1][ly] ) break;
 	    	lx--;
	    }
	    
	    // lx-rxの線分を描画
	    if(rx>lx){
	    	for ( i = lx ; i <= rx ; i++ ){
	    		rotate_enable_map[i][ly] = DRIVABLE;
	    		count++;
	    	}
		}
	
	    // 真上のスキャンラインを走査する
 	   	if ( ly - 1 >= 0 ){
	     	ScanLine( lx, rx, ly - 1);
		}

	
	    // 真下のスキャンラインを走査する
 		if ( ly + 1 <= MAP_SIZE_Y * 2 - SPACE ){
	    	ScanLine( lx, rx, ly + 1 );
	    }

	
 	 } while ( sIdx != eIdx );	

 	 printf("%d\n",count);
}




void Height_Map::ScanLine( int lx, int rx, int y)
{

  while ( lx <= rx ) {

    // 非領域色を飛ばす
    for ( ; lx <= rx ; lx++ ){
     	if ( rotate_enable_map[lx][y] ) break;
  	}
    if ( !rotate_enable_map[lx][y] ) break;

    // 領域色を飛ばす
    for ( ; lx <= rx ; lx++ )
      if ( !rotate_enable_map[lx][y] ) break;

    eIdx->sx = lx - 1;
    eIdx->sy = y;
    if ( ++eIdx == &buff[MAP_SIZE_X*2-1] ) eIdx = buff;
  }
 
}
*/


void Height_Map::InterpolateMap(){

	float height_sum = 0;
	float sum_num = 0;
	
	for(int i=2 ; i < MAP_SIZE_X * 2 - 2 ; i++){
		for(int j=2 ; j < MAP_SIZE_Y * 2 - 2 ;j++){

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
				if(sum_num>2){
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


	/*
	for(int i=2 ; i < MAP_SIZE_X * 2 - 2 ; i++){
		for(int j=2 ; j < MAP_SIZE_Y * 2 - 2 ;j++){
			interpolated_height_map[i][j] = height_map[i][j];
		}
	}
	*/

}


