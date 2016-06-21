#include "height_map2.h"

Height_Map::Height_Map(){
	// 設定パラメータ //
	dx = 0.085;   // 85X,Y方向の刻みを設定(XおよびYは共通の刻み幅)
	dz = 0.005;   // Z方向の刻みを設定
	
	MeshMinX = -3.000;   //
	MeshMaxX =  2.000;   //
	MeshMinY = -2.000;   //
	MeshMaxY =  2.000;   //
	MeshMinZ = -0.200;   //
	MeshMaxZ =  0.500;   //
	
	NumMeshX = int(float(MeshMaxX-MeshMinX)/dx)+1;
	NumMeshY = int(float(MeshMaxY-MeshMinY)/dx)+1;
	NumMeshZ = int(float(MeshMaxZ-MeshMinZ)/dz)+1;
	
        //重要設定項目
        float kugiri_z = float(dz);	//(set)高さ方向の刻み
        float kugiri = float(dx);	//(set)水平方向の刻み       %実車タイヤ幅
        float Tire_R = 0.1325;		//(set)タイヤ半径           %実車タイヤ半径
        
        div = 3;
        dx2 = dx/div;
	NumMesh2X = int(float(MeshMaxX-MeshMinX)/dx2)+1;
	NumMesh2Y = int(float(MeshMaxY-MeshMinY)/dx2)+1;
	
        //初期化
	center_x = 0;
    	center_y = 0;
    	
	Mesh3D.resize(NumMeshX);
	for(int i=0; i<NumMeshX; ++i){
		Mesh3D[i].resize(NumMeshY);
        	for(int j=0; j<NumMeshY; ++j){
        		Mesh3D[i][j].resize(NumMeshZ,0);
        	}
        }
        if(NumMeshX>NumMeshY){
        	Mesh3D_Buff.resize(NumMeshX);
		for(int i=0; i<NumMeshX; ++i){
			Mesh3D_Buff[i].resize(NumMeshX);
        		for(int j=0; j<NumMeshX; ++j){
        			Mesh3D_Buff[i][j].resize(NumMeshZ,0);
        		}
        	}
        }else{
        	Mesh3D_Buff.resize(NumMeshY);
		for(int i=0; i<NumMeshY; ++i){
			Mesh3D_Buff[i].resize(NumMeshY);
        		for(int j=0; j<NumMeshY; ++j){
        			Mesh3D_Buff[i][j].resize(NumMeshZ,0);
        		}
        	}
        }
        
	CanGoY1.resize(NumMeshY);
	CanGoY2.resize(NumMeshY);
	CanGoX1.resize(NumMeshY);
	CanGoX2.resize(NumMeshY);
	PointZ_Y1.resize(NumMeshY);
	PointZ_Y2.resize(NumMeshY);
	PointZ_X1.resize(NumMeshY);
	PointZ_X2.resize(NumMeshY);
	CanGoDirectionAll.resize(NumMeshY);
	go.resize(NumMeshY);
	
	for(int i=0; i<NumMeshY; ++i){
		CanGoY1[i].resize(NumMeshX);
		CanGoY2[i].resize(NumMeshX);
		CanGoX1[i].resize(NumMeshX);
		CanGoX2[i].resize(NumMeshX);
		PointZ_Y1[i].resize(NumMeshX);
		PointZ_Y2[i].resize(NumMeshX);
		PointZ_X1[i].resize(NumMeshX);
		PointZ_X2[i].resize(NumMeshX);
		CanGoDirectionAll[i].resize(NumMeshX);
		go[i].resize(NumMeshX);
        }
        
        height_map.resize(NumMesh2X);
	for(int i=0; i<NumMesh2X; ++i){
		height_map[i].resize(NumMesh2Y);
		for(int j=0; j<NumMesh2Y; ++j){
			height_map[i][j] = NOT_DETECT;
		}
	}
	
        initializeStereoCalc(Tire_R,kugiri,kugiri_z);
        
}

void Height_Map::initializeStereoCalc(float Tire_R, float kugiri, float kugiri_z){

	float Tire_D = Tire_R * 2;				//タイヤ直径
	int NumberOfSerchY = int(trunc(Tire_D / kugiri));		//Y方向探索数
	int NumberOfSerchZ;
	
	//準備する配列の大きさを計算
	int count = 0;
	for(int j=0; j<=NumberOfSerchY; ++j){
        	NumberOfSerchZ = int(trunc(sqrt(pow(Tire_D,2)-pow((j*kugiri),2))/kugiri_z));
        	for(int k=-NumberOfSerchZ; k<=NumberOfSerchZ; ++k){
            		++count;
        	}
    	}
    	
    	int CountMax = count-1;
    	vector < vector <float> > CircleCenter;
    	vector < vector <float> > ForwardTheta;
    	
	CircleCenter.resize(CountMax);
	ForwardTheta.resize(CountMax);
	
	for(int i=0; i<CountMax; ++i){
		CircleCenter[i].resize(2,0);
		ForwardTheta[i].resize(4,0);
	}
	//ころころデータの作成
	float Y1,Z1,Nolm1,Nolm2,Y2,Z2,Y3,Z3,Y4,Z4,Y5,Z5;
	count = 1;
	for(int j=0; j<=NumberOfSerchY; ++j){
        	NumberOfSerchZ = int(trunc(sqrt(pow(Tire_D,2)-pow((j*kugiri),2))/kugiri_z));
		for(int k=-NumberOfSerchZ; k<=NumberOfSerchZ; ++k){
			if(j!=0||k!=0){
				//中点のベクトル
				Y1 = float(j)*kugiri/2;
				Z1 = float(k)*kugiri_z/2;
				
				//中点のベクトルのノルム
				Nolm1 = sqrt(pow(Y1,2)+pow(Z1,2));
				
				//中点から円の中心へのベクトルのノルム
				Nolm2 = sqrt(pow(Tire_R,2)-pow(Nolm1,2));
				
				//中点から円の中心へのベクトル
				Y2 = (-Z1/Nolm1)*Nolm2;
				Z2 = (Y1/Nolm1)*Nolm2;
				
				//円の中心のベクトル
				Y3 = Y1 + Y2;
				Z3 = Z1 + Z2;
				
				CircleCenter[count-1][0] = Y3;
				CircleCenter[count-1][1] = Z3;
					
				//円の中心から前接触点のベクトル
				Y4 = float(j)*kugiri-Y3;
				Z4 = float(k)*kugiri_z-Z3;
				
				//前接触点における進行方向向きのベクトル
				Y5 = -Z4;
				Z5 = Y4;
				
				if(Y5<=0){
					ForwardTheta[count-1][0] = 90;
				}else{
					ForwardTheta[count-1][0] = atan(Z5/Y5)*180/3.1415926;
				}
				if(Y3<0){
					ForwardTheta[count-1][1] = 180+atan(Z3/Y3)*180/3.1415926;
				}else{
					ForwardTheta[count-1][1] = atan(Z3/Y3)*180/3.1415926;
				}
				ForwardTheta[count-1][2] = j;
				ForwardTheta[count-1][3] = k;
				
				++count;
			}
		}
	}
	
	for(int i=0; i<CountMax; ++i){
		ForwardTheta[i].push_back(CircleCenter[i][0]);
		ForwardTheta[i].push_back(CircleCenter[i][1]);
	}
	
	//Bubble sort
	//std::sort(ForwardThetabegin(), ForwardTheta.end(), [](const std::vector< float >& a, const std::vector< float >& b){ return a[1] > b[1]; } );
        for (int data=CountMax; data>1; --data){
                for (int i=0; i<data-1; ++i){
                        if (ForwardTheta[i][1]<ForwardTheta[i+1][1]){
                        	swap(ForwardTheta[i],ForwardTheta[i+1]);
                        }
                }
        }
        
        CoroCoroDate1.resize(CountMax);
        CoroCoroDate2.resize(CountMax);
	for(int i=0; i<CountMax; ++i){
		CoroCoroDate1[i].resize(2,0);
		CoroCoroDate2[i].resize(5,0);
		CoroCoroDate1[i][0] = int(ForwardTheta[i][2]);  // Y方向相対座標
		CoroCoroDate1[i][1] = int(ForwardTheta[i][3]);  // Z方向相対座標
		CoroCoroDate2[i][0] = ForwardTheta[i][0];  // 前方接触角度
		CoroCoroDate2[i][1] = ForwardTheta[i][4];  // 相対中心座標(Y)
		CoroCoroDate2[i][2] = ForwardTheta[i][5];  // 相対中心座標(Z)
		CoroCoroDate2[i][3] = ForwardTheta[i][1];  // 原点-中心角度
	}
        
        float SerchTheta_Y, SerchTheta_Z;
	for(int i=0; i<CountMax; ++i){
		SerchTheta_Y = CoroCoroDate2[i][1] - float(CoroCoroDate1[i][0])*kugiri;
		SerchTheta_Z = CoroCoroDate2[i][2] - float(CoroCoroDate1[i][1])*kugiri_z;
		if (SerchTheta_Y<=0){
			if(SerchTheta_Y==0&&SerchTheta_Z>0){
				CoroCoroDate2[i][4] = 90;
			}else if(SerchTheta_Y==0&&SerchTheta_Z<0){
				CoroCoroDate2[i][4] = 270;
			}else{
				CoroCoroDate2[i][4] = 180+atan( SerchTheta_Z/SerchTheta_Y )*180/3.1415926;
			}
		}else{
		CoroCoroDate2[i][4] = atan( SerchTheta_Z/SerchTheta_Y )*180/3.1415926;
		}
	}
	CoroCoroDateNum = CountMax;
	
}

void Height_Map::RecordSensorData(sensor_msgs::PointCloud laser_point_data){

	int DATA_NUM = laser_point_data.points.size();
	
	int x,y,z;
	for(int i=0;i<DATA_NUM;++i){
		x = int((( laser_point_data.points[i].x - center_x ) - MeshMinX) / dx2 );
		if(x>=0 && x<NumMesh2X){
			y = int((( laser_point_data.points[i].y - center_y ) - MeshMinY) / dx2 );
			if( y>=0 && y<NumMesh2Y){
				if(laser_point_data.points[i].z>height_map[x][y]){
					height_map[x][y] = laser_point_data.points[i].z;
				}
			}
		}
	}
	
	//InterpolateMap();
        
	/*
	for(int i=0;i<DATA_NUM;++i){
		x = int(trunc((( laser_point_data.points[i].x - center_x ) - MeshMinX) / dx));
		if(x>=0 && x<NumMeshX ){
			y = int(trunc((( laser_point_data.points[i].y - center_y ) - MeshMinY) / dx));
			if( y>=0 && y<NumMeshY ){
				z = int(trunc((laser_point_data.points[i].z - MeshMinZ) / dz));
				if( z>=0 && z<NumMeshZ ){
					Mesh3D[x][y][z] = 1;
				}
			}
		}
	}
	*/
	
	return;
}


void Height_Map::heightmap2Mesh3D(){
	int z;
        for(int i=0; i<NumMesh2X; ++i){
        	for(int j=0; j<NumMesh2Y; ++j){
			if(height_map[i][j] != NOT_DETECT){
				z = int(trunc((height_map[i][j] - MeshMinZ) / dz));
				if( z>=0 && z<NumMeshZ ){
					Mesh3D[int(float(i)/div)][int(float(j)/div)][z] = 1;
				}
			}
        	}
        }
}


void Height_Map::InterpolateMap(){

	float height_sum = 0;
	float sum_num = 0;
	
	vector < vector <float> > interpolated_height_map;
	
	interpolated_height_map = height_map;
	
	for(int i=2; i<NumMesh2X-2; ++i){
		for(int j=2; j<NumMesh2X-2; ++j){
			if(height_map[i][j] == NOT_DETECT){
				for(int n=-1; n<=1; ++n){
					for(int m=-1; m<=1; ++m){
						if(height_map[i+n][j+m] != NOT_DETECT){
							++sum_num;
							height_sum += height_map[i+n][j+m];
						}
					}
				}
				if(sum_num>14){
					interpolated_height_map[i][j] = height_sum / sum_num;
				}
				height_sum = 0;
				sum_num = 0;
			}
		}
	}
	 height_map = interpolated_height_map;
}


void Height_Map::HeightMapToPointCloud(sensor_msgs::PointCloud *out){
	
	for(int i=0; i<NumMesh2X; ++i){
		for(int j=0; j<NumMesh2Y; ++j){
        		if( height_map[i][j] != NOT_DETECT){
				out->points.push_back(TranslateIndexToRealCordinate2(i,j));
			}
		}
	}
	
	/*
	for(int i=0; i<NumMeshX; ++i){
        	for(int j=0; j<NumMeshY; ++j){
        		if(go[j][i]!=1){
        			for(int k=0; k<NumMeshZ; ++k){
        				if( Mesh3D[i][j][k] == 1){
						out->points.push_back(TranslateIndexToRealCordinate3(i,j,k));
					}
				}
        		}
        	}
        }
        */
        
}

inline geometry_msgs::Point32 Height_Map::TranslateIndexToRealCordinate2(int x_index,int y_index){
	geometry_msgs::Point32 point;
	point.x = float(x_index)*dx2 + MeshMinX + center_x;
	point.y = float(y_index)*dx2 + MeshMinY + center_y;
	point.z = height_map[x_index][y_index];
	return point;
}

inline geometry_msgs::Point32 Height_Map::TranslateIndexToRealCordinate3(int x_index,int y_index,int z_index){
	geometry_msgs::Point32 point;
	point.x = float(x_index)*dx + MeshMinX + center_x;
	point.y = float(y_index)*dx + MeshMinY + center_y;
	point.z = float(z_index)*dz + MeshMinZ;
	return point;
}


void Height_Map::MoveHeightMapCenter(float pos_x, float pos_y){

	vector < vector <float> > tmp_map;
	tmp_map.resize(NumMesh2X);
	for(int i=0; i<NumMesh2X; ++i){
		tmp_map[i].resize(NumMesh2Y);
		for(int j=0; j<NumMesh2Y; ++j){
			tmp_map[i][j] = height_map[i][j];
			height_map[i][j] = NOT_DETECT;
		}
	}

	// 複製vectorを移動させる
	int new_x_index,new_y_index;
	for(int i=0; i<NumMesh2X; ++i){
		for(int j=0; j<NumMesh2Y; ++j){
			if(tmp_map[i][j] != NOT_DETECT){
				new_x_index = int(float(i) + (center_x - pos_x)/dx2);
				if( 0 <= new_x_index && new_x_index < NumMesh2X ){
					new_y_index = int(float(j) + (center_y - pos_y)/dx2);
					if( 0 <= new_y_index && new_y_index < NumMesh2Y ){
						height_map[new_x_index][new_y_index] = tmp_map[i][j];
					}
				}
			}	
		}
	}
	
        
	/*
	vector < vector < vector <int> > > tmp_Mesh3D;
	tmp_Mesh3D.resize(NumMeshX);
        for(int i=0; i<NumMeshX; ++i){
		tmp_Mesh3D[i].resize(NumMeshY);
        	for(int j=0; j<NumMeshY; ++j){
        		tmp_Mesh3D[i][j].resize(NumMeshZ);
        		for(int k=0; k<NumMeshZ; ++k){
        			if( Mesh3D[i][j][k] == 1){
					tmp_Mesh3D[i][j][k] = Mesh3D[i][j][k];
					Mesh3D[i][j][k] = 0;
				}
        		}
        	}
        }
        
	int new_x,new_y;
        // 複製vectorを移動させる
        for(int i=0; i<NumMeshX; ++i){
        	for(int j=0; j<NumMeshY; ++j){
        		for(int k=0; k<NumMeshZ; ++k){
        			if( tmp_Mesh3D[i][j][k] == 1){
        				new_x = int(trunc(float(i) + (center_x - pos_x) / dx));
					if(new_x>=0 && new_x<NumMeshX ){
						new_y = int(trunc(float(j) + (center_y - pos_y) / dx));
						if(new_y>=0 && new_y<NumMeshY ){
							Mesh3D[new_x][new_y][k] = tmp_Mesh3D[i][j][k];
						}
					}
				}
        		}
        	}
        }
        */
	
	center_x = pos_x;
	center_y = pos_y;
}


void  Height_Map::RotateEnableAreaToPointCloud(sensor_msgs::PointCloud *out){
	for(int i=0; i<NumMeshX; ++i){
        	for(int j=0; j<NumMeshY; ++j){
        		if(go[j][i]==1){
        			for(int k=0; k<NumMeshZ; ++k){
        				if( Mesh3D[i][j][k] == 1){
						out->points.push_back(TranslateIndexToRealCordinate3(i,j,k));
					}
				}
        		}
        	}
        }
}


void Height_Map::UpdateRotateEnableMap(){

        InitializeMesh3D();
	heightmap2Mesh3D();
        InitializeGridParams();
        
        InitializeMesh3DBuff();
	RoteMesh(0);
	CoroCoro(NumMeshX, NumMeshY, NumMeshZ, 0);
	
        InitializeMesh3DBuff();
	RoteMesh(1);
	CoroCoro(NumMeshY, NumMeshX, NumMeshZ, 1);
	
        InitializeMesh3DBuff();
	RoteMesh(2);
	CoroCoro(NumMeshX, NumMeshY, NumMeshZ, 2);
	
        InitializeMesh3DBuff();
	RoteMesh(3);
	CoroCoro(NumMeshY, NumMeshX, NumMeshZ, 3);
    						
	JudgeDirectionAll();
}


void Height_Map::InitializeGridParams(){
	for(int i=0; i<NumMeshY; ++i){
		for(int j=0; j<NumMeshX; ++j){
			CanGoY1[i][j] = 0;
			CanGoY2[i][j] = 0;
			CanGoX1[i][j] = 0;
			CanGoX2[i][j] = 0;
			PointZ_Y1[i][j] = 0;
			PointZ_Y2[i][j] = 0;
			PointZ_X1[i][j] = 0;
			PointZ_X2[i][j] = 0;
			CanGoDirectionAll[i][j] = 0;
			go[i][j] = 0;
		}
        }
}


void Height_Map::InitializeMesh3DBuff(){
	if(NumMeshX>NumMeshY){
		for(int i=0; i<NumMeshX; ++i){
        		for(int j=0; j<NumMeshX; ++j){
        			for(int k=0; k<NumMeshZ; ++k){
        				Mesh3D_Buff[i][j][k] = 0;
        			}
        		}
        	}
        }else{
		for(int i=0; i<NumMeshY; ++i){
        		for(int j=0; j<NumMeshY; ++j){
        			for(int k=0; k<NumMeshZ; ++k){
        				Mesh3D_Buff[i][j][k] = 0;
        			}
        		}
        	}
        }
}

void Height_Map::InitializeMesh3D(){

	for(int i=0; i<NumMeshX; ++i){
        	for(int j=0; j<NumMeshY; ++j){
        		for(int k=0; k<NumMeshZ; ++k){
       				Mesh3D[i][j][k] = 0; 			
        		}
        	}
        }
}

void Height_Map::RoteMesh(int mode){
	switch(mode){
		case 0: // なにもしない(Y1)
			for(int i=0; i<NumMeshX; ++i){
				for(int j=0; j<NumMeshY; ++j){
					for(int k=0; k<NumMeshZ; ++k){
						Mesh3D_Buff[i][j][k] = Mesh3D[i][j][k];
					}
				}
			}
			break;
			
		case 1: // 左に90度回転(X1)
			for(int i=0; i<NumMeshX; ++i){
				for(int j=0; j<NumMeshY; ++j){
					for(int k=0; k<NumMeshZ; ++k){
						Mesh3D_Buff[NumMeshY-1-j][i][k] = Mesh3D[i][j][k];
					}
				}
			}
			break;
			
		case 2: // 180度回転(Y2)
			for(int i=0; i<NumMeshX; ++i){
				for(int j=0; j<NumMeshY; ++j){
					for(int k=0; k<NumMeshZ; ++k){
						Mesh3D_Buff[NumMeshX-1-i][NumMeshY-1-j][k] = Mesh3D[i][j][k];
					}
				}
			}
			break;
			
		case 3: // 右に90度回転(X2)
			for(int i=0; i<NumMeshX; ++i){
				for(int j=0; j<NumMeshY; ++j){
					for(int k=0; k<NumMeshZ; ++k){
						Mesh3D_Buff[j][NumMeshX-1-i][k] = Mesh3D[i][j][k];
					}
				}
			}
			break;
	}
}

void Height_Map::CoroCoro(int i_Size, int j_Size, int k_Size, int mode){

	int i,j,k;
	float SerchTheta;
	int break_flag, count, Local_j, Local_k;
	
	i = 0;
	while(1){
		j = 0;
    		k = k_Size-1;
    		// 初期位置探索
 	   	while(1){
    			if(k==0){
    				++j;
    				k = k_Size-1;
    				if(j==j_Size-1){
    					break;
    				}
    			}
    			if(Mesh3D_Buff[i][j][k]==1){
    				break;
    			}else{
    				--k;
    			}
    		}
    		SerchTheta = 180;
    	
    		
    		// ころころループ
    		while(1){
    			break_flag = 0;
    			count = 0;
    			// 探索開始角度の導出
    			while(1){
    				if(SerchTheta>CoroCoroDate2[count+1][3]){
    					break;
    				}else{
    				++count;
    				}
    			}
    			SerchTheta = 270;
    			
    			while(1){
    				Local_j = CoroCoroDate1[count][0];
    				Local_k = CoroCoroDate1[count][1];
    					if(j+Local_j<j_Size){  // 横方向の上限判定
    						if((k+Local_k<k_Size)&&(k+Local_k>=0)){  // 縦方向の上限判定
    							if(Mesh3D_Buff[i][j+Local_j][k+Local_k]==1){  // 現在のメッシュが1の場合
    								if(CoroCoroDate2[count][0] < 50){
    									for(int s=0; s<Local_j; ++s){
    										switch(mode){
    											case 0:
    												CanGoY1[j+s][i] = 1;
    												PointZ_Y1[j+s][i] = k+1;
    												break;
    											case 1:
    												CanGoX1[NumMeshY-1-i][j+s] = 1;
    												PointZ_X1[NumMeshY-1-i][j+s] = k+1;
    												break;
    											case 2:
    												CanGoY2[NumMeshY-1-(j+s)][NumMeshX-1-i] = 1;
    												PointZ_Y2[NumMeshY-1-(j+s)][NumMeshX-1-i] = k+1;
    												break;
    											case 3:
    												CanGoX2[i][NumMeshX-1-(j+s)] = 1;
    												PointZ_X2[i][NumMeshX-1-(j+s)] = k+1;
    												break;
    										}
    									}
    								}
    								SerchTheta = CoroCoroDate2[count][4];
    								if((Local_j==0)&&(Local_k>0)){   // 上り段差の場合
    									SerchTheta = 270;
    									++j;
    									k = k_Size;
    									if(j>=j_Size-1){
    										break_flag = 1;
    										break;
    									}
    									while(1){
    										if(k==0){
    											++j;
    											k = k_Size;
    											if( j >= (j_Size-1) ){
    												break_flag = 1;
    												break;
    											}
    										}
    										if(Mesh3D_Buff[i][j][k] == 1){
    											Local_j = 0;
    											Local_k = 0;
    											break;
    										}else{
    											--k;
    										}
    									}
    								}
    								break;
    							}else{
    							++count;
    							if(count >= CoroCoroDateNum){
    								++j;
    								if( j >= (j_Size-1) ){
    									break_flag = 1;
    									break;
    								}
    								while(1){
    									if(k==0){
    										++j;
    										k = k_Size-1;
    										if( j >= (j_Size-1) ){
    											break_flag = 1;
    											break;
    										}
    									}
    								if(Mesh3D_Buff[i][j][k] == 1){
    									Local_j = 0;
    									Local_k = 0;
    									break;
    								}else{
    									--k;
    								}
    							}
    							break;
    							}
    						}
    					}else{
    						++j;
    						k = k_Size-1;
    						if( j >= (j_Size-1) ){
    							break_flag = 1;
    							break;
    						}
    						while(1){
    							if(k == 1){
    								++j;
    								k = k_Size-1;
    								if( j >= (j_Size-1) ){
    									break_flag = 1;
    									break;
    								}
    							}
    							if(Mesh3D_Buff[i][j][k] == 1){
    								Local_j = 0;
    								Local_k = 0;
    								break;
    							}else{
    								--k;
    							}
    						}
    					}
    				}else{
    					break_flag = 1;
    					break;
    				}
    			}
    			if(break_flag == 1){
    				break;
    			}
    			j += Local_j;
 			k += Local_k;
    		}
    		++i;
    		if( i == (i_Size-1) ){
    			break;
    		}
    	}

}


void Height_Map::JudgeDirectionAll(){
	
	int Y1, Y2, X1, X2, Z_Y1, Z_Y2, Z_X1, Z_X2;
	
	for(int i=1; i<(NumMeshY-1); ++i){
		for(int j=1; j<(NumMeshX-1); ++j){
			Y1 = CanGoY1[i][j];
			X1 = CanGoX1[i][j];
			Y2 = CanGoY2[i][j];
			X2 = CanGoX2[i][j];
			Z_Y1 = PointZ_Y1[i][j];
			Z_X1 = PointZ_X1[i][j];
			Z_Y2 = PointZ_Y2[i][j];
			Z_X2 = PointZ_X2[i][j];
			Z_X1 = Z_Y1 - Z_X1;
			Z_X2 = Z_Y1 - Z_X2;
			Z_Y2 = Z_Y1 - Z_Y2;
			if( Y1==1 && Y2==1 && X1==1 && X2==1 ){
				if( Z_X1>(-100/5) && Z_X1<(100/5) ){
					if( Z_X2>(-100/5) && Z_X2<(100/5) ){
						if( Z_Y2>(-100/5) && Z_Y2<(100/5) ){
							CanGoDirectionAll[i][j] = 1;
						}
					}
				}
			}
		}
	}
}

void Height_Map::DetectRotateEnableAreaFromPoint(float pos_x,float pos_y){

	int X, Y;
	
	X = int(trunc(( pos_x - center_x - MeshMinX) / dx));
	Y = int(trunc(( pos_y - center_y - MeshMinY) / dx));
	
	for(int i=0; i<NumMeshY; ++i){
		for(int j=0; j<NumMeshX; ++j){
			go[i][j] = 0;
		}
        }
        
	q_map(X, Y);
}

void Height_Map::q_map(int X, int Y){
	
	unsigned long q[NumMeshX*NumMeshY];
	int i,j;
	int head,tail;
	unsigned long X_grid, Y_grid;
	
	j = 0;
	if(CanGoDirectionAll[Y][X]==1){
		go[Y][X] = 1;
		q[j] = (X<<16) + Y;
		++j;
		//break;
	}
	
	head = 0;
	tail = j;
	
	while(head != tail){
		X_grid = q[head]>>16;
		Y_grid = q[head]&0x0000ffff;
		
		++head;
		
		if(X_grid < (NumMeshX-1)){
			if(go[Y_grid][X_grid+1] == 0){
				if(CanGoDirectionAll[Y_grid][X_grid+1] == 1){
					go[Y_grid][X_grid+1] = 1;
					q[tail] = ((X_grid+1)<<16) + Y_grid;
					++ tail;
				}else{
					go[Y_grid][X_grid+1] = 2;
				}
			}
		}
		
		if(Y_grid < (NumMeshY-1)){
			if(go[Y_grid+1][X_grid] == 0){
				if(CanGoDirectionAll[Y_grid+1][X_grid] == 1){
					go[Y_grid+1][X_grid] = 1;
					q[tail] = ((X_grid)<<16) + (Y_grid+1);
					++tail;
				}else{
					go[Y_grid+1][X_grid] = 2;
				}
			}
		}
		
		if(X_grid > 0){
			if(go[Y_grid][X_grid-1] == 0){
				if(CanGoDirectionAll[Y_grid][X_grid-1] == 1){
					go[Y_grid][X_grid-1] = 1;
					q[tail] = ((X_grid-1)<<16) + Y_grid;
					++tail;
				}else{
					go[Y_grid][X_grid-1] = 2;
				}
			}
		}
		
		if(Y_grid > 0){
			if(go[Y_grid-1][X_grid] == 0){
				if(CanGoDirectionAll[Y_grid-1][X_grid] == 1){
					go[Y_grid-1][X_grid] = 1;
					q[tail] = ((X_grid)<<16) + (Y_grid-1);
					++tail;
				}else{
					go[Y_grid-1][X_grid] = 2;
				}
			}
		}
	}
}
