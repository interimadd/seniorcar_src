#ifndef __HEIGHT_MAP_H_INCLUDED__
#define __HEIGHT_MAP_H_INCLUDED__

#include <stdint.h>
#include <vector>
#include <stdio.h>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"
//#include <fstream>

using namespace std;

const float NOT_DETECT = -1000.0;

class Height_Map{

	public:
		float center_x;
		float center_y;
		
		//コンストラクタ
		Height_Map();
		
		//LRFの点群情報をHeightMapに格納する
		void RecordSensorData(sensor_msgs::PointCloud laser_point_data);
		
		//HeightMapの中心座標を移動させる,
		void MoveHeightMapCenter(float pos_x,float pos_y);
		
		//HeightMapをPointCloudの形で出力する
		void HeightMapToPointCloud(sensor_msgs::PointCloud *out);
		
		//転動可能な領域のマップを更新する
		void UpdateRotateEnableMap();

		//座標を指定して、そこから転動可能な領域を出力する
		void DetectRotateEnableAreaFromPoint(float pos_x,float pos_y);
		
		//転動可能な領域をPointCloudの形で出力する
		void RotateEnableAreaToPointCloud(sensor_msgs::PointCloud *out);

	private:
	
		// 設定パラメータ //
		float dx;    // 85X,Y方向の刻みを設定(XおよびYは共通の刻み幅)
		float dz;    // Z方向の刻みを設定
		float MeshMinX, MeshMaxX, MeshMinY, MeshMaxY, MeshMinZ, MeshMaxZ;
		int NumMeshX, NumMeshY,NumMeshZ;
		int CoroCoroDateNum;
		
		float div, dx2;
		int NumMesh2X, NumMesh2Y;
		
		vector < vector < vector <int> > > Mesh3D;
		vector < vector < vector <int> > > Mesh3D_Buff;
		
		vector < vector <int> > CanGoY1;
		vector < vector <int> > CanGoY2;
		vector < vector <int> > CanGoX1;
		vector < vector <int> > CanGoX2;
		vector < vector <int> > PointZ_Y1;
		vector < vector <int> > PointZ_Y2;
		vector < vector <int> > PointZ_X1;
		vector < vector <int> > PointZ_X2;
		vector < vector <int> > CanGoDirectionAll;
		vector < vector <int> > go;
		
		vector < vector <int> > CoroCoroDate1;
		vector < vector <float> > CoroCoroDate2;
		
		vector < vector <float> > height_map;
		
		void initializeStereoCalc(float Tire_R, float kugiri, float kugiri_z);
		void heightmap2Mesh3D();
		void InterpolateMap();
		void InitializeMesh3D();
		void InitializeGridParams();
		void InitializeMesh3DBuff();
		void RoteMesh(int mode);
		void CoroCoro(int i_Size, int j_Size, int k_Size, int mode);
		void JudgeDirectionAll();
		void q_map(int X_grid, int Y_grid);
		
		geometry_msgs::Point32 TranslateIndexToRealCordinate2(int x_index, int y_index);
		geometry_msgs::Point32 TranslateIndexToRealCordinate3(int x_index, int y_index, int z_index);
		
};


#endif
