#ifndef __PENETRATION_VOXEL_MAP_H_INCLUDED__
#define __PENETRATION_VOXEL_MAP_H_INCLUDED__

#include <stdint.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/types.h>
#include <dirent.h>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"

using namespace std;

/*
	 設定パラメータ値
*/
const float HORIZONTAL_RESOLUTION = 0.1;
const float VERTICAL_RESOLUTION   = 0.01;

const int MAP_SIZE_X_Y = 50; // centerから何マス広がっているか
const int MAP_SIZE_Z   = 50;

/*
	文字定義
*/
const int REFLECT = 0;
const int PASS = 1;

/*
	透過度を計算するためレーザ点群から各ボクセルの反射/通過を記録するVoxelMap
*/
class PenetrationVoxelMap{

	public:

		float center_x;
		float center_y;
		float center_z;

		float last_calc_x;
		float last_calc_y;

		PenetrationVoxelMap(float pos_x,float pos_y,float pos_z);

		/*
			【入力】LRFの点群情報をVoxelMapに格納する
		*/
		void RecordSensorData(geometry_msgs::Point LRFcoordinate,sensor_msgs::PointCloud laser_point_data);

		/*
			VoxelMapの中心座標を移動させる,
		*/
		void MoveVoxelMapCenter(float pos_x,float pos_y,float pos_z);

		/*
			実世界座標を配列の番号に変換
		*/
		void TranslateRealCordinateToIndex(int *return_index,float pose[3]);

		/*
			配列の番号のグリッドを実世界座標のポイントに変換
		*/
		geometry_msgs::Point32 TranslateIndexToRealCordinate(int x_index,int y_index,int z_index);


		vector < vector < vector < vector <int8_t> > > > voxel_map;	// [x_index][y_index][z_index][reflect/pass]
		vector < vector < vector < vector <int8_t> > > > voxel_map_for_copy;

		/*
			VoxelMapをPointCloudの形で出力する
		*/
		void VoxelMapToPointCloud(sensor_msgs::PointCloud *out);

		/*
			VoxelMap内のデータをプリント出力 デバッグ用
		*/
		void printVoxelMapData();

};


#endif