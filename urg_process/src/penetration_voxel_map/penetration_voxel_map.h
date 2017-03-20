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

const int MAP_SIZE_X_Y = 100; // centerから何マス広がっているか
const int MAP_SIZE_Z   = 100;


const float P_R_Z = 0.09;		// 反射したという情報が返ってきた時にそのグリッドが路面である確率 タイヤ高さ推定0.06、走行可能領域検出では0.09くらいがなんかよさそう
const float P_R_notZ = 0.0025;		// 通過したという情報が返ってきた時にそのグリッドが路面である確率
const float P_R_DEFAULT = 0.05;	// 何も情報が得られていないときにそのグリッドが路面である確率


//(0.2,0.01,0.1)
//(0.055,0.005,0.05)
//(0.0075,0.00015,0.001)
//(0.0065,0.00005,0.001)
//(0.0055,0.00000001,0.001)
//(0.09,0.0023,0.05)

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

		float center_x,center_y,center_z;

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
		void TranslateRealCordinateToIndex(int *return_index,float position_x,float position_y,float position_z);
		bool isInMap(int x_index,int y_index,int z_index);

		/*
			配列の番号のグリッドを実世界座標のポイントに変換
		*/
		geometry_msgs::Point32 TranslateIndexToRealCordinate(int x_index,int y_index,int z_index);

		vector < vector < vector < vector <int> > > > voxel_map;	// [x_index][y_index][z_index][reflect/pass]
		vector < vector < vector < vector <int> > > > voxel_map_for_copy;

		vector < vector < vector <float> > > voxel_odds_map;	// [x_index][y_index][z_index]
		vector < vector < vector <float> > > voxel_odds_map_for_copy;
		float l_0,l_occ,l_free;
		float translatePtoLogOdds(float p);
		float translateLogOddstoP(float log_odds);

		double returnPenetrationRate(int x_index,int y_index,int z_index);
		double returnReflectionRate(int x_index,int y_index,int z_index);
		int returnMeasuredTimes(int x_index,int y_index,int z_index);

		/*
			VoxelMapをPointCloudの形で出力する
		*/
		void VoxelMapToPointCloud(sensor_msgs::PointCloud *out);

		/*
			VoxelMap内のデータをプリント出力 デバッグ用
		*/
		void printVoxelMapData();

		double returnHighestVoxeclHeightInCordinate(float position_x,float position_y);

};


#endif