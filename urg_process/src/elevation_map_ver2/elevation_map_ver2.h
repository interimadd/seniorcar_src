#ifndef __ELEVATION_MAP_VER2_H_INCLUDED__
#define __ELEVATION_MAP_VER2_H_INCLUDED__

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

const int RECOAR_DATA_UPDATE_STRATEGY = 0; //0:そのまま 1：低い方

const float NOT_DETECT = -100.0;

/*
	レーザ点群を記録するだけのシンプルな機能だけに絞ったEevationMap
*/
class ElevationMap{

	public:

		float center_x;
		float center_y;

		float last_calc_x;
		float last_calc_y;

		ElevationMap(float pos_x,float pos_y,int map_size_x,int map_size_y,float horizontal_resolution);

		/*
			【入力】LRFの点群情報をHeightMapに格納する
		*/
		void RecordSensorData(sensor_msgs::PointCloud laser_point_data);

		/*
			HeightMapの中心座標を移動させる,
		*/
		void MoveHeightMapCenter(float pos_x,float pos_y);

		/*
			マップの隙間を補完する
			空いているマスの周りのマスを参照し、その中央値をとる
		*/
		void InterpolateMap();

		/*
			実世界座標を配列の番号に変換
		*/
		void TranslateRealCordinateToIndex(int *return_index,float pose[2]);

		/*
			配列の番号のグリッドを実世界座標のポイントに変換
		*/
		geometry_msgs::Point32 TranslateIndexToRealCordinate(int x_index,int y_index);

		int MAP_SIZE_X;
		int MAP_SIZE_Y;
		float HORIZONTAL_RESOLUTION;
		vector < vector <float> > height_map;
		vector < vector <float> > interpolated_height_map;

		/*
			HeightMapをPointCloudの形で出力する
		*/
		void HeightMapToPointCloud(sensor_msgs::PointCloud *out);

		/*
			Elevation Map内のデータをプリント出力 デバッグ用
		*/
		void printElevationMapData();

};


#endif