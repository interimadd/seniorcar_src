#ifndef __HEIGHT_MAP_H_INCLUDED__
#define __HEIGHT_MAP_H_INCLUDED__

#include <stdint.h>
#include <vector>
#include <stdio.h>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"

using namespace std;

const float NOT_DETECT = -100.0;
const float VEGETATION_THRESHOLD_MIN = 0.0005;
//const float VEGETATION_THRESHOLD_MIN = 0.0001;
const float VEGETATION_THRESHOLD_MAX = 10.01;

class ElevationMap{

	public:

		float center_x;
		float center_y;

		float last_calc_x;
		float last_calc_y;

		ElevationMap(float pos_x,float pos_y,int map_size_x,int map_size_y,float horizontal_resolution);

		/*
			LRFの点群情報をHeightMapに格納する
		*/
		void RecordSensorData(sensor_msgs::PointCloud laser_point_data);

		/*
			マップの隙間を補完する
			空いているマスの周りのマスを参照し、その中央値をとる
		*/
		void InterpolateMap();

		/*
			HeightMapの中心座標を移動させる,
		*/
		void MoveHeightMapCenter(float pos_x,float pos_y);

		/*
			HeightMapをPointCloudの形で出力する
		*/
		void HeightMapToPointCloud(sensor_msgs::PointCloud *out);
		void VarianceMapToPointCloud(sensor_msgs::PointCloud *out);
		void TypeMapToPointCloud(sensor_msgs::PointCloud *road,sensor_msgs::PointCloud *grass,sensor_msgs::PointCloud *others);

		/*
			実世界座標を配列の番号に変換する
		*/
		void TranslateRealCordinateToIndex(int *return_index,float pose[2]);

		int MAP_SIZE_X;
		int MAP_SIZE_Y;
		float HORIZONTAL_RESOLUTION;
		vector < vector <float> > height_map;
		vector < vector <float> > interpolated_height_map;
		vector < vector < vector <float> > > variance_map;  //(s^2,n,Σx_i^2,Σx_i)
		
		/*
			配列の番号から実世界座標に変換するのに使う
		*/
		geometry_msgs::Point32 TranslateIndexToRealCordinate(int x_index,int y_index);

		/*
			Elevation Map内のデータをプリント出力 デバッグ用
		*/
		void printElevationMapData();

};


#endif

/*

分散の更新に関しては以下のサイトを参照した
http://qpp.bitbucket.org/post/variance/
標本数が少ないと思われるので2番目の方法

*/