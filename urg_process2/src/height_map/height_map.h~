#ifndef __HEIGHT_MAP_H_INCLUDED__
#define __HEIGHT_MAP_H_INCLUDED__

#include <stdint.h>
#include <vector>
#include <stdio.h>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"

using namespace std;

const float NOT_DETECT = -100.0;

struct BufferPoint {
  int sx; /* 領域右端のX座標 */
  int sy; /* 領域のY座標 */
};

enum RotateEnable{
	DISABLE  = 0,
	ENABLE   = 1,
	UNKNOWN  = 2,
	DRIVABLE = 3, //  ENABLEの中でも連続している領域
};

class Height_Map{

	public:

		float center_x;
		float center_y;

		float last_calc_x;
		float last_calc_y;

		Height_Map(float pos_x,float pos_y,int map_size_x,int map_size_y,float horizontal_resolution);

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

		/*
			タイヤの大きさを指定して、転動の可否を判断するパラメータを計算
		*/
		void SetTireRadius(float tire_radius,float tire_width);

		/*
			転動可能な領域をPointCloudの形で出力する
		*/
		void RotateEnableAreaToPointCloud(sensor_msgs::PointCloud *out);

		/*
			転動可能な領域のマップを更新する
		*/
		void UpdateRotateEnableMap();

		/*
			座標を指定して、そこから転動可能な領域を出力する
		*/
		void DetectRotateEnableAreaFromPoint(float pos_x,float pos_y);


	private:

		int MAP_SIZE_X;
		int MAP_SIZE_Y;
		float HORIZONTAL_RESOLUTION;
		vector < vector <float> > height_map;
		vector < vector <float> > interpolated_height_map;
		vector < vector <int8_t> > rotate_enable_map;

		/*
			配列の番号から実世界座標に変換するのに使う
		*/
		geometry_msgs::Point32 TranslateIndexToRealCordinate(int x_index,int y_index);

		/*
			あるグリッド上でx方向,y方向にそれぞれ転動可能かを識別する
			tire_height:車輪中心を(0,0)としたとき、車輪の外周はグリッドの中で何マス目にあるのかを計算しておく
			enable_rolling_threshold:車輪のボクセルとの接触角度が60°以上で転動できないため、60°がグリッドになったタイヤのどこの座標に相等するかを計算しておく
		*/
		RotateEnable isEnableRolling(int x_index,int y_index);
		vector<float> tire_height;
		int tire_radius_in_grid;
		int half_tire_width_in_grid;
		int enable_rolling_threshold;

		/*
			scanLine : 線分からシードを探索してバッファに登録する
			int lx, rx : 線分のX座標の範囲
			int y : 線分のY座標
			unsigned int col : 領域色
		*/
		void ScanLine( int lx, int rx, int y);
		BufferPoint *buff; // シード登録用バッファ
		BufferPoint *sIdx, *eIdx;  // buffの先頭・末尾ポインタ

};


#endif