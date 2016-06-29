#ifndef __RISK_CALCULATER_H_INCLUDED__
#define __RISK_CALCULATER_H_INCLUDED__

#include "height_map.h" 
#include "seniorcar_param.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_listener.h>
#include "ultimate_seniorcar/SeniorcarState.h"
#include <tf/transform_datatypes.h>
#include <math.h>

/*
	各種パラメータ
*/
const float CALCULATE_TIME_STEP = 0.1;	// 何秒刻みの評価をするのか
const float CALCULATE_TIME_LENGTH = 3;	// 何秒後までの未来を評価するのか
const float CALCULATE_STEER_DEG_STEP = 4.0;	// 操舵角度何度ごとに計算を行うか
const float MAX_STEER_DEG_CHANGE = 20.0;		// 今の操舵角度から何度変化するところまで計算するか

const int PATH_POINT_NUM =   int( CALCULATE_TIME_LENGTH / CALCULATE_TIME_STEP ) + 1;	// 1つの経路を何個の点で表現するか
const int DEG_CALCULTE_NUM = int( MAX_STEER_DEG_CHANGE * 2.0 / CALCULATE_STEER_DEG_STEP ) + 1;	// 何個の経路を生成するか

const float DENGER_ANGLE = 0.4;  // 走れない角度
const float DENGER_Y_ZMP = SENIORCAR_HARF_TREAD_LENGTH*2 - 0.1; // ここまでZMPが来るとまずい閾値

const float MIN_VHEICLE_VELOCITY = 0.55;	//考慮する車両の最低速度

const float RAD_TO_DEG = 180.0 / M_PI;

/*
	タイヤの位置を格納するための型
*/
typedef struct{
	float vehicle_pose[3];			// 車両原点位置位置(x,y,th)
	float tire_pos[4][3];			// 車輪位置座標((x,y,z)*4)
	float calculated_roll_angle[2];	// 車輪位置から計算された車両の傾き（基本三点接地なので2通りある）
	float calculated_pitch_angle[2];
} CalculatedVehicleState;

/*
	セニアカーの転倒転落リスクをHeightMapを元に算出するためのクラス
*/
class RiskCalculater : public Height_Map
{
	public:

		RiskCalculater(float pos_x,float pos_y,int map_size_x,int map_size_y,float horizontal_resolution);
		float calculateRisk(geometry_msgs::Pose now_pos, ultimate_seniorcar::SeniorcarState now_state);

	private:

		vector < vector < vector <float> > > predicted_path; // 予想経路 predict_path[DEG_CALCULATE_NUM][PATH_POINT_NUM][3]
		void generatePath(geometry_msgs::Pose now_pos, ultimate_seniorcar::SeniorcarState now_state); // 予想経路更新
		void returnTirePositionAtGivenPose(CalculatedVehicleState *return_tire_pos,vector<float> pose);
		void calculateSlopeOfVehicle(CalculatedVehicleState *predicted_state);
		float calculateRollAngleFrom2Vectors(float vec1[3],float vec2[3]);
		float calculatePitchAngleFrom2Vectors(float vec1[3],float vec2[3]);
		float calculateZMP(float roll_angle,float vehicle_velocity,float steer_angle_deg);
		bool canDrive(float pitch_angle,float roll_angle,float y_zmp);

};


#endif