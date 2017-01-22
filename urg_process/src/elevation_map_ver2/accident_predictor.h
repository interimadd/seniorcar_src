#ifndef __ACCIDENT_PREDICTOR_H_INCLUDED__
#define __ACCIDENT_PREDICTOR_H_INCLUDED__

#include "elevation_map_ver2.h" 
#include "../parameter/seniorcar_param.h"
//#include "geometry_msgs/Pose.h"
//#include <tf/transform_listener.h>
#include "ultimate_seniorcar/SeniorcarState.h"
//#include <tf/transform_datatypes.h>
#include <math.h>
//#include <algorithm>
//#include "geometry_msgs/PoseArray.h"
#include <visualization_msgs/Marker.h>
#include <algorithm>


/*
	各種パラメータ
*/
const float CALCULATE_DISTANCE_STEP = 0.2;	// 何ｍ刻みで領域を評価するのか
const float CALCULATE_DISTANCE_LENGTH = 3.0;	// 何ｍ先までの領域を検出するのか

const float CALCULATE_STEER_DEG_STEP = 5.0;	// 操舵角度何度ごとに計算を行うか
const float MAX_STEER_DEG_CHANGE = 30.0;	// 今の操舵角度から何度変化するところまで計算するか

const float TIME_STEP_RESOLUTION = 0.1; // 矢印一個につき何秒刻みとするか

const int PATH_POINT_NUM =   int( CALCULATE_DISTANCE_LENGTH / CALCULATE_DISTANCE_STEP ) + 1;	// 1つの経路を何個の点で表現するか
const int DEG_CALCULTE_NUM = int( MAX_STEER_DEG_CHANGE * 2.0 / CALCULATE_STEER_DEG_STEP ) + 1;	// 何個の経路を生成するか

const float SENIORCAR_DRIVABLE_PITCH_ANGLE_THRESHOLD = 10.0 * 3.14 / 180;
const float SENIORCAR_DRIVABLE_ROLL_ANGLE_THRESHOLD = 10.0 * 3.14 / 180;
const float DANGER_Y_ZMP = SENIORCAR_HARF_TREAD_LENGTH*2 - 0.1; // ここまでZMPが来るとまずい閾値

const float MIN_VHEICLE_VELOCITY = 1.0;	//考慮する車両の最低速度

const float RAD_TO_DEG = 180.0 / M_PI;
const float NOT_SET = -100;

/*
	タイヤの位置を格納するための型
*/
typedef struct{
	float vehicle_pose[3];			// 車両原点位置位置(x,y,th)
	float tire_pos[4][4];			// 車輪位置座標((x,y,z,th)*4)
	float calculated_roll_angle[3];	// 車輪位置から計算された車両の傾き（基本三点接地なので2通りある）
	float calculated_pitch_angle[3];
	bool  is_fall;
	bool  is_collision;
	bool  is_rollover;
} CalculatedVehicleState;

typedef struct{
	float steer_angle[DEG_CALCULTE_NUM];
	float max_distance_to_go[DEG_CALCULTE_NUM];
} PredictResult;

enum ObjectMapStatus{
  UNKNOWN = 0,
  OBJECT  = 1,
  ROAD    = 2,
};

enum TireIndexNumber{
	FRONT_LEFT  = 0,
	FRONT_RIGHT = 1,
	BACK_LEFT	= 2,
	BACK_RIGHT	= 3,
};

/*
	セニアカーの転倒転落リスクをHeightMapを元に算出するためのクラス
*/
class AccidentPredictor : public ElevationMap
{
	public:

		AccidentPredictor(float pos_x,float pos_y,int map_size_x,int map_size_y,float horizontal_resolution);

		/*
			領域を走行可能か走行不可農家に識別する
			一旦全部不可能として根本から計算していく感じ？？
		*/
		void predictAccident(float pos_x,float pos_y,float pos_z,float roll,float pitch,float yaw,float vehicle_velocity);

		/*
			外部プログラムから結果を参照する用
		*/
		PredictResult returnPredictResult();

		/*
			可視化関連
		*/
		void returnObjectMapPointsMarker(visualization_msgs::Marker *points);
		void returnCalculatedVehicleState(visualization_msgs::Marker *triangles);

	private:

		vector < vector < vector <float> > > predicted_path; // 走行領域 predict_path[DEG_CALCULATE_NUM][PATH_POINT_NUM][3]
		vector < CalculatedVehicleState > calculated_state_array;
		float now_pos_z;

		vector<float> tire_height;
		int tire_radius_in_grid;
		int half_tire_width_in_grid;

		vector < vector <float> > tire_calculation_point; // [x0,y0,z0],[x1,y1,z1],[x2,y2,z2]...

		float C_f,C_b,C_a;	// ロール角計算用 φ= C_f * ⊿h_f + C_b * ⊿h_b + C_a * a_y
		
		void generatePath(float pos_x,float pos_y,float yaw,float vehicle_velocity); // 予想経路更新
		void returnTirePositionAtGivenPose(CalculatedVehicleState *return_tire_pos,vector<float> pose);
		void calculateSlopeOfVehicle(CalculatedVehicleState *predicted_state);
		float calculateRollAngleFrom2Vectors(float vec1[3],float vec2[3]);
		float calculatePitchAngleFrom2Vectors(float vec1[3],float vec2[3]);

		/*
			vehicle stateにおけるy方向の最大のZMP位置の絶対値を返す
		*/
		float calculateZMP(CalculatedVehicleState vehicle_state,float vehicle_velocity,float steer_angle_deg);

		/*
			四輪の中で閾値以上に高さが離れている車輪があれば転落判定を返す
		*/
		bool isFall(CalculatedVehicleState vehicle_state);

		/*
			障害物との衝突判定
		*/
		vector < vector <ObjectMapStatus> > object_map;
		void FindObjectFromMap();
		vector < vector <int> > collision_index;
		void setCollisionIndex(float yaw);
		bool isCollision(CalculatedVehicleState vehicle_state);
		bool isCollisionByTirePos(CalculatedVehicleState state_now,CalculatedVehicleState state_old);

		bool canDrive(float pitch_angle,float roll_angle,float y_zmp);
		float returnTireHeightInGrid(int x_index,int y_index);
		double returnTireHeightAtGivenPositionAndPose(double x_pos,double y_pos,double tire_theta);

		/*
			テスト用
		*/
		void printCalculatedState(CalculatedVehicleState state);
		void printPredictedFrontLeftTireHeight();
		void printPredictedVehicleAngle();

};


#endif