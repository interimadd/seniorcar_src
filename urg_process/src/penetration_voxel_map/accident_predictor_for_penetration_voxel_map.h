#ifndef __ACCIDENT_PREDICTOR_H_INCLUDED__
#define __ACCIDENT_PREDICTOR_H_INCLUDED__

#include "penetration_voxel_map.h" 
#include "../parameter/seniorcar_param.h"
//#include "geometry_msgs/Pose.h"
//#include <tf/transform_listener.h>
#include "ultimate_seniorcar/SeniorcarState.h"
//#include <tf/transform_datatypes.h>
#include <math.h>
//#include <algorithm>
//#include "geometry_msgs/PoseArray.h"
#include <visualization_msgs/Marker.h>
#include <stdlib.h> // rand, srand関数
#include <time.h>   // time関数


/*
	各種パラメータ
*/
const float CALCULATE_DISTANCE_STEP = 0.2;	// 何ｍ刻みで領域を評価するのか
const float CALCULATE_DISTANCE_LENGTH = 3.6;	// 何ｍ先までの領域を検出するのか

const float CALCULATE_STEER_DEG_STEP = 5.0;	// 操舵角度何度ごとに計算を行うか
const float MAX_STEER_DEG_CHANGE = 30.0;	// 今の操舵角度から何度変化するところまで計算するか

const float TIME_STEP_RESOLUTION = 0.25; // 矢印一個につき何秒刻みとするか

const int PATH_POINT_NUM =   int( CALCULATE_DISTANCE_LENGTH / CALCULATE_DISTANCE_STEP ) + 1;	// 1つの経路を何個の点で表現するか
const int DEG_CALCULTE_NUM = int( MAX_STEER_DEG_CHANGE * 2.0 / CALCULATE_STEER_DEG_STEP ) + 1;	// 何個の経路を生成するか

const int CALC_LOOP_NUM = 1; // 1つの予測通過点に対し何回計算するか

const float SENIORCAR_DRIVABLE_PITCH_ANGLE_THRESHOLD = 10.0 * 3.14 / 180;
const float DANGER_ANGLE = 0.25;  // 走れない角度
const float DANGER_Y_ZMP = SENIORCAR_HARF_TREAD_LENGTH*2 - 0.1; // ここまでZMPが来るとまずい閾値

const float MIN_VHEICLE_VELOCITY = 0.5;	//考慮する車両の最低速度

const float RAD_TO_DEG = 180.0 / M_PI;
const float NOT_SET = -100;

/*
	タイヤの位置を格納するための型
*/
typedef struct{
	float vehicle_pose[3];			// 車両原点位置位置(x,y,th)
	float tire_pos[4][4];			// 車輪位置座標((x,y,z,th)*4)
	float tire_height_probability_density[4][MAP_SIZE_Z*2];   // 車輪高さを確率密度分布で表したいときのやつ
	float calculated_roll_angle;	// 車輪位置から計算された車両の傾き
	float calculated_pitch_angle;
	bool  is_fall;
	bool  is_collision;
	bool  is_rollover;
	float accident_rate;
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
class AccidentPredictor : public PenetrationVoxelMap
{
	public:

		AccidentPredictor(float pos_x,float pos_y,float pos_z);

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
		void returnCalculatedVehicleState(visualization_msgs::Marker *triangles);

	private:

		vector < vector < vector <float> > > predicted_path; // 走行領域 predict_path[DEG_CALCULATE_NUM][PATH_POINT_NUM][3]
		vector < CalculatedVehicleState > calculated_state_array;
		float now_pos_z;

		vector < vector <float> > tire_calculation_point; // [x0,y0,z0],[x1,y1,z1],[x2,y2,z2]...
		vector < vector <int> >   tire_calculation_index; // [x0,y0,z0],[x1,y1,z1],[x2,y2,z2]...

		float C_f,C_b,C_a;	// ロール角計算用 φ= C_f * ⊿h_f + C_b * ⊿h_b + C_a * a_y
		
		void generatePath(float pos_x,float pos_y,float yaw,float vehicle_velocity); // 予想経路更新
		void returnTirePositionAtGivenPose(CalculatedVehicleState *return_tire_pos,vector<float> pose);
		void calculateSlopeOfVehicle(CalculatedVehicleState *predicted_state);

		/*
			vehicle stateにおけるy方向の最大のZMP位置の絶対値を返す
		*/
		float calculateZMP(CalculatedVehicleState vehicle_state,float vehicle_velocity,float steer_angle_deg);

		/*
			四輪の中で閾値以上に高さが離れている車輪があれば転落判定を返す
		*/
		bool isFall(CalculatedVehicleState vehicle_state);

		/*
			前輪の接地高さが前のステップから段差高さ以上に高くなっていると走行できない判定
		*/
		bool isCollisionByTirePos(CalculatedVehicleState state_now,CalculatedVehicleState state_old);

		double returnTireHeightAtGivenPositionAndPose(double x_pos,double y_pos,double tire_theta);

		// 車輪の高さを確率密度関数で計算するやつ
		void calculateTireHeightProbabilityDensity(CalculatedVehicleState *return_tire_height);
		// 車輪の高さを確率密度関数を基にランダムサンプリングしてtire_pos[Z]に代入する
		void randomSamplingTireHeighrByPropanilityDensity(CalculatedVehicleState *return_tire_height);
		// 車輪の高さを確率密度関数の中で最も確率が高い高さをtire_pos[Z]に代入する
		void mostLikelyTireHeightByPropanilityDensity(CalculatedVehicleState *return_tire_height);

		/*
			テスト用
		*/
		void printCalculatedState(CalculatedVehicleState state);
		void printTireHeightPropabilityDensity();
		void printFrontLeftTirePropabilityDensity();
		void printPredictedVehicleAngle();

};


#endif