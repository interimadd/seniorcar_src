# アクセル開度。単位は[]
# 取りうる値は0から127
int8 accel_opening

# 最高速度設定値。単位は[km/h]
# 取りうる値は2 km/hから6 km/h 
float32 max_velocity 

# 操舵角度。単位は[°]
# 取りうる値は-40°から40°
float64 steer_angle 

# 車両速度。単位は[m/s]
# 取りうる値は0 m/s〜
float64 vehicle_velocity

# 方向スイッチのオン/オフ
# 取りうる値は0か1
# 0のときに方向スイッチオフ（前進）、1のときに方向スイッチオン(後退)
int8 direction_switch 

# 各種スイッチのオン/オフ
# 上から、車両ランプ、左ウィンカー、右ウィンカー、警告音
int8 light_signal 
int8 left_winker
int8 right_winker
int8 hone_signal
