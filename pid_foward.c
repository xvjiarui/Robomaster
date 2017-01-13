typedef struct{
	int32_t raw_value;   									//±àÂëÆ÷²»¾­´¦ÀíµÄÔ­Ê¼Öµ
	int32_t last_raw_value;								//ÉÏÒ»´ÎµÄ±àÂëÆ÷Ô­Ê¼Öµ
	int32_t ecd_value;                       //¾­¹ý´¦ÀíºóÁ¬ÐøµÄ±àÂëÆ÷Öµ
	int32_t diff;													//Á½´Î±àÂëÆ÷Ö®¼äµÄ²îÖµ
	int32_t temp_count;                   //¼ÆÊýÓÃ
	uint8_t buf_count;								//ÂË²¨¸üÐÂbufÓÃ
	int32_t ecd_bias;											//³õÊ¼±àÂëÆ÷Öµ	
	int32_t ecd_raw_rate;									//Í¨¹ý±àÂëÆ÷¼ÆËãµÃµ½µÄËÙ¶ÈÔ­Ê¼Öµ
	int32_t rate_buf[RATE_BUF_SIZE];	//buf£¬for filter
	int32_t round_cnt;										//È¦Êý
	int32_t filter_rate;											//ËÙ¶È
	float ecd_angle;											//½Ç¶È
}Encoder;
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq,int16_t gimbal_x_iq);
void yaw_axis_pid_cal(s32 target_angle, s32 current_angle){
	
	float error = target_angle - current_angle;
	
	float Kout = error * yaw_Kp;
	
	yaw_temp_integral += error;
	
	float Iout = yaw_temp_integral * yaw_Ki;
	
	yaw_temp_derivative = error - yaw_pre_error;
	
	float Dout = yaw_temp_derivative * yaw_Kd;
	
	yaw_pid_output_angle = Dout + Iout + Kout; 

}
s16 angle_vel_pid_p=0;
s16 angle_vel_pid_i=0;
s16 angle_vel_pid_d=0;
s16 angle_vel_last_error=0;
bool angle_vel_first_time=true;
void control_angule_speed(s16 targ_angle_vel,s16 curr_angle_vel){
	s16 result=0;
	s16 angle_speed_Kp=1;
	s16 angle_speed_Ki=1;
	s16 angle_speed_Kd=1;
	angle_vel_pid_p=targ_angle_vel - curr_angle_vel;
	angle_vel_pid_i=angle_vel_pid_i + angle_vel_pid_p;
	angle_vel_pid_d=angle_vel_pid_p - angle_vel_last_error;
	if (angle_vel_first_time){
		result=angle_speed_Kp*angle_vel_pid_p + angle_speed_Ki*angle_vel_pid_i;
		angle_vel_first_time=false;
	}
	else result=angle_speed_Kp*angle_vel_pid_p + angle_speed_Ki*angle_vel_pid_i + angle_speed_Kd*angle_vel_pid_d;
	angle_vel_last_error=angle_vel_pid_p;
}
s32 target_angle=0;
s32 current_angle=0;
int16_t* control_remoter(int16_t ch0, int16_t ch1, int16_t ch2){
	ch2=ch2/5;
	int16_t result[4]={0,0,0,0};
	int16_t angle_to_ch=50;
	int16_t pid_to_ch=15;
	if (ch2<10 && ch2>-10)
	{
		ch2=0;
	}
	target_angle = target_angle +ch2/angle_to_ch;
	ch2=ch2+yaw_pid_output_angle/pid_to_ch;
	result[0]=ch1+ch0+ch2;
	result[1]=-(ch1-ch0-ch2);
	result[2]=-(ch1+ch0-ch2);
	result[3]=ch1-ch0+ch2;
	return result;
}
float power_pid_p=0;
float power_pid_i=0;
float power_pid_d=0;
float power_last_err=0;
bool power_pid_first_time=true;
int16_t control_power(float cur_v, float cur_i){
	int16_t result=0;
	float power_Kp=1;
	float power_Ki=1;
	float power_Kd=1;
	float limit_power=80;
	float cur_power=cur_i*cur_v;
	power_pid_p=limit_power - cur_power;
	power_pid_i=power_pid_i + power_pid_p;
	power_pid_d=power_pid_p - power_last_err;
	if(power_pid_first_time){
		result=power_Kp*power_pid_p + power_Ki*power_pid_i;
		power_pid_first_time=false;
	}
	else result=power_Kp*power_pid_p + power_Ki*power_pid_i + power_Kd*power_pid_d;
	power_last_err=power_pid_p;
	return result;
}
bool first_time=true;
int16_t pid_p[4]={0,0,0,0};
int16_t pid_i[4]={0,0,0,0};
int16_t last_err[4]={0,0,0,0};
int16_t pid_d[4]={0,0,0,0};
void control_car(int16_t ch0, int16_t ch1, int16_t ch2){
	int16_t* target_speed=control_remoter(ch0,ch1,ch2);
	int16_t power_change=control_power(InfantryJudge.RealVoltage, InfantryJudge.RealCurrent);
	int16_t power_ratio=10;
	current_angle = output_angle;
	yaw_axis_pid_cal(target_angle,current_angle);
	int16_t Kp=50;
	int16_t Ki=5;
	int16_t Kd=3;
	int16_t current_speed[4];
	current_speed[0]=CM1Encoder.filter_rate;
	current_speed[1]=CM2Encoder.filter_rate;
	current_speed[2]=CM3Encoder.filter_rate;
	current_speed[3]=CM4Encoder.filter_rate;
	for(int i=0;i<4;i++){
		pid_p[i]=target_speed[i]-current_speed[i];
		pid_i[i]+=pid_p[i];
		pid_d[i]=pid_p[i]-last_err[i];
	}
	int16_t input[4];
	for (int i = 0; i < 4; i++){	
		if(first_time){
			input[i]=Kp*pid_p[i]+pid_i[i]/Ki;
			first_time = false;
		}
		else input[i]=Kp*pid_p[i]+pid_i[i]/Ki+Kd*pid_d[i];
		input[i]=input[i]>25000? 25000: input[i];
		//input[i]=input[i]+ power_change*power_ratio;
	}
	for (int i = 0; i < 4; i++)
	{
		last_err[i]=pid_p[i];
	}
	Set_CM_Speed(CAN2, input[0],input[1], input[2], input[3]);
}

