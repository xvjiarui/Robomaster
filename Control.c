typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float target;
	float current;
	float last_err;
	float output;
	float p;
	float i;
	float d;
	float limit;
}PID;
void PID_init(PID* pid, float Kp_val, float Ki_val, float Kd_val, float limit){
	pid->Kp=Kp_val;
	pid->Ki=Ki_val;
	pid->Kd=Kd_val;
	pid->limit=limit;
	pid->target=0;
	pid->current=0;
	pid->last_err=0;
	pid->p=0;
	pid->i=0;
	pid->d=0;
}
float PID_output(PID* pid, float target_val){
	pid->target=target_val;
	pid->p=pid->target - pid->current;
	pid->i+=pid->p;
	pid->d=pid->p - pid->last_err;
	pid->last_err=pid->p;
	pid->output=pid->Kp * pid->p + pid->Ki * pid->i + pid->Kd * pid->d;

	return pid->output>pid->limit? pid->limit:pid->output;
}
float PID_output2(PID* pid, float target_val, float umax, float umin, float emax, float emin){
	float index=1;
	pid->target=target_val;
	pid->p=pid->target - pid->current;
	if(pid->current>umax){
		if(abs_s(pid->p)>emax){
			index=0;
		}
		else if(abs_s(pid->p)<emin){
			index=1;
			pid->i += pid->p < 0? pid->p:0;
		}
		else{
			index=(emax-abs_s(pid->p))/(emax-emin);
			pid->i += pid->p < 0? pid->p:0;
		}
	}
	else if(pid->current<umin){
		if(abs_s(pid->p)>emax){
			index=0;
		}
		else if(abs_s(pid->p)<emin){
			index=1;
			pid->i += pid->p > 0? pid->p:0;
		}
		else{
			index=(emax-abs_s(pid->p))/(emax-emin);
			pid->i += pid->p > 0? pid->p:0;
		}
	}
	else {
		if(abs_s(pid->p)>emax){
			index=0;
		}
		else if(abs_s(pid->p)<emin){
			index=1;
			pid->i += pid->p;
		}
		else{
			index=(emax-abs_s(pid->p))/(emax-emin);
			pid->i += pid->p;
		}
	}
	pid->d=pid->p - pid->last_err;
	pid->last_err=pid->p;
	pid->output=pid->Kp * pid->p + index * pid->Ki * pid->i + pid->Kd * pid->d;
	pid->output = pid->output>pid->limit? pid->limit:pid->output;
	return pid->output;
}
s32 target_angle=0;
s32 current_angle=0;
int16_t* control_remoter(int16_t ch0, int16_t ch1, int16_t ch2, float ratio0,float ratio1, float ratio2, int16_t delta){
	int16_t result[4]={0,0,0,0};
	if (ch2<10 && ch2>-10)
	{
		ch2=0;
	}
	target_angle += ch2/200;
	ch2 += delta;
	ch0 *= ratio0;
	ch1 *= ratio1;
	ch2 *= ratio2;
	result[0]=ch1+ch0+ch2;
	result[1]=-(ch1-ch0-ch2);
	result[2]=-(ch1+ch0-ch2);
	result[3]=ch1-ch0+ch2;
	return result;
}
void control_car(int16_t ch0, int16_t ch1, int16_t ch2){
	
	
	current_angle = output_angle;
	angle_pid.current=current_angle;

	
	
	ang_vel_pid.current=curr_ang_vel;

	
	
	angle_pid.current=InfantryJudge.RealVoltage * InfantryJudge.RealCurrent;
	int16_t* target_speed=control_remoter(ch0,ch1,ch2,1,1,0.01,0);
	//int16_t* target_speed=control_remoter(ch0,ch1,ch2,1,1,0.01,PID_output(angle_pid,target_angle)/50);
	//int16_t* target_speed=control_remoter(ch0,ch1,ch2,1,1,0.01,PID_output(ang_vel_pid, PID_output(angle_pid,target_angle)*0.1))
	
	
	wheels_pid[0].current=CM1Encoder.filter_rate;
	wheels_pid[1].current=CM2Encoder.filter_rate;
	wheels_pid[2].current=CM3Encoder.filter_rate;
	wheels_pid[3].current=CM4Encoder.filter_rate;
	int16_t input[4];
	for (int i = 0; i < 4; i++){	
		input[i]=PID_output(&wheels_pid[i],target_speed[i]);
		log_dug=input[i];
		//input[i] *= PID_output(&power_pid, 80)/80;
	}

	Set_CM_Speed(CAN2, input[0],input[1], input[2], input[3]);
}