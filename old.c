#include "main.h"
#include "function_list.h"
float abs_s(float a){
	if (a<0)
	{
		return -a;
	}
	return a;
}
static u32 ticks_msimg = (u32)-1;
float log_dug=0;
void init(){
	SysTick_Init();  
	Dbus_init();//usart1
	//buzzer_init();	 //initialization of buzzer
	tft_init(2,BLACK,GREEN,GREEN);
	LED_master_init();
	gyro_init();
	ADC1_init();
	judging_system_init(); //usart3
	gyro_init();
	pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();
	Quad_Encoder_Configuration();
	Encoder_Start1();
	Encoder_Start2();
	Friction_wheel_init();
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	DataMonitor_Init();
}
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
	// if(pid->output - pid->limit > 0){
	// 	pid->output = pid->limit;
	// }
	// else if(pid->output + pid->limit < 0){
	// 	pid->output = -(pid->limit);
	// }
	pid->output = pid->output > pid->limit? pid->limit : pid->output;
	pid->output = pid->output < (-pid->limit)? (-pid->limit) : pid->output;
	return pid->output;
}
float PID_output2(PID* pid, float target_val, float umax, float umin, float emax, float emin){
	float index=1;
	pid->target=target_val;
	pid->p=pid->target - pid->current;
	if(pid->last_err>umax){
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
	else if(pid->last_err<umin){
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
	pid->output = pid->output<(-pid->limit)? -pid->limit:pid->output;
	return pid->output;
}
s32 target_angle=0;
s32 current_angle=0;
int16_t* control_remoter(int16_t ch0, int16_t ch1, int16_t ch2, float ratio0,float ratio1, float ratio2, int16_t delta){
	int16_t result[4]={0,0,0,0};
	if(ch2<2 && ch2>-2){
		if(target_angle-current_angle>900){
			target_angle=current_angle+900;
		}
		ch2=delta;
	}
	else target_angle=output_angle;
	ch0 *= ratio0;
	ch1 *= ratio1;
	ch2 *= ratio2;
	result[0]=ch1+ch0+ch2;
	result[1]=-(ch1-ch0-ch2);
	result[2]=-(ch1+ch0-ch2);
	result[3]=ch1-ch0+ch2;
	return result;
}
PID wheels_pid[4];
PID angle_pid;
PID ang_vel_pid;
PID power_pid;
void control_car(int16_t ch0, int16_t ch1, int16_t ch2){
	current_angle = output_angle;
	angle_pid.current=current_angle;
	ang_vel_pid.current=curr_ang_vel;
	//power_pid.current=InfantryJudge.RealVoltage * InfantryJudge.RealCurrent;
	//int16_t* target_speed=control_remoter(ch0,ch1,ch2,1,1,1,0);
	int16_t* target_speed;
	if(DBUS_ReceiveData.rc.switch_right==3){
		target_speed=control_remoter(ch0,ch1,ch2,1.2,1.2,0.8,PID_output2(&angle_pid,target_angle,800,-800,30,-30));
	}
	else target_speed=control_remoter(ch0,ch1,ch2,1.2,1.2,0.8,PID_output(&angle_pid,target_angle));
	log_dug=angle_pid.output;
	wheels_pid[0].current=CM1Encoder.filter_rate;
	wheels_pid[1].current=CM2Encoder.filter_rate;
	wheels_pid[2].current=CM3Encoder.filter_rate;
	wheels_pid[3].current=CM4Encoder.filter_rate;
	int16_t input[4];
	for (int i = 0; i < 4; i++){	
		input[i]=PID_output(&wheels_pid[i],target_speed[i]);
		//input[i] *= PID_output(&power_pid, 80)/80;
	}
	Set_CM_Speed(CAN2, input[0],input[1], input[2], input[3]);
}


int main(void)
{	
	u8 dum[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '\n'};
	init();
	for (int i = 0; i < 4; i++){
		PID_init(&wheels_pid[i],50, 5, 3, 2000);

	}
	PID_init(&angle_pid,1,0.001,0,660);//5 20
	PID_init(&ang_vel_pid,0.01,0,0,1000);
	PID_init(&power_pid,0.1,0,0,100);
	while (1)  {	

		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			//buzzer_check();			
			
			if (ticks_msimg%500==0)
				DataMonitor_Send(dum, sizeof(dum));
			
			//TODO: nothing:-)
			if(ticks_msimg%50==0)
			{
				tft_clear();
				
				tft_prints(0,2,"Infantry V1.1");
				tft_clear();
				
				tft_prints(1,2,"rc0:%d rc1:%d",DBUS_ReceiveData.rc.ch0,DBUS_ReceiveData.rc.ch1);
				tft_prints(1,3,"rc2:%d",DBUS_ReceiveData.rc.ch2);
				//tft_prints(1,4,"rc2:%d",DBUS_ReceiveData.rc.ch2);
				//tft_prints(1,2,"V:%f",InfantryJudge.RealVoltage);
				//tft_prints(1,3,"I:%f",InfantryJudge.RealCurrent);
				//tft_prints(1,2,"rc0:%d",DBUS_ReceiveData.rc.switch_right);
				//tft_prints(1,3,"pidav:%d",pid_output_angle_speed);
				//tft_prints(1,4,"cav:%d",curr_ang_vel);
				tft_prints(1,4,"log:%f",log_dug);
				tft_prints(1,5,"1:%d 2:%d",CM1Encoder.filter_rate,CM2Encoder.filter_rate);
				tft_prints(1,6,"4:%d 3:%d",CM4Encoder.filter_rate,CM3Encoder.filter_rate);
				tft_prints(1,9,"cur:%d",current_angle);
				tft_prints(1,10,"tar:%d",target_angle);
				tft_prints(1,11,"pid:%f",yaw_pid_output_angle);
				tft_update();
				LED_blink(LED1);
			}		
			control_car(DBUS_ReceiveData.rc.ch0, DBUS_ReceiveData.rc.ch1, DBUS_ReceiveData.rc.ch2);
			if(DBUS_ReceiveData.rc.switch_left==1){
				control_car(0,0,0);
				tft_clear();
				tft_prints(0,2,"Shut down");
				Set_CM_Speed(CAN2,0,0,0,0);
				return 0;
			}
		}
	}	
}	
