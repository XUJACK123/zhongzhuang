#include "chassis_behaviour.h"
#include "remote_control.h"
#include "INS_task.h"
#include "user_lib.h"

extern INS_data_t INS_data;
extern TwoBoardControlGimbal_t TwoBoardControlGimbal;

#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }
float SetAngle;//小陀螺时设置的移动方向
float TopRoll=0.00f;//小陀螺时头晃是底盘不平，所以给个偏置，但是不能完全解决
float RollBias=0.0f;//莫名奇妙的偏置（应该是陀螺仪物理上倾斜了）
static void chassis_mode_CHASSIS_FOLLOW_GIMBAL(chassis_move_t *chassis_move_rc_to_vector);
static void chassis_mode_CHASSIS_TOP(chassis_move_t *chassis_move_rc_to_vector);
static void chassis_mode_CHASSIS_SELF_SAVING(chassis_move_t *chassis_move_rc_to_vector);
	
void chassis_behaviour_mode_set(chassis_move_t *chassis_move)
{
  if (chassis_move == NULL)
  {
    return;
  }
  
	chassis_move->last_chassis_mode=chassis_move->chassis_mode;
	
	if ((TwoBoardControlGimbal.mode)==2) // 左上
	{
		chassis_move->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
		//跳跃flag
//						chassis_move->JumpFlag=1;//通过flag控制控制跳跃、mode==9也只是间接控制flag
		//爬台阶flag
//						if(chassis_move->skip_ClimbTheStairsFlag==1)
//						{
//						chassis_move->ClimbTheStairsFlag=1;
//						}
	}
	else if ((TwoBoardControlGimbal.mode)==1) // 左中
	{
		chassis_move->chassis_mode = CHASSIS_FOLLOW_GIMBAL;

					chassis_move->JumpFlag=0;

						chassis_move->ClimbTheStairsFlag=0;
						chassis_move->skip_ClimbTheStairsFlag=1;
						chassis_move->chassis_climb_stage=FALL_DOWN;
	}
	else if ((TwoBoardControlGimbal.mode)==0) // 左下
	{
		chassis_move->chassis_mode = CHASSIS_ZERO_FORCE;
		chassis_move->UpFlag=0;
	}	
	else if((TwoBoardControlGimbal.mode)==8)
	{
		chassis_move->chassis_mode = CHASSIS_TOP;
	}
	else if((TwoBoardControlGimbal.mode)==9)
	{
		chassis_move->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
		chassis_move->JumpFlag=1;//通过flag控制控制跳跃、mode==9也只是间接控制flag
	}
	else if((TwoBoardControlGimbal.mode)==7)
	{
		chassis_move->chassis_mode = CHASSIS_FOLLOW_GIMBAL;
		if(chassis_move->skip_ClimbTheStairsFlag==1)
		{
		chassis_move->ClimbTheStairsFlag=1;
		}
	}
	else if((TwoBoardControlGimbal.mode)==6)
	{
		chassis_move->chassis_mode=CHASSIS_SELF_SAVING;
	}
	else if((TwoBoardControlGimbal.mode)==5)
	{
		chassis_move->chassis_mode=CHASSIS_FOLLOW_GIMBAL;
	}
	//0,1,2,6是自救，7蹭台阶、8小陀螺、跳跃,5是飞坡

	
}
void chassis_behaviour_control_set(chassis_move_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL)
	{
		return;
	}
  if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_ZERO_FORCE)
  {
	chassis_move_rc_to_vector->L_set=CHASSIS_LEG_MIN;
	chassis_move_rc_to_vector->chassis_state=ZERO_FORCE;
  }
  else if(chassis_move_rc_to_vector->chassis_mode == CHASSIS_FOLLOW_GIMBAL)
  {
		chassis_mode_CHASSIS_FOLLOW_GIMBAL(chassis_move_rc_to_vector);
  }
  else if(chassis_move_rc_to_vector->chassis_mode == CHASSIS_TOP)
  {
		chassis_mode_CHASSIS_TOP(chassis_move_rc_to_vector);
  }
  else if(chassis_move_rc_to_vector->chassis_mode == CHASSIS_SELF_SAVING)
  {
		chassis_mode_CHASSIS_SELF_SAVING(chassis_move_rc_to_vector);
  }
}

static void chassis_mode_CHASSIS_FOLLOW_GIMBAL(chassis_move_t *chassis_move_rc_to_vector)
{
	
	if(TwoBoardControlGimbal.legL_mode==2)
	{
		chassis_move_rc_to_vector->L_set=0.10f;//0.17f
		chassis_move_rc_to_vector->UpFlag=1;
	}
	else if (TwoBoardControlGimbal.legL_mode==3)
	{
		chassis_move_rc_to_vector->L_set=0.44f;//0.38f,0.45f
		chassis_move_rc_to_vector->UpFlag=1;
	}
	else
	{
		chassis_move_rc_to_vector->L_set=0.22f;//0.24
	}
	
	if(TwoBoardControlGimbal.mode==5)
	{
		chassis_move_rc_to_vector->L_set=0.26f;//0.24
	}

	if(TwoBoardControlGimbal.vx_set!=0)
	{
		chassis_move_rc_to_vector->UpFlag=1;
	}
	
	if(chassis_move_rc_to_vector->UpFlag==0)
	{
		chassis_move_rc_to_vector->L_set=0.15f;
	}
	
	//Roll轴补偿
	if(chassis_move_rc_to_vector->JumpFlyFlag==1)//空中时就不补偿了
	{
	chassis_move_rc_to_vector->Left_Leg.leg_L_set=chassis_move_rc_to_vector->Left_Leg.leg_L_set;
	chassis_move_rc_to_vector->Right_Leg.leg_L_set=chassis_move_rc_to_vector->Right_Leg.leg_L_set;
	}
	else
	{
	chassis_move_rc_to_vector->Left_Leg.leg_L_set=chassis_move_rc_to_vector->L_set;
	chassis_move_rc_to_vector->Right_Leg.leg_L_set=chassis_move_rc_to_vector->L_set;
	chassis_move_rc_to_vector->roll_set=TwoBoardControlGimbal.roll_set-RollBias;
	rc_deadband_limit(CHASSIS_WIDE*arm_sin_f32(INS_data.angle_roll+chassis_move_rc_to_vector->roll_set),chassis_move_rc_to_vector->delat_L,CHASSIS_ROLL_DEADBAND);
	chassis_move_rc_to_vector->Left_Leg.leg_L_set=chassis_move_rc_to_vector->Left_Leg.leg_L_set+chassis_move_rc_to_vector->delat_L;
	chassis_move_rc_to_vector->Right_Leg.leg_L_set=chassis_move_rc_to_vector->Right_Leg.leg_L_set-chassis_move_rc_to_vector->delat_L;
	}
	
	//腿长限位
	if(chassis_move_rc_to_vector->L_set > CHASSIS_LEG_MAX) chassis_move_rc_to_vector->L_set = CHASSIS_LEG_MAX;
	if(chassis_move_rc_to_vector->L_set < CHASSIS_LEG_MIN) chassis_move_rc_to_vector->L_set = CHASSIS_LEG_MIN;
	if(chassis_move_rc_to_vector->Left_Leg.leg_L_set > CHASSIS_LEG_MAX) chassis_move_rc_to_vector->Left_Leg.leg_L_set = CHASSIS_LEG_MAX;
	if(chassis_move_rc_to_vector->Left_Leg.leg_L_set < CHASSIS_LEG_MIN) chassis_move_rc_to_vector->Left_Leg.leg_L_set = CHASSIS_LEG_MIN;
	if(chassis_move_rc_to_vector->Right_Leg.leg_L_set > CHASSIS_LEG_MAX) chassis_move_rc_to_vector->Right_Leg.leg_L_set = CHASSIS_LEG_MAX;
	if(chassis_move_rc_to_vector->Right_Leg.leg_L_set < CHASSIS_LEG_MIN) chassis_move_rc_to_vector->Right_Leg.leg_L_set = CHASSIS_LEG_MIN;
	//前进速度设置
	chassis_move_rc_to_vector->vx_set=TwoBoardControlGimbal.vx_set;
}

static void chassis_mode_CHASSIS_TOP(chassis_move_t *chassis_move_rc_to_vector)
{
		//设置腿长
		if(TwoBoardControlGimbal.legL_mode==2)
		{
			chassis_move_rc_to_vector->L_set=0.15f;//
		}
		else if (TwoBoardControlGimbal.legL_mode==3)
		{
			chassis_move_rc_to_vector->L_set=0.3f;//0.38
		}
		else
		{
			chassis_move_rc_to_vector->L_set=0.2f;
		}
		
		chassis_move_rc_to_vector->Left_Leg.leg_L_set=chassis_move_rc_to_vector->L_set;
		chassis_move_rc_to_vector->Right_Leg.leg_L_set=chassis_move_rc_to_vector->L_set;
		chassis_move_rc_to_vector->roll_set=0-TopRoll;//陀螺仪roll不平
		rc_deadband_limit(CHASSIS_WIDE*arm_sin_f32(INS_data.angle_roll+chassis_move_rc_to_vector->roll_set),chassis_move_rc_to_vector->delat_L,CHASSIS_ROLL_DEADBAND);
		chassis_move_rc_to_vector->Left_Leg.leg_L_set=chassis_move_rc_to_vector->Left_Leg.leg_L_set+chassis_move_rc_to_vector->delat_L;
		chassis_move_rc_to_vector->Right_Leg.leg_L_set=chassis_move_rc_to_vector->Right_Leg.leg_L_set-chassis_move_rc_to_vector->delat_L;
		
		//腿长限位
		if(chassis_move_rc_to_vector->L_set > CHASSIS_LEG_MAX) chassis_move_rc_to_vector->L_set = CHASSIS_LEG_MAX;
		if(chassis_move_rc_to_vector->L_set < CHASSIS_LEG_MIN) chassis_move_rc_to_vector->L_set = CHASSIS_LEG_MIN;
		if(chassis_move_rc_to_vector->Left_Leg.leg_L_set > CHASSIS_LEG_MAX) chassis_move_rc_to_vector->Left_Leg.leg_L_set = CHASSIS_LEG_MAX;
		if(chassis_move_rc_to_vector->Left_Leg.leg_L_set < CHASSIS_LEG_MIN) chassis_move_rc_to_vector->Left_Leg.leg_L_set = CHASSIS_LEG_MIN;
		if(chassis_move_rc_to_vector->Right_Leg.leg_L_set > CHASSIS_LEG_MAX) chassis_move_rc_to_vector->Right_Leg.leg_L_set = CHASSIS_LEG_MAX;
		if(chassis_move_rc_to_vector->Right_Leg.leg_L_set < CHASSIS_LEG_MIN) chassis_move_rc_to_vector->Right_Leg.leg_L_set = CHASSIS_LEG_MIN;
			
		//小陀螺时的旋转方向
		float x_set,y_set;
		y_set=-(float)TwoBoardControlGimbal.vx_set;
		x_set=(float)TwoBoardControlGimbal.vy_set;
	//通过拨杆得到方向用于小陀螺旋转（这里感觉有点丑陋，应该有更好的办法）
	if (y_set == 0)
	{
		if(x_set > 0)
		{
			SetAngle = PI/2;
		}
		else if (x_set < 0)
		{
			SetAngle = -PI/2;
		}
		else
		{
			SetAngle = 0;
		}
	}
	else
	{
		if(y_set>0)
		{
		SetAngle = atan(x_set/y_set);
		}
		else
		{
			if(x_set>0)
			{
			SetAngle=-atan(y_set/x_set)+1.57;
			}
			else if(x_set<0)
			{
			SetAngle=-atan(y_set/x_set)-1.57;
			}
			else
			{
			SetAngle=3.14;	
			}
		}
	}
		//设置小陀螺前进速度
		if(y_set == 0&&x_set == 0)
		{
		chassis_move_rc_to_vector->vx_set=0;
		}
		else
		{
		chassis_move_rc_to_vector->vx_set=0.5f*arm_sin_f32(-SetAngle+TwoBoardControlGimbal.relative_angle_yaw);
		}
			//比较丑陋的变速小陀螺
		chassis_move_rc_to_vector->TopSwitchTime++;
		
			if(chassis_move_rc_to_vector->TopSwitchTime>=1)
			{
			chassis_move_rc_to_vector->wz_set=-13;//13
			}
			if(chassis_move_rc_to_vector->TopSwitchTime>=400)
			{
			chassis_move_rc_to_vector->wz_set=-9;//9
			}
			if(chassis_move_rc_to_vector->TopSwitchTime>=700)
			{
			chassis_move_rc_to_vector->wz_set=-5;//5
			}
			if(chassis_move_rc_to_vector->TopSwitchTime>=1100)
			{
			chassis_move_rc_to_vector->wz_set=-9;//9
			}
			if(chassis_move_rc_to_vector->TopSwitchTime>=1500)
			{
			chassis_move_rc_to_vector->wz_set=-13;//13
			chassis_move_rc_to_vector->TopSwitchTime=0;
			}
//		}
				//匀速旋转
//				chassis_move_rc_to_vector->wz_set=-15;//-13
		
}

static void chassis_mode_CHASSIS_SELF_SAVING(chassis_move_t *chassis_move_rc_to_vector)
{
 
  chassis_move_rc_to_vector->Left_Leg.leg_phi0_set=TwoBoardControlGimbal.LeftPhi0Set;
  chassis_move_rc_to_vector->Left_Leg.leg_L_set=TwoBoardControlGimbal.LeftLength;
  chassis_move_rc_to_vector->Right_Leg.leg_phi0_set=TwoBoardControlGimbal.RightPhi0Set;
  chassis_move_rc_to_vector->Right_Leg.leg_L_set=TwoBoardControlGimbal.RightLengthSet;
	  
  if(chassis_move_rc_to_vector->Left_Leg.leg_L_set > 0.4f) chassis_move_rc_to_vector->Left_Leg.leg_L_set = 0.4f;
  if(chassis_move_rc_to_vector->Left_Leg.leg_L_set < CHASSIS_LEG_MIN) chassis_move_rc_to_vector->Left_Leg.leg_L_set = CHASSIS_LEG_MIN;
  if(chassis_move_rc_to_vector->Right_Leg.leg_L_set > 0.4f) chassis_move_rc_to_vector->Right_Leg.leg_L_set = 0.4f;
  if(chassis_move_rc_to_vector->Right_Leg.leg_L_set < CHASSIS_LEG_MIN) chassis_move_rc_to_vector->Right_Leg.leg_L_set = CHASSIS_LEG_MIN;
}

