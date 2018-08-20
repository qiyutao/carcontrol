/***************************************************************************************
									文件开始
****************************************************************************************/

/*-------------------------------------文件名定义--------------------------------------*/
#ifndef _PRO_CAN_AL_H_
#define _PRO_CAN_AL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef _PRO_CAN_AL_
#define _PRO_CAN_AL_EXTERN_
#else
#define _PRO_CAN_AL_EXTERN_ extern
#endif

#include "pro_can_cfg.h"

#define TEST_CAN 0X08 /*BODY CAN MSG2 车身信息CAN*/
#define CH_CAN 1	  /*底盘信息CAN*/
#define T_CAN 0		  /*油门CAN*/

	/*-------------------------------------结构体定义--------------------------------------*/

	/* 车身信息结构体(包含转向灯信息) */
	typedef struct
	{
		unsigned char BCM_RightTurnSwitchrSt; /* 	右转向灯信号	灯状态：0 "Off"断开 1 "Closed"闭合 	 */
		unsigned char BCM_LeftTurnSwitchSt;   /* 	左转向灯信号	灯状态：0 "Off"断开 1 "Closed"闭合 	 */
	} BODY_CAN_DATA;

	/* 油门控制器接收结构体 */
	typedef struct
	{
		unsigned char ThrottleState;		 /* 油门当前状态；0：人工状态 1：自动状态，			*/
		unsigned char BrakeAndReverseSignal; /* 人工干预油门状态 0：人工未踩油门；1：人工踩下油门			*/
		float ThrottlePedalPercent;			 /* 当前油门踏板百分比 例67.4%	范围0-1000 转换公式：值/10	*/
	} THROTTLE_RX_DATA;

	/* 转向控制器的接收结构体 */
	typedef struct
	{
		float Torgue_Value;							 /* 方向盘转向力矩 范围0-14055 转换公式：值/1000-7 */
		float SteeringWheelAngle_back;				 /* 方向盘当前角度 范围0-17400 转换公式：值/10 -870*/
		unsigned short SteeringWheelAngleSpeed_back; /* 方向盘角速度 范围100-540 转换公式：值*10 */
		unsigned char EPS_State;					 /* 方向盘当前工作状态 */
	} EPSRX_DATA;

	/* 刹车信息的接收结构体 */
	typedef struct
	{
		float VehSpd;					 /* 车身速度 KM/H 范围0-51199 转换公式：值/100 */
		unsigned char AEBActive;		 /* AEB刹车响应状态 如果此位置为1则表明AEB刹车已经响应 */
		unsigned char AEBNotAvailable;   /* EBST是否可以响应ADAS的AEB刹车请求，即自身状态是否正常 */
		unsigned char AccNotAvailable;   /* ACC刹车响应状态 如果此位置为1则表明ACC刹车已经响应 */
		unsigned char AccBrakeActive;	/* EBST是否可以响应ADAS的ACC刹车请求，即自身状态是否正常 */
		unsigned char BrakePedalApplied; /* 人工干预刹车状态 1：人工踩下刹车，0：人工未踩刹车 */
	} EBSTMSG00_DATA;

	/* 档位控制器接收结构体 */
	typedef struct
	{
		unsigned char GSM_ParkSwitchSta_CH;		/* GSM的P档状态 */
		unsigned char GSM_ParkSwitchInvalid_CH; /* GSM的P档状态是否有效 */
		unsigned char GSM_ErrorSt;				/* GSM是否有故障      "0：无故障 1：故障" */
		unsigned char GSM_Fail_ApaNodeLost_CH;  /* ADAS的节点丢失状态 "0: APA 节点未丢失 1: APA 节点丢失" */
		unsigned char GSM_InterventionAPA;		/* GSM是否被人工干涉  "0: 未干涉 1: 干涉(包括P档干涉) " */
		unsigned char GSM_GearLeverPosition_CH; /* GSM档杆位置状态  0x0: Reserved 0x1: BW 0x2: H 0x3: FW 0x4: Reserved 0x5: Reserved 0x6: Reserved 0x7: Invalid  */
	} GSM_DATA;

	/* EPCM信息接收结构体(包含当前档位信息) */
	typedef struct
	{
		unsigned char DrivelineState_BD;  /* "当前档位的状态：0 P档 1 R挡 2 N档 3 D档 ;" */
		unsigned char DrivelineStateV_BD; /* 当前档位状态是否有效：0 "valid" 有效1 "invalid"无效 */
	} EPCM_DATA;

	typedef struct
	{
		EBSTMSG00_DATA brak_data;		/* 刹车信息结构体 */
		EPSRX_DATA eps_data;			/* 转向控制器的结构体 */
		THROTTLE_RX_DATA throttle_data; /* 油门控制器结构体 */
		BODY_CAN_DATA body_data;		/* 车身信息结构体(包含转向灯信息) */
		GSM_DATA gsm_data;				/* 档位控制器结构体 */
		EPCM_DATA epcm_data;			/* EPCM信息结构体(包含当前档位信息) */
	} CHASSISRX_DATA;

	/*********************************CAN接收用于控制车身的结构体*******************************************/

	/* 转向控制器的控制结构体 */
	typedef struct
	{
		/* float AssistTorgue_Value; */				/* 设置辅助力矩值           辅助力矩值设置;	Nm */
		unsigned short SteeringWheelAngleSpeed_ref; /* 设置方向盘的转角速度     方向盘目标角速度; degree/s  范围100-540 转换公式：值*10 */
		/* unsigned char AssistTorgueOpen_State; */ /* 设置辅助力矩             0x00:辅助力矩关闭；0x08：辅助力矩开启（注：此功能只有在手动模式下成立）;	 */
		unsigned char SteeringWheelAngle_state;		/* 转角指令是否正常         转角指令是否正常;0：不正常1：正常 ;	 */
		unsigned char ADAS_State;					/* 告知ADAS当前状态         "0：ADAS状态不正常 1：ADAS状态正常 ;"	 */
		unsigned char WorkMode;						/* 设置方向盘的工作状态指令 "EPS工作模式指令0：待机指令；1：自动驾驶模式指令；2：预留；3：预留；4：手动指令；5：手动介入恢复指令；6：清故障指令；7-15：预留；;" */
		float SteeringWheelAngle_ref;				/* 设置方向盘目标角度       方向盘目标角度指令;	degree  范围0-17400 转换公式：值/10 -870*/
	} EPSTX_DATA;

	/* 油门控制器的控制结构体 */
	typedef struct
	{
		unsigned char ADASState;			   /* ADAS状态          "0：人工驾驶状态；1：自动驾驶状态；2：ADAS故障"	 */
		unsigned char IsManualThrottleApprove; /* 是否允许人工踩油门 "0：允许人工踩油门；1：不允许人工踩油门"	 */
		float ThrottleCommand;				   /* 目标油门踏板开度   设置油门踏板百分比	% 例67.4%	范围0-1000 转换公式：值/10	 */
	} THROTTLE_TX_DATA;

	/* 刹车信息控制结构体 */
	typedef struct
	{
		unsigned char Deceleration_Request_Active /* ADAS请求减速 ADAS减速度请求标志位 1：ADAS发出请求，0：ADAS不请求 */
			float Deceleration_Target			  /* 减速度值     目标减速度  	m/s^2 范围0-64224 转换公式：值/1000	*/
	} EBSTMSG01_DATA;

	/* 档位控制器控制结构体 */
	typedef struct
	{
		unsigned char APA_SetLeverPosition_CH; /* "设置档位的状态：0 P档 1 R挡 2 N档 3 D档 0x7 无效;" */
		unsigned char APA_RequestEnable_CH;	/* P档请求指令使能            "0x0:Disabled  失能 0x1:Enabled   使能" */
		unsigned char APA_ReqParkSwitchSta_CH; /* P档请求                    "0x0:Park inactive    P档释放 0x1:Park active   P档挂起" */
	} GSM_TX_DATA;

	typedef struct
	{
		EBSTMSG01_DATA brak_Tdata;		 /* 刹车信息结构体 */
		EPSTX_DATA eps_Tdata;			 /* 转向控制器的结构体 */
		THROTTLE_TX_DATA throttle_Tdata; /* 油门控制器结构体 */
		GSM_TX_DATA gsm_Tdata;			 /* 档位控制器结构体 */
	} CHASSISTX_DATA;

	typedef struct
	{
		unsigned int Yaw_Rate; /* 	横摆角速度	Unsigned	32 */
		unsigned int X_Accel;  /* 	纵向加速度	Unsigned	32 */
		unsigned int Y_Accel;  /* 	横向加速度	Unsigned	32 */
	} GSENSER_DATA;

	typedef enum
	{
		UDS_CH_EBCM_Resp = 0x073d,
		EBCM_STA1_CH = 0x0218,
		EBCM_STA2_CH = 0x0208,
		EBCM_STA3_CH = 0x0258,
		SAS_CAL_ESP_CH = 0x03F0,
		EBCM_STA4_CH = 0x0259,
		EBCM_STA5_CH = 0x0219,
		EBCM_STA6_CH = 0x0217,
	} ENUM_UDSID;

	typedef enum
	{
		EPS_RX = 0x0133,
		/* 0X0215, */ /*ADAS TO EPS  方向转角系统*/
		EPS_TX = 0x0143,
		/* 0X0322, */ /*EPS TO ADAS  方向转角系统*/
		EBSTMSG00 = 0x0141,
		/* 0X0100, */		   /*EBS TO ADAS  刹车系统*/
		EBSTMSG01 = 0X0130,	/*ADAS TO EBS  刹车系统*/
		CH1_CH2 = 0X0600,	  /*IMU RECEIVE  加速度传感器信息*/
		CH3_CH4 = 0X0602,	  /*IMU RECEIVE  加速度传感器信息*/
		THROTTLE_RX = 0X0620,  /*ADAS TO THROTTLE  油门控制系统*/
		THROTTLE_TX = 0X0630,  /*THROTTLE TO ADAS  油门控制系统*/
		GSM_STAT_RX = 0x0132,  /*ADAS TO GSM  档位控制系统*/
		GSM_STAT_CH = 0X0250,  /*GSM TO ADAS  档位控制系统 DBT*/
		EPCM_STA1_BD = 0X0221, /*EPCM TO ADAS  EPCM档位信息*/

		BCM2 = 0X023A, /*BODY CAN MSG1 车身信息CAN*/
		BCM3 = 0X033C, /*BODY CAN MSG2 车身信息CAN*/
	} ENUM_CANID;

	/* BO_ 1264 Vehicle1: 8 Vehicle */
	typedef struct
	{
		unsigned char Steering_angle_validity;  /*47|1  @ 0+ (1,0) [0|0] "" Vector__XXX*/
		unsigned short Steering_angle_rate;		/*50|11 @ 0+ (1,0) [0|2047] "deg/s" Vector__XXX*/
		unsigned char Steering_angle_sign;		/*46|1  @ 0+ (1,0) [0|0] "" Vector__XXX*/
		unsigned char Steering_angle_rate_sign; /*30|1  @ 0+ (1,0) [0|0] "" Vector__XXX*/
		unsigned short Steering_angle;			/*45|11 @ 0+ (1,0) [0|2047] "deg" Vector__XXX*/
		unsigned short Radius_curvature;		/*29|14 @ 0- (1,0) [-8192|8191] "m" Vector__XXX*/
		unsigned char Yaw_rate_validity;		/*31|1  @ 0+ (1,0) [0|0] "" Vector__XXX*/
		unsigned short Yaw_rate;				/*11|12 @ 0- (0.0625,0) [-128|127.9375] "deg/s" Vector__XXX*/
		unsigned char Vehicle_speed_direction;  /*12|1  @ 0+ (1,0) [0|0] "" Vector__XXX*/
		unsigned short Vehicle_speed;			/*7|11  @ 0+ (0.0625,0) [0|127.9375] "m/s"  ESR*/
	} SG_77G_CAN;

	/*-------------------------------------外部函数声明--------------------------------------*/
	_PRO_CAN_AL_EXTERN_ unsigned char can_al_init(void);	   /*init APP layer */
	_PRO_CAN_AL_EXTERN_ void can_rev_msg(CAN_PACKET *msg_ptr); /*处理接收报文*/
	_PRO_CAN_AL_EXTERN_ void send_can_msg(unsigned int send_ID);

	_PRO_CAN_AL_EXTERN_ void CanRevBCMMsg(unsigned char *Data8);								 /* BCM车身数据解析 */
	_PRO_CAN_AL_EXTERN_ void CanRevEPCMMsg(unsigned char *Data8);								 /* 档位状态数据解析 */
	_PRO_CAN_AL_EXTERN_ void CanRevGSMMsg(unsigned char *Data8);								 /* GSM控制器数据解析 */
	_PRO_CAN_AL_EXTERN_ void CanRevThrottleMsg(unsigned char *Data8);							 /* 油门数据解析 */
	_PRO_CAN_AL_EXTERN_ void CanRevBreakMsg(unsigned char *Data8);								 /* 刹车数据解析 */
	_PRO_CAN_AL_EXTERN_ void CanRevSteelMsg(unsigned char *Data8);								 /* 转向数据解析 */
	_PRO_CAN_AL_EXTERN_ void CanSendSteelMsg(float steerling_value, unsigned short steer_speed); /* 转向数据封装发送 */

#ifdef __cplusplus
}
#endif

#endif /* _PRO_CAN_AL_H_ */

/***************************************************************************************
      																文件结尾
****************************************************************************************/
