//////////////////////////////////////////////////////////////////////////////////////////
// File:     WBIK_Functions.h
// Date:     11 June 2013     
// Description:  h file for kinematic functions
// Author:  Inhyeok Kim <inhyeok@rainbow.re.kr>      
// ----------------------------------------------------------------------------
// Copyright (c) 2013, Rainbow Co.
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the Rainbow Co.
// 4. Neither the name of the Rainbow Co. nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY Inhyeok Kim ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL Inhyeok Kim BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////////////////


#ifndef WBIK_FUNCTIONS_H
#define WBIK_FUNCTIONS_H

#include "CommonDefinition.h"

//typedef unsigned int BOOL;
//#define TRUE		1
//#define FALSE		0

#define DRC_BI_MODE	1
#define DRC_QUAD_MODE	2

// #define PI			3.141592653589793f
#define D2R			0.017453292519943f
#define R2D			57.295779513082323f
#define MARGIN		D2R

#define DT			0.005f

#define gAcc		9.81f

//------------------------------ feedback control gains
#define KP1		1600.f				// wn^2, 
#define KD1		80.f				// 2*zeta*wn

#define KP2		400.f				
#define KD2		40.f

#define KP3		0.f				
#define KD3		40.f


//---------------------------------------------- for walking pattern
#define WTG_MODE					3		// WTG_MODE = 0 : statically stable region is co


//------------------------------ link parameters
#define CX_PEL		-0.0192f // -0.0171f
#define CY_PEL		0.
#define CZ_PEL		0.1330f  // 0.128f

#define CX_TOR		-0.0115f
#define CY_TOR		0.f
#define CZ_TOR		 0.1347f

#define CX_UARM		0.0044f
#define CY_UARM		-0.01f
#define CZ_UARM		-0.089f

#define CX_LARM		-0.0288f
#define CY_LARM		0.0422f
#define CZ_LARM		-0.1802f

#define CX_HAND		-0.0001f
#define CY_HAND		0.0051f
#define CZ_HAND		-0.0537f

#define CX_ULEG		0.016f   //0.0155f
#define CY_ULEG		0.0056f  //-0.0071f
#define CZ_ULEG		-0.1216f //-0.1162f

#define CX_LLEG		0.0133f  //0.0121f
#define CY_LLEG		0.0104f  //0.0156f
#define CZ_LLEG		-0.2157f  //-0.2247f

#define CX_FOOT		0.0174f  //0.0122f
#define CY_FOOT		-0.0013f //0.f
#define CZ_FOOT		-0.03f   //-0.0217f
	
#define L1_PEL		0.177f
#define L_LEG	  	0.33f
#define L2_PEL		0.173f
#define L1_TOR		0.4590f
#define L2_TOR		0.1845f
#define L_FOOT		0.105f  //0.14f //0.101f
#define OFF_ELB		0.03f
#define L_UARM		0.3f
#define L_LARM		0.3f
#define L_HAND		0.12f
#define L_STICK		0.095f

#define m_PEL		4.238f //3.465f
#define m_TOR		7.202f
#define m_ULEG		5.789f //5.222f
#define m_LLEG		1.575f //1.307f
#define m_FOOT		2.725f //2.6983f
#define m_UARM		2.6373f
#define m_LARM		1.318f
#define m_LHAND		1.2f
#define m_RHAND		1.4f
#define m_TOTAL		(2.f*(m_ULEG+m_LLEG+m_UARM+m_LARM+m_FOOT)+m_LHAND+m_RHAND+m_TOR+m_PEL)

#define m_dummy		0.001f
#define idummy		(float)1e-6
#define iLUA11		0.0339f
#define iLUA22		0.0335f
#define iLUA33		0.0036f
#define iLLA11		0.0153f
#define iLLA22		0.0136f
#define iLLA33		0.0224f
#define iLH11		0.0033f
#define iLH22		0.0033f
#define iLH33		0.0006f 

#define iRUA11		0.0339f
#define iRUA22		0.0335f
#define iRUA33		0.0036f
#define iRLA11		0.0153f
#define iRLA22		0.0136f
#define iRLA33		0.0224f
#define iRH11		0.0033f
#define iRH22		0.0033f
#define iRH33		0.0006f 


//-------- initial position in degree
#define OFFSET_RSR	-0.1f//-10.f
#define OFFSET_LSR	0.1f//10.f
#define OFFSET_REB	-0.1f//-20.f
#define OFFSET_LEB	-0.1f//-20.f
//---------------


//--------------------------------------------- joint constraints
#define JOINT_LIMIT_DUTY	0.2f
#define JOINT_LIMIT_RANGE	(5.f*D2R)
#define	QP_MAX			12.f // rad/s

#define WSTmax          1.06f  // 61.f*D2R
#define WSTmin          -1.06f // -61.f*D2R
#define WSTpmax         6.28f
#define WSTppmax        62.8f

#define RSPmax          4.71f
#define RSPmin          -4.71f
#define RSPpmax         6.28f
#define RSPppmax        62.8f

#define RSRmax          0.17f
#define RSRmin          -3.14f
#define RSRpmax         6.28f
#define RSRppmax        62.8f

#define RSYmax          (165.f*D2R)
#define RSYmin          (-165.f*D2R)
#define RSYpmax         6.28f
#define RSYppmax        62.8f

#define REBmax          0.f
#define REBmin          -2.96f
#define REBpmax         6.28f
#define REBppmax        62.8f

#define RWYmax          (155.f*D2R)
#define RWYmin          (-155.f*D2R)
#define RWYpmax         6.28f
#define RWYppmax        62.8f

#define RWPmax			(120.f*D2R)
#define RWPmin			(-187.f*D2R)
#define RWPpmax			6.28f
#define RWPppmax		62.8f

#define RWY2max         (160.f*D2R)
#define RWY2min         (-160.f*D2R)
#define RWY2pmax         6.28f
#define RWY2ppmax        62.8f

#define LSPmax          RSPmax
#define LSPmin          RSPmin
#define LSPpmax         RSPpmax
#define LSPppmax        RSPppmax

#define LSRmax          -(RSRmin)
#define LSRmin          -(RSRmax)
#define LSRpmax         RSRpmax
#define LSRppmax        RSRppmax

#define LSYmax          -(RSYmin)
#define LSYmin          -(RSYmax)
#define LSYpmax         RSYpmax
#define LSYppmax        RSYppmax

#define LEBmax          REBmax
#define LEBmin          REBmin
#define LEBpmax         REBpmax
#define LEBppmax        REBppmax

#define LWYmax          -(RWYmin)
#define LWYmin          -(RWYmax)
#define LWYpmax         RWYpmax
#define LWYppmax        RWYppmax

#define LWPmax			RWPmax
#define LWPmin			RWPmin
#define LWPpmax			RWPpmax
#define	LWPppmax		RWPppmax

#define LWY2max          -(RWY2min)
#define LWY2min          -(RWY2max)
#define LWY2pmax         RWY2pmax
#define LWY2ppmax        RWY2ppmax

#define RHYmax          (92.f*D2R) // 32.f*D2R //0.55f
#define RHYmin          (-92.f*D2R)  // -32.f*D2R //-0.55f
#define RHYpmax         6.28f
#define RHYppmax        62.8f

#define RHRmax          (52.f*D2R)//0.36f  
#define RHRmin          (-52.f*D2R)-0.6f  
#define RHRpmax         6.28f
#define RHRppmax        62.8f

#define RHPmax          (91.f*D2R)
#define RHPmin          (-110.f*D2R) //
#define RHPpmax         6.28f
#define RHPppmax        62.8f

#define RKNmax          2.6f
#define RKNmin          0.f
#define RKNpmax         6.28f
#define RKNppmax        62.8f

#define RAPmax          1.91f
#define RAPmin          -1.91f
#define RAPpmax         6.28f
#define RAPppmax        62.8f

#define RARmax          (52.f*D2R)//0.43f
#define RARmin          (-52.f*D2R)//-0.43f
#define RARpmax         6.28f
#define RARppmax        62.8f

#define LHYmax          RHYmax
#define LHYmin          RHYmin
#define LHYpmax         RHYpmax
#define LHYppmax        RHYppmax

#define LHRmax          -(RHRmin)
#define LHRmin          -(RHRmax)
#define LHRpmax         RHRpmax
#define LHRppmax        RHRppmax

#define LHPmax          RHPmax
#define LHPmin          RHPmin
#define LHPpmax         RHPpmax
#define LHPppmax        RHPppmax

#define LKNmax          RKNmax
#define LKNmin          RKNmin
#define LKNpmax         RKNpmax
#define LKNppmax        RKNppmax

#define LAPmax          RAPmax
#define LAPmin          RAPmin
#define LAPpmax         RAPpmax
#define LAPppmax        RAPppmax

#define LARmax          -(RARmin)
#define LARmin          -(RARmax)
#define LARpmax         RARpmax
#define LARppmax		RARppmax


#define NKYmin			-180.f
#define NKYmax			180.f
#define NK1min			-30.f
#define NK1max			70.f
#define NK2min			-45.f
#define NK2max			45.f
//-----------------------------------------------------------




//---------------------------------------------- for walking pattern
#define WTG_MODE					3		// WTG_MODE = 0 : statically stable region is considered in the sway motion
											// WTG_MODE = 1 : statically stable region is not considered in the sway motion
									
#define dFORWARD_MAX				0.4f
#define dYAW_MAX					0.5235988f  // 30degree
#define STEP_BUNDLE_RING_SIZE		20
#define FOOT_SIZE					0.22f
#define RFSP						-1
#define LFSP						1
#define DFSP						2
#define RHSP						-1
#define LHSP						1
#define DHSP						2
#define T_STEP_MAX					5.f
#define T_STEP_MIN					0.5f
#define T_DFSP						0.1f
#define WALK_TRAJ_SIZE				2000
#define dT_IDLE					1.f
//#define ZMPXMAX					0.14f
//#define ZMPXMIN					-0.08f
//#define ZMPYMAX					0.1685f
//#define ZMPYMIN					-0.1685f
#define ZMP_SAG_MARGIN			0.1f
#define ZMP_FRON_MARGIN			0.f
#define ZMP_OFFSET				0.05f //0.013f
#define dLIFT					0.05f
#define MOVING_FOOT_CUTOFF		1e-4
#define K_SFSP					516.6295f		// Nm/rad
#define L_COM					0.595f	
#define K_PELVIS_ROLL			11500.f
#define K_PELVIS_YAW			9000.f


#define PWM_SIGN_LSP				-1.f
#define PWM_SIGN_LSR				-1.f
#define PWM_SIGN_LSY				-1.f
#define PWM_SIGN_LEB				-1.f
#define PWM_SIGN_LWY				-1.f
#define PWM_SIGN_LWP				-1.f
#define PWM_SIGN_LWY2				-1.f

#define PWM_SIGN_RSP				-1.f
#define PWM_SIGN_RSR				-1.f
#define PWM_SIGN_RSY				-1.f
#define PWM_SIGN_REB				-1.f
#define PWM_SIGN_RWY				-1.f
#define PWM_SIGN_RWP				-1.f
#define PWM_SIGN_RWY2				-1.f

#define PWM_SIGN_RHY				-1.f
#define PWM_SIGN_RHR				-1.f
#define PWM_SIGN_RHP				-1.f
#define PWM_SIGN_RKN				-1.f
#define PWM_SIGN_RAP				-1.f
#define PWM_SIGN_RAR				-1.f

#define PWM_SIGN_LHY				-1.f
#define PWM_SIGN_LHR				-1.f
#define PWM_SIGN_LHP				-1.f
#define PWM_SIGN_LKN				-1.f
#define PWM_SIGN_LAP				-1.f
#define PWM_SIGN_LAR				-1.f


//-----------------------------

int poly5(float ti, float tf, float yi, float yf, float ypi, float ypf, float yppi, float yppf, float *coeff_result_6x1);	// 5th order polynomial, y(t)=coeff(1) + coeff(2)*t + coeff(3)*t^2 + coeff(4)*t^3 + coeff(5)*t^4 + coeff(6)*t^5


//------------------------------- for the offline test
#define OFFLINE_LENGTH			10000
#define OFFLINE_MAX_CHANGES		100
typedef struct{
	float drvPEL[OFFLINE_LENGTH][4];
	float dpCOMx[OFFLINE_LENGTH];	
	float dpCOMy[OFFLINE_LENGTH];	
	float dpRFx[OFFLINE_LENGTH];
	float dpRFy[OFFLINE_LENGTH];
	float dpRFz[OFFLINE_LENGTH];
	float dpLFx[OFFLINE_LENGTH];
	float dpLFy[OFFLINE_LENGTH];
	float dpLFz[OFFLINE_LENGTH];
	float dpRHx[OFFLINE_LENGTH];
	float dpRHy[OFFLINE_LENGTH];
	float dpRHz[OFFLINE_LENGTH];
	float dpLHx[OFFLINE_LENGTH];
	float dpLHy[OFFLINE_LENGTH];
	float dpLHz[OFFLINE_LENGTH];
	float dpPELz[OFFLINE_LENGTH];
	unsigned int i;	
	unsigned int length;
	unsigned int n_changes;
	unsigned int i_change[OFFLINE_MAX_CHANGES][2];
	int des_fsp[OFFLINE_MAX_CHANGES];
	float offsetZMPs[OFFLINE_LENGTH];
	float offsetZMPf[OFFLINE_LENGTH];
} OFFLINE_TRAJ;

int LoadOfflineTraj(int mode);
int LoadControllerParamter(void);
int LoadControllerParamter_DRC(char drc_walking_mode);
int WBIK_FreqTest(unsigned int n);
int WBIK_Offline_DRC(unsigned int n);
int WBIK_DRC_steering(unsigned int n);

#define COP_CUTOFF		30.f   // rad/s
#define N_TRANSITION	25
#define N3_TRANSITION	400

#define WEIGHT_PEL_SQRT_INV		1.f  //0.01f  // 1/sqrt(W) = 1/sqrt(10000)
#define WEIGHT_PCZ_SQRT_INV		1.f 
#define WEIGHT_WST_SQRT_INV		1.f  //0.0316f // 1/sqrt(100)

#define GLOBAL		0
#define LOCAL		1
//-------------------------------


#define X_34		1
#define Y_34		2
#define Z_34		3
#define Q1_34		4
#define Q2_34		5
#define Q3_34		6
#define Q4_34		7
#define WST_34		8
#define	RHY_34		9
#define RHR_34		10
#define RHP_34		11
#define	RKN_34		12
#define RAP_34		13
#define RAR_34		14
#define LHY_34		15
#define LHR_34		16
#define LHP_34		17
#define LKN_34		18
#define LAP_34		19
#define LAR_34		20
#define RSP_34		21
#define RSR_34		22
#define RSY_34		23
#define REB_34		24
#define RWY_34		25
#define RWP_34		26
#define LSP_34		27
#define LSR_34		28
#define LSY_34		29
#define LEB_34		30
#define LWY_34		31
#define LWP_34		32
#define RWY2_34		33
#define LWY2_34		34


#define X_33		1
#define Y_33		2
#define Z_33		3
#define WX_33		4
#define WY_33		5
#define WZ_33		6
#define WST_33		7
#define	RHY_33		8
#define RHR_33		9
#define RHP_33		10
#define	RKN_33		11
#define RAP_33		12
#define RAR_33		13
#define LHY_33		14
#define LHR_33		15
#define LHP_33		16
#define LKN_33		17
#define LAP_33		18
#define LAR_33		19
#define RSP_33		20
#define RSR_33		21
#define RSY_33		22
#define REB_33		23
#define RWY_33		24
#define RWP_33		25
#define LSP_33		26
#define LSR_33		27
#define LSY_33		28
#define LEB_33		29
#define LWY_33		30
#define LWP_33		31
#define RWY2_33		32
#define LWY2_33		33


int InitGlobalMotionVariables(void);
int FreeGlobalMotionVariables(void);

int UpdateGlobalMotionVariables(void);

int FKine_Whole(void);
int FKine_COM(const float *Q_34x1, float *pCOM_3x1);
int FKine_Hand(char ref_coord, const float *Q_34x1, float *pRH_3x1, float *qRH_4x1, float *pLH_3x1, float *qLH_4x1);
int FKine_Stick(char ref_coord, const float *Q_34x1, float *pRS_3x1, float *qRS_4x1, float *pLS_3x1, float *qLS_4x1);
int FKine_Wrist(char ref_coord, const float *Q_34x1, float *pRWR_3x1, float *qRWR_4x1, float *pLWR_3x1, float *qLWR_4x1);

int QT2DC(const float *qt_4x1, float **DC_3x3);		// convert a quaternion to a direction cosine matrix
int DC2QT(const float **DC_3x3, float *qt_4x1);		// convert a direction cosine matrix to a quaternion
int QTdel(const float *des_qt_4x1, const float *qt_4x1, float *result_3x1); // delta quaternion
int Wq(int ref, const float *qt_4x1, float **Wq_3x4); // quaternion rate matrix. (ref==0: global frame, ref==1:body frame)
int Qq(int ref, const float *qt_4x1, float **Qq_4x4); // quaternion matrix. (ref==0: global frame, ref==1:body frame)
int QTcross(const float *QT1_4x1, const float *QT2_4x1, float *result_4x1);
int QTbar(const float *qt_4x1, float *result_4x1);

int RX(float theta, float **R_3x3);
int RY(float theta, float **R_3x3);
int RZ(float theta, float **R_3x3);

int RVALS(float x, float xp, float xpp_ref, float xpmax, float xppmax, float xmin, float xmax, float margin, float *xpp_result, float *bound_l, float *bound_u);	// Range, Velocity and Acceleration Limit Strategy
int RVALS3(float x, float xp, float xpmax, float xppmax, float xmin, float xmax, float margin, float *bound_l, float *bound_u);	

int UpdatePassiveCoord_DSP(void);
int UpdatePassiveCoord_SSP(int LorR); // left:LorR=1,  right:LorR=-1

void wberror(char error_text[]);

int derive3(float x0, float x1, float x2, float *xd1, float *xdd1, float dt);
int derive3QT(float *q_past_4x1, float *q_4x1, float *q_next_4x1, float *w_result_3x1, float dt);

float limitDuty(float limit, float duty);
int GetZMP_WB(FT rf, FT lf, FT rw, FT lw);

int getGravityTorque(const float *Q_34x1, float *gravity_33x1);
int getFricCompen(const float *Qp_33x1, float *viscous_33x1);
int getDuty4JointLimit(float Q_34x1[], float duty_joint_limit_33x1[]);
int WBIK_DRC_ladder_climbing(unsigned int n);
int WBIK_DRC_quad(unsigned int n);

int checkJointRange(float Q_34x1[]);
float torque2duty(int joint_no, float torque);
int LoadParameter_ComputedTorque(void);
float one_cos(float t_sec, float mag, float T_sec);
int one_cos_orientation(float t_sec, const float *qt0_4x1, const float *qt1_4x1, float T_sec, float *result_4x1);
int QT2RV(const float *qt_4x1, float* rv); // quaternion to rotational vector
int RV2QT(const float *rv, float *qt_4x1); // rotational vector to quaternion
//------------------------------------------------------------------

#endif




