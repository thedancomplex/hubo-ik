// Written by Inhyeok Kim, Rainbow orginally in Window.

#include <stdio.h>
//#include <stdlib.h>
#include <math.h>
#include "WBIK_Functions.h"
#include "nrutil.h"
#include "CommonDefinition.h"
#include "SharedMemory.h"
#include "APIs.h"


#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include "nrutil.h"

#include "CommonDefinition.h"
#include "CAN.h"

#define NR_END 1
#define FREE_ARG char*
PSHARED_DATA	pSharedMemory;

extern JOINT Joint[NO_OF_JOINT];		// Joint struct variable
extern FT FTSensor[NO_OF_FT];		// FT sensor struct variable
extern IMU IMUSensor[NO_OF_IMU];	// IMU sensor struct variable

extern bool SendRunStopCMD(unsigned char _boardID, unsigned char _runStop);

const float _AXIS_X[4] = {0., 1.f, 0., 0.};
const float _AXIS_Y[4] = {0., 0., 1.f, 0.};
const float _AXIS_Z[4] = {0., 0., 0., 1.f};


//------------- Constants for link geometries
const float _LINK_LEG[4] = {0., 0., 0., -L_LEG};
const float _LINK_RPEL[4] = {0., 0., -L1_PEL*0.5f, 0.};
const float _LINK_LPEL[4] = {0., 0., L1_PEL*0.5f, 0.};
const float _LINK_FOOT[4] = {0., 0., 0., -L_FOOT};
const float _LINK_WST[4] =  {0., 0., 0., L2_PEL};
const float _LINK_RSHLD[4] =  {0., 0., -L1_TOR*0.5f, L2_TOR};
const float _LINK_LSHLD[4] =  {0., 0., L1_TOR*0.5f, L2_TOR};
const float _LINK_UARM[4] =   {0., OFF_ELB, 0., -L_UARM};
const float _LINK_LARM[4] =   {0., -OFF_ELB, 0., -L_LARM};
const float _LINK_HAND[4] =   {0., 0., 0., -L_HAND};
const float _LINK_STICK[4] =   {0., 0., 0.,-L_STICK};
//-------------


//------------- Local coordinates of mass centers 
const float _C_PEL[4] = {0., CX_PEL, CY_PEL, CZ_PEL};
const float _C_RUL[4] = {0., CX_ULEG, -CY_ULEG, CZ_ULEG};
const float _C_RLL[4] = {0., CX_LLEG, -CY_LLEG, CZ_LLEG};
const float _C_RF[4] = {0., CX_FOOT, -CY_FOOT, CZ_FOOT};
const float _C_LUL[4] = {0., CX_ULEG, CY_ULEG, CZ_ULEG};
const float _C_LLL[4] = {0., CX_LLEG, CY_LLEG, CZ_LLEG};
const float _C_LF[4] = {0., CX_FOOT, CY_FOOT, CZ_FOOT};
const float _C_TOR[4] = {0., CX_TOR, CY_TOR, CZ_TOR};
const float _C_RUA[4] = {0., CX_UARM, -CY_UARM, CZ_UARM};
const float _C_RLA[4] = {0., CX_LARM+OFF_ELB, -CY_LARM, CZ_LARM};
const float _C_RHAND[4] = {0., CX_HAND, -CY_HAND, CZ_HAND};
const float _C_LUA[4] = {0., CX_UARM, CY_UARM, CZ_UARM};
const float _C_LLA[4] = {0., CX_LARM+OFF_ELB, CY_LARM, CZ_LARM};
const float _C_LHAND[4] = {0., CX_HAND, CY_HAND, CZ_HAND};
//--------------



float RHYRefAngleCurrent=0.0;
float RHRRefAngleCurrent=0.0;
float RHPRefAngleCurrent=0.0;
float RKNRefAngleCurrent=0.0;
float RAPRefAngleCurrent=0.0;
float RARRefAngleCurrent=0.0;
float LHYRefAngleCurrent=0.0;
float LHRRefAngleCurrent=0.0;
float LHPRefAngleCurrent=0.0;
float LKNRefAngleCurrent=0.0;
float LAPRefAngleCurrent=0.0;
float LARRefAngleCurrent=0.0;

float WSTRefAngleCurrent=0.0;
float RSPRefAngleCurrent=0.0;
float RSRRefAngleCurrent=0.0;
float RSYRefAngleCurrent=0.0;
float REBRefAngleCurrent=0.0;
float RWYRefAngleCurrent=0.0;
float RWPRefAngleCurrent=0.0;
float LSPRefAngleCurrent=0.0;
float LSRRefAngleCurrent=0.0;
float LSYRefAngleCurrent=0.0;
float LEBRefAngleCurrent=0.0;
float LWYRefAngleCurrent=0.0;
float LWPRefAngleCurrent=0.0;
float RWY2RefAngleCurrent=0.0;
float LWY2RefAngleCurrent=0.0;

float RHYEncoderValue=0.0;
float RHREncoderValue=0.0;
float RHPEncoderValue=0.0;
float RKNEncoderValue=0.0;
float RAPEncoderValue=0.0;
float RAREncoderValue=0.0;
float LHYEncoderValue=0.0;
float LHREncoderValue=0.0;
float LHPEncoderValue=0.0;
float LKNEncoderValue=0.0;
float LAPEncoderValue=0.0;
float LAREncoderValue=0.0;

float WSTEncoderValue=0.0;
float RSPEncoderValue=0.0;
float RSREncoderValue=0.0;
float RSYEncoderValue=0.0;
float REBEncoderValue=0.0;
float RWYEncoderValue=0.0;
float RWPEncoderValue=0.0;
float LSPEncoderValue=0.0;
float LSREncoderValue=0.0;
float LSYEncoderValue=0.0;
float LEBEncoderValue=0.0;
float LWYEncoderValue=0.0;
float LWPEncoderValue=0.0;
float RWY2EncoderValue=0.0;
float LWY2EncoderValue=0.0;

//-------------- For kinematics
float **_Rz_RHY_3x3, **_Rx_RHR_3x3, **_Ry_RHP_3x3, **_Ry_RKN_3x3, **_Ry_RAP_3x3, **_Rx_RAR_3x3;
float **_Rz_LHY_3x3, **_Rx_LHR_3x3, **_Ry_LHP_3x3, **_Ry_LKN_3x3, **_Ry_LAP_3x3, **_Rx_LAR_3x3;
float **_Rz_WST_3x3, **_Ry_RSP_3x3, **_Rx_RSR_3x3, **_Rz_RSY_3x3, **_Ry_REB_3x3, **_Rz_RWY_3x3, **_Ry_RWP_3x3, **_Rz_RWY2_3x3;
float **_Ry_LSP_3x3, **_Rx_LSR_3x3, **_Rz_LSY_3x3, **_Ry_LEB_3x3, **_Rz_LWY_3x3, **_Ry_LWP_3x3, **_Rz_LWY2_3x3, **_Ry_PI_3x3;
float **_dcPEL_3x3, **_dcRUL_3x3, **_dcRLL_3x3, **_dcRF_3x3;
float **_dcLUL_3x3, **_dcLLL_3x3, **_dcLF_3x3;
float **_dcTOR_3x3, **_dcRUA_3x3, **_dcRLA_3x3, **_dcRH_3x3, **_dcRS_3x3;
float **_dcLUA_3x3, **_dcLLA_3x3, **_dcLH_3x3, **_dcLS_3x3;

float _pRF_3x1[4], _pLF_3x1[4], _qRF_4x1[5], _qLF_4x1[5], _pRH_3x1[4], _pLH_3x1[4], _qRH_4x1[5], _qLH_4x1[5], _pCOM_3x1[4];
float _pRS_3x1[4], _pLS_3x1[4], _qRS_4x1[5], _qLS_4x1[5];
float _pRWR_3x1[4], _pLWR_3x1[4], _pRANK_3x1[4], _pLANK_3x1[4], _qRWR_4x1[5], _qLWR_4x1[5];
float _pRF_L_3x1[4], _pLF_L_3x1[4], _qRF_L_4x1[5], _qLF_L_4x1[5], _pRH_L_3x1[4], _pLH_L_3x1[4], _qRH_L_4x1[5], _qLH_L_4x1[5], _pRS_L_3x1[4], _pLS_L_3x1[4], _qRS_L_4x1[5], _qLS_L_4x1[5], _qRWR_L_4x1[5], _qLWR_L_4x1[5]; // local coordinates
float _pRWR_L_3x1[4], _pLWR_L_3x1[4], _pRANK_L_3x1[4], _pLANK_L_3x1[4]; // local coordinates
char _FKineUpdatedFlag;
char _PassiveUpdatedFlag; 

float **_jRF_6x33, **_jLF_6x33, **_jCOM_3x33, **_jRH_6x33, **_jLH_6x33, **_jRS_6x33, **_jLS_6x33, **_jRWR_6x33, **_jLWR_6x33;
float **_jRF_old_6x33, **_jLF_old_6x33, **_jCOM_old_3x33, **_jRH_old_6x33, **_jLH_old_6x33, **_jRS_old_6x33, **_jLS_old_6x33, **_jRWR_old_6x33, **_jLWR_old_6x33;
float **_jRF_old2_6x33, **_jLF_old2_6x33, **_jCOM_old2_3x33, **_jRH_old2_6x33, **_jLH_old2_6x33, **_jRS_old2_6x33, **_jLS_old2_6x33, **_jRWR_old2_6x33, **_jLWR_old2_6x33;
float **_jRFp_6x33, **_jLFp_6x33, **_jCOMp_3x33, **_jRHp_6x33, **_jLHp_6x33, **_jRSp_6x33, **_jLSp_6x33, **_jRWRp_6x33, **_jLWRp_6x33;

float **_jT1_33x33, **_jT1inv_33x33, **_N1_33x33;
float **_jT2_33x33, **_jT2inv_33x33;
//---------------


//--------------------- temporary variables
float **_TEMP1_34x34, **_TEMP2_34x34, **_TEMP3_34x34, **_TEMP4_34x34;
float **_EYE_33;
//----------------------


//--------------------- Joint variables
float _Q_34x1[35]; // [x,y,z,qPEL[4],wst,rhy,rhr,rhp,rkn,rap,rar,lhy,lhr,lhp,lkn,lap,lar, rsp,rsr,rsy,reb,rwy,rwp, lsp,lsr,lsy,leb,lwy,lwp,rwy2, lwy2]
float _Qp_33x1[34]; // [x,y,z,wPEL[3],wst,rhy,rhr,rhp,rkn,rap,rar,lhy,lhr,lhp,lkn,lap,lar, rsp,rsr,rsy,reb,rwy,rwp, lsp,lsr,lsy,leb,lwy,lwp,rwyp2, lwyp2]
//----------------------


//--------------------- initial values
float _Q0_34x1[35];
float _pPEL0_3x1[4], _pCOM0_3x1[4];
float _pRF0_3x1[4], _pLF0_3x1[4], _qRF0_4x1[5], _qLF0_4x1[5];
float _pRH0_3x1[4], _pLH0_3x1[4], _qRH0_4x1[5], _qLH0_4x1[5];
//---------------------


//---------------------- For walking pattern
float _pZMP_3x1[4], _copRF_3x1[4], _copLF_3x1[4];
float _copRF_filt_2x1[3], _copLF_filt_2x1[3];
char _FSP, _HSP;		// Foot Supporting Phase, Hand Supporting Phase
float _fRS_3x1[4], _fLS_3x1[4];
//---------------------


//---------------------- For the offline test
OFFLINE_TRAJ _offline_traj;
float _log_temp[30];
float _Xhat_DSP_F_4x1[5], _Xhat_DSP_S_4x1[5], _Xhat_SSP_S_4x1[5], _Xhat_SSP_F_4x1[5];
float _Ahat_DSP_F_BI_4x4[5][5], _Bhat_DSP_F_BI_4x1[5], _Lo_DSP_F_BI_4x1[5], _Kfb_DSP_F_BI_1x4[5], _Bss_DSP_F_BI_4x1[5];
float _Ahat_DSP_S_BI_4x4[5][5], _Bhat_DSP_S_BI_4x1[5], _Lo_DSP_S_BI_4x1[5], _Kfb_DSP_S_BI_1x4[5], _Bss_DSP_S_BI_4x1[5];
float _Ahat_SSP_S_BI_4x4[5][5], _Bhat_SSP_S_BI_4x1[5], _Lo_SSP_S_BI_4x1[5], _Kfb_SSP_S_BI_1x4[5], _Bss_SSP_S_BI_4x1[5];
float _Ahat_SSP_F_BI_4x4[5][5], _Bhat_SSP_F_BI_4x1[5], _Lo_SSP_F_BI_4x1[5], _Kfb_SSP_F_BI_1x4[5], _Bss_SSP_F_BI_4x1[5];

float _Ahat_DSP_F_QUAD_4x4[5][5], _Bhat_DSP_F_QUAD_4x1[5], _Lo_DSP_F_QUAD_4x1[5], _Kfb_DSP_F_QUAD_1x4[5], _Bss_DSP_F_QUAD_4x1[5];
float _Ahat_DSP_S_QUAD_4x4[5][5], _Bhat_DSP_S_QUAD_4x1[5], _Lo_DSP_S_QUAD_4x1[5], _Kfb_DSP_S_QUAD_1x4[5], _Bss_DSP_S_QUAD_4x1[5];
float _Ahat_SSP_S_QUAD_4x4[5][5], _Bhat_SSP_S_QUAD_4x1[5], _Lo_SSP_S_QUAD_4x1[5], _Kfb_SSP_S_QUAD_1x4[5], _Bss_SSP_S_QUAD_4x1[5];
float _Ahat_SSP_F_QUAD_4x4[5][5], _Bhat_SSP_F_QUAD_4x1[5], _Lo_SSP_F_QUAD_4x1[5], _Kfb_SSP_F_QUAD_1x4[5], _Bss_SSP_F_QUAD_4x1[5];

float _Ahat_DSP_F_DRC_4x4[5][5], _Bhat_DSP_F_DRC_4x1[5], _Lo_DSP_F_DRC_4x1[5], _Kfb_DSP_F_DRC_1x4[5], _Bss_DSP_F_DRC_4x1[5];
float _Ahat_DSP_S_DRC_4x4[5][5], _Bhat_DSP_S_DRC_4x1[5], _Lo_DSP_S_DRC_4x1[5], _Kfb_DSP_S_DRC_1x4[5], _Bss_DSP_S_DRC_4x1[5];
float _Ahat_SSP_S_DRC_4x4[5][5], _Bhat_SSP_S_DRC_4x1[5], _Lo_SSP_S_DRC_4x1[5], _Kfb_SSP_S_DRC_1x4[5], _Bss_SSP_S_DRC_4x1[5];
float _Ahat_SSP_F_DRC_4x4[5][5], _Bhat_SSP_F_DRC_4x1[5], _Lo_SSP_F_DRC_4x1[5], _Kfb_SSP_F_DRC_1x4[5], _Bss_SSP_F_DRC_4x1[5];
//----------------------


//--------------------- DRC mode
char _DRC_walking_mode;
//---------------------

//---------------------- torque control
float _gain_gravity[34], _gain_task[34], _para_viscous[34][2];
//----------------------









//-----------------------  added by Inhyeok Kim
double **_nrTEMP1_34x34, **_nrTEMP2_34x34, **_nrTEMP3_34x34;
double **_A_56x56, **_Aa_27x27, **_A0_54x27, **_N_27x27, **_EYE_34;
int _izrov_54x1[55], _iposv_54x1[55], _l1_55x1[56], _l3_54x1[55], _ia_54x1[55];
double _ba_27x1[28], _b0_54x1[55], _d_27x1[28], _grad_27x1[28], _lambda_27x1[28], _Ax_54x1[55], _temp_54x1[55], _temp2_54x1[55]; 
int _indxc[35],_indxr[35],_ipiv[35];
double _rv1[35];
double **_nrTEMP4_34x34, **_nrTEMP5_34x34, _s_33x1[34];
extern float** _EYE_33;
//-----------------------





int InitGlobalNRutilVariables(void)  // added by Inhyeok Kim
{
	int i,j;

	_A_56x56 = dmatrix(1,56,1,56);
	_Aa_27x27 = dmatrix(1,27,1,27);
	_A0_54x27 = dmatrix(1,54,1,27);
	_N_27x27 = dmatrix(1,27,1,27);
	_EYE_34 = dmatrix(1,34,1,34);
	_nrTEMP1_34x34 = dmatrix(1,34,1,34);
	_nrTEMP2_34x34 = dmatrix(1,34,1,34);
	_nrTEMP3_34x34 = dmatrix(1,34,1,34);
	_nrTEMP4_34x34 = dmatrix(1,34,1,34);
	_nrTEMP5_34x34 = dmatrix(1,34,1,34);
	

	for(i=1; i<=34; i++)
		for(j=1; j<=34; j++)
		{
			if(i == j)
				_EYE_34[i][j] = 1.;
			else
				_EYE_34[i][j] = 0.;	
		}

	return 0;
}

//---------------------------------------

int FreeGlobalNRutilVariables(void)  // added by Inhyeok Kim
{
	free_dmatrix(_A_56x56,1,56,1,56);
	free_dmatrix(_Aa_27x27,1,27,1,27);
	free_dmatrix(_A0_54x27,1,54,1,27);
	free_dmatrix(_N_27x27,1,27,1,27);
	free_dmatrix(_EYE_34,1,34,1,34);
	free_dmatrix(_nrTEMP1_34x34, 1,34,1,34);
	free_dmatrix(_nrTEMP2_34x34, 1,34,1,34);
	free_dmatrix(_nrTEMP3_34x34, 1,34,1,34);
	free_dmatrix(_nrTEMP4_34x34, 1,34,1,34);
	free_dmatrix(_nrTEMP5_34x34, 1,34,1,34);

	return 0;
}

//---------------------------------------

void nrerror(char error_text[])
/* Numerical Recipes standard error handler */
{
	printf("\nNumerical Recipes run-time error...\n");
	printf("%s\n",error_text);
}

//---------------------------------------

float *vector(unsigned int nl, unsigned int nh)
/* allocate a float vector with subscript range v[nl..nh] */
{
	float *v;

	v=(float *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(float)));
	if (!v) nrerror("allocation failure in vector()");
	return v-nl+NR_END;
}

//---------------------------------------

int *ivector(unsigned int nl, unsigned int nh)
/* allocate an int vector with subscript range v[nl..nh] */
{
	int *v;

	v=(int *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(int)));
	if (!v) nrerror("allocation failure in ivector()");
	return v-nl+NR_END;
}

//---------------------------------------

unsigned char *cvector(long nl, long nh)
/* allocate an unsigned char vector with subscript range v[nl..nh] */
{
	unsigned char *v;

	v=(unsigned char *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(unsigned char)));
	if (!v) nrerror("allocation failure in cvector()");
	return v-nl+NR_END;
}

//---------------------------------------

unsigned long *lvector(long nl, long nh)
/* allocate an unsigned long vector with subscript range v[nl..nh] */
{
	unsigned long *v;

	v=(unsigned long *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(long)));
	if (!v) nrerror("allocation failure in lvector()");
	return v-nl+NR_END;
}

//---------------------------------------

double *dvector(long nl, long nh)
/* allocate a double vector with subscript range v[nl..nh] */
{
	double *v;

	v=(double *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(double)));
	if (!v) nrerror("allocation failure in dvector()");
	return v-nl+NR_END;
}

//---------------------------------------

float **matrix(unsigned int nrl, unsigned int nrh, unsigned int ncl, unsigned int nch)
/* allocate a float matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	unsigned int i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	float **m;

	/* allocate pointers to rows */
	m=(float **) malloc((size_t)((nrow+NR_END)*sizeof(float*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;

	/* allocate rows and set pointers to them */
	m[nrl]=(float *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(float)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

//---------------------------------------

double **dmatrix(long nrl, long nrh, long ncl, long nch)
/* allocate a double matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	double **m;

	/* allocate pointers to rows */
	m=(double **) malloc((size_t)((nrow+NR_END)*sizeof(double*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;

	/* allocate rows and set pointers to them */
	m[nrl]=(double *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(double)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

//---------------------------------------

int **imatrix(long nrl, long nrh, long ncl, long nch)
/* allocate a int matrix with subscript range m[nrl..nrh][ncl..nch] */
{
	long i, nrow=nrh-nrl+1,ncol=nch-ncl+1;
	int **m;

	/* allocate pointers to rows */
	m=(int **) malloc((size_t)((nrow+NR_END)*sizeof(int*)));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m += NR_END;
	m -= nrl;


	/* allocate rows and set pointers to them */
	m[nrl]=(int *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(int)));
	if (!m[nrl]) nrerror("allocation failure 2 in matrix()");
	m[nrl] += NR_END;
	m[nrl] -= ncl;

	for(i=nrl+1;i<=nrh;i++) m[i]=m[i-1]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

//---------------------------------------

float **submatrix(float **a, long oldrl, long oldrh, long oldcl, long oldch,
	long newrl, long newcl)
/* point a submatrix [newrl..][newcl..] to a[oldrl..oldrh][oldcl..oldch] */
{
	long i,j,nrow=oldrh-oldrl+1,ncol=oldcl-newcl;
	float **m;

	/* allocate array of pointers to rows */
	m=(float **) malloc((size_t) ((nrow+NR_END)*sizeof(float*)));
	if (!m) nrerror("allocation failure in submatrix()");
	m += NR_END;
	m -= newrl;

	/* set pointers to rows */
	for(i=oldrl,j=newrl;i<=oldrh;i++,j++) m[j]=a[i]+ncol;

	/* return pointer to array of pointers to rows */
	return m;
}

//---------------------------------------

float **convert_matrix(float *a, long nrl, long nrh, long ncl, long nch)
/* allocate a float matrix m[nrl..nrh][ncl..nch] that points to the matrix
declared in the standard C manner as a[nrow][ncol], where nrow=nrh-nrl+1
and ncol=nch-ncl+1. The routine should be called with the address
&a[0][0] as the first argument. */
{
	long i,j,nrow=nrh-nrl+1,ncol=nch-ncl+1;
	float **m;

	/* allocate pointers to rows */
	m=(float **) malloc((size_t) ((nrow+NR_END)*sizeof(float*)));
	if (!m) nrerror("allocation failure in convert_matrix()");
	m += NR_END;
	m -= nrl;

	/* set pointers to rows */
	m[nrl]=a-ncl;
	for(i=1,j=nrl+1;i<nrow;i++,j++) m[j]=m[j-1]+ncol;
	/* return pointer to array of pointers to rows */
	return m;
}

//---------------------------------------

float ***f3tensor(long nrl, long nrh, long ncl, long nch, long ndl, long ndh)
/* allocate a float 3tensor with range t[nrl..nrh][ncl..nch][ndl..ndh] */
{
	long i,j,nrow=nrh-nrl+1,ncol=nch-ncl+1,ndep=ndh-ndl+1;
	float ***t;

	/* allocate pointers to pointers to rows */
	t=(float ***) malloc((size_t)((nrow+NR_END)*sizeof(float**)));
	if (!t) nrerror("allocation failure 1 in f3tensor()");
	t += NR_END;
	t -= nrl;

	/* allocate pointers to rows and set pointers to them */
	t[nrl]=(float **) malloc((size_t)((nrow*ncol+NR_END)*sizeof(float*)));
	if (!t[nrl]) nrerror("allocation failure 2 in f3tensor()");
	t[nrl] += NR_END;
	t[nrl] -= ncl;

	/* allocate rows and set pointers to them */
	t[nrl][ncl]=(float *) malloc((size_t)((nrow*ncol*ndep+NR_END)*sizeof(float)));
	if (!t[nrl][ncl]) nrerror("allocation failure 3 in f3tensor()");
	t[nrl][ncl] += NR_END;
	t[nrl][ncl] -= ndl;

	for(j=ncl+1;j<=nch;j++) t[nrl][j]=t[nrl][j-1]+ndep;
	for(i=nrl+1;i<=nrh;i++) {
		t[i]=t[i-1]+ncol;
		t[i][ncl]=t[i-1][ncl]+ncol*ndep;
		for(j=ncl+1;j<=nch;j++) t[i][j]=t[i][j-1]+ndep;
	}

	/* return pointer to array of pointers to rows */
	return t;
}

//---------------------------------------

void free_vector(float *v, unsigned int nl, unsigned int nh)
/* free a float vector allocated with vector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_ivector(int *v, unsigned int nl, unsigned int nh)
/* free an int vector allocated with ivector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_cvector(unsigned char *v, long nl, long nh)
/* free an unsigned char vector allocated with cvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_lvector(unsigned long *v, long nl, long nh)
/* free an unsigned long vector allocated with lvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_dvector(double *v, long nl, long nh)
/* free a double vector allocated with dvector() */
{
	free((FREE_ARG) (v+nl-NR_END));
}

//---------------------------------------

void free_matrix(float **m, unsigned int nrl, unsigned int nrh, unsigned int ncl, unsigned int nch)
/* free a float matrix allocated by matrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}

//---------------------------------------

void free_dmatrix(double **m, long nrl, long nrh, long ncl, long nch)
/* free a double matrix allocated by dmatrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}

//---------------------------------------

void free_imatrix(int **m, long nrl, long nrh, long ncl, long nch)
/* free an int matrix allocated by imatrix() */
{
	free((FREE_ARG) (m[nrl]+ncl-NR_END));
	free((FREE_ARG) (m+nrl-NR_END));
}

//---------------------------------------

void free_submatrix(float **b, long nrl, long nrh, long ncl, long nch)
/* free a submatrix allocated by submatrix() */
{
	free((FREE_ARG) (b+nrl-NR_END));
}

//---------------------------------------

void free_convert_matrix(float **b, long nrl, long nrh, long ncl, long nch)
/* free a matrix allocated by convert_matrix() */
{
	free((FREE_ARG) (b+nrl-NR_END));
}

//---------------------------------------

void free_f3tensor(float ***t, long nrl, long nrh, long ncl, long nch,
	long ndl, long ndh)
/* free a float f3tensor allocated by f3tensor() */
{
	free((FREE_ARG) (t[nrl][ncl]+ndl-NR_END));
	free((FREE_ARG) (t[nrl]+ncl-NR_END));
	free((FREE_ARG) (t+nrl-NR_END));
}

//---------------------------------------

void simp1(double **a, int mm, int ll[], int nll, int iabf, int *kp, double *bmax)
{
	int k;
	double test;

	if (nll <= 0)
		*bmax=0.0;
	else {
		*kp=ll[1];
		*bmax=a[mm+1][*kp+1];
		for (k=2;k<=nll;k++) {
			if (iabf == 0)
				test=a[mm+1][ll[k]+1]-(*bmax);
			else
				test = (double)(fabs(a[mm+1][ll[k]+1])-fabs(*bmax));
			if (test > 0.) {
				*bmax=a[mm+1][ll[k]+1];
				*kp=ll[k];
			}
		}
	}
}

//---------------------------------------

void simp2(double **a, int m, int n, int *ip, int kp)
{
	int k,i;
	double qp,q0,q,q1;

	*ip=0;
	for (i=1;i<=m;i++)
		if (a[i+1][kp+1] < -EPS) break;
	if (i>m) return;
	q1 = -a[i+1][1]/a[i+1][kp+1];
	*ip=i;
	for (i=*ip+1;i<=m;i++) {
		if (a[i+1][kp+1] < -EPS) {
			q = -a[i+1][1]/a[i+1][kp+1];
			if (q < q1) {
				*ip=i;
				q1=q;
			} else if (q == q1) {
				for (k=1;k<=n;k++) {
					qp = -a[*ip+1][k+1]/a[*ip+1][kp+1];
					q0 = -a[i+1][k+1]/a[i+1][kp+1];
					if (q0 != qp) break;
				}
				if (q0 < qp) *ip=i;
			}
		}
	}
}

//---------------------------------------

void simp3(double **a, int i1, int k1, int ip, int kp)
{
	int kk,ii;
	double piv;

	piv=1.0f/a[ip+1][kp+1];
	for (ii=1;ii<=i1+1;ii++)
		if (ii-1 != ip) {
			a[ii][kp+1] *= piv;
			for (kk=1;kk<=k1+1;kk++)
				if (kk-1 != kp)
					a[ii][kk] -= a[ip+1][kk]*a[ii][kp+1];
		}
	for (kk=1;kk<=k1+1;kk++)
		if (kk-1 != kp) a[ip+1][kk] *= -piv;
	a[ip+1][kp+1]=piv;
}



//---------------------------------------
int mult_mv(const float **matrix_mxn, unsigned int m, unsigned int n, const float *vector_nx1, float *result_mx1)	// matrix x vector  //added by Inhyeok Kim
{
	unsigned int i, j;
	float sum = 0.;

	for(i=1; i<=m; i++)
	{
		sum = 0.;		
		for(j=1; j<=n; j++)
			sum += matrix_mxn[i][j]*vector_nx1[j];
		
		result_mx1[i] = sum;
	}
	return 0;
}


//---------------------------------------
int mult_mv_(const double **matrix_mxn, unsigned int m, unsigned int n, const double *vector_nx1, double *result_mx1)	// matrix x vector //added by Inhyeok Kim
{
	unsigned int i, j;
	double sum = 0.;

	for(i=1; i<=m; i++)
	{
		sum = 0.;		
		for(j=1; j<=n; j++)
			sum += matrix_mxn[i][j]*vector_nx1[j];
		
		result_mx1[i] = sum;
	}
	return 0;
}

//---------------------------------------
int mult_mm(const float **matrix_m1xn1, unsigned int m1, unsigned int n1, const float *matrix_n1xn2[], unsigned int n2, float *result_m1xn2[]) // matrix x matrix //added by Inhyeok Kim
{
	unsigned int i,j,k;
	float sum = 0.;

	for(i=1; i<=m1; i++)
	{				
		for(j=1; j<=n2; j++)
		{
			sum = 0.;
			for(k=1; k<=n1; k++)
				sum += matrix_m1xn1[i][k]*matrix_n1xn2[k][j];
			
			result_m1xn2[i][j] = sum;
		}
	}
	return 0;
}

//---------------------------------------
int mult_mm_(const double **matrix_m1xn1, unsigned int m1, unsigned int n1, const double *matrix_n1xn2[], unsigned int n2, double *result_m1xn2[]) // matrix x matrix //added by Inhyeok Kim
{
	unsigned int i,j,k;
	double sum = 0.;

	for(i=1; i<=m1; i++)
	{				
		for(j=1; j<=n2; j++)
		{
			sum = 0.;
			for(k=1; k<=n1; k++)
				sum += matrix_m1xn1[i][k]*matrix_n1xn2[k][j];
			
			result_m1xn2[i][j] = sum;
		}
	}
	return 0;
}

//---------------------------------------
float dot(const float *vector1_nx1, unsigned int n, const float *vector2_nx1) // vector dot vector  //added by Inhyeok Kim
{
	unsigned int i;
	float sum = 0.;

	for(i=1; i<=n; i++)
		sum += vector1_nx1[i]*vector2_nx1[i];

	return sum;
}

//---------------------------------------
int mult_vvt(float scalar, const float *vector_mx1, unsigned int m, const float *vector_nx1, unsigned int n, float **result_mxn) // vector * vector transpose //added by Inhyeok Kim
{
	unsigned int i, j;

	if(scalar != 1.f)
	{
		for(i=1; i<=m; i++)
			for(j=1; j<=n; j++)
				result_mxn[i][j] = vector_mx1[i]*vector_nx1[j];
	}
	else
	{
		for(i=1; i<=m; i++)
			for(j=1; j<=n; j++)
				result_mxn[i][j] = scalar*vector_mx1[i]*vector_nx1[j];		
	}
	

	return 0;

}

//---------------------------------------
int mult_vtm(float scalar, const float *vector_mx1, unsigned int m, const float **matrix_mxn, unsigned int n, float *result_nx1) // scalar * vector transpose * matrix //added by Inhyeok Kim
{
	unsigned int i, j;
	float sum;

	if(scalar == 1.f)
	{
		for(i=1; i<=n; i++)
		{
			sum = 0.;
			for(j=1; j<=m; j++)
				sum += vector_mx1[j]*matrix_mxn[j][i];
			
			result_nx1[i] = sum;
		}
	}
	else
	{
		for(i=1; i<=n; i++)
		{
			sum = 0.;
			for(j=1; j<=m; j++)
				sum += vector_mx1[j]*matrix_mxn[j][i];
			
			result_nx1[i] = scalar*sum;
		}		
	}
	return 0;
}

//---------------------------------------
int trans(float scalar, const float **matrix_mxn, unsigned int m, unsigned int n, float **result_nxm) // scalar * matrix transpose //added by Inhyeok Kim
{
	unsigned int i,j;

	if(scalar == 1.f)
	{
		for(i=1; i<=m; i++)
			for(j=1; j<=n; j++)
				result_nxm[j][i] = matrix_mxn[i][j];	
	}
	else
	{
		for(i=1; i<=m; i++)
			for(j=1; j<=n; j++)
				result_nxm[j][i] = scalar*matrix_mxn[i][j];		
	}

	return 0;
}

//---------------------------------------
int trans_(double scalar, const double **matrix_mxn, unsigned int m, unsigned int n, double **result_nxm) // scalar * matrix transpose  //added by Inhyeok Kim
{
	unsigned int i,j;

	if(scalar == 1.)
	{
		for(i=1; i<=m; i++)
			for(j=1; j<=n; j++)
				result_nxm[j][i] = matrix_mxn[i][j];	
	}
	else
	{
		for(i=1; i<=m; i++)
			for(j=1; j<=n; j++)
				result_nxm[j][i] = scalar*matrix_mxn[i][j];		
	}

	return 0;
}

//---------------------------------------
int trans2(float scalar, float **matrix_mxn, unsigned int m, unsigned int n) // scalar * matrix transpose  //added by Inhyeok Kim
{
	float temp;
	unsigned int i,j;
	unsigned int k = (unsigned int)IMAX(m,n);

	if(scalar == 1.f)
	{
		for(i=1; i<=k; i++)
		{
			for(j=1; j<i; j++)
			{
				temp = matrix_mxn[i][j];
				matrix_mxn[i][j] = matrix_mxn[j][i];
				matrix_mxn[j][i] = temp;
			}
		}
	}
	else
	{
		for(i=1; i<=k; i++)
		{
			for(j=1; j<i; j++)
			{
				temp = matrix_mxn[i][j];
				matrix_mxn[i][j] = scalar*matrix_mxn[j][i];
				matrix_mxn[j][i] = scalar*temp;
			}
		}		
	}

	return 0;
}

//-------------------------------------
int trans2_(double scalar, double **matrix_mxn, unsigned int m, unsigned int n) // scalar * matrix transpose  ////added by Inhyeok Kim
{
	double temp;
	unsigned int i,j;
	unsigned int k = (unsigned int)IMAX(m,n);

	if(scalar == 1.)
	{
		for(i=1; i<=k; i++)
		{
			for(j=1; j<i; j++)
			{
				temp = matrix_mxn[i][j];
				matrix_mxn[i][j] = matrix_mxn[j][i];
				matrix_mxn[j][i] = temp;
			}
		}
	}
	else
	{
		for(i=1; i<=k; i++)
		{
			for(j=1; j<i; j++)
			{
				temp = matrix_mxn[i][j];
				matrix_mxn[i][j] = scalar*matrix_mxn[j][i];
				matrix_mxn[j][i] = scalar*temp;
			}
		}		
	}

	return 0;
}

//---------------------------------------
int sum_mm(const float **matrix1_mxn, unsigned int m, unsigned int n, const float **matrix2_mxn, float **result_mxn) // matrix + matrix  //added by Inhyeok Kim
{
	unsigned int i,j;

	for (i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = matrix1_mxn[i][j] + matrix2_mxn[i][j];

	return 0;
}

//---------------------------------------
int sum_smsm(float scalar1, const float **matrix1_mxn, unsigned int m, unsigned int n, float scalar2, const float **matrix2_mxn, float **result_mxn) // scalar1*matrix + scalar2*matrix  //added by Inhyeok Kim
{
	unsigned int i,j;

	for (i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = scalar1*matrix1_mxn[i][j] + scalar2*matrix2_mxn[i][j];

	return 0;
}

//---------------------------------------
int sum_smsm_(double scalar1, const double **matrix1_mxn, unsigned int m, unsigned int n, double scalar2, const double **matrix2_mxn, double **result_mxn) // scalar1*matrix + scalar2*matrix  //added by Inhyeok Kim
{
	unsigned int i,j;

	for (i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = scalar1*matrix1_mxn[i][j] + scalar2*matrix2_mxn[i][j];

	return 0;
}

//---------------------------------------
int diff_mm(const float **matrix1_mxn, unsigned int m, unsigned int n, const float **matrix2_mxn, float **result_mxn) // matrix - matrix  //added by Inhyeok Kim
{
	unsigned int i,j;

	for (i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = matrix1_mxn[i][j] - matrix2_mxn[i][j];

	return 0;
}

//---------------------------------------
int diff_mm_(const double **matrix1_mxn, unsigned int m, unsigned int n, const double **matrix2_mxn, double **result_mxn) // matrix - matrix //added by Inhyeok Kim
{
	unsigned int i,j;

	for (i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = matrix1_mxn[i][j] - matrix2_mxn[i][j];

	return 0;
}

//---------------------------------------
int sum_vv(const float *vector1_mx1, unsigned int m, const float *vector2_mx1, float *result_mx1) // vector + vector  //added by Inhyeok Kim
{
	unsigned int i;

	for(i=1; i<=m; i++)
		result_mx1[i] = vector1_mx1[i] + vector2_mx1[i];

	return 0;
}

//---------------------------------------
int diff_vv(const float *vector1_mx1, unsigned int m, const float *vector2_mx1, float *result_mx1) // vector - vector  //added by Inhyeok Kim
{
	unsigned int i;

	for(i=1; i<=m; i++)
		result_mx1[i] = vector1_mx1[i] - vector2_mx1[i];

	return 0;
}

//---------------------------------------
int cross(float scalar, const float vector1_3x1[4], const float vector2_3x1[4], float result_3x1[4])	// scalar * vector cross product  //added by Inhyeok Kim
{
	if(scalar == 1.f)
	{			
		result_3x1[1] = (vector1_3x1[2]*vector2_3x1[3] - vector1_3x1[3]*vector2_3x1[2]);
		result_3x1[2] = (vector1_3x1[3]*vector2_3x1[1] - vector1_3x1[1]*vector2_3x1[3]);
		result_3x1[3] = (vector1_3x1[1]*vector2_3x1[2] - vector1_3x1[2]*vector2_3x1[1]);
	}
	else
	{
		result_3x1[1] = scalar*(vector1_3x1[2]*vector2_3x1[3] - vector1_3x1[3]*vector2_3x1[2]);
		result_3x1[2] = scalar*(vector1_3x1[3]*vector2_3x1[1] - vector1_3x1[1]*vector2_3x1[3]);
		result_3x1[3] = scalar*(vector1_3x1[1]*vector2_3x1[2] - vector1_3x1[2]*vector2_3x1[1]);
	}

	return 0;
}

//---------------------------------------
int mult_sv(const float *vector_nx1, unsigned int n, float scalar, float *result_nx1)		// scalar * vector  //added by Inhyeok Kim
{
	unsigned int i;

	for(i=1; i<=n; i++)
		result_nx1[i] = scalar*vector_nx1[i];

	return 0;
}

//---------------------------------------
int mult_sm(const float **matrix_mxn, unsigned int m, unsigned int n, float scalar, float **result_mxn) // scalar * matrix  //added by Inhyeok Kim
{
	unsigned int i,j;

	for(i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = scalar*matrix_mxn[i][j];

	return 0;
}

//---------------------------------------
int subs_v(const float *vector_nx1, unsigned int n, float *result_nx1)		// result_nx1 = vector_nx1  //added by Inhyeok Kim
{
	unsigned int i;
	
	for(i=1; i<=n; i++)
		result_nx1[i] = vector_nx1[i];

	return 0;
}

//---------------------------------------
int subs_sv(float scalar, const float *vector_nx1, unsigned int n, float *result_nx1)		// result_nx1 = scalar*vector_nx1  //added by Inhyeok Kim
{
	unsigned int i;
	
	for(i=1; i<=n; i++)
		result_nx1[i] = scalar*vector_nx1[i];

	return 0;
}

//---------------------------------------
int subs_sv_(double scalar, const double *vector_nx1, unsigned int n, double *result_nx1)		// result_nx1 = scalar*vector_nx1  //added by Inhyeok Kim
{
	unsigned int i;
	
	for(i=1; i<=n; i++)
		result_nx1[i] = scalar*vector_nx1[i];

	return 0;
}

//---------------------------------------
int subs_m(const float **matrix_mxn, unsigned int m, unsigned int n, float **result_mxn) // result_mxn = matrix_mxn   //added by Inhyeok Kim
{

	unsigned int i,j;

	for(i=1; i<=m; i++)
		for(j=1; j<=n; j++)
			result_mxn[i][j] = matrix_mxn[i][j];

	return 0;
}

//---------------------------------------
int sum_svsv(float scalar1, const float *vector1_nx1, unsigned int n, float scalar2, const float *vector2_nx1, float *result_nx1) // s1*v1 + s2*v2  //added by Inhyeok Kim
{
	unsigned int i;

	for(i=1; i<=n; i++)
		result_nx1[i] = scalar1*vector1_nx1[i] + scalar2*vector2_nx1[i];

	return 0;
}

//---------------------------------------
int sum_svsv_(double scalar1, const double *vector1_nx1, unsigned int n, double scalar2, const double *vector2_nx1, double *result_nx1) // s1*v1 + s2*v2  //added by Inhyeok Kim
{
	unsigned int i;

	for(i=1; i<=n; i++)
		result_nx1[i] = scalar1*vector1_nx1[i] + scalar2*vector2_nx1[i];

	return 0;
}

//---------------------------------------
int mult_smv(float scalar, const float **matrix_mxn, unsigned int m, unsigned int n, const float *vector_nx1, float *result_mx1)	// scalar * matrix * vector  //added by Inhyeok Kim
{
	unsigned int i, j;
	float sum = 0.;

	for(i=1; i<=m; i++)
	{
		sum = 0.;		
		for(j=1; j<=n; j++)
			sum += matrix_mxn[i][j]*vector_nx1[j];
		
		result_mx1[i] = scalar*sum;
	}
	return 0;
}

//---------------------------------------
int mult_smv_(double scalar, const double **matrix_mxn, unsigned int m, unsigned int n, const double *vector_nx1, double *result_mx1)	// scalar * matrix * vector  //added by Inhyeok Kim
{
	unsigned int i, j;
	double sum = 0.;

	for(i=1; i<=m; i++)
	{
		sum = 0.;		
		for(j=1; j<=n; j++)
			sum += matrix_mxn[i][j]*vector_nx1[j];
		
		result_mx1[i] = scalar*sum;
	}
	return 0;
}

//---------------------------------------
int mult_smm(float scalar, const float **matrix_m1xn1, unsigned int m1, unsigned int n1, const float **matrix_n1xn2, unsigned int n2, float **result_m1xn2) // scalar*matrix*matrix  //added by Inhyeok Kim
{
	unsigned int i,j,k;
	float sum = 0.;

	for(i=1; i<=m1; i++)
	{				
		for(j=1; j<=n2; j++)
		{
			sum = 0.;
			for(k=1; k<=n1; k++)
				sum += matrix_m1xn1[i][k]*matrix_n1xn2[k][j];
			
			result_m1xn2[i][j] = scalar*sum;
		}
	}
	return 0;
}

//---------------------------------------
int accu_vv(float scalar, const float *vector_n1x1, unsigned int n1, const float *vector_n2x1, unsigned int n2, float *result_n1n2x1) // accumulate two vectors  //added by Inhyeok Kim
{
	unsigned int i;

	if(scalar == 1.f)
	{
		for(i=1; i<=n1; i++)
			result_n1n2x1[i] = vector_n1x1[i];

		for(i=1; i<=n2; i++)
			result_n1n2x1[n1+i] = vector_n2x1[i];
	}
	else
	{
		for(i=1; i<=n1; i++)
			result_n1n2x1[i] = scalar*vector_n1x1[i];

		for(i=1; i<=n2; i++)
			result_n1n2x1[n1+i] = scalar*vector_n2x1[i];
	}

	return 0;
}

//---------------------------------------
int aug_vv(float scalar, const float *vector1_nx1, unsigned int n, const float *vector2_nx1, float **result_nx2) // augment a vector  //added by Inhyeok Kim
{
	unsigned int i;

	if(scalar == 1.f)
	{
		for(i=1; i<=n; i++)
		{
			result_nx2[i][1] = vector1_nx1[i];
			result_nx2[i][2] = vector2_nx1[i];
		}
	}
	else
	{
		for(i=1; i<=n; i++)
		{
			result_nx2[i][1] = scalar*vector1_nx1[i];
			result_nx2[i][2] = scalar*vector2_nx1[i];
		}
	}

	

	return 0;
}

//---------------------------------------
int accu_mm(float scalar, const float **matrix_m1xn, unsigned int m1, unsigned int n, const float **matrix_m2xn, unsigned int m2, float **result_m1m2xn) // accumulate two matrices  //added by Inhyeok Kim
{
	unsigned int i, j;

	if(scalar == 1.f)
	{		
		for(j=1; j<=n; j++)
		{
			for(i=1; i<=m1; i++)		
				result_m1m2xn[i][j] = matrix_m1xn[i][j];

			for(i=1; i<=m2; i++)
				result_m1m2xn[m1+i][j] = matrix_m2xn[i][j];
		}
	}
	else
	{		
		for(j=1; j<=n; j++)
		{
			for(i=1; i<=m1; i++)		
				result_m1m2xn[i][j] = scalar*matrix_m1xn[i][j];

			for(i=1; i<=m2; i++)
				result_m1m2xn[m1+i][j] = scalar*matrix_m2xn[i][j];
		}
	}

	return 0;
}

//---------------------------------------
int aug_mm(float scalar, const float **matrix_mxn1, unsigned int m, unsigned int n1, const float **matrix_mxn2, unsigned int n2, float **result_mxn1n2) // augment a matrix  //added by Inhyeok Kim
{
	unsigned int i, j;

	if(scalar == 1.f)
	{
		for(i=1; i<=m; i++)
		{
			for(j=1; j<=n1; j++)
				result_mxn1n2[i][j] = matrix_mxn1[i][j];

			for(j=1; j<=n2; j++)
				result_mxn1n2[i][j+n1] = matrix_mxn2[i][j];
		}
	}
	else
	{
		for(i=1; i<=m; i++)
		{
			for(j=1; j<=n1; j++)
				result_mxn1n2[i][j] = scalar*matrix_mxn1[i][j];

			for(j=1; j<=n2; j++)
				result_mxn1n2[i][j+n1] = scalar*matrix_mxn2[i][j];
		}
	}


	return 0;
}

//---------------------------------------
int aug_mv(float scalar, const float **matrix_mxn, unsigned int m, unsigned int n, const float *vector_mx1, float **result_mxn1) // augment a matrix  //added by Inhyeok Kim
{
	unsigned int i,j;

	if(scalar == 1.f)
	{
		for(i=1; i<=m; i++)
		{
			for(j=1; j<=n; j++)
				result_mxn1[i][j] = matrix_mxn[i][j];

			result_mxn1[i][n+1] = vector_mx1[i];
		}
	}
	else
	{
		for(i=1; i<=m; i++)
		{
			for(j=1; j<=n; j++)
				result_mxn1[i][j] = scalar*matrix_mxn[i][j];

			result_mxn1[i][n+1] = scalar*vector_mx1[i];
		}
	}

	
	return 0;
}
/*
//---------------------------------------
#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int gaussj_mod(float **A, int n, float *X)	// n<=29  //gaussj.c modified by Inhyeok Kim
{	
	int i,icol,irow,j,k,l,ll;
	float big,dum,pivinv,temp;
//	int *indxc,*indxr,*ipiv;

//	indxc=ivector(1,n);
//	indxr=ivector(1,n);
//	ipiv=ivector(1,n);
	for (j=1;j<=n;j++) 
		_ipiv[j]=0;
	
	for (i=1;i<=n;i++)
	{
		big=0.;
		for (j=1;j<=n;j++)
		{
			if (_ipiv[j] != 1)
			{
				for (k=1;k<=n;k++)
				{
					if (_ipiv[k] == 0)
					{
						if (fabs(A[j][k]) >= big)
						{
							big=(float)fabs(A[j][k]);
							irow=j;	
							icol=k;
						}
					}
				}
			}
		}
		++(_ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=n;l++)
				SWAP(A[irow][l],A[icol][l])
			SWAP(X[irow],X[icol])
		}
		_indxr[i]=irow;
		_indxc[i]=icol;
		if (A[icol][icol] == 0.0) 
			return 1;
		pivinv=1.f/A[icol][icol];
		A[icol][icol]=1.f;
		for (l=1;l<=n;l++) 
			A[icol][l] *= pivinv;
		X[icol] *= pivinv;
		for (ll=1;ll<=n;ll++)
			if (ll != icol) {
				dum=A[ll][icol];
				A[ll][icol]=0.0;
				for (l=1;l<=n;l++) 
					A[ll][l] -= A[icol][l]*dum;
				X[ll] -= X[icol]*dum;
			}
	}

	for (l=n;l>=1;l--) {
		if (_indxr[l] != _indxc[l])
			for (k=1;k<=n;k++)
				SWAP(A[k][_indxr[l]],A[k][_indxc[l]]);
	}
//	free_ivector(ipiv,1,n);
//	free_ivector(indxr,1,n);
//	free_ivector(indxc,1,n);

	return 0;
}
#undef SWAP
*/
//---------------------------------------
#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int inv(float **Ai, int n)   //gaussj.c modified by Inhyeok Kim
{
//	int *indxc,*indxr,*ipiv;
	int i,icol,irow,j,k,l,ll;
	float big,dum,pivinv,temp;
	
//	indxc=ivector(1,n);
//	indxr=ivector(1,n);
//	ipiv=ivector(1,n);
	for (j=1;j<=n;j++) 
		_ipiv[j]=0;
	
	for (i=1;i<=n;i++)
	{
		big=0.;
		for (j=1;j<=n;j++)
		{
			if (_ipiv[j] != 1)
			{
				for (k=1;k<=n;k++)
				{
					if (_ipiv[k] == 0)
					{
						if (fabs(Ai[j][k]) >= big)
						{
							big=(float)fabs(Ai[j][k]);
							irow=j;	
							icol=k;
						}
					}
				}
			}
		}
		++(_ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=n;l++)
				SWAP(Ai[irow][l],Ai[icol][l])
		}
		_indxr[i]=irow;
		_indxc[i]=icol;
		if (fabs(Ai[icol][icol]) <= EPS) 
			return 1;
		pivinv=1.f/Ai[icol][icol];
		Ai[icol][icol]=1.f;
		for (l=1;l<=n;l++) 
			Ai[icol][l] *= pivinv;
		for (ll=1;ll<=n;ll++)
			if (ll != icol) {
				dum=Ai[ll][icol];
				Ai[ll][icol]=0.0;
				for (l=1;l<=n;l++) 
					Ai[ll][l] -= Ai[icol][l]*dum;
			}
	}

	for (l=n;l>=1;l--) {
		if (_indxr[l] != _indxc[l])
			for (k=1;k<=n;k++)
				SWAP(Ai[k][_indxr[l]],Ai[k][_indxc[l]]);
	}
//	free_ivector(ipiv,1,n);
//	free_ivector(indxr,1,n);
//	free_ivector(indxc,1,n);
	
	return 0;
}
#undef SWAP

//---------------------------------------

//---------------------------------------

#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int inv_(double **Ai, int n) //gaussj.c modified by Inhyeok Kim
{
//	int *indxc,*indxr,*ipiv;
	int i,icol,irow,j,k,l,ll;
	double big,dum,pivinv,temp;
	
//	indxc=ivector(1,n);
//	indxr=ivector(1,n);
//	ipiv=ivector(1,n);
	for (j=1;j<=n;j++) 
		_ipiv[j]=0;
	
	for (i=1;i<=n;i++)
	{
		big=0.;
		for (j=1;j<=n;j++)
		{
			if (_ipiv[j] != 1)
			{
				for (k=1;k<=n;k++)
				{
					if (_ipiv[k] == 0)
					{
						if (fabs(Ai[j][k]) >= big)
						{
							big=fabs(Ai[j][k]);
							irow=j;	
							icol=k;
						}
					}
				}
			}
		}
		++(_ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=n;l++)
				SWAP(Ai[irow][l],Ai[icol][l])
		}
		_indxr[i]=irow;
		_indxc[i]=icol;
		if (Ai[icol][icol] == 0.0) 
			return 1;
		pivinv=1.f/Ai[icol][icol];
		Ai[icol][icol]=1.f;
		for (l=1;l<=n;l++) 
			Ai[icol][l] *= pivinv;
		for (ll=1;ll<=n;ll++)
			if (ll != icol) {
				dum=Ai[ll][icol];
				Ai[ll][icol]=0.0;
				for (l=1;l<=n;l++) 
					Ai[ll][l] -= Ai[icol][l]*dum;
			}
	}

	for (l=n;l>=1;l--) {
		if (_indxr[l] != _indxc[l])
			for (k=1;k<=n;k++)
				SWAP(Ai[k][_indxr[l]],Ai[k][_indxc[l]]);
	}
//	free_ivector(ipiv,1,n);
//	free_ivector(indxr,1,n);
//	free_ivector(indxc,1,n);
	
	return 0;
}
#undef SWAP

//---------------------------------------
#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
int inv2(const float **A, int n, float **Ai)  //gaussj.c modified by Inhyeok Kim
{
//	int *indxc,*indxr,*ipiv;
	int i,icol,irow,j,k,l,ll;
	float big,dum,pivinv,temp;

	subs_m(A,n,n, Ai);

//	indxc=ivector(1,n);
//	indxr=ivector(1,n);
//	ipiv=ivector(1,n);
	for (j=1;j<=n;j++) 
		_ipiv[j]=0;
	
	for (i=1;i<=n;i++)
	{
		big=0.;
		for (j=1;j<=n;j++)
		{
			if (_ipiv[j] != 1)
			{
				for (k=1;k<=n;k++)
				{
					if (_ipiv[k] == 0)
					{
						if (fabs(Ai[j][k]) >= big)
						{
							big=(float)fabs(Ai[j][k]);
							irow=j;	
							icol=k;
						}
					}
				}
			}
		}
		++(_ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=n;l++)
				SWAP(Ai[irow][l],Ai[icol][l])
		}
		_indxr[i]=irow;
		_indxc[i]=icol;
		if (Ai[icol][icol] == 0.0) 
			return 1;
		pivinv=1.f/Ai[icol][icol];
		Ai[icol][icol]=1.f;
		for (l=1;l<=n;l++) 
			Ai[icol][l] *= pivinv;
		for (ll=1;ll<=n;ll++)
			if (ll != icol) {
				dum=Ai[ll][icol];
				Ai[ll][icol]=0.0;
				for (l=1;l<=n;l++) 
					Ai[ll][l] -= Ai[icol][l]*dum;
			}
	}

	for (l=n;l>=1;l--) {
		if (_indxr[l] != _indxc[l])
			for (k=1;k<=n;k++)
				SWAP(Ai[k][_indxr[l]],Ai[k][_indxc[l]]);
	}
//	free_ivector(ipiv,1,n);
//	free_ivector(indxr,1,n);
//	free_ivector(indxc,1,n);
	
	return 0;
}
#undef SWAP


//---------------------------------------

int svdcmp(float **a, int m, int n, float w[], float **v)  //svdcmp.c modified by Inhyeok Kim
{	
	int flag,i,its,j,jj,k,l,nm;
	float anorm,c,f,g,h,s,scale,x,y,z;
//	float *rv1;

//	rv1=vector(1,n);
	g=scale=anorm=0.0;
	for (i=1;i<=n;i++) 
	{
		l=i+1;
		_rv1[i]=scale*g;
		g=s=scale=0.0;
		if (i <= m) 
		{
			for (k=i;k<=m;k++) 
				scale += (float)fabs(a[k][i]);
			if (scale) 
			{
				for (k=i;k<=m;k++) 
				{
					a[k][i] /= scale;
					s += a[k][i]*a[k][i];
				}
				f=a[i][i];
				g = -(float)SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][i]=f-g;
				for (j=l;j<=n;j++) 
				{
					for (s=0.0,k=i;k<=m;k++) 
						s += a[k][i]*a[k][j];
					f=s/h;
					for (k=i;k<=m;k++) 
						a[k][j] += f*a[k][i];
				}
				for (k=i;k<=m;k++) 
					a[k][i] *= scale;
			}
		}
		w[i]=scale *g;
		g=s=scale=0.0;
		if (i <= m && i != n) 
		{
			for (k=l;k<=n;k++) 
				scale += (float)fabs(a[i][k]);
			if (scale) 
			{
				for (k=l;k<=n;k++) 
				{
					a[i][k] /= scale;
					s += a[i][k]*a[i][k];
				}
				f=a[i][l];
				g = -(float)SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][l]=f-g;
				for (k=l;k<=n;k++) 
					_rv1[k]=a[i][k]/h;
				for (j=l;j<=m;j++) 
				{
					for (s=0.0,k=l;k<=n;k++) 
						s += a[j][k]*a[i][k];
					for (k=l;k<=n;k++) 
						a[j][k] += s*(float)_rv1[k];
				}
				for (k=l;k<=n;k++) 
					a[i][k] *= scale;
			}
		}
		anorm = (float)FMAX(anorm,(float)(fabs(w[i])+fabs(_rv1[i])));
	}
	for (i=n;i>=1;i--) 
	{
		if (i < n) 
		{
			if (g) 
			{
				for (j=l;j<=n;j++)
					v[j][i]=(a[i][j]/a[i][l])/g;
				for (j=l;j<=n;j++) 
				{
					for (s=0.0,k=l;k<=n;k++) 
						s += a[i][k]*v[k][j];
					for (k=l;k<=n;k++) 
						v[k][j] += s*v[k][i];
				}
			}
			for (j=l;j<=n;j++) 
				v[i][j]=v[j][i]=0.0;
		}
		v[i][i]=1.0;
		g=(float)_rv1[i];
		l=i;
	}
	for (i=IMIN(m,n);i>=1;i--) 
	{
		l=i+1;
		g=w[i];
		for (j=l;j<=n;j++) 
			a[i][j]=0.0;
		if (g) {
			g=1.f/g;
			for (j=l;j<=n;j++) 
			{
				for (s=0.0,k=l;k<=m;k++) 
					s += a[k][i]*a[k][j];
				f=(s/a[i][i])*g;
				for (k=i;k<=m;k++) 
					a[k][j] += f*a[k][i];
			}
			for (j=i;j<=m;j++) 
				a[j][i] *= g;
		} 
		else 
			for (j=i;j<=m;j++) a[j][i]=0.0;
		++a[i][i];
	}
	for (k=n;k>=1;k--) 
	{
		for (its=1;its<=30;its++) 
		{
			flag=1;
			for (l=k;l>=1;l--) 
			{
				nm=l-1;
				//if ((float)(fabs(_rv1[l])+anorm) == anorm) 
				if (fabs(_rv1[l]) <= EPS*0.5f) 
				{
					flag=0;
					break;
				}
				//if ((float)(fabs(w[nm])+anorm) == anorm) 
				if (fabs(w[nm])<= EPS*0.5f) 
					break;
			}
			if (flag) 
			{
				c=0.0;
				s=1.0;
				for (i=l;i<=k;i++) 
				{
					f=s*(float)_rv1[i];
					_rv1[i]=c*_rv1[i];
					//if ((float)(fabs(f)+anorm) == anorm) 
					if (fabs(f)<= EPS*0.5f) 
						break;
					g=w[i];
					h=pythag(f,g);
					w[i]=h;
					h=1.f/h;
					c=g*h;
					s = -f*h;
					for (j=1;j<=m;j++) 
					{
						y=a[j][nm];
						z=a[j][i];
						a[j][nm]=y*c+z*s;
						a[j][i]=z*c-y*s;
					}
				}
			}
			z=w[k];
			if (l == k) 
			{
				if (z < 0.0) 
				{
					w[k] = -z;
					for (j=1;j<=n;j++) 
						v[j][k] = -v[j][k];
				}
				break;
			}
			if (its == 30) 
			{
				nrerror("no convergence in 30 svdcmp iterations");
				return -1;
			}
			x=w[l];
			nm=k-1;
			y=w[nm];
			g=(float)_rv1[nm];
			h=(float)_rv1[k];
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.f*h*y);
			g=pythag(f,1.f);
			f=((x-z)*(x+z)+h*((y/(f+(float)SIGN(g,f)))-h))/x;
			c=s=1.f;
			for (j=l;j<=nm;j++) 
			{
				i=j+1;
				g=(float)_rv1[i];
				y=w[i];
				h=s*g;
				g=c*g;
				z=pythag(f,h);
				_rv1[j]=z;
				c=f/z;
				s=h/z;
				f=x*c+g*s;
				g = g*c-x*s;
				h=y*s;
				y *= c;
				for (jj=1;jj<=n;jj++) 
				{
					x=v[jj][j];
					z=v[jj][i];
					v[jj][j]=x*c+z*s;
					v[jj][i]=z*c-x*s;
				}
				z=pythag(f,h);
				w[j]=z;
				if (z) 
				{
					z=1.f/z;
					c=f*z;
					s=h*z;
				}
				f=c*g+s*y;
				x=c*y-s*g;
				for (jj=1;jj<=m;jj++) 
				{
					y=a[jj][j];
					z=a[jj][i];
					a[jj][j]=y*c+z*s;
					a[jj][i]=z*c-y*s;
				}
			}
			_rv1[l]=0.0;
			_rv1[k]=f;
			w[k]=x;
		}
	}
	
//	free_vector(rv1,1,n);
	return 0;
}


int svdcmp_(double **a, int m, int n, double w[], double **v) //svdcmp.c modified by Inhyeok Kim
{	
	int flag,i,its,j,jj,k,l,nm;
	double anorm,c,f,g,h,s,scale,x,y,z;
//	float *rv1;

//	rv1=vector(1,n);
	g=scale=anorm=0.0;
	for (i=1;i<=n;i++) 
	{
		l=i+1;
		_rv1[i]=scale*g;
		g=s=scale=0.0;
		if (i <= m) 
		{
			for (k=i;k<=m;k++) 
				scale += fabs(a[k][i]);
			if (scale) 
			{
				for (k=i;k<=m;k++) 
				{
					a[k][i] /= scale;
					s += a[k][i]*a[k][i];
				}
				f=a[i][i];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][i]=f-g;
				for (j=l;j<=n;j++) 
				{
					for (s=0.0,k=i;k<=m;k++) 
						s += a[k][i]*a[k][j];
					f=s/h;
					for (k=i;k<=m;k++) 
						a[k][j] += f*a[k][i];
				}
				for (k=i;k<=m;k++) 
					a[k][i] *= scale;
			}
		}
		w[i]=scale *g;
		g=s=scale=0.0;
		if (i <= m && i != n) 
		{
			for (k=l;k<=n;k++) 
				scale += fabs(a[i][k]);
			if (scale) 
			{
				for (k=l;k<=n;k++) 
				{
					a[i][k] /= scale;
					s += a[i][k]*a[i][k];
				}
				f=a[i][l];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][l]=f-g;
				for (k=l;k<=n;k++) 
					_rv1[k]=a[i][k]/h;
				for (j=l;j<=m;j++) 
				{
					for (s=0.0,k=l;k<=n;k++) 
						s += a[j][k]*a[i][k];
					for (k=l;k<=n;k++) 
						a[j][k] += s*_rv1[k];
				}
				for (k=l;k<=n;k++) 
					a[i][k] *= scale;
			}
		}
		anorm = DMAX(anorm,(fabs(w[i])+fabs(_rv1[i])));
	}
	for (i=n;i>=1;i--) 
	{
		if (i < n) 
		{
			if (g) 
			{
				for (j=l;j<=n;j++)
					v[j][i]=(a[i][j]/a[i][l])/g;
				for (j=l;j<=n;j++) 
				{
					for (s=0.0,k=l;k<=n;k++) 
						s += a[i][k]*v[k][j];
					for (k=l;k<=n;k++) 
						v[k][j] += s*v[k][i];
				}
			}
			for (j=l;j<=n;j++) 
				v[i][j]=v[j][i]=0.0;
		}
		v[i][i]=1.0;
		g=_rv1[i];
		l=i;
	}
	for (i=IMIN(m,n);i>=1;i--) 
	{
		l=i+1;
		g=w[i];
		for (j=l;j<=n;j++) 
			a[i][j]=0.0;
		if (g) {
			g=1.f/g;
			for (j=l;j<=n;j++) 
			{
				for (s=0.0,k=l;k<=m;k++) 
					s += a[k][i]*a[k][j];
				f=(s/a[i][i])*g;
				for (k=i;k<=m;k++) 
					a[k][j] += f*a[k][i];
			}
			for (j=i;j<=m;j++) 
				a[j][i] *= g;
		} 
		else 
			for (j=i;j<=m;j++) a[j][i]=0.0;
		++a[i][i];
	}
	for (k=n;k>=1;k--) 
	{
		for (its=1;its<=30;its++) 
		{
			flag=1;
			for (l=k;l>=1;l--) 
			{
				nm=l-1;
				//if ((float)(fabs(_rv1[l])+anorm) == anorm) 
				if (fabs(_rv1[l]) <= EPS*0.5f) 
				{
					flag=0;
					break;
				}
				//if ((float)(fabs(w[nm])+anorm) == anorm) 
				if (fabs(w[nm])<= EPS*0.5f) 
					break;
			}
			if (flag) 
			{
				c=0.0;
				s=1.0;
				for (i=l;i<=k;i++) 
				{
					f=s*_rv1[i];
					_rv1[i]=c*_rv1[i];
					//if ((float)(fabs(f)+anorm) == anorm) 
					if (fabs(f)<= EPS*0.5f) 
						break;
					g=w[i];
					h=pythag_(f,g);
					w[i]=h;
					h=1.f/h;
					c=g*h;
					s = -f*h;
					for (j=1;j<=m;j++) 
					{
						y=a[j][nm];
						z=a[j][i];
						a[j][nm]=y*c+z*s;
						a[j][i]=z*c-y*s;
					}
				}
			}
			z=w[k];
			if (l == k) 
			{
				if (z < 0.0) 
				{
					w[k] = -z;
					for (j=1;j<=n;j++) 
						v[j][k] = -v[j][k];
				}
				break;
			}
			if (its == 30) 
			{
				nrerror("no convergence in 30 svdcmp iterations");
				return -1;
			}
			x=w[l];
			nm=k-1;
			y=w[nm];
			g=_rv1[nm];
			h=_rv1[k];
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.f*h*y);
			g=pythag_(f,1.f);
			f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
			c=s=1.f;
			for (j=l;j<=nm;j++) 
			{
				i=j+1;
				g=_rv1[i];
				y=w[i];
				h=s*g;
				g=c*g;
				z=pythag_(f,h);
				_rv1[j]=z;
				c=f/z;
				s=h/z;
				f=x*c+g*s;
				g = g*c-x*s;
				h=y*s;
				y *= c;
				for (jj=1;jj<=n;jj++) 
				{
					x=v[jj][j];
					z=v[jj][i];
					v[jj][j]=x*c+z*s;
					v[jj][i]=z*c-x*s;
				}
				z=pythag_(f,h);
				w[j]=z;
				if (z) 
				{
					z=1.f/z;
					c=f*z;
					s=h*z;
				}
				f=c*g+s*y;
				x=c*y-s*g;
				for (jj=1;jj<=m;jj++) 
				{
					y=a[jj][j];
					z=a[jj][i];
					a[jj][j]=y*c+z*s;
					a[jj][i]=z*c-y*s;
				}
			}
			_rv1[l]=0.0;
			_rv1[k]=f;
			w[k]=x;
		}
	}
	
//	free_vector(rv1,1,n);
	return 0;
}

//---------------------------------------

float pythag(float a, float b)
{
	float absa,absb;
	absa = (float)fabs(a);
	absb = (float)fabs(b);
	if (absa > absb) 
		return (float)(absa*sqrt(1.0+SQR(absb/absa)));
	else 
		return (float)(absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
}

double pythag_(double a, double b) //pythag.c modified by Inhyeok Kim
{
	double absa,absb;
	absa = fabs(a);
	absb = fabs(b);
	if (absa > absb) 
		return (absa*sqrt(1.0+DSQR(absb/absa)));
	else 
		return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+DSQR(absa/absb)));
}

//---------------------------------------

int pinv(const float **A, int m, int n, float **Ai)  //pseudo inverse // added by Inhyeok Kim
{
	int temp;
	int i,j;

	trans(1.f, A, m,n, (float**)_nrTEMP1_34x34);
	mult_mm(A,m,n, (const float**)_nrTEMP1_34x34,m, (float**)_nrTEMP2_34x34);
	temp = inv((float**)_nrTEMP2_34x34, m);
	mult_mm((const float**)_nrTEMP1_34x34,n,m, (const float**)_nrTEMP2_34x34,m, Ai);

	if(temp == 1)	// singularity
	{
		printf("\n singularity in pinv()\n");
		for(i=1; i<=m; i++)
		{
			for(j=1; j<=n; j++)
				printf("%f ", A[i][j]);
			printf("\n");
		}

		return -1;
	}
	return 0;
}


int pinv_SR(const float **A, int m, int n, float lambda, float **Ai) //damped least-square pseudo inverse //added by Inhyeok Kim
{
	int temp;
	int i,j;

        //double **_2nrTEMP4_34x34;

	if(m>33 || n>33)
	{
		printf("\n error - matrix dimension is out of scope!");
		return -2;
	}

        //printf("ei %d %d\n",m,n);
	for(i=1; i<=m; i++)
		for(j=1; j<=n; j++)
                        {
                        //printf("ei %d %d\n",i,j);
			_nrTEMP4_34x34[i][j] = A[i][j];
                        }

        //printf("eif\n\n");
	trans_(1.f, (const double**)_nrTEMP4_34x34, m,n, _nrTEMP1_34x34);
	mult_mm_((const double**)_nrTEMP4_34x34,m,n, (const double**)_nrTEMP1_34x34,m, _nrTEMP2_34x34);
	sum_smsm_(1.f, (const double**)_nrTEMP2_34x34,m,m, lambda, (const double**)_EYE_34, _nrTEMP2_34x34);
	temp = inv_(_nrTEMP2_34x34, m);
	mult_mm_((const double**)_nrTEMP1_34x34,n,m, (const double**)_nrTEMP2_34x34,m, _nrTEMP4_34x34);

	if(temp != 0)	// singularity
	{
		printf("\n singularity in pinv_SR()\n");
		/*for(i=1; i<=m; i++)
		{
			for(j=1; j<=n; j++)
				printf("%f ", A[i][j]);
			printf("\n");
		}
		*/
		return -1;
	}
	else
	{
		for(i=1; i<=n; i++)
			for(j=1; j<=m; j++)
				Ai[i][j] = (float)_nrTEMP4_34x34[i][j];
	}

	return 0;
}

//---------------------------------------

int pinv_svd(const float **A, int m, int n, float **Ai)  //pseudo-inverse using SVD, added by Inhyeok Kim
{
	int i,j;
	double f_temp = 10.*EPS;

	for(i=1;i<=m;i++)
		for(j=1;j<=n;j++)
			_nrTEMP4_34x34[i][j] = A[i][j];

	if(svdcmp_(_nrTEMP4_34x34,m,n,_s_33x1,_nrTEMP5_34x34) == -1)
		return -1;

	for(i=1; i<=n; i++)
	{
		if(_s_33x1[i]>=f_temp)
		{
			for(j=1;j<=n;j++)
				_nrTEMP5_34x34[j][i] /= _s_33x1[i];
		}
		else
		{
			for(j=1;j<=n;j++)
			{
				_nrTEMP5_34x34[j][i] = 0.;			
				_nrTEMP4_34x34[j][i] = 0.;
			}
		}
	}


	trans2_(1.f, _nrTEMP4_34x34,m,n);

	mult_mm_((const double**)_nrTEMP5_34x34,n,n, (const double**)_nrTEMP4_34x34,m, _nrTEMP1_34x34);

	for(i=1;i<=n;i++)
		for(j=1;j<=m;j++)
			Ai[i][j] = (float)_nrTEMP1_34x34[i][j];

	return 0;
}

//---------------------------------------

#define SWAP(a,b) temp=(a);(a)=(b);(b)=temp;
#define SWAPi(a,b) tempi=(a);(a)=(b);(b)=tempi;
// example 
// arr_4x1[] = {0, 4, 3, 2, 1}
// index_4x1[] = {0, 1, 2, 3, 4}
//
// nrselect(1,4, arr_4x1, index_4x1)
// arr_4x1 : {0, 1, 2, 3, 4}
// index_4x1 : {0, 4, 3, 2, 1}
// returned value : 1

float nrselect(unsigned int k, unsigned int n, float arr[], unsigned int index[])	
{
	unsigned int i,ir,j,l,mid;
	float a,temp;
	unsigned int ind, tempi;

	l=1;
	ir=n;
	for (;;) {
		if (ir <= l+1) {
			if (ir == l+1 && arr[ir] < arr[l]) {
				SWAP(arr[l],arr[ir])
				SWAPi(index[l],index[ir])
			}
			return arr[k];
		} else {
			mid=(l+ir) >> 1;
			SWAP(arr[mid],arr[l+1])
			SWAPi(index[mid], index[l+1])
			if (arr[l] > arr[ir]) {
				SWAP(arr[l],arr[ir])
				SWAPi(index[l],index[ir])
			}
			if (arr[l+1] > arr[ir]) {
				SWAP(arr[l+1],arr[ir])
				SWAPi(index[l+1],index[ir])
			}
			if (arr[l] > arr[l+1]) {
				SWAP(arr[l],arr[l+1])
				SWAPi(index[l],index[l+1])
			}
			i=l+1;
			j=ir;
			a=arr[l+1];
			ind = index[l+1];
			for (;;) {
				do i++; while (arr[i] < a);
				do j--; while (arr[j] > a);
				if (j < i) break;
				SWAP(arr[i],arr[j])
				SWAPi(index[i],index[j])
			}
			arr[l+1]=arr[j];
			index[l+1] = index[j];
			arr[j]=a;
			index[j] = ind;
			if (j >= k) ir=j-1;
			if (j <= k) l=i;
		}
	}
}
#undef SWAP
#undef SWAPi

//---------------------------------------

float norm_v(const float* vector_nx1, unsigned int n) //added by Inhyeok Kim
{
	unsigned int i;
	float sum = 0.;

	for(i=1; i<=n; i++)
		sum += vector_nx1[i]*vector_nx1[i];

	return((float)sqrt(sum));
}

//---------------------------------------

double norm_v_(const double* vector_nx1, unsigned int n) //added by Inhyeok Kim
{
	unsigned int i;
	double sum = 0.;

	for(i=1; i<=n; i++)
		sum += vector_nx1[i]*vector_nx1[i];

	return((double)sqrt(sum));
}

//---------------------------------------

int findminmax(float data[], unsigned int n, float *result_max, float *result_min, unsigned int *i_max, unsigned int *i_min)   //added by Inhyeok Kim
{
	unsigned int i, n_max, n_min;
	float max = -1.e9;
	float min = 1.e9;

	for(i=0; i<n; i++)
	{
		if(max < data[i])
		{
			max = data[i];
			n_max = i;
		}		
		
		if(min > data[i])
		{
			min = data[i];
			n_min = i;
		}
	}

	*result_max = max;
	*i_max = n_max;

	*result_min = min;
	*i_min = n_min;

	return 0;
}

//---------------------------------------

int findmax(float data[], unsigned int n, float *result_max, unsigned int *i_max) //added by Inhyeok Kim
{
	unsigned int i, n_max;
	float max = -1.e9;

	for(i=0; i<n; i++)
	{
		if(max < data[i])
		{
			max = data[i];
			n_max = i;
		}				
	}

	*result_max = max;
	*i_max = n_max;

	return 0;
}

//---------------------------------------

int findmin(float data[], unsigned int n, float *result_min, unsigned int *i_min)  //added by Inhyeok Kim
{
	unsigned int i, n_min;
	float min = 1.e9;

	for(i=0; i<n; i++)
	{
		if(min > data[i])
		{
			min = data[i];
			n_min = i;
		}				
	}

	*result_min = min;
	*i_min = n_min;

	return 0;
}



int InitGlobalMotionVariables(void)
{
	int i,j;

	_PassiveUpdatedFlag = 0;
	_FKineUpdatedFlag = 0;

	InitGlobalNRutilVariables();	

	_jRF_6x33 = matrix(1,6,1,33);
	_jLF_6x33 = matrix(1,6,1,33);
	_jCOM_3x33 = matrix(1,3,1,33);
	_jRH_6x33 = matrix(1,6,1,33);
	_jLH_6x33 = matrix(1,6,1,33);
	_jRS_6x33 = matrix(1,6,1,33);
	_jLS_6x33 = matrix(1,6,1,33);
	_jRWR_6x33 = matrix(1,6,1,33);
	_jLWR_6x33 = matrix(1,6,1,33);

	_jRF_old_6x33 = matrix(1,6,1,33);
	_jLF_old_6x33 = matrix(1,6,1,33);
	_jCOM_old_3x33 = matrix(1,3,1,33);
	_jRH_old_6x33 = matrix(1,6,1,33);
	_jLH_old_6x33 = matrix(1,6,1,33);
	_jRS_old_6x33 = matrix(1,6,1,33);
	_jLS_old_6x33 = matrix(1,6,1,33);
	_jRWR_old_6x33 = matrix(1,6,1,33);
	_jLWR_old_6x33 = matrix(1,6,1,33);

	_jRF_old2_6x33 = matrix(1,6,1,33);
	_jLF_old2_6x33 = matrix(1,6,1,33);
	_jCOM_old2_3x33 = matrix(1,3,1,33);
	_jRH_old2_6x33 = matrix(1,6,1,33);
	_jLH_old2_6x33 = matrix(1,6,1,33);
	_jRS_old2_6x33 = matrix(1,6,1,33);
	_jLS_old2_6x33 = matrix(1,6,1,33);
	_jRWR_old2_6x33 = matrix(1,6,1,33);
	_jLWR_old2_6x33 = matrix(1,6,1,33);

	_jRFp_6x33 = matrix(1,6,1,33);
	_jLFp_6x33 = matrix(1,6,1,33);
	_jCOMp_3x33 = matrix(1,3,1,33);
	_jRHp_6x33 = matrix(1,6,1,33);
	_jLHp_6x33 = matrix(1,6,1,33);
	_jRSp_6x33 = matrix(1,6,1,33);
	_jLSp_6x33 = matrix(1,6,1,33);
	_jRWRp_6x33 = matrix(1,6,1,33);
	_jLWRp_6x33 = matrix(1,6,1,33);

	_Rz_RHY_3x3 = matrix(1,3,1,3);
	_Rx_RHR_3x3 = matrix(1,3,1,3);
	_Ry_RHP_3x3 = matrix(1,3,1,3);
	_Ry_RKN_3x3 = matrix(1,3,1,3);
	_Ry_RAP_3x3 = matrix(1,3,1,3);
	_Rx_RAR_3x3 = matrix(1,3,1,3);

	_Rz_LHY_3x3 = matrix(1,3,1,3);
	_Rx_LHR_3x3 = matrix(1,3,1,3);
	_Ry_LHP_3x3 = matrix(1,3,1,3);
	_Ry_LKN_3x3 = matrix(1,3,1,3);
	_Ry_LAP_3x3 = matrix(1,3,1,3);
	_Rx_LAR_3x3 = matrix(1,3,1,3);

	_Rz_WST_3x3 = matrix(1,3,1,3);
	_Ry_RSP_3x3 = matrix(1,3,1,3);
	_Rx_RSR_3x3 = matrix(1,3,1,3);
	_Rz_RSY_3x3 = matrix(1,3,1,3);
	_Ry_REB_3x3 = matrix(1,3,1,3);
	_Rz_RWY_3x3 = matrix(1,3,1,3);
	_Ry_RWP_3x3 = matrix(1,3,1,3);
	_Rz_RWY2_3x3 = matrix(1,3,1,3);

	_Ry_LSP_3x3 = matrix(1,3,1,3);
	_Rx_LSR_3x3 = matrix(1,3,1,3);
	_Rz_LSY_3x3 = matrix(1,3,1,3);
	_Ry_LEB_3x3 = matrix(1,3,1,3);
	_Rz_LWY_3x3 = matrix(1,3,1,3);
	_Ry_LWP_3x3 = matrix(1,3,1,3);
	_Rz_LWY2_3x3 = matrix(1,3,1,3);
	_Ry_PI_3x3 = matrix(1,3,1,3);

	_dcPEL_3x3 = matrix(1,3,1,3);
	_dcRUL_3x3 = matrix(1,3,1,3);
	_dcRLL_3x3 = matrix(1,3,1,3);
	_dcRF_3x3 = matrix(1,3,1,3);
	_dcLUL_3x3 = matrix(1,3,1,3);
	_dcLLL_3x3 = matrix(1,3,1,3);
	_dcLF_3x3 = matrix(1,3,1,3);

	_dcTOR_3x3 = matrix(1,3,1,3);
	_dcRUA_3x3 = matrix(1,3,1,3);
	_dcRLA_3x3 = matrix(1,3,1,3);
	_dcRH_3x3 = matrix(1,3,1,3);
	_dcLUA_3x3 = matrix(1,3,1,3);
	_dcLLA_3x3 = matrix(1,3,1,3);
	_dcLH_3x3 = matrix(1,3,1,3);
	_dcRS_3x3 = matrix(1,3,1,3);
	_dcLS_3x3 = matrix(1,3,1,3);

	_TEMP1_34x34 = matrix(1,34,1,34);
	_TEMP2_34x34 = matrix(1,34,1,34);
	_TEMP3_34x34 = matrix(1,34,1,34);
	_TEMP4_34x34 = matrix(1,34,1,34);
	_EYE_33 = matrix(1,33,1,33);

	_jT1_33x33 = matrix(1,33,1,33);
	_jT1inv_33x33 = matrix(1,33,1,33);
	_N1_33x33 = matrix(1,33,1,33);
	_jT2_33x33 = matrix(1,33,1,33);
	_jT2inv_33x33 = matrix(1,33,1,33);

	for(j=1; j<=33; j++)
	{
		for(i=1; i<=6; i++)
		{			
			_jRF_6x33[i][j] = 0.;
			_jLF_6x33[i][j] = 0.;

			_jRFp_6x33[i][j] = 0.;
			_jLFp_6x33[i][j] = 0.;

			_jRH_6x33[i][j] = 0.;
			_jLH_6x33[i][j] = 0.;

			_jRS_6x33[i][j] = 0.;
			_jLS_6x33[i][j] = 0.;

			_jRWR_6x33[i][j] = 0.;
			_jLWR_6x33[i][j] = 0.;

			_jRHp_6x33[i][j] = 0.;
			_jLHp_6x33[i][j] = 0.;

			_jRSp_6x33[i][j] = 0.;
			_jLSp_6x33[i][j] = 0.;

			_jRWRp_6x33[i][j] = 0.;
			_jLWRp_6x33[i][j] = 0.;
		}

		for(i=1; i<=3; i++)
		{	
			_jCOM_3x33[i][j] = 0.;
			_jCOMp_3x33[i][j] = 0.;		
		}

		for(i=1; i<=33; i++)
		{
			if(i == j)
				_EYE_33[i][j] = 1.f;
			else
				_EYE_33[i][j] = 0.;	

			_jT1_33x33[i][j] = 0.;
			_jT1inv_33x33[i][j] = 0.;
			_N1_33x33[i][j] = 0.;
			_jT2_33x33[i][j] = 0.;
			_jT2inv_33x33[i][j] = 0.;
		}
	}

	_copRF_filt_2x1[1] = 0.;
	_copRF_filt_2x1[2] = 0.;
	_copLF_filt_2x1[1] = 0.;
	_copLF_filt_2x1[2] = 0.;
	return 0;
}


int UpdateGlobalMotionVariables(void)
{
	int i;

	subs_m((const float**)_jRF_6x33,6,33, _jRF_old_6x33);
	subs_m((const float**)_jLF_6x33,6,33, _jLF_old_6x33);
	subs_m((const float**)_jRH_6x33,6,33, _jRH_old_6x33);
	subs_m((const float**)_jLH_6x33,6,33, _jLH_old_6x33);
	subs_m((const float**)_jRS_6x33,6,33, _jRS_old_6x33);
	subs_m((const float**)_jLS_6x33,6,33, _jLS_old_6x33);
	subs_m((const float**)_jRWR_6x33,6,33, _jRWR_old_6x33);
	subs_m((const float**)_jLWR_6x33,6,33, _jLWR_old_6x33);
	subs_m((const float**)_jCOM_3x33,3,33, _jCOM_old_3x33);

	subs_m((const float**)_jRF_6x33,6,33, _jRF_old2_6x33);
	subs_m((const float**)_jLF_6x33,6,33, _jLF_old2_6x33);
	subs_m((const float**)_jRH_6x33,6,33, _jRH_old2_6x33);
	subs_m((const float**)_jLH_6x33,6,33, _jLH_old2_6x33);
	subs_m((const float**)_jRS_6x33,6,33, _jRS_old2_6x33);
	subs_m((const float**)_jLS_6x33,6,33, _jLS_old2_6x33);
	subs_m((const float**)_jRWR_6x33,6,33, _jRWR_old2_6x33);
	subs_m((const float**)_jLWR_6x33,6,33, _jLWR_old2_6x33);
	subs_m((const float**)_jCOM_3x33,3,33, _jCOM_old2_3x33);

	_Q_34x1[RHY_34] = RHYRefAngleCurrent*D2R;
	_Q_34x1[RHR_34] = RHRRefAngleCurrent*D2R;
	_Q_34x1[RHP_34] = RHPRefAngleCurrent*D2R;
	_Q_34x1[RKN_34] = RKNRefAngleCurrent*D2R;
	_Q_34x1[RAP_34] = RAPRefAngleCurrent*D2R;
	_Q_34x1[RAR_34] = RARRefAngleCurrent*D2R;
	_Q_34x1[LHY_34] = LHYRefAngleCurrent*D2R;
	_Q_34x1[LHR_34] = LHRRefAngleCurrent*D2R;
	_Q_34x1[LHP_34] = LHPRefAngleCurrent*D2R;
	_Q_34x1[LKN_34] = LKNRefAngleCurrent*D2R;
	_Q_34x1[LAP_34] = LAPRefAngleCurrent*D2R;
	_Q_34x1[LAR_34] = LARRefAngleCurrent*D2R;

	_Q_34x1[WST_34] = WSTRefAngleCurrent*D2R;
	_Q_34x1[RSP_34] = RSPRefAngleCurrent*D2R;
	_Q_34x1[RSR_34] = (RSRRefAngleCurrent + OFFSET_RSR)*D2R;
	_Q_34x1[RSY_34] = RSYRefAngleCurrent*D2R;
	_Q_34x1[REB_34] = (REBRefAngleCurrent + OFFSET_REB)*D2R;
	_Q_34x1[RWY_34] = RWYRefAngleCurrent*D2R;
	_Q_34x1[RWP_34] = RWPRefAngleCurrent*D2R;
	_Q_34x1[LSP_34] = LSPRefAngleCurrent*D2R;
	_Q_34x1[LSR_34] = (LSRRefAngleCurrent + OFFSET_LSR)*D2R;
	_Q_34x1[LSY_34] = LSYRefAngleCurrent*D2R;
	_Q_34x1[LEB_34] = (LEBRefAngleCurrent + OFFSET_LEB)*D2R;
	_Q_34x1[LWY_34] = LWYRefAngleCurrent*D2R;
	_Q_34x1[LWP_34] = LWPRefAngleCurrent*D2R;
	_Q_34x1[RWY2_34] = RWY2RefAngleCurrent*D2R;
	_Q_34x1[LWY2_34] = LWY2RefAngleCurrent*D2R;
	
	//------------------ initial joint pos
	for(i=8; i<=34; i++)
		_Q0_34x1[i] = _Q_34x1[i];

	for(i=1; i<=33; i++)
		_Qp_33x1[i] = 0.f;

	UpdatePassiveCoord_SSP(LFSP);
	//---------------------
	
	return 0;
}


int FreeGlobalMotionVariables(void)
{
	free_matrix(_jRF_6x33, 1,6,1,33);
	free_matrix(_jLF_6x33, 1,6,1,33);
	free_matrix(_jCOM_3x33, 1,3,1,33);
	free_matrix(_jRH_6x33, 1,6,1,33);
	free_matrix(_jLH_6x33, 1,6,1,33);
	free_matrix(_jRS_6x33, 1,6,1,33);
	free_matrix(_jLS_6x33, 1,6,1,33);
	free_matrix(_jRWR_6x33, 1,6,1,33);
	free_matrix(_jLWR_6x33, 1,6,1,33);

	free_matrix(_jRF_old_6x33, 1,6,1,33);
	free_matrix(_jLF_old_6x33, 1,6,1,33);
	free_matrix(_jCOM_old_3x33, 1,3,1,33);
	free_matrix(_jRH_old_6x33, 1,6,1,33);
	free_matrix(_jLH_old_6x33, 1,6,1,33);
	free_matrix(_jRS_old_6x33, 1,6,1,33);
	free_matrix(_jLS_old_6x33, 1,6,1,33);
	free_matrix(_jRWR_old_6x33, 1,6,1,33);
	free_matrix(_jLWR_old_6x33, 1,6,1,33);

	free_matrix(_jRF_old2_6x33, 1,6,1,33);
	free_matrix(_jLF_old2_6x33, 1,6,1,33);
	free_matrix(_jCOM_old2_3x33, 1,3,1,33);
	free_matrix(_jRH_old2_6x33, 1,6,1,33);
	free_matrix(_jLH_old2_6x33, 1,6,1,33);
	free_matrix(_jRS_old2_6x33, 1,6,1,33);
	free_matrix(_jLS_old2_6x33, 1,6,1,33);
	free_matrix(_jRWR_old2_6x33, 1,6,1,33);
	free_matrix(_jLWR_old2_6x33, 1,6,1,33);

	free_matrix(_jRFp_6x33, 1,6,1,33);
	free_matrix(_jLFp_6x33, 1,6,1,33);
	free_matrix(_jCOMp_3x33, 1,3,1,33);
	free_matrix(_jRHp_6x33, 1,6,1,33);
	free_matrix(_jLHp_6x33, 1,6,1,33);
	free_matrix(_jRSp_6x33, 1,6,1,33);
	free_matrix(_jLSp_6x33, 1,6,1,33);
	free_matrix(_jRWRp_6x33, 1,6,1,33);
	free_matrix(_jLWRp_6x33, 1,6,1,33);

	free_matrix(_Rz_RHY_3x3,1,3,1,3);
	free_matrix(_Rx_RHR_3x3,1,3,1,3);
	free_matrix(_Ry_RHP_3x3,1,3,1,3);
	free_matrix(_Ry_RKN_3x3,1,3,1,3);
	free_matrix(_Ry_RAP_3x3,1,3,1,3);
	free_matrix(_Rx_RAR_3x3,1,3,1,3);

	free_matrix(_Rz_LHY_3x3,1,3,1,3);
	free_matrix(_Rx_LHR_3x3,1,3,1,3);
	free_matrix(_Ry_LHP_3x3,1,3,1,3);
	free_matrix(_Ry_LKN_3x3,1,3,1,3);
	free_matrix(_Ry_LAP_3x3,1,3,1,3);
	free_matrix(_Rx_LAR_3x3,1,3,1,3);

	free_matrix(_Rz_WST_3x3,1,3,1,3);
	free_matrix(_Ry_RSP_3x3,1,3,1,3);
	free_matrix(_Rx_RSR_3x3,1,3,1,3);
	free_matrix(_Rz_RSY_3x3,1,3,1,3);
	free_matrix(_Ry_REB_3x3,1,3,1,3);
	free_matrix(_Rz_RWY_3x3,1,3,1,3);
	free_matrix(_Ry_RWP_3x3,1,3,1,3);
	free_matrix(_Rz_RWY2_3x3,1,3,1,3);

	free_matrix(_Ry_LSP_3x3,1,3,1,3);
	free_matrix(_Rx_LSR_3x3,1,3,1,3);
	free_matrix(_Rz_LSY_3x3,1,3,1,3);
	free_matrix(_Ry_LEB_3x3,1,3,1,3);
	free_matrix(_Rz_LWY_3x3,1,3,1,3);
	free_matrix(_Ry_LWP_3x3,1,3,1,3);
	free_matrix(_Rz_LWY2_3x3,1,3,1,3);
	free_matrix(_Ry_PI_3x3,1,3,1,3);

	free_matrix(_dcPEL_3x3,1,3,1,3);
	free_matrix(_dcRUL_3x3,1,3,1,3);
	free_matrix(_dcRLL_3x3,1,3,1,3);
	free_matrix(_dcRF_3x3,1,3,1,3);
	free_matrix(_dcLUL_3x3,1,3,1,3);
	free_matrix(_dcLLL_3x3,1,3,1,3);
	free_matrix(_dcLF_3x3,1,3,1,3);

	free_matrix(_dcTOR_3x3,1,3,1,3);
	free_matrix(_dcRUA_3x3,1,3,1,3);
	free_matrix(_dcRLA_3x3,1,3,1,3);
	free_matrix(_dcRH_3x3,1,3,1,3);
	free_matrix(_dcLUA_3x3,1,3,1,3);
	free_matrix(_dcLLA_3x3,1,3,1,3);
	free_matrix(_dcLH_3x3,1,3,1,3);
	free_matrix(_dcRS_3x3,1,3,1,3);
	free_matrix(_dcLS_3x3,1,3,1,3);

	free_matrix(_TEMP1_34x34,1,34,1,34);
	free_matrix(_TEMP2_34x34,1,34,1,34);
	free_matrix(_TEMP3_34x34,1,34,1,34);
	free_matrix(_TEMP4_34x34,1,34,1,34);
	free_matrix(_EYE_33, 1,33,1,33);

	free_matrix(_jT1_33x33, 1,33,1,33);
	free_matrix(_jT1inv_33x33, 1,33,1,33);
	free_matrix(_N1_33x33, 1,33,1,33);
	free_matrix(_jT2_33x33, 1,33,1,33);
	free_matrix(_jT2inv_33x33, 1,33,1,33);

	FreeGlobalNRutilVariables();
	return 0;
}




int FKine_Whole(void)
{
	float wst, rhy, rhp, rhr, rkn, rap, rar, lhy, lhp, lhr, lkn, lap, lar, rsp, rsr, rsy, reb, rwy, lsp, lsr, lsy, leb, lwy, rwp, lwp, rwy2, lwy2;
	float axis_rhy[4], axis_rhr[4], axis_rhp[4], axis_rkn[4], axis_rap[4], axis_rar[4];	 
	float axis_lhy[4], axis_lhr[4], axis_lhp[4], axis_lkn[4], axis_lap[4], axis_lar[4];	
	float axis_wst[4], axis_rsp[4], axis_rsr[4], axis_rsy[4], axis_reb[4], axis_rwy[4], axis_rwp[4], axis_rwy2[4];
	float axis_lsp[4], axis_lsr[4], axis_lsy[4], axis_leb[4], axis_lwy[4], axis_lwp[4], axis_lwy2[4];

	float pRPEL[4], pRKN[4], pLPEL[4], pLKN[4], pWST[4], pRSHLD[4], pRELB[4], pLSHLD[4], pLELB[4], pRELB2[4], pLELB2[4];
	float cPEL[4], cTOR[4], cRUL[4], cRLL[4], cRF[4], cLUL[4], cLLL[4], cLF[4], cRUA[4], cRLA[4], cLUA[4], cLLA[4], cRH[4], cLH[4];
	float temp3_3x1[4], temp4_3x1[4];

	//--- Jacobian for [xp,yp,zp,wPELx,wPELy,wPELz,wstp,rhyp,rhrp,rhpp,rknp,rapp,rarp,lhyp,lhrp,lhpp,lknp,lapp,larp,rspp,rsrp,rsyp,rebp,rwyp,rwpp,lspp,lsrp,lsyp,lebp,lwyp,lwpp]' : 31x1
	float **jCOM_PEL_3x33, **jCOM_RUL_3x33, **jCOM_RLL_3x33, **jCOM_RF_3x33, **jCOM_LUL_3x33, **jCOM_LLL_3x33, **jCOM_LF_3x33;
	float **jCOM_TOR_3x33, **jCOM_RUA_3x33, **jCOM_RLA_3x33, **jCOM_LUA_3x33, **jCOM_LLA_3x33, **jCOM_LH_3x33, **jCOM_RH_3x33;

	const float qPEL[5] = {0., _Q_34x1[4], _Q_34x1[5], _Q_34x1[6], _Q_34x1[7]};
	const float pPC[4] = {0., _Q_34x1[1], _Q_34x1[2], _Q_34x1[3]};

	int i, j;
	float ftemp;

	jCOM_PEL_3x33 = matrix(1,3,1,33);
	jCOM_RUL_3x33 = matrix(1,3,1,33);
	jCOM_RLL_3x33 = matrix(1,3,1,33);
	jCOM_RF_3x33 = matrix(1,3,1,33);
	jCOM_LUL_3x33 = matrix(1,3,1,33);
	jCOM_LLL_3x33 = matrix(1,3,1,33);
	jCOM_LF_3x33 = matrix(1,3,1,33);
	jCOM_TOR_3x33 = matrix(1,3,1,33);
	jCOM_RUA_3x33 = matrix(1,3,1,33);
	jCOM_RLA_3x33 = matrix(1,3,1,33);
	jCOM_LUA_3x33 = matrix(1,3,1,33);
	jCOM_LLA_3x33 = matrix(1,3,1,33);
	jCOM_LH_3x33 = matrix(1,3,1,33);
	jCOM_RH_3x33 = matrix(1,3,1,33);
	
	for(i=1; i<=3; i++)
	{
		for(j=1; j<=33; j++)
		{
			_jCOM_3x33[i][j] = 0.;
			_jCOM_3x33[i][j] = 0.;
			_jCOM_3x33[i][j] = 0.;

			jCOM_PEL_3x33[i][j] = 0.;
			jCOM_RUL_3x33[i][j] = 0.;
			jCOM_RLL_3x33[i][j] = 0.;
			jCOM_RF_3x33[i][j] = 0.;
			jCOM_LUL_3x33[i][j] = 0.;
			jCOM_LLL_3x33[i][j] = 0.;
			jCOM_LF_3x33[i][j] = 0.;
			jCOM_TOR_3x33[i][j] = 0.;
			jCOM_RUA_3x33[i][j] = 0.;
			jCOM_RLA_3x33[i][j] = 0.;
			jCOM_LUA_3x33[i][j] = 0.;
			jCOM_LLA_3x33[i][j] = 0.;
			jCOM_RH_3x33[i][j] = 0.;
			jCOM_LH_3x33[i][j] = 0.;	
			
			_jRF_6x33[i][j] = 0.;
			_jLF_6x33[i][j] = 0.;
			_jRH_6x33[i][j] = 0.;
			_jLH_6x33[i][j] = 0.;
			_jRS_6x33[i][j] = 0.;
			_jLS_6x33[i][j] = 0.;
			_jRWR_6x33[i][j] = 0.;
			_jLWR_6x33[i][j] = 0.;

			_jRF_6x33[i+3][j] = 0.;
			_jLF_6x33[i+3][j] = 0.;
			_jRH_6x33[i+3][j] = 0.;
			_jLH_6x33[i+3][j] = 0.;
			_jRWR_6x33[i+3][j] = 0.;
			_jLWR_6x33[i+3][j] = 0.;
		}
	}

	_jCOM_3x33[1][1] = 1.f;
	_jCOM_3x33[2][2] = 1.f;
	_jCOM_3x33[3][3] = 1.f;

	rhy = _Q_34x1[RHY_34];
	rhr = _Q_34x1[RHR_34];
	rhp = _Q_34x1[RHP_34];
	rkn = _Q_34x1[RKN_34];
	rap = _Q_34x1[RAP_34];
	rar = _Q_34x1[RAR_34];

	lhy = _Q_34x1[LHY_34];
	lhr = _Q_34x1[LHR_34];
	lhp = _Q_34x1[LHP_34];
	lkn = _Q_34x1[LKN_34];
	lap = _Q_34x1[LAP_34];
	lar = _Q_34x1[LAR_34];

	wst = _Q_34x1[WST_34];
	rsp = _Q_34x1[RSP_34];
	rsr = _Q_34x1[RSR_34];
	rsy = _Q_34x1[RSY_34];
	reb = _Q_34x1[REB_34];
	rwy = _Q_34x1[RWY_34];
	rwp = _Q_34x1[RWP_34];
	lsp = _Q_34x1[LSP_34];
	lsr = _Q_34x1[LSR_34];
	lsy = _Q_34x1[LSY_34];
	leb = _Q_34x1[LEB_34];
	lwy = _Q_34x1[LWY_34];
	lwp = _Q_34x1[LWP_34];
	rwy2 = _Q_34x1[RWY2_34];
	lwy2 = _Q_34x1[LWY2_34];

        //printf("angle %f %f %f %f %f %f %f\n",rsp,rsr,rsy,reb,rwy,rwp,rwy2); 
   
	QT2DC(qPEL, _dcPEL_3x3);
	
	RZ(rhy, _Rz_RHY_3x3);
	RX(rhr, _Rx_RHR_3x3);
	RY(rhp, _Ry_RHP_3x3);
	RY(rkn, _Ry_RKN_3x3);
	RY(rap, _Ry_RAP_3x3);
	RX(rar, _Rx_RAR_3x3);

	RZ(lhy, _Rz_LHY_3x3);
	RX(lhr, _Rx_LHR_3x3);
	RY(lhp, _Ry_LHP_3x3);
	RY(lkn, _Ry_LKN_3x3);
	RY(lap, _Ry_LAP_3x3);
	RX(lar, _Rx_LAR_3x3);

	RZ(wst, _Rz_WST_3x3);
	
	RY(rsp, _Ry_RSP_3x3);
	RX(rsr, _Rx_RSR_3x3);
	RZ(rsy, _Rz_RSY_3x3);
	RY(reb, _Ry_REB_3x3);
	RZ(rwy, _Rz_RWY_3x3);
	RY(rwp, _Ry_RWP_3x3);

	RY(lsp, _Ry_LSP_3x3);
	RX(lsr, _Rx_LSR_3x3);
	RZ(lsy, _Rz_LSY_3x3);
	RY(leb, _Ry_LEB_3x3);
	RZ(lwy, _Rz_LWY_3x3);
	RY(lwp, _Ry_LWP_3x3);

	RZ(rwy2, _Rz_RWY2_3x3);
	RZ(lwy2, _Rz_LWY2_3x3);

	RY(PI, _Ry_PI_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_RHR_3x3,3,3, (const float**)_Ry_RHP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcRUL_3x3);

	mult_mm((const float**)_dcRUL_3x3,3,3, (const float**)_Ry_RKN_3x3,3, _dcRLL_3x3);
	
	mult_mm((const float**)_Ry_RAP_3x3,3,3, (const float**)_Rx_RAR_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_dcRLL_3x3,3,3, (const float**)_TEMP1_34x34,3, _dcRF_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_LHR_3x3,3,3, (const float**)_Ry_LHP_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcLUL_3x3);

	mult_mm((const float**)_dcLUL_3x3,3,3, (const float**)_Ry_LKN_3x3,3, _dcLLL_3x3);
	
	mult_mm((const float**)_Ry_LAP_3x3,3,3, (const float**)_Rx_LAR_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_dcLLL_3x3,3,3, (const float**)_TEMP1_34x34,3, _dcLF_3x3);

	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_WST_3x3,3, _dcTOR_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_RSR_3x3,3,3, (const float**)_Rz_RSY_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcRUA_3x3);
	
	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_RWY_3x3,3, _dcRLA_3x3);
	mult_mm((const float**)_dcRLA_3x3,3,3, (const float**)_Ry_RWP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_RWY2_3x3,3, _dcRH_3x3);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Ry_PI_3x3,3, _dcRS_3x3);

	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_Rx_LSR_3x3,3,3, (const float**)_Rz_LSY_3x3,3, _TEMP2_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_TEMP2_34x34,3, _dcLUA_3x3);

	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_LWY_3x3,3, _dcLLA_3x3);
	mult_mm((const float**)_dcLLA_3x3,3,3, (const float**)_Ry_LWP_3x3,3, _TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Rz_LWY2_3x3,3, _dcLH_3x3);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_Ry_PI_3x3,3, _dcLS_3x3);
	
	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_RPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pRPEL);

	mult_mv((const float**)_dcRUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRPEL,3, temp3_3x1, pRKN);

	mult_mv((const float**)_dcRLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, _pRANK_3x1);

	mult_mv((const float**)_dcRF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(_pRANK_3x1,3, temp3_3x1, _pRF_3x1);


	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_LPEL, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pLPEL);

	mult_mv((const float**)_dcLUL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLPEL,3, temp3_3x1, pLKN);

	mult_mv((const float**)_dcLLL_3x3,3,3, _LINK_LEG, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, _pLANK_3x1);

	mult_mv((const float**)_dcLF_3x3,3,3, _LINK_FOOT, temp3_3x1);
	sum_vv(_pLANK_3x1,3, temp3_3x1, _pLF_3x1);


	mult_mv((const float**)_dcPEL_3x3,3,3, _LINK_WST, temp3_3x1); 
	sum_vv(pPC,3, temp3_3x1, pWST);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_RSHLD, temp3_3x1); 
	sum_vv(pWST,3, temp3_3x1, pRSHLD);

	mult_mv((const float**)_dcRUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pRSHLD,3, temp3_3x1, pRELB);

	temp4_3x1[1] = _LINK_LARM[1];
	temp4_3x1[2] = 0.;
	temp4_3x1[3] = 0.;
	mult_mm((const float**)_dcRUA_3x3,3,3, (const float**)_Ry_REB_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pRELB,3, temp3_3x1, pRELB2);
	temp4_3x1[1] = 0.;
	temp4_3x1[2] = _LINK_LARM[2];
	temp4_3x1[3] = _LINK_LARM[3];
	mult_mv((const float**)_dcRLA_3x3,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pRELB2,3, temp3_3x1, _pRWR_3x1);

	mult_mv((const float**)_dcRH_3x3,3,3, _LINK_HAND, temp3_3x1); 
	sum_vv(_pRWR_3x1,3, temp3_3x1, _pRH_3x1);

	mult_mv((const float**)_dcRS_3x3,3,3, _LINK_STICK, temp3_3x1); 
	sum_vv(_pRWR_3x1,3, temp3_3x1, _pRS_3x1);

	mult_mv((const float**)_dcTOR_3x3,3,3, _LINK_LSHLD, temp3_3x1); 
	sum_vv(pWST,3, temp3_3x1, pLSHLD);

	mult_mv((const float**)_dcLUA_3x3,3,3, _LINK_UARM, temp3_3x1); 
	sum_vv(pLSHLD,3, temp3_3x1, pLELB);

	temp4_3x1[1] = _LINK_LARM[1];
	temp4_3x1[2] = 0.;
	temp4_3x1[3] = 0.;
	mult_mm((const float**)_dcLUA_3x3,3,3, (const float**)_Ry_LEB_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pLELB,3, temp3_3x1, pLELB2);
	temp4_3x1[1] = 0.;
	temp4_3x1[2] = _LINK_LARM[2];
	temp4_3x1[3] = _LINK_LARM[3];
	mult_mv((const float**)_dcLLA_3x3,3,3, temp4_3x1, temp3_3x1); 
	sum_vv(pLELB2,3, temp3_3x1, _pLWR_3x1);

	mult_mv((const float**)_dcLH_3x3,3,3, _LINK_HAND, temp3_3x1); 
	sum_vv(_pLWR_3x1,3, temp3_3x1, _pLH_3x1);

	mult_mv((const float**)_dcLS_3x3,3,3, _LINK_STICK, temp3_3x1); 
	sum_vv(_pLWR_3x1,3, temp3_3x1, _pLS_3x1);

	mult_mv((const float**)_dcPEL_3x3,3,3, _C_PEL, temp3_3x1);
	sum_vv(pPC,3, temp3_3x1, cPEL);

	mult_mv((const float**)_dcTOR_3x3,3,3, _C_TOR, temp3_3x1);
	sum_vv(pWST,3, temp3_3x1, cTOR);

	mult_mv((const float**)_dcRUL_3x3,3,3, _C_RUL, temp3_3x1);
	sum_vv(pRPEL,3, temp3_3x1, cRUL);

	mult_mv((const float**)_dcRLL_3x3,3,3, _C_RLL, temp3_3x1);
	sum_vv(pRKN,3, temp3_3x1, cRLL);

	mult_mv((const float**)_dcRF_3x3,3,3, _C_RF, temp3_3x1);
	sum_vv(_pRANK_3x1,3, temp3_3x1, cRF);

	mult_mv((const float**)_dcLUL_3x3,3,3, _C_LUL, temp3_3x1);
	sum_vv(pLPEL,3, temp3_3x1, cLUL);

	mult_mv((const float**)_dcLLL_3x3,3,3, _C_LLL, temp3_3x1);
	sum_vv(pLKN,3, temp3_3x1, cLLL);

	mult_mv((const float**)_dcLF_3x3,3,3, _C_LF, temp3_3x1);
	sum_vv(_pLANK_3x1,3, temp3_3x1, cLF);

	mult_mv((const float**)_dcRUA_3x3,3,3, _C_RUA, temp3_3x1);
	sum_vv(pRSHLD,3, temp3_3x1, cRUA);

	mult_mv((const float**)_dcRLA_3x3,3,3, _C_RLA, temp3_3x1); 
	sum_vv(pRELB2,3, temp3_3x1, cRLA);

	mult_mv((const float**)_dcRH_3x3,3,3, _C_RHAND, temp3_3x1);
	sum_vv(_pRWR_3x1,3, temp3_3x1, cRH);

	mult_mv((const float**)_dcLUA_3x3,3,3, _C_LUA, temp3_3x1);
	sum_vv(pLSHLD,3, temp3_3x1, cLUA);

	mult_mv((const float**)_dcLLA_3x3,3,3, _C_LLA, temp3_3x1); 
	sum_vv(pLELB2,3, temp3_3x1, cLLA);
	
	mult_mv((const float**)_dcLH_3x3,3,3, _C_LHAND, temp3_3x1);
	sum_vv(_pLWR_3x1,3, temp3_3x1, cLH);

	mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_wst);

	mult_mv((const float**)_dcPEL_3x3,3,3, _AXIS_Z, axis_rhy);
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_RHY_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, _AXIS_X, axis_rhr);
	mult_mv((const float**)_dcRUL_3x3,3,3, _AXIS_Y, axis_rhp);
	axis_rkn[1] = axis_rhp[1];
	axis_rkn[2] = axis_rhp[2];
	axis_rkn[3] = axis_rhp[3];
	axis_rap[1] = axis_rhp[1];
	axis_rap[2] = axis_rhp[2];
	axis_rap[3] = axis_rhp[3];
	mult_mv((const float**)_dcRF_3x3,3,3, _AXIS_X, axis_rar);

	axis_lhy[1] = axis_rhy[1];
	axis_lhy[2] = axis_rhy[2];
	axis_lhy[3] = axis_rhy[3];
	mult_mm((const float**)_dcPEL_3x3,3,3, (const float**)_Rz_LHY_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, _AXIS_X, axis_lhr);
	mult_mv((const float**)_dcLUL_3x3,3,3, _AXIS_Y, axis_lhp);
	axis_lkn[1] = axis_lhp[1];
	axis_lkn[2] = axis_lhp[2];
	axis_lkn[3] = axis_lhp[3];
	axis_lap[1] = axis_lhp[1];
	axis_lap[2] = axis_lhp[2];
	axis_lap[3] = axis_lhp[3];
	mult_mv((const float**)_dcLF_3x3,3,3, _AXIS_X, axis_lar);

	mult_mv((const float**)_dcTOR_3x3,3,3, _AXIS_Y, axis_rsp);
	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_RSP_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, _AXIS_X, axis_rsr);
	mult_mv((const float**)_dcRUA_3x3,3,3, _AXIS_Z, axis_rsy);
	mult_mv((const float**)_dcRUA_3x3,3,3, _AXIS_Y, axis_reb);
	mult_mv((const float**)_dcRLA_3x3,3,3, _AXIS_Z, axis_rwy);
	mult_mv((const float**)_dcRLA_3x3,3,3, _AXIS_Y, axis_rwp);
	mult_mv((const float**)_dcRH_3x3,3,3, _AXIS_Z, axis_rwy2);

	axis_lsp[1] = axis_rsp[1];
	axis_lsp[2] = axis_rsp[2];
	axis_lsp[3] = axis_rsp[3];
	mult_mm((const float**)_dcTOR_3x3,3,3, (const float**)_Ry_LSP_3x3,3, _TEMP1_34x34);
	mult_mv((const float**)_TEMP1_34x34,3,3, _AXIS_X, axis_lsr);
	mult_mv((const float**)_dcLUA_3x3,3,3, _AXIS_Z, axis_lsy);
	mult_mv((const float**)_dcLUA_3x3,3,3, _AXIS_Y, axis_leb);
	mult_mv((const float**)_dcLLA_3x3,3,3, _AXIS_Z, axis_lwy);
	mult_mv((const float**)_dcLLA_3x3,3,3, _AXIS_Y, axis_lwp);
	mult_mv((const float**)_dcLH_3x3,3,3, _AXIS_Z, axis_lwy2);

	ftemp = (m_PEL/m_TOTAL);
	jCOM_PEL_3x33[1][1] = ftemp;
	jCOM_PEL_3x33[2][2] = ftemp;
	jCOM_PEL_3x33[3][3] = ftemp;
	diff_vv(cPEL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_PEL_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_PEL_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_PEL_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_PEL_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_PEL_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_PEL_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_PEL_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_PEL_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_PEL_3x33[3][6] = ftemp*temp4_3x1[3];

	ftemp = (m_TOR/m_TOTAL);
	jCOM_TOR_3x33[1][1] = ftemp;
	jCOM_TOR_3x33[2][2] = ftemp;
	jCOM_TOR_3x33[3][3] = ftemp;
	diff_vv(cTOR,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cTOR,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_TOR_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_TOR_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_TOR_3x33[3][7] = ftemp*temp4_3x1[3];

	ftemp = (m_ULEG/m_TOTAL);
	jCOM_RUL_3x33[1][1] = ftemp;
	jCOM_RUL_3x33[2][2] = ftemp;
	jCOM_RUL_3x33[3][3] = ftemp;
	diff_vv(cRUL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRUL,3,pRPEL, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x33[1][8] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x33[2][8] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x33[3][8] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x33[1][9] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x33[2][9] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x33[3][9] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jCOM_RUL_3x33[1][10] = ftemp*temp4_3x1[1];
	jCOM_RUL_3x33[2][10] = ftemp*temp4_3x1[2];
	jCOM_RUL_3x33[3][10] = ftemp*temp4_3x1[3];

	ftemp = (m_LLEG/m_TOTAL);
	jCOM_RLL_3x33[1][1] = ftemp;
	jCOM_RLL_3x33[2][2] = ftemp;
	jCOM_RLL_3x33[3][3] = ftemp;
	diff_vv(cRLL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRLL,3,pRPEL, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][8] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][8] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][8] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][9] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][9] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][9] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][10] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][10] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][10] = ftemp*temp4_3x1[3];
	diff_vv(cRLL,3,pRKN, temp3_3x1);
	cross(1.f,axis_rkn, temp3_3x1, temp4_3x1);
	jCOM_RLL_3x33[1][11] = ftemp*temp4_3x1[1];
	jCOM_RLL_3x33[2][11] = ftemp*temp4_3x1[2];
	jCOM_RLL_3x33[3][11] = ftemp*temp4_3x1[3];

	ftemp = (m_FOOT/m_TOTAL);
	jCOM_RF_3x33[1][1] = ftemp;
	jCOM_RF_3x33[2][2] = ftemp;
	jCOM_RF_3x33[3][3] = ftemp;
	diff_vv(cRF,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRF,3,pRPEL, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][8] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][8] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][8] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][9] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][9] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][9] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][10] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][10] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][10] = ftemp*temp4_3x1[3];
	diff_vv(cRF,3,pRKN, temp3_3x1);
	cross(1.f,axis_rkn, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][11] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][11] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][11] = ftemp*temp4_3x1[3];
	diff_vv(cRF,3,_pRANK_3x1, temp3_3x1);
	cross(1.f,axis_rap, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][12] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][12] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][12] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rar, temp3_3x1, temp4_3x1);
	jCOM_RF_3x33[1][13] = ftemp*temp4_3x1[1];
	jCOM_RF_3x33[2][13] = ftemp*temp4_3x1[2];
	jCOM_RF_3x33[3][13] = ftemp*temp4_3x1[3];

	ftemp = (m_ULEG/m_TOTAL);
	jCOM_LUL_3x33[1][1] = ftemp;
	jCOM_LUL_3x33[2][2] = ftemp;
	jCOM_LUL_3x33[3][3] = ftemp;
	diff_vv(cLUL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLUL,3,pLPEL, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x33[1][14] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x33[2][14] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x33[3][14] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x33[1][15] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x33[2][15] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x33[3][15] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jCOM_LUL_3x33[1][16] = ftemp*temp4_3x1[1];
	jCOM_LUL_3x33[2][16] = ftemp*temp4_3x1[2];
	jCOM_LUL_3x33[3][16] = ftemp*temp4_3x1[3];

	ftemp = (m_LLEG/m_TOTAL);
	jCOM_LLL_3x33[1][1] = ftemp;
	jCOM_LLL_3x33[2][2] = ftemp;
	jCOM_LLL_3x33[3][3] = ftemp;
	diff_vv(cLLL,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLLL,3,pLPEL, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][14] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][14] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][14] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][15] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][15] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][15] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][16] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][16] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][16] = ftemp*temp4_3x1[3];
	diff_vv(cLLL,3,pLKN, temp3_3x1);
	cross(1.f,axis_lkn, temp3_3x1, temp4_3x1);
	jCOM_LLL_3x33[1][17] = ftemp*temp4_3x1[1];
	jCOM_LLL_3x33[2][17] = ftemp*temp4_3x1[2];
	jCOM_LLL_3x33[3][17] = ftemp*temp4_3x1[3];

	ftemp = (m_FOOT/m_TOTAL);
	jCOM_LF_3x33[1][1] = ftemp;
	jCOM_LF_3x33[2][2] = ftemp;
	jCOM_LF_3x33[3][3] = ftemp;
	diff_vv(cLF,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLF,3,pLPEL, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][14] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][14] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][14] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][15] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][15] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][15] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][16] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][16] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][16] = ftemp*temp4_3x1[3];
	diff_vv(cLF,3,pLKN, temp3_3x1);
	cross(1.f,axis_lkn, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][17] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][17] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][17] = ftemp*temp4_3x1[3];
	diff_vv(cLF,3,_pLANK_3x1, temp3_3x1);
	cross(1.f,axis_lap, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][18] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][18] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][18] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lar, temp3_3x1, temp4_3x1);
	jCOM_LF_3x33[1][19] = ftemp*temp4_3x1[1];
	jCOM_LF_3x33[2][19] = ftemp*temp4_3x1[2];
	jCOM_LF_3x33[3][19] = ftemp*temp4_3x1[3];

	ftemp = (m_UARM/m_TOTAL);
	jCOM_RUA_3x33[1][1] = ftemp;
	jCOM_RUA_3x33[2][2] = ftemp;
	jCOM_RUA_3x33[3][3] = ftemp;
	diff_vv(cRUA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRUA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cRUA,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][20] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][20] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][20] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][21] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][21] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][21] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	jCOM_RUA_3x33[1][22] = ftemp*temp4_3x1[1];
	jCOM_RUA_3x33[2][22] = ftemp*temp4_3x1[2];
	jCOM_RUA_3x33[3][22] = ftemp*temp4_3x1[3];


	ftemp = (m_LARM/m_TOTAL);
	jCOM_RLA_3x33[1][1] = ftemp;
	jCOM_RLA_3x33[2][2] = ftemp;
	jCOM_RLA_3x33[3][3] = ftemp;
	diff_vv(cRLA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][20] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][20] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][20] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][21] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][21] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][21] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][22] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][22] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][22] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pRELB, temp3_3x1);
	cross(1.f,axis_reb, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][23] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][23] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][23] = ftemp*temp4_3x1[3];
	diff_vv(cRLA,3,pRELB2, temp3_3x1);
	cross(1.f,axis_rwy, temp3_3x1, temp4_3x1);
	jCOM_RLA_3x33[1][24] = ftemp*temp4_3x1[1];
	jCOM_RLA_3x33[2][24] = ftemp*temp4_3x1[2];
	jCOM_RLA_3x33[3][24] = ftemp*temp4_3x1[3];

	ftemp = (m_RHAND/m_TOTAL);
	jCOM_RH_3x33[1][1] = ftemp;
	jCOM_RH_3x33[2][2] = ftemp;
	jCOM_RH_3x33[3][3] = ftemp;
	diff_vv(cRH,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cRH,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cRH,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][20] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][20] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][20] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][21] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][21] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][21] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][22] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][22] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][22] = ftemp*temp4_3x1[3];
	diff_vv(cRH,3,pRELB, temp3_3x1);
	cross(1.f,axis_reb, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][23] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][23] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][23] = ftemp*temp4_3x1[3];
	diff_vv(cRH,3,pRELB2, temp3_3x1);
	cross(1.f,axis_rwy, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][24] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][24] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][24] = ftemp*temp4_3x1[3];
	diff_vv(cRH,3,_pRWR_3x1, temp3_3x1);
	cross(1.f,axis_rwp, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][25] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][25] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][25] = ftemp*temp4_3x1[3];
	cross(1.f,axis_rwy2, temp3_3x1, temp4_3x1);
	jCOM_RH_3x33[1][32] = ftemp*temp4_3x1[1];
	jCOM_RH_3x33[2][32] = ftemp*temp4_3x1[2];
	jCOM_RH_3x33[3][32] = ftemp*temp4_3x1[3];

	ftemp = (m_UARM/m_TOTAL);
	jCOM_LUA_3x33[1][1] = ftemp;
	jCOM_LUA_3x33[2][2] = ftemp;
	jCOM_LUA_3x33[3][3] = ftemp;
	diff_vv(cLUA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLUA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cLUA,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][26] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][26] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][26] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][27] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][27] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][27] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	jCOM_LUA_3x33[1][28] = ftemp*temp4_3x1[1];
	jCOM_LUA_3x33[2][28] = ftemp*temp4_3x1[2];
	jCOM_LUA_3x33[3][28] = ftemp*temp4_3x1[3];

	ftemp = (m_LARM/m_TOTAL);
	jCOM_LLA_3x33[1][1] = ftemp;
	jCOM_LLA_3x33[2][2] = ftemp;
	jCOM_LLA_3x33[3][3] = ftemp;
	diff_vv(cLLA,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][26] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][26] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][26] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][27] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][27] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][27] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][28] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][28] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][28] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pLELB, temp3_3x1);
	cross(1.f,axis_leb, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][29] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][29] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][29] = ftemp*temp4_3x1[3];
	diff_vv(cLLA,3,pLELB2, temp3_3x1);
	cross(1.f,axis_lwy, temp3_3x1, temp4_3x1);
	jCOM_LLA_3x33[1][30] = ftemp*temp4_3x1[1];
	jCOM_LLA_3x33[2][30] = ftemp*temp4_3x1[2];
	jCOM_LLA_3x33[3][30] = ftemp*temp4_3x1[3];
	
	ftemp = (m_LHAND/m_TOTAL);
	jCOM_LH_3x33[1][1] = ftemp;
	jCOM_LH_3x33[2][2] = ftemp;
	jCOM_LH_3x33[3][3] = ftemp;
	diff_vv(cLH,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][4] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][4] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][4] = ftemp*temp4_3x1[3];	
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][5] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][5] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][5] = ftemp*temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][6] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][6] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][6] = ftemp*temp4_3x1[3];
	diff_vv(cLH,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][7] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][7] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][7] = ftemp*temp4_3x1[3];
	diff_vv(cLH,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][26] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][26] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][26] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][27] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][27] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][27] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][28] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][28] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][28] = ftemp*temp4_3x1[3];
	diff_vv(cLH,3,pLELB, temp3_3x1);
	cross(1.f,axis_leb, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][29] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][29] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][29] = ftemp*temp4_3x1[3];
	diff_vv(cLH,3,pLELB2, temp3_3x1);
	cross(1.f,axis_lwy, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][30] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][30] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][30] = ftemp*temp4_3x1[3];
	diff_vv(cLH,3,_pLWR_3x1, temp3_3x1);
	cross(1.f,axis_lwp, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][31] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][31] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][31] = ftemp*temp4_3x1[3];
	cross(1.f,axis_lwy2, temp3_3x1, temp4_3x1);
	jCOM_LH_3x33[1][33] = ftemp*temp4_3x1[1];
	jCOM_LH_3x33[2][33] = ftemp*temp4_3x1[2];
	jCOM_LH_3x33[3][33] = ftemp*temp4_3x1[3];

	for(i=1; i<=3; i++)
	{
		_jCOM_3x33[i][4] = jCOM_PEL_3x33[i][4] + jCOM_TOR_3x33[i][4] + jCOM_RUL_3x33[i][4] + jCOM_RLL_3x33[i][4] + jCOM_RF_3x33[i][4] + jCOM_LUL_3x33[i][4] + jCOM_LLL_3x33[i][4] + jCOM_LF_3x33[i][4] + jCOM_RUA_3x33[i][4] + jCOM_RLA_3x33[i][4] + jCOM_LUA_3x33[i][4] + jCOM_LLA_3x33[i][4] + jCOM_RH_3x33[i][4] + jCOM_LH_3x33[i][4];
		_jCOM_3x33[i][5] = jCOM_PEL_3x33[i][5] + jCOM_TOR_3x33[i][5] + jCOM_RUL_3x33[i][5] + jCOM_RLL_3x33[i][5] + jCOM_RF_3x33[i][5] + jCOM_LUL_3x33[i][5] + jCOM_LLL_3x33[i][5] + jCOM_LF_3x33[i][5] + jCOM_RUA_3x33[i][5] + jCOM_RLA_3x33[i][5] + jCOM_LUA_3x33[i][5] + jCOM_LLA_3x33[i][5] + jCOM_RH_3x33[i][5] + jCOM_LH_3x33[i][5];
		_jCOM_3x33[i][6] = jCOM_PEL_3x33[i][6] + jCOM_TOR_3x33[i][6] + jCOM_RUL_3x33[i][6] + jCOM_RLL_3x33[i][6] + jCOM_RF_3x33[i][6] + jCOM_LUL_3x33[i][6] + jCOM_LLL_3x33[i][6] + jCOM_LF_3x33[i][6] + jCOM_RUA_3x33[i][6] + jCOM_RLA_3x33[i][6] + jCOM_LUA_3x33[i][6] + jCOM_LLA_3x33[i][6] + jCOM_RH_3x33[i][6] + jCOM_LH_3x33[i][6];
		
		_jCOM_3x33[i][8] = jCOM_RUL_3x33[i][8] + jCOM_RLL_3x33[i][8] + jCOM_RF_3x33[i][8];
		_jCOM_3x33[i][9] = jCOM_RUL_3x33[i][9] + jCOM_RLL_3x33[i][9] + jCOM_RF_3x33[i][9];
		_jCOM_3x33[i][10] = jCOM_RUL_3x33[i][10] + jCOM_RLL_3x33[i][10] + jCOM_RF_3x33[i][10];
		_jCOM_3x33[i][11] = jCOM_RLL_3x33[i][11] + jCOM_RF_3x33[i][11];
		_jCOM_3x33[i][12] = jCOM_RF_3x33[i][12];
		_jCOM_3x33[i][13] = jCOM_RF_3x33[i][13];
		_jCOM_3x33[i][14] = jCOM_LUL_3x33[i][14] + jCOM_LLL_3x33[i][14] + jCOM_LF_3x33[i][14];
		_jCOM_3x33[i][15] = jCOM_LUL_3x33[i][15] + jCOM_LLL_3x33[i][15] + jCOM_LF_3x33[i][15];
		_jCOM_3x33[i][16] = jCOM_LUL_3x33[i][16] + jCOM_LLL_3x33[i][16] + jCOM_LF_3x33[i][16];
		_jCOM_3x33[i][17] = jCOM_LLL_3x33[i][17] + jCOM_LF_3x33[i][17];
		_jCOM_3x33[i][18] = jCOM_LF_3x33[i][18];
		_jCOM_3x33[i][19] = jCOM_LF_3x33[i][19];
		
		_jCOM_3x33[i][7] = jCOM_TOR_3x33[i][7] + jCOM_RUA_3x33[i][7] + jCOM_RLA_3x33[i][7] + jCOM_LUA_3x33[i][7] + jCOM_LLA_3x33[i][7] + jCOM_RH_3x33[i][7] + jCOM_LH_3x33[i][7];
		_jCOM_3x33[i][20] = jCOM_RUA_3x33[i][20] + jCOM_RLA_3x33[i][20] + jCOM_RH_3x33[i][20];
		_jCOM_3x33[i][21] = jCOM_RUA_3x33[i][21] + jCOM_RLA_3x33[i][21] + jCOM_RH_3x33[i][21];
		_jCOM_3x33[i][22] = jCOM_RUA_3x33[i][22] + jCOM_RLA_3x33[i][22] + jCOM_RH_3x33[i][22];
		_jCOM_3x33[i][23] = jCOM_RLA_3x33[i][23] + jCOM_RH_3x33[i][23];
		_jCOM_3x33[i][24] = jCOM_RLA_3x33[i][24] + jCOM_RH_3x33[i][24];
		_jCOM_3x33[i][25] = jCOM_RH_3x33[i][25];

		_jCOM_3x33[i][26] = jCOM_LUA_3x33[i][26] + jCOM_LLA_3x33[i][26] + jCOM_LH_3x33[i][26];
		_jCOM_3x33[i][27] = jCOM_LUA_3x33[i][27] + jCOM_LLA_3x33[i][27] + jCOM_LH_3x33[i][27];	
		_jCOM_3x33[i][28] = jCOM_LUA_3x33[i][28] + jCOM_LLA_3x33[i][28] + jCOM_LH_3x33[i][28];	
		_jCOM_3x33[i][29] = jCOM_LLA_3x33[i][29] + jCOM_LH_3x33[i][29];	
		_jCOM_3x33[i][30] = jCOM_LLA_3x33[i][30] + jCOM_LH_3x33[i][30];
		_jCOM_3x33[i][31] = jCOM_LH_3x33[i][31];

		_jCOM_3x33[i][32] = jCOM_RH_3x33[i][32];
		_jCOM_3x33[i][33] = jCOM_LH_3x33[i][33];
	}

	_pCOM_3x1[1] = ((m_PEL*cPEL[1] +m_TOR*cTOR[1] + m_ULEG*(cRUL[1]+cLUL[1]) + m_LLEG*(cRLL[1]+cLLL[1]) +m_FOOT*(cRF[1]+cLF[1]) + m_UARM*(cRUA[1]+cLUA[1]) + m_LARM*(cRLA[1]+cLLA[1]) + m_RHAND*cRH[1] + m_LHAND*cLH[1])/m_TOTAL);
	_pCOM_3x1[2] = ((m_PEL*cPEL[2] +m_TOR*cTOR[2] + m_ULEG*(cRUL[2]+cLUL[2]) + m_LLEG*(cRLL[2]+cLLL[2]) +m_FOOT*(cRF[2]+cLF[2]) + m_UARM*(cRUA[2]+cLUA[2]) + m_LARM*(cRLA[2]+cLLA[2]) + m_RHAND*cRH[2] + m_LHAND*cLH[2])/m_TOTAL);
	_pCOM_3x1[3] = ((m_PEL*cPEL[3] +m_TOR*cTOR[3] + m_ULEG*(cRUL[3]+cLUL[3]) + m_LLEG*(cRLL[3]+cLLL[3]) +m_FOOT*(cRF[3]+cLF[3]) + m_UARM*(cRUA[3]+cLUA[3]) + m_LARM*(cRLA[3]+cLLA[3]) + m_RHAND*cRH[3] + m_LHAND*cLH[3])/m_TOTAL);

	_jRF_6x33[1][1] = 1.f;
	_jRF_6x33[2][2] = 1.f;
	_jRF_6x33[3][3] = 1.f;
	_jRF_6x33[4][4] = 1.f;
	_jRF_6x33[5][5] = 1.f;
	_jRF_6x33[6][6] = 1.f;

	diff_vv(_pRF_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][4] = temp4_3x1[1];
	_jRF_6x33[2][4] = temp4_3x1[2];
	_jRF_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][5] = temp4_3x1[1];
	_jRF_6x33[2][5] = temp4_3x1[2];
	_jRF_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][6] = temp4_3x1[1];
	_jRF_6x33[2][6] = temp4_3x1[2];
	_jRF_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pRF_3x1,3,pRPEL, temp3_3x1);
	cross(1.f,axis_rhy, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][8] = temp4_3x1[1];
	_jRF_6x33[2][8] = temp4_3x1[2];
	_jRF_6x33[3][8] = temp4_3x1[3];
	_jRF_6x33[4][8] = axis_rhy[1];
	_jRF_6x33[5][8] = axis_rhy[2];
	_jRF_6x33[6][8] = axis_rhy[3];
	cross(1.f,axis_rhr, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][9] = temp4_3x1[1];
	_jRF_6x33[2][9] = temp4_3x1[2];
	_jRF_6x33[3][9] = temp4_3x1[3];
	_jRF_6x33[4][9] = axis_rhr[1];
	_jRF_6x33[5][9] = axis_rhr[2];
	_jRF_6x33[6][9] = axis_rhr[3];
	cross(1.f,axis_rhp, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][10] = temp4_3x1[1];
	_jRF_6x33[2][10] = temp4_3x1[2];
	_jRF_6x33[3][10] = temp4_3x1[3];
	_jRF_6x33[4][10] = axis_rhp[1];
	_jRF_6x33[5][10] = axis_rhp[2];
	_jRF_6x33[6][10] = axis_rhp[3];

	diff_vv(_pRF_3x1,3,pRKN, temp3_3x1);
	cross(1.f,axis_rkn, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][11] = temp4_3x1[1];
	_jRF_6x33[2][11] = temp4_3x1[2];
	_jRF_6x33[3][11] = temp4_3x1[3];
	_jRF_6x33[4][11] = axis_rkn[1];
	_jRF_6x33[5][11] = axis_rkn[2];
	_jRF_6x33[6][11] = axis_rkn[3];

	diff_vv(_pRF_3x1,3,_pRANK_3x1, temp3_3x1);
	cross(1.f,axis_rap, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][12] = temp4_3x1[1];
	_jRF_6x33[2][12] = temp4_3x1[2];
	_jRF_6x33[3][12] = temp4_3x1[3];
	_jRF_6x33[4][12] = axis_rap[1];
	_jRF_6x33[5][12] = axis_rap[2];
	_jRF_6x33[6][12] = axis_rap[3];
	cross(1.f,axis_rar, temp3_3x1, temp4_3x1);
	_jRF_6x33[1][13] = temp4_3x1[1];
	_jRF_6x33[2][13] = temp4_3x1[2];
	_jRF_6x33[3][13] = temp4_3x1[3];
	_jRF_6x33[4][13] = axis_rar[1];
	_jRF_6x33[5][13] = axis_rar[2];
	_jRF_6x33[6][13] = axis_rar[3];


	_jLF_6x33[1][1] = 1.f;
	_jLF_6x33[2][2] = 1.f;
	_jLF_6x33[3][3] = 1.f;
	_jLF_6x33[4][4] = 1.f;
	_jLF_6x33[5][5] = 1.f;
	_jLF_6x33[6][6] = 1.f;

	diff_vv(_pLF_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][4] = temp4_3x1[1];
	_jLF_6x33[2][4] = temp4_3x1[2];
	_jLF_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][5] = temp4_3x1[1];
	_jLF_6x33[2][5] = temp4_3x1[2];
	_jLF_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][6] = temp4_3x1[1];
	_jLF_6x33[2][6] = temp4_3x1[2];
	_jLF_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pLF_3x1,3,pLPEL, temp3_3x1);
	cross(1.f,axis_lhy, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][14] = temp4_3x1[1];
	_jLF_6x33[2][14] = temp4_3x1[2];
	_jLF_6x33[3][14] = temp4_3x1[3];
	_jLF_6x33[4][14] = axis_lhy[1];
	_jLF_6x33[5][14] = axis_lhy[2];
	_jLF_6x33[6][14] = axis_lhy[3];
	cross(1.f,axis_lhr, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][15] = temp4_3x1[1];
	_jLF_6x33[2][15] = temp4_3x1[2];
	_jLF_6x33[3][15] = temp4_3x1[3];
	_jLF_6x33[4][15] = axis_lhr[1];
	_jLF_6x33[5][15] = axis_lhr[2];
	_jLF_6x33[6][15] = axis_lhr[3];
	cross(1.f,axis_lhp, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][16] = temp4_3x1[1];
	_jLF_6x33[2][16] = temp4_3x1[2];
	_jLF_6x33[3][16] = temp4_3x1[3];
	_jLF_6x33[4][16] = axis_lhp[1];
	_jLF_6x33[5][16] = axis_lhp[2];
	_jLF_6x33[6][16] = axis_lhp[3];

	diff_vv(_pLF_3x1,3,pLKN, temp3_3x1);
	cross(1.f,axis_lkn, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][17] = temp4_3x1[1];
	_jLF_6x33[2][17] = temp4_3x1[2];
	_jLF_6x33[3][17] = temp4_3x1[3];
	_jLF_6x33[4][17] = axis_lkn[1];
	_jLF_6x33[5][17] = axis_lkn[2];
	_jLF_6x33[6][17] = axis_lkn[3];

	diff_vv(_pLF_3x1,3,_pLANK_3x1, temp3_3x1);
	cross(1.f,axis_lap, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][18] = temp4_3x1[1];
	_jLF_6x33[2][18] = temp4_3x1[2];
	_jLF_6x33[3][18] = temp4_3x1[3];
	_jLF_6x33[4][18] = axis_lap[1];
	_jLF_6x33[5][18] = axis_lap[2];
	_jLF_6x33[6][18] = axis_lap[3];
	cross(1.f,axis_lar, temp3_3x1, temp4_3x1);
	_jLF_6x33[1][19] = temp4_3x1[1];
	_jLF_6x33[2][19] = temp4_3x1[2];
	_jLF_6x33[3][19] = temp4_3x1[3];
	_jLF_6x33[4][19] = axis_lar[1];
	_jLF_6x33[5][19] = axis_lar[2];
	_jLF_6x33[6][19] = axis_lar[3];

	_jRH_6x33[1][1] = 1.f;
	_jRH_6x33[2][2] = 1.f;
	_jRH_6x33[3][3] = 1.f;
	_jRH_6x33[4][4] = 1.f;
	_jRH_6x33[5][5] = 1.f;
	_jRH_6x33[6][6] = 1.f;



	diff_vv(_pRH_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][4] = temp4_3x1[1];
	_jRH_6x33[2][4] = temp4_3x1[2];
	_jRH_6x33[3][4] = temp4_3x1[3];

	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][5] = temp4_3x1[1];
	_jRH_6x33[2][5] = temp4_3x1[2];
	_jRH_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][6] = temp4_3x1[1];
	_jRH_6x33[2][6] = temp4_3x1[2];
	_jRH_6x33[3][6] = temp4_3x1[3];
        /*
        printf("jax %f %f %f\n",_jRH_6x33[1][4],_jRH_6x33[2][4],_jRH_6x33[3][4]);
        printf("jay %f %f %f\n",_jRH_6x33[1][5],_jRH_6x33[2][5],_jRH_6x33[3][5]);
        printf("jaz %f %f %f\n",_jRH_6x33[1][6],_jRH_6x33[2][6],_jRH_6x33[3][6]);
	*/
        diff_vv(_pRH_3x1,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][7] = temp4_3x1[1];
	_jRH_6x33[2][7] = temp4_3x1[2];
	_jRH_6x33[3][7] = temp4_3x1[3];
	_jRH_6x33[4][7] = axis_wst[1];
	_jRH_6x33[5][7] = axis_wst[2];
	_jRH_6x33[6][7] = axis_wst[3];

        //printf("jawst1 %f %f %f\n",_jRH_6x33[1][7],_jRH_6x33[2][7],_jRH_6x33[3][7]);
        //printf("jawst2 %f %f %f\n",_jRH_6x33[4][7],_jRH_6x33[5][7],_jRH_6x33[6][7]);


	diff_vv(_pRH_3x1,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][20] = temp4_3x1[1];
	_jRH_6x33[2][20] = temp4_3x1[2];
	_jRH_6x33[3][20] = temp4_3x1[3];
	_jRH_6x33[4][20] = axis_rsp[1];
	_jRH_6x33[5][20] = axis_rsp[2];
	_jRH_6x33[6][20] = axis_rsp[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][21] = temp4_3x1[1];
	_jRH_6x33[2][21] = temp4_3x1[2];
	_jRH_6x33[3][21] = temp4_3x1[3];
	_jRH_6x33[4][21] = axis_rsr[1];
	_jRH_6x33[5][21] = axis_rsr[2];
	_jRH_6x33[6][21] = axis_rsr[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][22] = temp4_3x1[1];
	_jRH_6x33[2][22] = temp4_3x1[2];
	_jRH_6x33[3][22] = temp4_3x1[3];
	_jRH_6x33[4][22] = axis_rsy[1];
	_jRH_6x33[5][22] = axis_rsy[2];
	_jRH_6x33[6][22] = axis_rsy[3];

	diff_vv(_pRH_3x1,3,pRELB, temp3_3x1);
	cross(1.f,axis_reb, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][23] = temp4_3x1[1];
	_jRH_6x33[2][23] = temp4_3x1[2];
	_jRH_6x33[3][23] = temp4_3x1[3];
	_jRH_6x33[4][23] = axis_reb[1];
	_jRH_6x33[5][23] = axis_reb[2];
	_jRH_6x33[6][23] = axis_reb[3];
	
	diff_vv(_pRH_3x1,3, pRELB2, temp3_3x1);
	cross(1.f,axis_rwy, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][24] = temp4_3x1[1];
	_jRH_6x33[2][24] = temp4_3x1[2];
	_jRH_6x33[3][24] = temp4_3x1[3];
	_jRH_6x33[4][24] = axis_rwy[1];
	_jRH_6x33[5][24] = axis_rwy[2];
	_jRH_6x33[6][24] = axis_rwy[3];

	diff_vv(_pRH_3x1,3,_pRWR_3x1, temp3_3x1);
	cross(1.f,axis_rwp, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][25] = temp4_3x1[1];
	_jRH_6x33[2][25] = temp4_3x1[2];
	_jRH_6x33[3][25] = temp4_3x1[3];
	_jRH_6x33[4][25] = axis_rwp[1];
	_jRH_6x33[5][25] = axis_rwp[2];
	_jRH_6x33[6][25] = axis_rwp[3];
	cross(1.f,axis_rwy2, temp3_3x1, temp4_3x1);
	_jRH_6x33[1][32] = temp4_3x1[1];
	_jRH_6x33[2][32] = temp4_3x1[2];
	_jRH_6x33[3][32] = temp4_3x1[3];
	_jRH_6x33[4][32] = axis_rwy2[1];
	_jRH_6x33[5][32] = axis_rwy2[2];
	_jRH_6x33[6][32] = axis_rwy2[3];
	
	_jLH_6x33[1][1] = 1.f;
	_jLH_6x33[2][2] = 1.f;
	_jLH_6x33[3][3] = 1.f;
	_jLH_6x33[4][4] = 1.f;
	_jLH_6x33[5][5] = 1.f;
	_jLH_6x33[6][6] = 1.f;

	diff_vv(_pLH_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][4] = temp4_3x1[1];
	_jLH_6x33[2][4] = temp4_3x1[2];
	_jLH_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][5] = temp4_3x1[1];
	_jLH_6x33[2][5] = temp4_3x1[2];
	_jLH_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][6] = temp4_3x1[1];
	_jLH_6x33[2][6] = temp4_3x1[2];
	_jLH_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pLH_3x1,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][7] = temp4_3x1[1];
	_jLH_6x33[2][7] = temp4_3x1[2];
	_jLH_6x33[3][7] = temp4_3x1[3];
	_jLH_6x33[4][7] = axis_wst[1];
	_jLH_6x33[5][7] = axis_wst[2];
	_jLH_6x33[6][7] = axis_wst[3];

	diff_vv(_pLH_3x1,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][26] = temp4_3x1[1];
	_jLH_6x33[2][26] = temp4_3x1[2];
	_jLH_6x33[3][26] = temp4_3x1[3];
	_jLH_6x33[4][26] = axis_lsp[1];
	_jLH_6x33[5][26] = axis_lsp[2];
	_jLH_6x33[6][26] = axis_lsp[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][27] = temp4_3x1[1];
	_jLH_6x33[2][27] = temp4_3x1[2];
	_jLH_6x33[3][27] = temp4_3x1[3];
	_jLH_6x33[4][27] = axis_lsr[1];
	_jLH_6x33[5][27] = axis_lsr[2];
	_jLH_6x33[6][27] = axis_lsr[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][28] = temp4_3x1[1];
	_jLH_6x33[2][28] = temp4_3x1[2];
	_jLH_6x33[3][28] = temp4_3x1[3];
	_jLH_6x33[4][28] = axis_lsy[1];
	_jLH_6x33[5][28] = axis_lsy[2];
	_jLH_6x33[6][28] = axis_lsy[3];

	diff_vv(_pLH_3x1,3,pLELB, temp3_3x1);
	cross(1.f,axis_leb, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][29] = temp4_3x1[1];
	_jLH_6x33[2][29] = temp4_3x1[2];
	_jLH_6x33[3][29] = temp4_3x1[3];
	_jLH_6x33[4][29] = axis_leb[1];
	_jLH_6x33[5][29] = axis_leb[2];
	_jLH_6x33[6][29] = axis_leb[3];

	diff_vv(_pLH_3x1,3, pLELB2, temp3_3x1);
	cross(1.f,axis_lwy, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][30] = temp4_3x1[1];
	_jLH_6x33[2][30] = temp4_3x1[2];
	_jLH_6x33[3][30] = temp4_3x1[3];
	_jLH_6x33[4][30] = axis_lwy[1];
	_jLH_6x33[5][30] = axis_lwy[2];
	_jLH_6x33[6][30] = axis_lwy[3];

	diff_vv(_pLH_3x1,3,_pLWR_3x1, temp3_3x1);
	cross(1.f,axis_lwp, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][31] = temp4_3x1[1];
	_jLH_6x33[2][31] = temp4_3x1[2];
	_jLH_6x33[3][31] = temp4_3x1[3];
	_jLH_6x33[4][31] = axis_lwp[1];
	_jLH_6x33[5][31] = axis_lwp[2];
	_jLH_6x33[6][31] = axis_lwp[3];
	cross(1.f,axis_lwy2, temp3_3x1, temp4_3x1);
	_jLH_6x33[1][33] = temp4_3x1[1];
	_jLH_6x33[2][33] = temp4_3x1[2];
	_jLH_6x33[3][33] = temp4_3x1[3];
	_jLH_6x33[4][33] = axis_lwy2[1];
	_jLH_6x33[5][33] = axis_lwy2[2];
	_jLH_6x33[6][33] = axis_lwy2[3];

	_jRS_6x33[1][1] = 1.f;
	_jRS_6x33[2][2] = 1.f;
	_jRS_6x33[3][3] = 1.f;
	_jRS_6x33[4][4] = 1.f;
	_jRS_6x33[5][5] = 1.f;
	_jRS_6x33[6][6] = 1.f;

	diff_vv(_pRS_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][4] = temp4_3x1[1];
	_jRS_6x33[2][4] = temp4_3x1[2];
	_jRS_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][5] = temp4_3x1[1];
	_jRS_6x33[2][5] = temp4_3x1[2];
	_jRS_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][6] = temp4_3x1[1];
	_jRS_6x33[2][6] = temp4_3x1[2];
	_jRS_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pRS_3x1,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][7] = temp4_3x1[1];
	_jRS_6x33[2][7] = temp4_3x1[2];
	_jRS_6x33[3][7] = temp4_3x1[3];
	_jRS_6x33[4][7] = axis_wst[1];
	_jRS_6x33[5][7] = axis_wst[2];
	_jRS_6x33[6][7] = axis_wst[3];

	diff_vv(_pRS_3x1,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][20] = temp4_3x1[1];
	_jRS_6x33[2][20] = temp4_3x1[2];
	_jRS_6x33[3][20] = temp4_3x1[3];
	_jRS_6x33[4][20] = axis_rsp[1];
	_jRS_6x33[5][20] = axis_rsp[2];
	_jRS_6x33[6][20] = axis_rsp[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][21] = temp4_3x1[1];
	_jRS_6x33[2][21] = temp4_3x1[2];
	_jRS_6x33[3][21] = temp4_3x1[3];
	_jRS_6x33[4][21] = axis_rsr[1];
	_jRS_6x33[5][21] = axis_rsr[2];
	_jRS_6x33[6][21] = axis_rsr[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][22] = temp4_3x1[1];
	_jRS_6x33[2][22] = temp4_3x1[2];
	_jRS_6x33[3][22] = temp4_3x1[3];
	_jRS_6x33[4][22] = axis_rsy[1];
	_jRS_6x33[5][22] = axis_rsy[2];
	_jRS_6x33[6][22] = axis_rsy[3];

	diff_vv(_pRS_3x1,3,pRELB, temp3_3x1);
	cross(1.f,axis_reb, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][23] = temp4_3x1[1];
	_jRS_6x33[2][23] = temp4_3x1[2];
	_jRS_6x33[3][23] = temp4_3x1[3];
	_jRS_6x33[4][23] = axis_reb[1];
	_jRS_6x33[5][23] = axis_reb[2];
	_jRS_6x33[6][23] = axis_reb[3];
	
	diff_vv(_pRS_3x1,3, pRELB2, temp3_3x1);
	cross(1.f,axis_rwy, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][24] = temp4_3x1[1];
	_jRS_6x33[2][24] = temp4_3x1[2];
	_jRS_6x33[3][24] = temp4_3x1[3];
	_jRS_6x33[4][24] = axis_rwy[1];
	_jRS_6x33[5][24] = axis_rwy[2];
	_jRS_6x33[6][24] = axis_rwy[3];

	diff_vv(_pRS_3x1,3,_pRWR_3x1, temp3_3x1);
	cross(1.f,axis_rwp, temp3_3x1, temp4_3x1);
	_jRS_6x33[1][25] = temp4_3x1[1];
	_jRS_6x33[2][25] = temp4_3x1[2];
	_jRS_6x33[3][25] = temp4_3x1[3];
	_jRS_6x33[4][25] = axis_rwp[1];
	_jRS_6x33[5][25] = axis_rwp[2];
	_jRS_6x33[6][25] = axis_rwp[3];
	
	_jLS_6x33[1][1] = 1.f;
	_jLS_6x33[2][2] = 1.f;
	_jLS_6x33[3][3] = 1.f;
	_jLS_6x33[4][4] = 1.f;
	_jLS_6x33[5][5] = 1.f;
	_jLS_6x33[6][6] = 1.f;

	diff_vv(_pLS_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][4] = temp4_3x1[1];
	_jLS_6x33[2][4] = temp4_3x1[2];
	_jLS_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][5] = temp4_3x1[1];
	_jLS_6x33[2][5] = temp4_3x1[2];
	_jLS_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][6] = temp4_3x1[1];
	_jLS_6x33[2][6] = temp4_3x1[2];
	_jLS_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pLS_3x1,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][7] = temp4_3x1[1];
	_jLS_6x33[2][7] = temp4_3x1[2];
	_jLS_6x33[3][7] = temp4_3x1[3];
	_jLS_6x33[4][7] = axis_wst[1];
	_jLS_6x33[5][7] = axis_wst[2];
	_jLS_6x33[6][7] = axis_wst[3];

	diff_vv(_pLS_3x1,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][26] = temp4_3x1[1];
	_jLS_6x33[2][26] = temp4_3x1[2];
	_jLS_6x33[3][26] = temp4_3x1[3];
	_jLS_6x33[4][26] = axis_lsp[1];
	_jLS_6x33[5][26] = axis_lsp[2];
	_jLS_6x33[6][26] = axis_lsp[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][27] = temp4_3x1[1];
	_jLS_6x33[2][27] = temp4_3x1[2];
	_jLS_6x33[3][27] = temp4_3x1[3];
	_jLS_6x33[4][27] = axis_lsr[1];
	_jLS_6x33[5][27] = axis_lsr[2];
	_jLS_6x33[6][27] = axis_lsr[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][28] = temp4_3x1[1];
	_jLS_6x33[2][28] = temp4_3x1[2];
	_jLS_6x33[3][28] = temp4_3x1[3];
	_jLS_6x33[4][28] = axis_lsy[1];
	_jLS_6x33[5][28] = axis_lsy[2];
	_jLS_6x33[6][28] = axis_lsy[3];

	diff_vv(_pLS_3x1,3,pLELB, temp3_3x1);
	cross(1.f,axis_leb, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][29] = temp4_3x1[1];
	_jLS_6x33[2][29] = temp4_3x1[2];
	_jLS_6x33[3][29] = temp4_3x1[3];
	_jLS_6x33[4][29] = axis_leb[1];
	_jLS_6x33[5][29] = axis_leb[2];
	_jLS_6x33[6][29] = axis_leb[3];

	diff_vv(_pLS_3x1,3, pLELB2, temp3_3x1);
	cross(1.f,axis_lwy, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][30] = temp4_3x1[1];
	_jLS_6x33[2][30] = temp4_3x1[2];
	_jLS_6x33[3][30] = temp4_3x1[3];
	_jLS_6x33[4][30] = axis_lwy[1];
	_jLS_6x33[5][30] = axis_lwy[2];
	_jLS_6x33[6][30] = axis_lwy[3];

	diff_vv(_pLS_3x1,3,_pLWR_3x1, temp3_3x1);
	cross(1.f,axis_lwp, temp3_3x1, temp4_3x1);
	_jLS_6x33[1][31] = temp4_3x1[1];
	_jLS_6x33[2][31] = temp4_3x1[2];
	_jLS_6x33[3][31] = temp4_3x1[3];
	_jLS_6x33[4][31] = axis_lwp[1];
	_jLS_6x33[5][31] = axis_lwp[2];
	_jLS_6x33[6][31] = axis_lwp[3];


	_jRWR_6x33[1][1] = 1.f;
	_jRWR_6x33[2][2] = 1.f;
	_jRWR_6x33[3][3] = 1.f;
	_jRWR_6x33[4][4] = 1.f;
	_jRWR_6x33[5][5] = 1.f;
	_jRWR_6x33[6][6] = 1.f;

	diff_vv(_pRWR_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][4] = temp4_3x1[1];
	_jRWR_6x33[2][4] = temp4_3x1[2];
	_jRWR_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][5] = temp4_3x1[1];
	_jRWR_6x33[2][5] = temp4_3x1[2];
	_jRWR_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][6] = temp4_3x1[1];
	_jRWR_6x33[2][6] = temp4_3x1[2];
	_jRWR_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pRWR_3x1,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][7] = temp4_3x1[1];
	_jRWR_6x33[2][7] = temp4_3x1[2];
	_jRWR_6x33[3][7] = temp4_3x1[3];
	_jRWR_6x33[4][7] = axis_wst[1];
	_jRWR_6x33[5][7] = axis_wst[2];
	_jRWR_6x33[6][7] = axis_wst[3];

	diff_vv(_pRWR_3x1,3,pRSHLD, temp3_3x1);
	cross(1.f,axis_rsp, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][20] = temp4_3x1[1];
	_jRWR_6x33[2][20] = temp4_3x1[2];
	_jRWR_6x33[3][20] = temp4_3x1[3];
	_jRWR_6x33[4][20] = axis_rsp[1];
	_jRWR_6x33[5][20] = axis_rsp[2];
	_jRWR_6x33[6][20] = axis_rsp[3];
	cross(1.f,axis_rsr, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][21] = temp4_3x1[1];
	_jRWR_6x33[2][21] = temp4_3x1[2];
	_jRWR_6x33[3][21] = temp4_3x1[3];
	_jRWR_6x33[4][21] = axis_rsr[1];
	_jRWR_6x33[5][21] = axis_rsr[2];
	_jRWR_6x33[6][21] = axis_rsr[3];
	cross(1.f,axis_rsy, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][22] = temp4_3x1[1];
	_jRWR_6x33[2][22] = temp4_3x1[2];
	_jRWR_6x33[3][22] = temp4_3x1[3];
	_jRWR_6x33[4][22] = axis_rsy[1];
	_jRWR_6x33[5][22] = axis_rsy[2];
	_jRWR_6x33[6][22] = axis_rsy[3];

	diff_vv(_pRWR_3x1,3,pRELB, temp3_3x1);
	cross(1.f,axis_reb, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][23] = temp4_3x1[1];
	_jRWR_6x33[2][23] = temp4_3x1[2];
	_jRWR_6x33[3][23] = temp4_3x1[3];
	_jRWR_6x33[4][23] = axis_reb[1];
	_jRWR_6x33[5][23] = axis_reb[2];
	_jRWR_6x33[6][23] = axis_reb[3];
	
	diff_vv(_pRWR_3x1,3, pRELB2, temp3_3x1);
	cross(1.f,axis_rwy, temp3_3x1, temp4_3x1);
	_jRWR_6x33[1][24] = temp4_3x1[1];
	_jRWR_6x33[2][24] = temp4_3x1[2];
	_jRWR_6x33[3][24] = temp4_3x1[3];
	_jRWR_6x33[4][24] = axis_rwy[1];
	_jRWR_6x33[5][24] = axis_rwy[2];
	_jRWR_6x33[6][24] = axis_rwy[3];


	_jLWR_6x33[1][1] = 1.f;
	_jLWR_6x33[2][2] = 1.f;
	_jLWR_6x33[3][3] = 1.f;
	_jLWR_6x33[4][4] = 1.f;
	_jLWR_6x33[5][5] = 1.f;
	_jLWR_6x33[6][6] = 1.f;

	diff_vv(_pLWR_3x1,3,pPC, temp3_3x1);
	cross(1.f,_AXIS_X, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][4] = temp4_3x1[1];
	_jLWR_6x33[2][4] = temp4_3x1[2];
	_jLWR_6x33[3][4] = temp4_3x1[3];
	cross(1.f,_AXIS_Y, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][5] = temp4_3x1[1];
	_jLWR_6x33[2][5] = temp4_3x1[2];
	_jLWR_6x33[3][5] = temp4_3x1[3];
	cross(1.f,_AXIS_Z, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][6] = temp4_3x1[1];
	_jLWR_6x33[2][6] = temp4_3x1[2];
	_jLWR_6x33[3][6] = temp4_3x1[3];

	diff_vv(_pLWR_3x1,3,pWST, temp3_3x1);
	cross(1.f,axis_wst, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][7] = temp4_3x1[1];
	_jLWR_6x33[2][7] = temp4_3x1[2];
	_jLWR_6x33[3][7] = temp4_3x1[3];
	_jLWR_6x33[4][7] = axis_wst[1];
	_jLWR_6x33[5][7] = axis_wst[2];
	_jLWR_6x33[6][7] = axis_wst[3];

	diff_vv(_pLWR_3x1,3,pLSHLD, temp3_3x1);
	cross(1.f,axis_lsp, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][26] = temp4_3x1[1];
	_jLWR_6x33[2][26] = temp4_3x1[2];
	_jLWR_6x33[3][26] = temp4_3x1[3];
	_jLWR_6x33[4][26] = axis_lsp[1];
	_jLWR_6x33[5][26] = axis_lsp[2];
	_jLWR_6x33[6][26] = axis_lsp[3];
	cross(1.f,axis_lsr, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][27] = temp4_3x1[1];
	_jLWR_6x33[2][27] = temp4_3x1[2];
	_jLWR_6x33[3][27] = temp4_3x1[3];
	_jLWR_6x33[4][27] = axis_lsr[1];
	_jLWR_6x33[5][27] = axis_lsr[2];
	_jLWR_6x33[6][27] = axis_lsr[3];
	cross(1.f,axis_lsy, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][28] = temp4_3x1[1];
	_jLWR_6x33[2][28] = temp4_3x1[2];
	_jLWR_6x33[3][28] = temp4_3x1[3];
	_jLWR_6x33[4][28] = axis_lsy[1];
	_jLWR_6x33[5][28] = axis_lsy[2];
	_jLWR_6x33[6][28] = axis_lsy[3];

	diff_vv(_pLWR_3x1,3,pLELB, temp3_3x1);
	cross(1.f,axis_leb, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][29] = temp4_3x1[1];
	_jLWR_6x33[2][29] = temp4_3x1[2];
	_jLWR_6x33[3][29] = temp4_3x1[3];
	_jLWR_6x33[4][29] = axis_leb[1];
	_jLWR_6x33[5][29] = axis_leb[2];
	_jLWR_6x33[6][29] = axis_leb[3];

	diff_vv(_pLWR_3x1,3, pLELB2, temp3_3x1);
	cross(1.f,axis_lwy, temp3_3x1, temp4_3x1);
	_jLWR_6x33[1][30] = temp4_3x1[1];
	_jLWR_6x33[2][30] = temp4_3x1[2];
	_jLWR_6x33[3][30] = temp4_3x1[3];
	_jLWR_6x33[4][30] = axis_lwy[1];
	_jLWR_6x33[5][30] = axis_lwy[2];
	_jLWR_6x33[6][30] = axis_lwy[3];
	
	DC2QT((const float**)_dcRF_3x3, _qRF_4x1);
	DC2QT((const float**)_dcLF_3x3, _qLF_4x1);

	DC2QT((const float**)_dcRH_3x3, _qRH_4x1);
	DC2QT((const float**)_dcLH_3x3, _qLH_4x1);

	DC2QT((const float**)_dcRS_3x3, _qRS_4x1);
	DC2QT((const float**)_dcLS_3x3, _qLS_4x1);

	DC2QT((const float**)_dcRLA_3x3, _qRWR_4x1);
	DC2QT((const float**)_dcLLA_3x3, _qLWR_4x1);
	
	//-------------------------------------- local coordinates
        /*printf("\nCOM %f %f %f %f\n",_pCOM_3x1[1],_pCOM_3x1[2],_pCOM_3x1[3],pPC);
        printf("prh %f %f %f %f\n",_pRH_3x1[1],_pRH_3x1[2],_pRH_3x1[3],pPC);
        printf("plh %f %f %f %f\n",_pLH_3x1[1],_pLH_3x1[2],_pLH_3x1[3],pPC);
        printf("prF %f %f %f %f\n",_pRF_3x1[1],_pRF_3x1[2],_pRF_3x1[3],pPC);
        printf("plF %f %f %f %f\n",_pLF_3x1[1],_pLF_3x1[2],_pLF_3x1[3],pPC);
*/
	diff_vv(_pRH_3x1,3, pPC, _pRH_L_3x1);
	diff_vv(_pLH_3x1,3, pPC, _pLH_L_3x1);
	diff_vv(_pRWR_3x1,3, pPC, _pRWR_L_3x1);
	diff_vv(_pLWR_3x1,3, pPC, _pLWR_L_3x1);
	diff_vv(_pRF_3x1,3, pPC, _pRF_L_3x1);
	diff_vv(_pLF_3x1,3, pPC, _pLF_L_3x1);
	diff_vv(_pRANK_3x1,3, pPC, _pRANK_L_3x1);
	diff_vv(_pLANK_3x1,3, pPC, _pLANK_L_3x1);
	diff_vv(_pRS_3x1,3, pPC, _pRS_L_3x1);
	diff_vv(_pLS_3x1,3, pPC, _pLS_L_3x1);

	trans(1.f, (const float**)_dcPEL_3x3, 3,3,_TEMP1_34x34);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcRF_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qRF_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcLF_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qLF_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcRH_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qRH_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcLH_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qLH_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcRS_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qRS_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcLS_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qLS_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcRLA_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qRWR_L_4x1);
	mult_mm((const float**)_TEMP1_34x34,3,3, (const float**)_dcLLA_3x3,3, _TEMP2_34x34);
	DC2QT((const float**)_TEMP2_34x34, _qLWR_L_4x1);

	free_matrix(jCOM_PEL_3x33, 1,3,1,33);
	free_matrix(jCOM_RUL_3x33, 1,3,1,33);
	free_matrix(jCOM_RLL_3x33, 1,3,1,33);
	free_matrix(jCOM_RF_3x33, 1,3,1,33);
	free_matrix(jCOM_LUL_3x33, 1,3,1,33);
	free_matrix(jCOM_LLL_3x33, 1,3,1,33);
	free_matrix(jCOM_LF_3x33, 1,3,1,33);
	free_matrix(jCOM_TOR_3x33, 1,3,1,33);
	free_matrix(jCOM_RUA_3x33, 1,3,1,33);
	free_matrix(jCOM_RLA_3x33, 1,3,1,33);
	free_matrix(jCOM_LUA_3x33, 1,3,1,33);
	free_matrix(jCOM_LLA_3x33, 1,3,1,33);
	free_matrix(jCOM_LH_3x33, 1,3,1,33);
	free_matrix(jCOM_RH_3x33, 1,3,1,33);

	return 1;
}





int QT2DC(const float qt_4x1[5], float **DC_3x3)		// convert a quaternion to a direction cosine matrix
{
	float q0 = qt_4x1[1];
	float q1 = qt_4x1[2];
	float q2 = qt_4x1[3];
	float q3 = qt_4x1[4];

	DC_3x3[1][1] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
	DC_3x3[1][2] = 2*(q1*q2-q0*q3);
	DC_3x3[1][3] = 2*(q1*q3+q0*q2);
	DC_3x3[2][1] = 2*(q1*q2+q0*q3);
	DC_3x3[2][2] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
	DC_3x3[2][3] = 2*(q2*q3-q0*q1);
	DC_3x3[3][1] = 2*(q1*q3-q0*q2);
	DC_3x3[3][2] = 2*(q2*q3+q0*q1);
	DC_3x3[3][3] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	return 0;
}



int DC2QT(const float **DC_3x3, float qt_4x1[5])
{
	unsigned int index;
	float q_sq[4], temp, max;

	q_sq[0] = 0.25f*(1.f + DC_3x3[1][1] + DC_3x3[2][2] + DC_3x3[3][3]);
	q_sq[1] = 0.25f*(1.f + DC_3x3[1][1] - DC_3x3[2][2] - DC_3x3[3][3]);
	q_sq[2] = 0.25f*(1.f - DC_3x3[1][1] + DC_3x3[2][2] - DC_3x3[3][3]);
	q_sq[3] = 0.25f*(1.f - DC_3x3[1][1] - DC_3x3[2][2] + DC_3x3[3][3]);

	findmax(q_sq, 4, &max, &index);

	switch(index)
	{
	case 0:
		qt_4x1[1] = (float)sqrt(max);
		temp = 4.f*qt_4x1[1];
		qt_4x1[2] = (DC_3x3[3][2]-DC_3x3[2][3])/temp;
		qt_4x1[3] = (DC_3x3[1][3]-DC_3x3[3][1])/temp;
		qt_4x1[4] = (DC_3x3[2][1]-DC_3x3[1][2])/temp;
		break;
	case 1:
		qt_4x1[2] = (float)sqrt(max);
		temp = 4.f*qt_4x1[2];
		qt_4x1[1] = (DC_3x3[3][2]-DC_3x3[2][3])/temp;
		qt_4x1[3] = (DC_3x3[2][1]+DC_3x3[1][2])/temp;
		qt_4x1[4] = (DC_3x3[1][3]+DC_3x3[3][1])/temp;
		break;
	case 2:
		qt_4x1[3] = (float)sqrt(max);
		temp = 4.f*qt_4x1[3];
		qt_4x1[1] = (DC_3x3[1][3]-DC_3x3[3][1])/temp;
		qt_4x1[2] = (DC_3x3[2][1]+DC_3x3[1][2])/temp;
		qt_4x1[4] = (DC_3x3[3][2]+DC_3x3[2][3])/temp;
		break;
	case 3:
		qt_4x1[4] = (float)sqrt(max);
		temp = 4.f*qt_4x1[4];
		qt_4x1[1] = (DC_3x3[2][1]-DC_3x3[1][2])/temp;
		qt_4x1[2] = (DC_3x3[1][3]-DC_3x3[3][1])/temp;
		qt_4x1[3] = (DC_3x3[3][2]+DC_3x3[2][3])/temp;
		break;
	}

	return 0;
}


int QTdel(const float *des_qt_4x1, const float *qt_4x1, float *result_3x1) // delta quaternion
{
	/*//----------------- original
	float q0;
	float des_q0 = des_qt_4x1[1];
	float q[4], des_q[4];
	float temp[4];
	float qt_4x1[5];

	float **temp_3x3;
	temp_3x3 = matrix(1,3,1,3);

	QT2DC((const float*)qt_4x1, temp_3x3);
	DC2QT((const float**)temp_3x3, qt_4x1);

	q0 = qt_4x1[1];
	q[1] = qt_4x1[2];
	q[2] = qt_4x1[3];
	q[3] = qt_4x1[4];

	des_q[1] = des_qt_4x1[2];
	des_q[2] = des_qt_4x1[3];
	des_q[3] = des_qt_4x1[4];

	cross(1.f, q, des_q, temp);

	result_3x1[1] = q0*des_q[1] - des_q0*q[1] - temp[1];
	result_3x1[2] = q0*des_q[2] - des_q0*q[2] - temp[2];
	result_3x1[3] = q0*des_q[3] - des_q0*q[3] - temp[3];

	free_matrix(temp_3x3, 1,3,1,3);

	return 0;
	//--------------*/

	//----------------using DC
	float temp1[4];	
	float **temp3_3x3, **temp4_3x3;
	temp3_3x3 = matrix(1,3,1,3);
	temp4_3x3 = matrix(1,3,1,3);

	result_3x1[1] = 0.f;  
	result_3x1[2] = 0.f;
	result_3x1[3] = 0.f;

	QT2DC((const float*)qt_4x1, temp3_3x3);
	QT2DC((const float*)des_qt_4x1, temp4_3x3);

	trans2(1.f, temp3_3x3, 3,3);
	trans2(1.f, temp4_3x3, 3,3);

	cross(1.f, (const float*)temp3_3x3[1], (const float*)temp4_3x3[1], result_3x1);  
	cross(1.f, (const float*)temp3_3x3[2], (const float*)temp4_3x3[2], temp1);
	result_3x1[1] += temp1[1];
	result_3x1[2] += temp1[2];
	result_3x1[3] += temp1[3];

	cross(1.f, (const float*)temp3_3x3[3], (const float*)temp4_3x3[3], temp1);
	result_3x1[1] += temp1[1];
	result_3x1[2] += temp1[2];
	result_3x1[3] += temp1[3];

	result_3x1[1] *= 0.5f;
	result_3x1[2] *= 0.5f;
	result_3x1[3] *= 0.5f;

	free_matrix(temp3_3x3, 1,3,1,3);
	free_matrix(temp4_3x3, 1,3,1,3);

	return 0;
}


int Wq(int ref, const float *qt_4x1, float **Wq_3x4)
{
  if(ref == 0) // global frame  
  {
    Wq_3x4[1][1] = -qt_4x1[2];
    Wq_3x4[1][2] = qt_4x1[1];
    Wq_3x4[1][3] = -qt_4x1[4];
    Wq_3x4[1][4] = qt_4x1[3];
    Wq_3x4[2][1] = -qt_4x1[3];
    Wq_3x4[2][2] = qt_4x1[4];
    Wq_3x4[2][3] = qt_4x1[1];
    Wq_3x4[2][4] = -qt_4x1[2];
    Wq_3x4[3][1] = -qt_4x1[4];
    Wq_3x4[3][2] = -qt_4x1[3];
    Wq_3x4[3][3] = qt_4x1[2];
    Wq_3x4[3][4] = qt_4x1[1];
  }
  else // body frame
  {
    Wq_3x4[1][1] = -qt_4x1[2];
    Wq_3x4[1][2] = qt_4x1[1];
    Wq_3x4[1][3] = qt_4x1[4];
    Wq_3x4[1][4] = -qt_4x1[3];
    Wq_3x4[2][1] = -qt_4x1[3];
    Wq_3x4[2][2] = -qt_4x1[4];
    Wq_3x4[2][3] = qt_4x1[1];
    Wq_3x4[2][4] = qt_4x1[2];
    Wq_3x4[3][1] = -qt_4x1[4];
    Wq_3x4[3][2] = qt_4x1[3];
    Wq_3x4[3][3] = -qt_4x1[2];
    Wq_3x4[3][4] = qt_4x1[1];
  }
  return 0;
}


int Qq(int ref, const float *qt_4x1, float **Qq_4x4) // quaternion matrix
{
  // qt1 * qt2 = Qq(qt1)*qt2 = Qq_(qt2)*qt1
  // dc1*dc2 = Qq_(qt1)*qt2
	
  if(ref==0)  // return Qq
  {
    Qq_4x4[1][1] = qt_4x1[1];
    Qq_4x4[1][2] = -qt_4x1[2];
    Qq_4x4[1][3] = -qt_4x1[3];
    Qq_4x4[1][4] = -qt_4x1[4];
    
    Qq_4x4[2][1] = qt_4x1[2];
    Qq_4x4[2][2] = qt_4x1[1];
    Qq_4x4[2][3] = qt_4x1[4];
    Qq_4x4[2][4] = -qt_4x1[3];
    
    Qq_4x4[3][1] = qt_4x1[3];
    Qq_4x4[3][2] = -qt_4x1[4];
    Qq_4x4[3][3] = qt_4x1[1];
    Qq_4x4[3][4] = qt_4x1[2];
    
    Qq_4x4[4][1] = qt_4x1[4];
    Qq_4x4[4][2] = qt_4x1[3];
    Qq_4x4[4][3] = -qt_4x1[2];
    Qq_4x4[4][4] = qt_4x1[1];
    
  }
  else  // return Qq_
  {
    Qq_4x4[1][1] = qt_4x1[1];
    Qq_4x4[1][2] = -qt_4x1[2];
    Qq_4x4[1][3] = -qt_4x1[3];
    Qq_4x4[1][4] = -qt_4x1[4];
    
    Qq_4x4[2][1] = qt_4x1[2];
    Qq_4x4[2][2] = qt_4x1[1];
    Qq_4x4[2][3] = -qt_4x1[4];
    Qq_4x4[2][4] = qt_4x1[3];
    
    Qq_4x4[3][1] = qt_4x1[3];
    Qq_4x4[3][2] = qt_4x1[4];
    Qq_4x4[3][3] = qt_4x1[1];
    Qq_4x4[3][4] = -qt_4x1[2];
    
    Qq_4x4[4][1] = qt_4x1[4];
    Qq_4x4[4][2] = -qt_4x1[3];
    Qq_4x4[4][3] = qt_4x1[2];
    Qq_4x4[4][4] = qt_4x1[1];
  }
  return 0;
}


int QTcross(const float *qt1_4x1, const float *qt2_4x1, float *result_4x1)
{
	result_4x1[1] = qt1_4x1[1]*qt2_4x1[1] - qt1_4x1[2]*qt2_4x1[2] - qt1_4x1[3]*qt2_4x1[3] - qt1_4x1[4]*qt2_4x1[4];
	result_4x1[2] = qt1_4x1[1]*qt2_4x1[2] + qt1_4x1[2]*qt2_4x1[1] + qt1_4x1[3]*qt2_4x1[4] - qt1_4x1[4]*qt2_4x1[3];
	result_4x1[3] = qt1_4x1[1]*qt2_4x1[3] + qt1_4x1[3]*qt2_4x1[1] - qt1_4x1[2]*qt2_4x1[4] + qt1_4x1[4]*qt2_4x1[2];
	result_4x1[4] = qt1_4x1[1]*qt2_4x1[4] + qt1_4x1[2]*qt2_4x1[3] - qt1_4x1[3]*qt2_4x1[2] + qt1_4x1[4]*qt2_4x1[1];

	return 0;
}

int QTbar(const float *qt_4x1, float *result_4x1)
{
	result_4x1[1] = qt_4x1[1];
	result_4x1[2] = -qt_4x1[2];
	result_4x1[3] = -qt_4x1[3];
	result_4x1[4] = -qt_4x1[4];

	return 0;
}


int RX(float _theta, float **R_3x3)
{
	R_3x3[1][1] = 1.f;
	R_3x3[1][2] = 0;
	R_3x3[1][3] = 0;

	R_3x3[2][1] = 0;
	R_3x3[2][2] = (float)cos(_theta);
	R_3x3[2][3] = -(float)sin(_theta);

	R_3x3[3][1] = 0;
	R_3x3[3][2] = -R_3x3[2][3];
	R_3x3[3][3] = R_3x3[2][2];

	return 0;
}


int RY(float _theta, float **R_3x3)
{
	R_3x3[1][1] = (float)cos(_theta);
	R_3x3[1][2] = 0;
	R_3x3[1][3] = (float)sin(_theta);

	R_3x3[2][1] = 0;
	R_3x3[2][2] = 1.f;
	R_3x3[2][3] = 0;

	R_3x3[3][1] = -R_3x3[1][3];
	R_3x3[3][2] = 0;
	R_3x3[3][3] = R_3x3[1][1];

	return 0;
}


int RZ(float _theta, float **R_3x3)
{
	R_3x3[1][1] = (float)cos(_theta);
	R_3x3[1][2] = -(float)sin(_theta);
	R_3x3[1][3] = 0;

	R_3x3[2][1] = -R_3x3[1][2];
	R_3x3[2][2] = R_3x3[1][1];
	R_3x3[2][3] = 0;

	R_3x3[3][1] = 0;
	R_3x3[3][2] = 0;
	R_3x3[3][3] = 1.f;

	return 0;
}



int RVALS(float x, float xp, float xpp_ref, float xpmax, float xppmax, float xmin, float xmax, float margin, float *xpp_result, float *bound_l, float *bound_u)	// Operational Range, Velocity and Acceleration Limit Strategy
{
	float Aupper_temp[2], Alower_temp[2];
	float f_temp;

	Aupper_temp[0] = (xpmax-xp)/DT;
	if(xmax < (x+xp*DT+0.5f*xppmax*DT*DT))
		Aupper_temp[1] = (float)FMAX((0.f-xp)/DT, -xppmax);
	else
		Aupper_temp[1] = (float)FMAX( ((float)sqrtf(2.f*xppmax*(xmax-margin-(x+xp*DT+0.5f*xppmax*DT*DT)))-xp)/DT ,-xppmax);

	Alower_temp[0] = (-xpmax-xp)/DT;
	if(xmin > (x+xp*DT-0.5f*xppmax*DT*DT))
		Alower_temp[1] = (float)FMIN((0.f-xp)/DT, xppmax);
	else
		Alower_temp[1] = (float)FMIN( (-(float)sqrtf(2.f*xppmax*((x+xp*DT+0.5f*xppmax*DT*DT)-xmin-margin))-xp)/DT , xppmax);

	//----------------------- position, velocity and acceleration ranges are limited
	f_temp = (float)FMIN(Aupper_temp[0],Aupper_temp[1]);
	*bound_u = (float)FMIN(f_temp, xppmax);
	f_temp = (float)FMAX(Alower_temp[0],Alower_temp[1]);
	*bound_l = (float)FMAX(f_temp, -xppmax);
	//----------------

	//----------------------- position ranges are limited
	//*bound_u = Aupper_temp[1];
	//*bound_l = Alower_temp[1];
	//----------------
  
	if(*bound_l > *bound_u)
	{
		if(*bound_u < 0.)
			*bound_u = *bound_l;
		else
			*bound_l = *bound_u;
	}
  
  if(xpp_ref < *bound_l)
    *xpp_result = *bound_l;
  else if(xpp_ref > *bound_u)
    *xpp_result = *bound_u;
  else
    *xpp_result = xpp_ref;
    
	return 0;
}


int RVALS3(float x, float xp, float xpmax, float xppmax, float xmin, float xmax, float margin, float *bound_l, float *bound_u)
{
	float Aupper_temp[2], Alower_temp[2];

	Aupper_temp[0] = (xpmax-xp)/DT;
	if(xmax < (x+xp*DT+0.5f*xppmax*DT*DT))
		Aupper_temp[1] = (float)FMAX((0.f-xp)/DT, -xppmax);
	else
		Aupper_temp[1] = (float)FMAX( ((float)sqrtf(2.f*xppmax*(xmax-margin-(x+xp*DT+0.5f*xppmax*DT*DT)))-xp)/DT ,-xppmax);

	Alower_temp[0] = (-xpmax-xp)/DT;
	if(xmin > (x+xp*DT-0.5f*xppmax*DT*DT))
		Alower_temp[1] = (float)FMIN((0.f-xp)/DT, xppmax);
	else
		Alower_temp[1] = (float)FMIN( (-(float)sqrtf(2.f*xppmax*((x+xp*DT+0.5f*xppmax*DT*DT)-xmin-margin))-xp)/DT , xppmax);

	//----------------------- position, velocity and acceleration ranges are limited
	// f_temp = (float)FMIN(Aupper_temp[0],Aupper_temp[1]);
	// *bound_u = (float)FMIN(f_temp, xppmax);
	// f_temp = (float)FMAX(Alower_temp[0],Alower_temp[1]);
	// *bound_l = (float)FMAX(f_temp, -xppmax);
	//----------------

	//----------------------- position ranges are limited
	*bound_u = Aupper_temp[1];
	*bound_l = Alower_temp[1];
	//----------------

	if(*bound_l > *bound_u)
	{
		if(*bound_u < 0.)
			*bound_u = *bound_l;
		else
			*bound_l = *bound_u;
	}

	return 0;
}


int UpdatePassiveCoord_SSP(int LorR)
{
	int i;

	_Q_34x1[1] = 0.f;
	_Q_34x1[2] = 0.f;
	_Q_34x1[3] = 0.f;
	_Q_34x1[4] = 1.f;
	_Q_34x1[5] = 0.f;
	_Q_34x1[6] = 0.f;
	_Q_34x1[7] = 0.f;
	
	_Q0_34x1[1] = _Q_34x1[1] = 0.f;
	_Q0_34x1[2] = _Q_34x1[2] = 0.f;

	FKine_Whole();

	if(LorR == 1)
	{
		_Q0_34x1[3] = _Q_34x1[3] = -_pLF_3x1[3];
		QT2DC(_qLF_4x1, _TEMP1_34x34);
		trans(1.f, (const float**)_TEMP1_34x34,3,3, _TEMP2_34x34);
	}
	else
	{
		_Q0_34x1[3] = _Q_34x1[3] = -_pRF_3x1[3];		
		QT2DC(_qRF_4x1, _TEMP1_34x34);
		trans(1.f, (const float**)_TEMP1_34x34,3,3, _TEMP2_34x34);
	}

	DC2QT((const float**)_TEMP2_34x34, &_Q_34x1[3]);
	_Q0_34x1[4] = _Q_34x1[4];
	_Q0_34x1[5] = _Q_34x1[5];
	_Q0_34x1[6] = _Q_34x1[6];
	_Q0_34x1[7] = _Q_34x1[7];

	FKine_Whole();

	for(i=1; i<=3; i++)
	{
		_pCOM0_3x1[i] = _pCOM_3x1[i];
		_pRF0_3x1[i] = _pRF_3x1[i];
		_pLF0_3x1[i] = _pLF_3x1[i];
		_pRH0_3x1[i] = _pRH_3x1[i];
		_pLH0_3x1[i] = _pLH_3x1[i];
		_qRF0_4x1[i] = _qRF_4x1[i];
		_qLF0_4x1[i] = _qLF_4x1[i];
		_qRH0_4x1[i] = _qRH_4x1[i];
		_qLH0_4x1[i] = _qLH_4x1[i];
	}
	_qRF0_4x1[4] = _qRF_4x1[4];
	_qLF0_4x1[4] = _qLF_4x1[4];
	_qRH0_4x1[4] = _qRH_4x1[4];
	_qLH0_4x1[4] = _qLH_4x1[4];

	_PassiveUpdatedFlag = 1;

	return 0;
}


void wberror(char error_text[])
/* whole-body motion error handler */
{
	printf("\nWhole-body motion run-time error...\n");
	printf("%s\n",error_text);
}



int derive3(float x0_in, float x1_in, float x2_in, float *xd1_out, float *xdd1_out, float dt_in)
{
	*xd1_out = (x2_in-x0_in)/(2.f*dt_in);
	*xdd1_out = (x2_in-2.f*x1_in+x0_in)/(dt_in*dt_in);
	
	return 0;
}

int derive3QT(float *q_past_4x1, float *q_4x1, float *q_next_4x1, float *w_result_3x1, float dt)
{
	QT2DC(q_past_4x1, _TEMP3_34x34);
	QT2DC(q_next_4x1, _TEMP4_34x34);
	diff_mm((const float**)_TEMP4_34x34,3,3, (const float**)_TEMP3_34x34, _TEMP3_34x34);
	QT2DC(q_4x1, _TEMP4_34x34);
	trans2(1.f, _TEMP4_34x34,3,3);
	mult_smm(0.5f/dt, (const float**)_TEMP3_34x34, 3,3, (const float**)_TEMP4_34x34,3, _TEMP2_34x34);

	w_result_3x1[1] = _TEMP2_34x34[3][2];
	w_result_3x1[2] = _TEMP2_34x34[1][3];
	w_result_3x1[3] = _TEMP2_34x34[2][1];

	return 0;
}

/*
int poly5(float ti, float tf, float yi, float yf, float ypi, float ypf, float yppi, float yppf, float *coeff_result_6x1)	
{
	// 5th order polynomial, y(t)=coeff(1) + coeff(2)*t + coeff(3)*t^2 + coeff(4)*t^3 + coeff(5)*t^4 + coeff(6)*t^5
	_TEMP3_34x34[1][1] = 1.f;
	_TEMP3_34x34[1][2] = ti;
	_TEMP3_34x34[1][3] = ti*ti;
	_TEMP3_34x34[1][4] = (float)pow(ti,3);
	_TEMP3_34x34[1][5] = (float)pow(ti,4);
	_TEMP3_34x34[1][6] = (float)pow(ti,5);

	_TEMP3_34x34[2][1] = 1.f;
	_TEMP3_34x34[2][2] = tf;
	_TEMP3_34x34[2][3] = tf*tf;
	_TEMP3_34x34[2][4] = (float)pow(tf,3);
	_TEMP3_34x34[2][5] = (float)pow(tf,4);
	_TEMP3_34x34[2][6] = (float)pow(tf,5);

	_TEMP3_34x34[3][1] = 0.;
	_TEMP3_34x34[3][2] = 1.f;
	_TEMP3_34x34[3][3] = 2.f*ti;
	_TEMP3_34x34[3][4] = 3.f*ti*ti;
	_TEMP3_34x34[3][5] = 4.f*(float)pow(ti,3);
	_TEMP3_34x34[3][6] = 5.f*(float)pow(ti,4);

	_TEMP3_34x34[4][1] = 0.;
	_TEMP3_34x34[4][2] = 1.f;
	_TEMP3_34x34[4][3] = 2.f*tf;
	_TEMP3_34x34[4][4] = 3.f*tf*tf;
	_TEMP3_34x34[4][5] = 4.f*(float)pow(tf,3);
	_TEMP3_34x34[4][6] = 5.f*(float)pow(tf,4);

	_TEMP3_34x34[5][1] = 0.;
	_TEMP3_34x34[5][2] = 0.;
	_TEMP3_34x34[5][3] = 2.f;
	_TEMP3_34x34[5][4] = 6.f*ti;
	_TEMP3_34x34[5][5] = 12.f*ti*ti;
	_TEMP3_34x34[5][6] = 20.f*(float)pow(ti,3);

	_TEMP3_34x34[6][1] = 0.;
	_TEMP3_34x34[6][2] = 0.;
	_TEMP3_34x34[6][3] = 2.f;
	_TEMP3_34x34[6][4] = 6.f*tf;
	_TEMP3_34x34[6][5] = 12.f*tf*tf;
	_TEMP3_34x34[6][6] = 20.f*(float)pow(tf,3);

	coeff_result_6x1[1] = yi;
	coeff_result_6x1[2] = yf;
	coeff_result_6x1[3] = ypi;
	coeff_result_6x1[4] = ypf;
	coeff_result_6x1[5] = yppi;
	coeff_result_6x1[6] = yppf;

	return (gaussj_mod(_TEMP3_34x34,6, coeff_result_6x1));
}
*/


int checkJointRange(float Q_34x1[])
{
	if(Q_34x1[WST_34] > WSTmax || Q_34x1[WST_34] < WSTmin)	{printf("\n WST limit error!!"); return -1;}
	if(Q_34x1[RHY_34] > RHYmax || Q_34x1[RHY_34] < RHYmin)	{printf("\n RHY limit error!!"); return -1;}
	if(Q_34x1[RHR_34] > RHRmax || Q_34x1[RHR_34] < RHRmin)	{printf("\n RHR limit error!!"); return -1;}
	if(Q_34x1[RHP_34] > RHPmax || Q_34x1[RHP_34] < RHPmin)	{printf("\n RHP limit error!!"); return -1;}
	if(Q_34x1[RKN_34] > RKNmax || Q_34x1[RKN_34] < RKNmin)	{printf("\n RKN limit error!!"); return -1;}
	if(Q_34x1[RAP_34] > RAPmax || Q_34x1[RAP_34] < RAPmin)	{printf("\n RAP limit error!!"); return -1;}
	if(Q_34x1[RAR_34] > RARmax || Q_34x1[RAR_34] < RARmin)	{printf("\n RAR limit error!!"); return -1;}
	if(Q_34x1[LHY_34] > LHYmax || Q_34x1[LHY_34] < LHYmin)	{printf("\n LHY limit error!!"); return -1;}
	if(Q_34x1[LHR_34] > LHRmax || Q_34x1[LHR_34] < LHRmin)	{printf("\n LHR limit error!!"); return -1;}
	if(Q_34x1[LHP_34] > LHPmax || Q_34x1[LHP_34] < LHPmin)	{printf("\n LHP limit error!!"); return -1;}
	if(Q_34x1[LKN_34] > LKNmax || Q_34x1[LKN_34] < LKNmin)	{printf("\n LKN limit error!!"); return -1;}
	if(Q_34x1[LAP_34] > LAPmax || Q_34x1[LAP_34] < LAPmin)	{printf("\n LAP limit error!!"); return -1;}
	if(Q_34x1[LAR_34] > LARmax || Q_34x1[LAR_34] < LARmin)	{printf("\n LAR limit error!!"); return -1;}
	if(Q_34x1[RSP_34] > RSPmax || Q_34x1[RSP_34] < RSPmin)	{printf("\n RSP limit error!!"); return -1;}
	if(Q_34x1[RSR_34] > RSRmax || Q_34x1[RSR_34] < RSRmin)	{printf("\n RSR limit error!!"); return -1;}
	if(Q_34x1[RSY_34] > RSYmax || Q_34x1[RSY_34] < RSYmin)	{printf("\n RSY limit error!!"); return -1;}
	if(Q_34x1[REB_34] > REBmax || Q_34x1[REB_34] < REBmin)	{printf("\n REB limit error!!"); return -1;}
	if(Q_34x1[RWY_34] > RWYmax || Q_34x1[RWY_34] < RWYmin)	{printf("\n RWY limit error!!"); return -1;}
	if(Q_34x1[RWP_34] > RWPmax || Q_34x1[RWP_34] < RWPmin)	{printf("\n RWP limit error!!"); return -1;}
	if(Q_34x1[LSP_34] > LSPmax || Q_34x1[LSP_34] < LSPmin)	{printf("\n LSP limit error!!"); return -1;}
	if(Q_34x1[LSR_34] > LSRmax || Q_34x1[LSR_34] < LSRmin)	{printf("\n LSR limit error!!"); return -1;}
	if(Q_34x1[LSY_34] > LSYmax || Q_34x1[LSY_34] < LSYmin)	{printf("\n LSY limit error!!"); return -1;}
	if(Q_34x1[LEB_34] > LEBmax || Q_34x1[LEB_34] < LEBmin)	{printf("\n LEB limit error!!"); return -1;}
	if(Q_34x1[LWY_34] > LWYmax || Q_34x1[LWY_34] < LWYmin)	{printf("\n LWY limit error!!"); return -1;}
	if(Q_34x1[LWP_34] > LWPmax || Q_34x1[LWP_34] < LWPmin)	{printf("\n LWP limit error!!"); return -1;}
	if(Q_34x1[RWY2_34] > RWY2max || Q_34x1[RWY2_34] < RWY2min)	{printf("\n RWY2 limit error!!"); return -1;}
	if(Q_34x1[LWY2_34] > LWY2max || Q_34x1[LWY2_34] < LWY2min)	{printf("\n LWY2 limit error!!"); return -1;}

	return 0;
}


float one_cos(float t_sec, float mag, float T_sec)
{
	float rtn;

	if(t_sec < 0.f)
		return 0.f;
	else if(t_sec < T_sec)
		rtn = mag*0.5f*(1.f - (float)cosf(PI/T_sec*t_sec));
	else
		rtn = mag;

	return rtn;
}


int one_cos_orientation(float t_sec, const float *qt0_4x1, const float *qt1_4x1, float T_sec, float *result_4x1)
{
	float qt0_bar_4x1[5], qt_del_4x1[5], rv[4];

	QTbar(qt0_4x1, qt0_bar_4x1);
	QTcross(qt0_bar_4x1, qt1_4x1, qt_del_4x1);
	QT2RV(qt_del_4x1, rv);
	if(t_sec < 0.f)
	{
		result_4x1[1] = qt0_4x1[1];
		result_4x1[2] = qt0_4x1[2];
		result_4x1[3] = qt0_4x1[3];
		result_4x1[4] = qt0_4x1[4];

	}
	else if(t_sec < T_sec)
	{
		rv[0] = one_cos(t_sec, rv[0], T_sec);
		RV2QT(rv, qt_del_4x1);
		QTcross(qt0_4x1, qt_del_4x1, result_4x1);
	}
	else
	{
		result_4x1[1] = qt1_4x1[1];
		result_4x1[2] = qt1_4x1[2];
		result_4x1[3] = qt1_4x1[3];
		result_4x1[4] = qt1_4x1[4];
	}

	return 0;
}





int QT2RV(const float *qt_4x1, float *rv)
{
	float temp;
	rv[0] = acosf(qt_4x1[1])*2.f;

	if(fabs(sinf(rv[0]/2.f)) < EPS)
	{
		rv[1] = qt_4x1[2];
		rv[2] = qt_4x1[3];
		rv[3] = qt_4x1[4];
	}
	else
	{
		rv[1] = qt_4x1[2]/sinf(rv[0]/2.f);
		rv[2] = qt_4x1[3]/sinf(rv[0]/2.f);
		rv[3] = qt_4x1[4]/sinf(rv[0]/2.f);

		temp = norm_v(rv,3);
		rv[1] /= temp;
		rv[2] /= temp;
		rv[3] /= temp;
	}

	return 0;
}


int RV2QT(const float *rv, float *qt_4x1)
{
	float temp = sqrtf(rv[1]*rv[1]+rv[2]*rv[2]+rv[3]*rv[3]);

	if(temp > 0.5f)
	{
		qt_4x1[1] = cosf(rv[0]/2.f);
		qt_4x1[2] = rv[1]/temp*sinf(rv[0]/2.f);
		qt_4x1[3] = rv[2]/temp*sinf(rv[0]/2.f);
		qt_4x1[4] = rv[3]/temp*sinf(rv[0]/2.f);		
	}
	else
	{
		qt_4x1[1] = 1.f;
		qt_4x1[2] = 0.f;
		qt_4x1[3] = 0.f;
		qt_4x1[4] = 0.f;
	}
	
	return 0;
}





int _OfflineCount = 0;
float	ps_move_sec;
float	ps_kp[25],ps_kd[25];
int ps_position_mode_flag = 0;
int ps_torque_mode_flag = 0;
int ps_ladder_demo = 0;

float			ps_drvRF[4], ps_drvLF[4], ps_dpRF[3], ps_dpLF[3];
float			ps_drvRH[4], ps_drvLH[4], ps_dpRH[3], ps_dpLH[3], ps_dpCOM[3], ps_dpPELz, ps_drvPEL[4];
float			ps_off_traj_dpPELz[MAX_OFF_TRAJ], ps_off_traj_dpCOM[MAX_OFF_TRAJ][2], ps_off_traj_dpRH[MAX_OFF_TRAJ][3], ps_off_traj_dpLH[MAX_OFF_TRAJ][3], ps_off_traj_dpRF[MAX_OFF_TRAJ][3], ps_off_traj_dpLF[MAX_OFF_TRAJ][3], ps_off_traj_drvRH[MAX_OFF_TRAJ][4], ps_off_traj_drvLH[MAX_OFF_TRAJ][4], ps_off_traj_drvRF[MAX_OFF_TRAJ][4], ps_off_traj_drvLF[MAX_OFF_TRAJ][4];
unsigned int	ps_off_traj_length, ps_off_traj_count;

	float			ps_ref_fLH_3x1[4], ps_ref_fRH_3x1[4];


int WBIK_DRC_ladder_climbing(unsigned int n)  
{
	int i,j;
	
	float Qpp_33x1[34], meas_Q_34x1[35], meas_Qp_33x1[34];
	float X1_33x1[34], Xrf_6x1[7], Xlf_6x1[7], Xrh_6x1[7], Xlh_6x1[7], Xcom_3x1[4], Xpel_3x1[4], Xwst, Xpelz, X2_33x1[34];  // task variables
	float ub_27x1[28], lb_27x1[28], qpp_limited[28];  // joint upper-bound, lower-bound
	int index_limited[28];
	
	float des_pCOM_3x1[4], des_vCOM_3x1[4];
	float des_pRF_3x1[4], des_pLF_3x1[4], des_vRF_3x1[4], des_vLF_3x1[4];
	float des_qRF_4x1[5], des_qLF_4x1[5], des_wRF_3x1[4], des_wLF_3x1[4];
	float des_pRH_3x1[4], des_pLH_3x1[4], des_vRH_3x1[4], des_vLH_3x1[4];
	float des_qRH_4x1[5], des_qLH_4x1[5], des_wRH_3x1[4], des_wLH_3x1[4];
	float des_WST, des_WSTp, des_qPEL_4x1[5];
	float des_pPELz, des_vPELz;
	float vCOM_3x1[4];

	int isLimited = 0;
	int dim_primary_task, dim_redundant_task;

	float lL, lR, fRFz, fLFz;
	float rhr_compen = 0.f;
	float lhr_compen = 0.f;
	float rhy_compen = 0.f;
	float lhy_compen = 0.f;
	
	float viscous_33x1[34], ct_33x1[34], gravity_33x1[34], duty_33x1[34], duty_joint_limit_33x1[34];

	static char half_flag, reset_neutral_flag;
	static int count;
	static float neutral_Q_34x1[35], Q_old_34x1[35], Q_old2_34x1[35];
	static float pCOM1_3x1[4], pPELz1, pRF1_3x1[4], pLF1_3x1[4], qRF1_4x1[5], qLF1_4x1[5];
	static float pRH1_3x1[4], pLH1_3x1[4], qRH1_4x1[5], qLH1_4x1[5];
	static float pRH2_3x1[4], pLH2_3x1[4], qRH2_4x1[5], qLH2_4x1[5];
	static float t;

	float dpRF_3x1[4], dpLF_3x1[4], dqRF_4x1[5], dqLF_4x1[5];
	float dpRH_3x1[4], dpLH_3x1[4], dqRH_4x1[5], dqLH_4x1[5];
	float dpPELz, dpCOM_3x1[4];

	float ftemp1_7x1[8], ftemp2_7x1[8], ftemp3_7x1[8], ftemp4_7x1[8];
	float pre_gain = 0.f;

	if(_FKineUpdatedFlag != 1)
		return -4;
	
	if(n<=2)
	{
		neutral_Q_34x1[WST_34] = WSTRefAngleCurrent*D2R;

		neutral_Q_34x1[RSP_34] = RSPRefAngleCurrent*D2R;
		neutral_Q_34x1[RSR_34] = (RSRRefAngleCurrent + OFFSET_RSR)*D2R;
		neutral_Q_34x1[RSY_34] = RSYRefAngleCurrent*D2R;
		neutral_Q_34x1[REB_34] = (REBRefAngleCurrent + OFFSET_REB)*D2R;
		neutral_Q_34x1[RWY_34] = RWYRefAngleCurrent*D2R;
		neutral_Q_34x1[RWP_34] = RWPRefAngleCurrent*D2R;
		neutral_Q_34x1[RWY2_34] = RWY2RefAngleCurrent*D2R;
		
		neutral_Q_34x1[LSP_34] = LSPRefAngleCurrent*D2R;
		neutral_Q_34x1[LSR_34] = (LSRRefAngleCurrent + OFFSET_LSR)*D2R;
		neutral_Q_34x1[LSY_34] = LSYRefAngleCurrent*D2R;
		neutral_Q_34x1[LEB_34] = (LEBRefAngleCurrent + OFFSET_LEB)*D2R;
		neutral_Q_34x1[LWY_34] = LWYRefAngleCurrent*D2R;
		neutral_Q_34x1[LWP_34] = LWPRefAngleCurrent*D2R;
		neutral_Q_34x1[LWY2_34] = LWY2RefAngleCurrent*D2R;

		_pPEL0_3x1[1] = _Q_34x1[1];
		_pPEL0_3x1[2] = _Q_34x1[2];
		pPELz1 = _pPEL0_3x1[3] = _Q_34x1[3];

		pCOM1_3x1[1] = _pCOM0_3x1[1] = _pCOM_3x1[1];
		pCOM1_3x1[2] = _pCOM0_3x1[2] = _pCOM_3x1[2];
//pCOM1_3x1[3] = _pCOM0_3x1[3] = _pCOM_3x1[3];

		pRF1_3x1[1] = _pRF0_3x1[1] = _pRF_3x1[1];
		pRF1_3x1[2] = _pRF0_3x1[2] = _pRF_3x1[2];
		pRF1_3x1[3] = _pRF0_3x1[3] = _pRF_3x1[3];

		qRF1_4x1[1] = _qRF0_4x1[1] = _qRF_4x1[1];
		qRF1_4x1[2] = _qRF0_4x1[2] = _qRF_4x1[2];
		qRF1_4x1[3] = _qRF0_4x1[3] = _qRF_4x1[3];
		qRF1_4x1[4] = _qRF0_4x1[4] = _qRF_4x1[4];

		pLF1_3x1[1] = _pLF0_3x1[1] = _pLF_3x1[1];
		pLF1_3x1[2] = _pLF0_3x1[2] = _pLF_3x1[2];
		pLF1_3x1[3] = _pLF0_3x1[3] = _pLF_3x1[3];

		qLF1_4x1[1] = _qLF0_4x1[1] = _qLF_4x1[1];
		qLF1_4x1[2] = _qLF0_4x1[2] = _qLF_4x1[2];
		qLF1_4x1[3] = _qLF0_4x1[3] = _qLF_4x1[3];
		qLF1_4x1[4] = _qLF0_4x1[4] = _qLF_4x1[4];

		pRH1_3x1[1] = _pRH0_3x1[1] = _pRH_3x1[1];
		pRH1_3x1[2] = _pRH0_3x1[2] = _pRH_3x1[2];
		pRH1_3x1[3] = _pRH0_3x1[3] = _pRH_3x1[3];

		qRH1_4x1[1] = _qRH0_4x1[1] = _qRH_4x1[1];
		qRH1_4x1[2] = _qRH0_4x1[2] = _qRH_4x1[2];
		qRH1_4x1[3] = _qRH0_4x1[3] = _qRH_4x1[3];
		qRH1_4x1[4] = _qRH0_4x1[4] = _qRH_4x1[4];

		pLH1_3x1[1] = _pLH0_3x1[1] = _pLH_3x1[1];
		pLH1_3x1[2] = _pLH0_3x1[2] = _pLH_3x1[2];
		pLH1_3x1[3] = _pLH0_3x1[3] = _pLH_3x1[3];

		qLH1_4x1[1] = _qLH0_4x1[1] = _qLH_4x1[1];
		qLH1_4x1[2] = _qLH0_4x1[2] = _qLH_4x1[2];
		qLH1_4x1[3] = _qLH0_4x1[3] = _qLH_4x1[3];
		qLH1_4x1[4] = _qLH0_4x1[4] = _qLH_4x1[4];

		meas_Q_34x1[RSP_34] = ((float)RSPEncoderValue)*D2R;
		meas_Q_34x1[RSR_34] = ((float)RSREncoderValue)*D2R + OFFSET_RSR*D2R;
		meas_Q_34x1[RSY_34] = ((float)RSYEncoderValue)*D2R;
		meas_Q_34x1[REB_34] = ((float)REBEncoderValue)*D2R + OFFSET_REB*D2R;
		meas_Q_34x1[RWY_34] = ((float)RWYEncoderValue)*D2R;
		meas_Q_34x1[RWP_34] = ((float)RWPEncoderValue)*D2R;
		meas_Q_34x1[RWY2_34] = ((float)RWY2EncoderValue)*D2R;

		meas_Q_34x1[LSP_34] = ((float)LSPEncoderValue)*D2R;
		meas_Q_34x1[LSR_34] = ((float)LSREncoderValue)*D2R + OFFSET_LSR*D2R;
		meas_Q_34x1[LSY_34] = ((float)LSYEncoderValue)*D2R;
		meas_Q_34x1[LEB_34] = ((float)LEBEncoderValue)*D2R + OFFSET_LEB*D2R;
		meas_Q_34x1[LWY_34] = ((float)LWYEncoderValue)*D2R;
		meas_Q_34x1[LWP_34] = ((float)LWPEncoderValue)*D2R;
		meas_Q_34x1[LWY2_34] = ((float)LWY2EncoderValue)*D2R;

		Q_old2_34x1[RSP_34] = Q_old_34x1[RSP_34] = meas_Q_34x1[RSP_34];
		Q_old2_34x1[RSR_34] = Q_old_34x1[RSR_34] = meas_Q_34x1[RSR_34];
		Q_old2_34x1[RSY_34] = Q_old_34x1[RSY_34] = meas_Q_34x1[RSY_34];
		Q_old2_34x1[REB_34] = Q_old_34x1[REB_34] = meas_Q_34x1[REB_34];
		Q_old2_34x1[RWY_34] = Q_old_34x1[RWY_34] = meas_Q_34x1[RWY_34];
		Q_old2_34x1[RWP_34] = Q_old_34x1[RWP_34] = meas_Q_34x1[RWP_34];
		Q_old2_34x1[RWY2_34] = Q_old_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

		Q_old2_34x1[LSP_34] = Q_old_34x1[LSP_34] = meas_Q_34x1[LSP_34];
		Q_old2_34x1[LSR_34] = Q_old_34x1[LSR_34] = meas_Q_34x1[LSR_34];
		Q_old2_34x1[LSY_34] = Q_old_34x1[LSY_34] = meas_Q_34x1[LSY_34];
		Q_old2_34x1[LEB_34] = Q_old_34x1[LEB_34] = meas_Q_34x1[LEB_34];
		Q_old2_34x1[LWY_34] = Q_old_34x1[LWY_34] = meas_Q_34x1[LWY_34];
		Q_old2_34x1[LWP_34] = Q_old_34x1[LWP_34] = meas_Q_34x1[LWP_34];
		Q_old2_34x1[LWY2_34] = Q_old_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

		diff_mm((const float**)_jRH_6x33,6,33, (const float**)_jRH_6x33, _jRHp_6x33);
		subs_m((const float**)_jRH_6x33,6,33, _jRH_old_6x33);
		subs_m((const float**)_jRH_6x33,6,33, _jRH_old2_6x33);
		
		diff_mm((const float**)_jLH_6x33,6,33, (const float**)_jLH_6x33, _jLHp_6x33);
		subs_m((const float**)_jLH_6x33,6,33, _jLH_old_6x33);
		subs_m((const float**)_jLH_6x33,6,33, _jLH_old2_6x33);

		diff_mm((const float**)_jRF_6x33,6,33, (const float**)_jRF_6x33, _jRFp_6x33);
		subs_m((const float**)_jRF_6x33,6,33, _jRF_old_6x33);
		subs_m((const float**)_jRF_6x33,6,33, _jRF_old2_6x33);

		diff_mm((const float**)_jLF_6x33,6,33, (const float**)_jLF_6x33, _jLFp_6x33);
		subs_m((const float**)_jLF_6x33,6,33, _jLF_old_6x33);
		subs_m((const float**)_jLF_6x33,6,33, _jLF_old2_6x33);
		
		diff_mm((const float**)_jCOM_3x33,3,33, (const float**)_jCOM_3x33, _jCOMp_3x33);
		subs_m((const float**)_jCOM_3x33,3,33, _jCOM_old_3x33);
		subs_m((const float**)_jCOM_3x33,3,33, _jCOM_old2_3x33);
		
                //for(i=1;i<=34;i++)
		//	neutral_Q_34x1[i] = _Q_34x1[i];


		half_flag = 0;
		t = 0.f;
		count = 0;
		return 0;
	}
	diff_mm((const float**)_jRF_6x33,6,33, (const float**)_jRF_old2_6x33, _jRFp_6x33);
	subs_m((const float**)_jRF_old_6x33,6,33, _jRF_old2_6x33);
	subs_m((const float**)_jRF_6x33,6,33, _jRF_old_6x33);
	mult_sm((const float**)_jRFp_6x33,6,33, 1.f/(2.f*DT), _jRFp_6x33);
	
	diff_mm((const float**)_jLF_6x33,6,33, (const float**)_jLF_old2_6x33, _jLFp_6x33);
	subs_m((const float**)_jLF_old_6x33,6,33, _jLF_old2_6x33);
	subs_m((const float**)_jLF_6x33,6,33, _jLF_old_6x33);
	mult_sm((const float**)_jLFp_6x33,6,33, 1.f/(2.f*DT), _jLFp_6x33);

	diff_mm((const float**)_jCOM_3x33,3,33, (const float**)_jCOM_old2_3x33, _jCOMp_3x33);
	subs_m((const float**)_jCOM_old_3x33,3,33, _jCOM_old2_3x33);
	subs_m((const float**)_jCOM_3x33,3,33, _jCOM_old_3x33);
	mult_sm((const float**)_jCOMp_3x33,3,33, 1.f/(2.f*DT), _jCOMp_3x33);
	
	dpPELz = 0.f;

	dpCOM_3x1[1] = 0.f;
	dpCOM_3x1[2] = 0.f;
	dpCOM_3x1[3] = 0.f;
	
	dpRF_3x1[1] = 0.f;
	dpRF_3x1[2] = 0.f;
	dpRF_3x1[3] = 0.f;
	
	dpLF_3x1[1] = 0.f;
	dpLF_3x1[2] = 0.f;
	dpLF_3x1[3] = 0.f;
	
	dpRH_3x1[1] = 0.f;
	dpRH_3x1[2] = 0.f;
	dpRH_3x1[3] = 0.f;
	
	dpLH_3x1[1] = 0.f;
	dpLH_3x1[2] = 0.f;
	dpLH_3x1[3] = 0.f;
	
	dqRF_4x1[1] = 1.f;
	dqRF_4x1[2] = 0.f;
	dqRF_4x1[3] = 0.f;
	dqRF_4x1[4] = 0.f;
	
	dqLF_4x1[1] = 1.f;
	dqLF_4x1[2] = 0.f;
	dqLF_4x1[3] = 0.f;
	dqLF_4x1[4] = 0.f;
	
	dqRH_4x1[1] = 1.f;
	dqRH_4x1[2] = 0.f;
	dqRH_4x1[3] = 0.f;
	dqRH_4x1[4] = 0.f;
	
	dqLH_4x1[1] = 1.f;
	dqLH_4x1[2] = 0.f;
	dqLH_4x1[3] = 0.f;
	dqLH_4x1[4] = 0.f;

	reset_neutral_flag = 0;
	if(ps_position_mode_flag == 0) // staying
	{	
		t = 0.f;
	}
	else if(ps_position_mode_flag == 1)  // move hands
	{	
		
		dpRH_3x1[1] = one_cos(t, ps_dpRH[0], ps_move_sec);
		dpRH_3x1[2] = one_cos(t, ps_dpRH[1], ps_move_sec);
		dpRH_3x1[3] = one_cos(t, ps_dpRH[2], ps_move_sec);

		dpLH_3x1[1] = one_cos(t, ps_dpLH[0], ps_move_sec);
		dpLH_3x1[2] = one_cos(t, ps_dpLH[1], ps_move_sec);
		dpLH_3x1[3] = one_cos(t, ps_dpLH[2], ps_move_sec);

		ftemp1_7x1[0] = one_cos(t, ps_drvRH[0], ps_move_sec);
		ftemp1_7x1[1] = ps_drvRH[1];
		ftemp1_7x1[2] = ps_drvRH[2];
		ftemp1_7x1[3] = ps_drvRH[3];
		RV2QT(ftemp1_7x1, dqRH_4x1);

		ftemp1_7x1[0] = one_cos(t, ps_drvLH[0], ps_move_sec);
		ftemp1_7x1[1] = ps_drvLH[1];
		ftemp1_7x1[2] = ps_drvLH[2];
		ftemp1_7x1[3] = ps_drvLH[3];
		RV2QT(ftemp1_7x1, dqLH_4x1);
		
		t += DT;

		if(t > ps_move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			ps_position_mode_flag = 0;
		}
	}
	else if(ps_position_mode_flag == 2)  //move RF and LF
	{
		dpRF_3x1[1] = one_cos(t, ps_dpRF[0], ps_move_sec);
		dpRF_3x1[2] = one_cos(t, ps_dpRF[1], ps_move_sec);
		dpRF_3x1[3] = one_cos(t, ps_dpRF[2], ps_move_sec);

		dpLF_3x1[1] = one_cos(t, ps_dpLF[0], ps_move_sec);
		dpLF_3x1[2] = one_cos(t, ps_dpLF[1], ps_move_sec);
		dpLF_3x1[3] = one_cos(t, ps_dpLF[2], ps_move_sec);

		ftemp1_7x1[0] = one_cos(t, ps_drvRF[0], ps_move_sec);
		ftemp1_7x1[1] = ps_drvRF[1];
		ftemp1_7x1[2] = ps_drvRF[2];
		ftemp1_7x1[3] = ps_drvRF[3];
		RV2QT(ftemp1_7x1, dqRF_4x1);

		ftemp1_7x1[0] = one_cos(t, ps_drvLF[0], ps_move_sec);
		ftemp1_7x1[1] = ps_drvLF[1];
		ftemp1_7x1[2] = ps_drvLF[2];
		ftemp1_7x1[3] = ps_drvLF[3];
		RV2QT(ftemp1_7x1, dqLF_4x1);

		t += DT;
		if(t > ps_move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			ps_position_mode_flag = 0;
		}
	}
	else if(ps_position_mode_flag == 3)  //move COM
	{	
		dpCOM_3x1[1] = one_cos(t, ps_dpCOM[0], ps_move_sec);
		dpCOM_3x1[2] = one_cos(t, ps_dpCOM[1], ps_move_sec);
	
		t += DT;
		if(t > ps_move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			ps_position_mode_flag = 0;
		}
	}
	else if(ps_position_mode_flag == 4)  //move pPELz
	{
		dpPELz = one_cos(t, ps_dpPELz, ps_move_sec);
	
		t += DT;
		if(t > ps_move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			ps_position_mode_flag = 0;
		}
	}

	else if(ps_position_mode_flag == 6)  //move pPELz
	{
                dpCOM_3x1[1] = one_cos(t, ps_dpCOM[0], ps_move_sec);
		dpCOM_3x1[2] = one_cos(t, ps_dpCOM[1], ps_move_sec);
		dpPELz = one_cos(t, ps_dpPELz, ps_move_sec);
	
		t += DT;
		if(t > ps_move_sec)
		{
			t = 0.f;
			reset_neutral_flag = 1;
			ps_position_mode_flag = 0;
		}
	}

	else if(ps_position_mode_flag == 5)  //offline traj
	{
		dpPELz = ps_off_traj_dpPELz[ps_off_traj_count];
		dpCOM_3x1[1] = ps_off_traj_dpCOM[ps_off_traj_count][0];
		dpCOM_3x1[2] = ps_off_traj_dpCOM[ps_off_traj_count][1];
		dpRH_3x1[1] = ps_off_traj_dpRH[ps_off_traj_count][0];
		dpRH_3x1[2] = ps_off_traj_dpRH[ps_off_traj_count][1];
		dpRH_3x1[3] = ps_off_traj_dpRH[ps_off_traj_count][2];
		dpLH_3x1[1] = ps_off_traj_dpLH[ps_off_traj_count][0];
		dpLH_3x1[2] = ps_off_traj_dpLH[ps_off_traj_count][1];
		dpLH_3x1[3] = ps_off_traj_dpLH[ps_off_traj_count][2];
		dpRF_3x1[1] = ps_off_traj_dpRF[ps_off_traj_count][0];
		dpRF_3x1[2] = ps_off_traj_dpRF[ps_off_traj_count][1];
		dpRF_3x1[3] = ps_off_traj_dpRF[ps_off_traj_count][2];
		dpLF_3x1[1] = ps_off_traj_dpLF[ps_off_traj_count][0];
		dpLF_3x1[2] = ps_off_traj_dpLF[ps_off_traj_count][1];
		dpLF_3x1[3] = ps_off_traj_dpLF[ps_off_traj_count][2];
		RV2QT(ps_off_traj_drvRH[ps_off_traj_count], dqRH_4x1);
		RV2QT(ps_off_traj_drvLH[ps_off_traj_count], dqLH_4x1);
		RV2QT(ps_off_traj_drvRF[ps_off_traj_count], dqRF_4x1);
		RV2QT(ps_off_traj_drvLF[ps_off_traj_count], dqLF_4x1);
		ps_off_traj_count++;
		if(ps_off_traj_count >= ps_off_traj_length)
		{
			reset_neutral_flag = 1;
			ps_off_traj_count = 0;
			ps_position_mode_flag = 0;
		}
	}
	
	des_WST = 0.f;
	des_WSTp = 0.f;
	des_qPEL_4x1[1] = 1.f;
	des_qPEL_4x1[2] = 0.f;
	des_qPEL_4x1[3] = 0.f;
	des_qPEL_4x1[4] = 0.f;

	des_pPELz = pPELz1 + dpPELz;

        //printf("co %f %f\n",des_pCOM_3x1[1],pCOM1_3x1[1]);
        //printf("co %f %f\n",des_pCOM_3x1[2],pCOM1_3x1[2]);
        //printf("co %f %f\n",des_pCOM_3x1[3],pCOM1_3x1[3]);

	des_pCOM_3x1[1] = pCOM1_3x1[1] + dpCOM_3x1[1];
	des_pCOM_3x1[2] = pCOM1_3x1[2] + dpCOM_3x1[2];
	des_pCOM_3x1[3] = pCOM1_3x1[2] + dpCOM_3x1[3];
/*
        printf("CoM %f %f\n",des_pCOM_3x1[1],pCOM1_3x1[1]);
        printf("CoM %f %f\n",des_pCOM_3x1[2],pCOM1_3x1[2]);
        printf("CoM %f %f\n",des_pCOM_3x1[3],pCOM1_3x1[3]);
*/
	des_pRF_3x1[1] = pRF1_3x1[1] + dpRF_3x1[1];
	des_pRF_3x1[2] = pRF1_3x1[2] + dpRF_3x1[2];
	des_pRF_3x1[3] = pRF1_3x1[3] + dpRF_3x1[3];
/*	
        printf("Rf %f %f\n",des_pRF_3x1[1],pRF1_3x1[1]);
        printf("Rf %f %f\n",des_pRF_3x1[2],pRF1_3x1[2]);
        printf("Rf %f %f\n",des_pRF_3x1[3],pRF1_3x1[3]);
*/
	des_pLF_3x1[1] = pLF1_3x1[1] + dpLF_3x1[1];
	des_pLF_3x1[2] = pLF1_3x1[2] + dpLF_3x1[2];
	des_pLF_3x1[3] = pLF1_3x1[3] + dpLF_3x1[3];
/*	
        printf("Lf %f %f\n",des_pLF_3x1[1],pLF1_3x1[1]);
        printf("Lf %f %f\n",des_pLF_3x1[2],pLF1_3x1[2]);
        printf("Lf %f %f\n",des_pLF_3x1[3],pLF1_3x1[3]);
*/
	des_pRH_3x1[1] = pRH1_3x1[1] + dpRH_3x1[1];
	des_pRH_3x1[2] = pRH1_3x1[2] + dpRH_3x1[2];
	des_pRH_3x1[3] = pRH1_3x1[3] + dpRH_3x1[3];
/*
        printf("rH %f %f\n",des_pRH_3x1[1],pRH1_3x1[1]);
        printf("rH %f %f\n",des_pRH_3x1[2],pRH1_3x1[2]);
        printf("rH %f %f\n",des_pRH_3x1[3],pRH1_3x1[3]);
*/
	des_pLH_3x1[1] = pLH1_3x1[1] + dpLH_3x1[1];
	des_pLH_3x1[2] = pLH1_3x1[2] + dpLH_3x1[2];
	des_pLH_3x1[3] = pLH1_3x1[3] + dpLH_3x1[3];
/*
        printf("LH %f %f\n",des_pLH_3x1[1],pLH1_3x1[1]);
        printf("LH %f %f\n",des_pLH_3x1[2],pLH1_3x1[2]);
        printf("LH %f %f\n",des_pLH_3x1[3],pLH1_3x1[3]);
*/
	QTcross(dqRH_4x1, qRH1_4x1, des_qRH_4x1);
	QTcross(dqLH_4x1, qLH1_4x1, des_qLH_4x1);
	QTcross(dqRF_4x1, qRF1_4x1, des_qRF_4x1);
	QTcross(dqLF_4x1, qLF1_4x1, des_qLF_4x1);
		

	if(reset_neutral_flag==1)
	{
		for(i=1;i<=34;i++)
			neutral_Q_34x1[i] = _Q_34x1[i];

		pPELz1 = des_pPELz;

		pCOM1_3x1[1] = des_pCOM_3x1[1];
		pCOM1_3x1[2] = des_pCOM_3x1[2];

		pRF1_3x1[1] = des_pRF_3x1[1];
		pRF1_3x1[2] = des_pRF_3x1[2];
		pRF1_3x1[3] = des_pRF_3x1[3];

		pLF1_3x1[1] = des_pLF_3x1[1];
		pLF1_3x1[2] = des_pLF_3x1[2];
		pLF1_3x1[3] = des_pLF_3x1[3];

		pRH1_3x1[1] = des_pRH_3x1[1];
		pRH1_3x1[2] = des_pRH_3x1[2];
		pRH1_3x1[3] = des_pRH_3x1[3];

		pLH1_3x1[1] = des_pLH_3x1[1];
		pLH1_3x1[2] = des_pLH_3x1[2];
		pLH1_3x1[3] = des_pLH_3x1[3];

		qRF1_4x1[1] = des_qRF_4x1[1];
		qRF1_4x1[2] = des_qRF_4x1[2];
		qRF1_4x1[3] = des_qRF_4x1[3];
		qRF1_4x1[4] = des_qRF_4x1[4];

		qLF1_4x1[1] = des_qLF_4x1[1];
		qLF1_4x1[2] = des_qLF_4x1[2];
		qLF1_4x1[3] = des_qLF_4x1[3];
		qLF1_4x1[4] = des_qLF_4x1[4];

		qRH1_4x1[1] = des_qRH_4x1[1];
		qRH1_4x1[2] = des_qRH_4x1[2];
		qRH1_4x1[3] = des_qRH_4x1[3];
		qRH1_4x1[4] = des_qRH_4x1[4];

		qLH1_4x1[1] = des_qLH_4x1[1];
		qLH1_4x1[2] = des_qLH_4x1[2];
		qLH1_4x1[3] = des_qLH_4x1[3];
		qLH1_4x1[4] = des_qLH_4x1[4];

		printf("\n move done");
		reset_neutral_flag = 0;
	}

	des_vPELz = 0.f;

	des_vCOM_3x1[1] = 0.;
	des_vCOM_3x1[2] = 0.; 
	des_vCOM_3x1[3] = 0.;

	des_vRF_3x1[1] = 0.;
	des_vRF_3x1[2] = 0.;
	des_vRF_3x1[3] = 0.; 	
	des_wRF_3x1[1] = 0.;
	des_wRF_3x1[2] = 0.;
	des_wRF_3x1[3] = 0.;

	des_vLF_3x1[1] = 0.;
	des_vLF_3x1[2] = 0.;
	des_vLF_3x1[3] = 0.;
	des_wLF_3x1[1] = 0.;
	des_wLF_3x1[2] = 0.;
	des_wLF_3x1[3] = 0.;
	
	des_vRH_3x1[1] = 0.;
	des_vRH_3x1[2] = 0.;
	des_vRH_3x1[3] = 0.; 	
	des_wRH_3x1[1] = 0.;
	des_wRH_3x1[2] = 0.;
	des_wRH_3x1[3] = 0.;

	des_vLH_3x1[1] = 0.;
	des_vLH_3x1[2] = 0.;
	des_vLH_3x1[3] = 0.;
	des_wLH_3x1[1] = 0.;
	des_wLH_3x1[2] = 0.;
	des_wLH_3x1[3] = 0.;

	//------------------------------ compensation for hip roll joints
	lL = des_pLF_3x1[2] - des_pCOM_3x1[2];
	lR = des_pCOM_3x1[2] - des_pRF_3x1[2];
	fRFz = lL*m_TOTAL*9.81f/(lR+lL);
	fLFz = lR*m_TOTAL*9.81f/(lR+lL);

	if(lL < lR)
	{
		lhr_compen = (0.5f*m_TOTAL*9.81f-fRFz)/K_PELVIS_ROLL;
		rhr_compen = 0.;
	}
	else
	{
		lhr_compen = 0.;
		rhr_compen = -(0.5f*m_TOTAL*9.81f-fLFz)/K_PELVIS_ROLL;
	}
	//-------------------------------------

	//--------- Pelvis attitude
	QTdel(des_qPEL_4x1, &_Q_34x1[3], ftemp3_7x1);
	Xpel_3x1[1] = ps_kd[9]*(-_Qp_33x1[4]) + ps_kp[9]*ftemp3_7x1[1];
	Xpel_3x1[2] = ps_kd[9]*(-_Qp_33x1[5]) + ps_kp[9]*ftemp3_7x1[2];
	Xpel_3x1[3] = ps_kd[9]*(-_Qp_33x1[6]) + ps_kp[9]*ftemp3_7x1[3];
	//-----------------	

	//------------------ Waist
	RVALS(_Q_34x1[WST_34],  _Qp_33x1[WST_33],  ps_kd[10]*(des_WSTp-_Qp_33x1[WST_33])+ps_kp[10]*(des_WST-_Q_34x1[WST_34]),   WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &Xwst, &ftemp1_7x1[0], &ftemp1_7x1[1]);
	//-----------------

	//--------- Pelvis height
	Xpelz = ps_kd[11]*(des_vPELz-_Qp_33x1[3]) + ps_kp[11]*(des_pPELz-_Q_34x1[3]);
	//-----------------
	
	//------------ Xcom				
	mult_mv((const float**)_jCOM_3x33,3,33, _Qp_33x1, vCOM_3x1);	
	ftemp2_7x1[1] = ps_kd[12]*(des_vCOM_3x1[1]-vCOM_3x1[1]) + ps_kp[12]*(des_pCOM_3x1[1]-_pCOM_3x1[1]);
	ftemp2_7x1[2] = ps_kd[12]*(des_vCOM_3x1[2]-vCOM_3x1[2]) + ps_kp[12]*(des_pCOM_3x1[2]-_pCOM_3x1[2]);
	ftemp2_7x1[3] = ps_kd[12]*(des_vCOM_3x1[3]-vCOM_3x1[3]) + ps_kp[12]*(des_pCOM_3x1[3]-_pCOM_3x1[3]);
	mult_mv((const float**)_jCOMp_3x33, 3,33, _Qp_33x1, ftemp1_7x1);
	Xcom_3x1[1] = ftemp2_7x1[1] - ftemp1_7x1[1];
	Xcom_3x1[2] = ftemp2_7x1[2] - ftemp1_7x1[2];
	Xcom_3x1[3] = ftemp2_7x1[3] - ftemp1_7x1[3];
	//-----------------

	//------------ Foot
	diff_vv(des_pRF_3x1,3, _pRF_3x1, ftemp3_7x1);
	mult_mv((const float**)_jRF_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vRF_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(ps_kp[13], ftemp3_7x1,3, ps_kd[13],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jRFp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xrf_6x1);

	QTdel(des_qRF_4x1, _qRF_4x1, ftemp3_7x1);	
	diff_vv(des_wRF_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(ps_kp[13],ftemp3_7x1,3, ps_kd[13],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xrf_6x1[3]);
	
	diff_vv(des_pLF_3x1,3, _pLF_3x1, ftemp3_7x1);
	mult_mv((const float**)_jLF_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vLF_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(ps_kp[13], ftemp3_7x1,3, ps_kd[13],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jLFp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xlf_6x1);

	QTdel(des_qLF_4x1, _qLF_4x1, ftemp3_7x1);	
	diff_vv(des_wLF_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(ps_kp[13],ftemp3_7x1,3, ps_kd[13],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xlf_6x1[3]);
	//-----------------
	
	//------------ Hand
	diff_vv(des_pRH_3x1,3, _pRH_3x1, ftemp3_7x1);
	mult_mv((const float**)_jRH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
	diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(ps_kp[14], ftemp3_7x1,3, ps_kd[14],ftemp1_7x1, ftemp3_7x1);
	mult_mv((const float**)_jRHp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xrh_6x1);

	QTdel(des_qRH_4x1, _qRH_4x1, ftemp3_7x1);	
	diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(ps_kp[14],ftemp3_7x1,3, ps_kd[14],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xrh_6x1[3]);
	
	diff_vv(des_pLH_3x1,3, _pLH_3x1, ftemp3_7x1);  
	mult_mv((const float**)_jLH_6x33,6,33, _Qp_33x1, ftemp2_7x1);  
	diff_vv(des_vLH_3x1,3, ftemp2_7x1, ftemp1_7x1);
	sum_svsv(ps_kp[14], ftemp3_7x1,3, ps_kd[14],ftemp1_7x1, ftemp3_7x1);  
	mult_mv((const float**)_jLHp_6x33,6,33, _Qp_33x1, ftemp1_7x1);	  
	diff_vv(ftemp3_7x1,3, ftemp1_7x1, Xlh_6x1);

	QTdel(des_qLH_4x1, _qLH_4x1, ftemp3_7x1);	
	diff_vv(des_wLH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
	sum_svsv(ps_kp[14],ftemp3_7x1,3, ps_kd[14],ftemp4_7x1, ftemp2_7x1);
	diff_vv(ftemp2_7x1,3, &ftemp1_7x1[3], &Xlh_6x1[3]);
	//-----------------
	
	X1_33x1[1] = Xrf_6x1[1];
	X1_33x1[2] = Xrf_6x1[2];
	X1_33x1[3] = Xrf_6x1[3];
	X1_33x1[4] = Xrf_6x1[4];
	X1_33x1[5] = Xrf_6x1[5];
	X1_33x1[6] = Xrf_6x1[6];
	X1_33x1[7] = Xlf_6x1[1];
	X1_33x1[8] = Xlf_6x1[2];
	X1_33x1[9] = Xlf_6x1[3];
	X1_33x1[10] = Xlf_6x1[4];
	X1_33x1[11] = Xlf_6x1[5];
	X1_33x1[12] = Xlf_6x1[6];
	X1_33x1[13] = Xrh_6x1[1];
	X1_33x1[14] = Xrh_6x1[2];
	X1_33x1[15] = Xrh_6x1[3];
	X1_33x1[16] = Xrh_6x1[4];
	X1_33x1[17] = Xrh_6x1[5];
	X1_33x1[18] = Xrh_6x1[6];
	X1_33x1[19] = Xlh_6x1[1];
	X1_33x1[20] = Xlh_6x1[2];
	X1_33x1[21] = Xlh_6x1[3];	
	X1_33x1[22] = Xlh_6x1[4];
	X1_33x1[23] = Xlh_6x1[5];
	X1_33x1[24] = Xlh_6x1[6];
	X1_33x1[25] = Xcom_3x1[1];
	X1_33x1[26] = Xcom_3x1[2];	
	X1_33x1[27] = Xpelz;
	//X1_33x1[28] = Xpel_3x1[1];
	//X1_33x1[29] = Xpel_3x1[2];
	//X1_33x1[30] = Xpel_3x1[3];
	dim_primary_task = 27;

	//----------------- redundant tasks
	RVALS(_Q_34x1[WST_34],  _Qp_33x1[WST_33],  ps_kd[15]*(-_Qp_33x1[WST_33])+ps_kp[15]*(neutral_Q_34x1[WST_34]-_Q_34x1[WST_34]),		WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &X2_33x1[1], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSP_34],  _Qp_33x1[RSP_33],  ps_kd[15]*(-_Qp_33x1[RSP_33])+ps_kp[15]*(neutral_Q_34x1[RSP_34]-_Q_34x1[RSP_34]),   RSPpmax, RSPppmax, RSPmin, RSPmax, D2R, &X2_33x1[2], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSR_34],  _Qp_33x1[RSR_33],  ps_kd[15]*(-_Qp_33x1[RSR_33])+ps_kp[15]*(neutral_Q_34x1[RSR_34]-_Q_34x1[RSR_34]),   RSRpmax, RSRppmax, RSRmin, RSRmax, D2R, &X2_33x1[3], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RSY_34],  _Qp_33x1[RSY_33],  ps_kd[15]*(-_Qp_33x1[RSY_33])+ps_kp[15]*(neutral_Q_34x1[RSY_34]-_Q_34x1[RSY_34]),   RSYpmax, RSYppmax, RSYmin, RSYmax, D2R, &X2_33x1[4], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[REB_34],  _Qp_33x1[REB_33],  ps_kd[15]*(-_Qp_33x1[REB_33])+ps_kp[15]*(neutral_Q_34x1[REB_34]-_Q_34x1[REB_34]),   REBpmax, REBppmax, REBmin, REBmax, D2R, &X2_33x1[5], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWY_34],  _Qp_33x1[RWY_33],  ps_kd[15]*(-_Qp_33x1[RWY_33])+ps_kp[15]*(neutral_Q_34x1[RWY_34]-_Q_34x1[RWY_34]),   RWYpmax, RWYppmax, RWYmin, RWYmax, D2R, &X2_33x1[6], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWP_34],  _Qp_33x1[RWP_33],  ps_kd[15]*(-_Qp_33x1[RWP_33])+ps_kp[15]*(neutral_Q_34x1[RWP_34]-_Q_34x1[RWP_34]),   RWPpmax, RWPppmax, RWPmin, RWPmax, D2R, &X2_33x1[7], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSP_34],  _Qp_33x1[LSP_33],  ps_kd[15]*(-_Qp_33x1[LSP_33])+ps_kp[15]*(neutral_Q_34x1[LSP_34]-_Q_34x1[LSP_34]),   LSPpmax, LSPppmax, LSPmin, LSPmax, D2R, &X2_33x1[8], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSR_34],  _Qp_33x1[LSR_33],  ps_kd[15]*(-_Qp_33x1[LSR_33])+ps_kp[15]*(neutral_Q_34x1[LSR_34]-_Q_34x1[LSR_34]),   LSRpmax, LSRppmax, LSRmin, LSRmax, D2R, &X2_33x1[9], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LSY_34],  _Qp_33x1[LSY_33],  ps_kd[15]*(-_Qp_33x1[LSY_33])+ps_kp[15]*(neutral_Q_34x1[LSY_34]-_Q_34x1[LSY_34]),   LSYpmax, LSYppmax, LSYmin, LSYmax, D2R, &X2_33x1[10], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LEB_34],  _Qp_33x1[LEB_33],  ps_kd[15]*(-_Qp_33x1[LEB_33])+ps_kp[15]*(neutral_Q_34x1[LEB_34]-_Q_34x1[LEB_34]),   LEBpmax, LEBppmax, LEBmin, LEBmax, D2R, &X2_33x1[11], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWY_34],  _Qp_33x1[LWY_33],  ps_kd[15]*(-_Qp_33x1[LWY_33])+ps_kp[15]*(neutral_Q_34x1[LWY_34]-_Q_34x1[LWY_34]),   LWYpmax, LWYppmax, LWYmin, LWYmax, D2R, &X2_33x1[12], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWP_34],  _Qp_33x1[LWP_33],  ps_kd[15]*(-_Qp_33x1[LWP_33])+ps_kp[15]*(neutral_Q_34x1[LWP_34]-_Q_34x1[LWP_34]),   LWPpmax, LWPppmax, LWPmin, LWPmax, D2R, &X2_33x1[13], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[RWY2_34],  _Qp_33x1[RWY2_33],  ps_kd[15]*(-_Qp_33x1[RWY2_33])+ps_kp[15]*(neutral_Q_34x1[RWY2_34]-_Q_34x1[RWY2_34]),   RWY2pmax, RWY2ppmax, RWY2min, RWY2max, D2R, &X2_33x1[14], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	RVALS(_Q_34x1[LWY2_34],  _Qp_33x1[LWY2_33],  ps_kd[15]*(-_Qp_33x1[LWY2_33])+ps_kp[15]*(neutral_Q_34x1[LWY2_34]-_Q_34x1[LWY2_34]),   LWY2pmax, LWY2ppmax, LWY2min, LWY2max, D2R, &X2_33x1[15], &ftemp1_7x1[0], &ftemp1_7x1[1]);
	X2_33x1[16] = Xpel_3x1[1];
	X2_33x1[17] = Xpel_3x1[2];
	X2_33x1[18] = Xpel_3x1[3];
	dim_redundant_task = 18;
	//--------------------

	for(i=1; i<=33; i++)
	{
		_jT1_33x33[1][i] = _jRF_6x33[1][i];
		_jT1_33x33[2][i] = _jRF_6x33[2][i];
		_jT1_33x33[3][i] = _jRF_6x33[3][i];
		_jT1_33x33[4][i] = _jRF_6x33[4][i];
		_jT1_33x33[5][i] = _jRF_6x33[5][i];
		_jT1_33x33[6][i] = _jRF_6x33[6][i];			

		_jT1_33x33[7][i] = _jLF_6x33[1][i];
		_jT1_33x33[8][i] = _jLF_6x33[2][i];
		_jT1_33x33[9][i] = _jLF_6x33[3][i];
		_jT1_33x33[10][i] = _jLF_6x33[4][i];
		_jT1_33x33[11][i] = _jLF_6x33[5][i];
		_jT1_33x33[12][i] = _jLF_6x33[6][i];

		_jT1_33x33[13][i] = _jRH_6x33[1][i];
		_jT1_33x33[14][i] = _jRH_6x33[2][i];
		_jT1_33x33[15][i] = _jRH_6x33[3][i];
		_jT1_33x33[16][i] = _jRH_6x33[4][i];			
		_jT1_33x33[17][i] = _jRH_6x33[5][i];			
		_jT1_33x33[18][i] = _jRH_6x33[6][i];			

		_jT1_33x33[19][i] = _jLH_6x33[1][i];
		_jT1_33x33[20][i] = _jLH_6x33[2][i];
		_jT1_33x33[21][i] = _jLH_6x33[3][i];
		_jT1_33x33[22][i] = _jLH_6x33[4][i];
		_jT1_33x33[23][i] = _jLH_6x33[5][i];
		_jT1_33x33[24][i] = _jLH_6x33[6][i];

		_jT1_33x33[25][i] = _jCOM_3x33[1][i];
		_jT1_33x33[26][i] = _jCOM_3x33[2][i];

		_jT1_33x33[27][i] = 0.f;
		//_jT1_33x33[28][i] = 0.f;
		//_jT1_33x33[29][i] = 0.f;
		//_jT1_33x33[30][i] = 0.f;   
		for(j=1;j<=18;j++)
			_jT2_33x33[j][i] = 0.f;
	}
	_jT1_33x33[27][3] = 1.f;
  //_jT1_33x33[28][4] = 1.f;
  //_jT1_33x33[29][5] = 1.f;
  //_jT1_33x33[30][6] = 1.f;

	for(i=1; i<=27; i++)
	{
		_jT1_33x33[i][4] = 0.f;  // exclude pelvis orientation from the solution
		_jT1_33x33[i][5] = 0.f; 
		_jT1_33x33[i][6] = 0.f; 
	}

	for(i=13; i<=24; i++)
	{
		_jT1_33x33[i][7] = 0.f;  // exclude WST from the solution
	}

	for(j=20; j<=33; j++)
	{
		_jT1_33x33[25][j] = 0.f; // exclude the upper body from COM solution
		_jT1_33x33[26][j] = 0.f;
	}
	_jT1_33x33[25][7] = 0.f;
	_jT1_33x33[26][7] = 0.f;

	_jT2_33x33[1][WST_33] = 1.f;
	_jT2_33x33[2][RSP_33] = 1.f;
	_jT2_33x33[3][RSR_33] = 1.f;
	_jT2_33x33[4][RSY_33] = 1.f;
	_jT2_33x33[5][REB_33] = 1.f;
	_jT2_33x33[6][RWY_33] = 1.f;
	_jT2_33x33[7][RWP_33] = 1.f;
	_jT2_33x33[8][LSP_33] = 1.f;
	_jT2_33x33[9][LSR_33] = 1.f;	
	_jT2_33x33[10][LSY_33] = 1.f;
	_jT2_33x33[11][LEB_33] = 1.f;
	_jT2_33x33[12][LWY_33] = 1.f;
	_jT2_33x33[13][LWP_33] = 1.f;
	_jT2_33x33[14][RWY2_33] = 1.f;
	_jT2_33x33[15][LWY2_33] = 1.f;	
	_jT2_33x33[16][WX_33] = 1.f;
	_jT2_33x33[17][WY_33] = 1.f;
	_jT2_33x33[18][WZ_33] = 1.f;
	
	for(i=1; i<=33; i++)
	{
		_jT1_33x33[i][WX_33] *= WEIGHT_PEL_SQRT_INV;
		_jT1_33x33[i][WY_33] *= WEIGHT_PEL_SQRT_INV;
		_jT1_33x33[i][WZ_33] *= WEIGHT_PEL_SQRT_INV;
		_jT1_33x33[i][WST_33] *= WEIGHT_WST_SQRT_INV;

		_jT2_33x33[i][WX_33] *= WEIGHT_PEL_SQRT_INV;
		_jT2_33x33[i][WY_33] *= WEIGHT_PEL_SQRT_INV;
		_jT2_33x33[i][WZ_33] *= WEIGHT_PEL_SQRT_INV;			
		_jT2_33x33[i][WST_33] *= WEIGHT_WST_SQRT_INV;
	}
	
	if(pinv_SR((const float**)_jT1_33x33, dim_primary_task, 33, (float)1.e-10, _jT1inv_33x33) != 0)
		return -2; // singularity occurred
	mult_mm((const float**)_jT1inv_33x33,33,dim_primary_task, (const float**)_jT1_33x33, 33, _TEMP1_34x34);
	diff_mm((const float**)_EYE_33, 33,33, (const float**)_TEMP1_34x34, _N1_33x33);	

	//pinv_svd((const float**)_jT2_33x33,18,33, _jT2inv_33x33);
	trans(1.f, (const float**)_jT2_33x33, dim_redundant_task,33, _jT2inv_33x33);

	//mult_mv((const float**)_jT1inv_33x33,33,dim_primary_task, (const float*)X1_33x1, Qpp_33x1);
	mult_mv((const float**)_jT1inv_33x33,33,dim_primary_task, (const float*)X1_33x1, _TEMP1_34x34[1]);
	mult_mv((const float**)_jT2inv_33x33,33,dim_redundant_task, (const float*)X2_33x1, _TEMP2_34x34[1]);
	mult_mv((const float**)_N1_33x33,33,33, (const float*)_TEMP2_34x34[1], Qpp_33x1);
	sum_vv((const float*)_TEMP1_34x34[1], 33, Qpp_33x1, Qpp_33x1);
	Qpp_33x1[WX_33] *= WEIGHT_PEL_SQRT_INV;
	Qpp_33x1[WY_33] *= WEIGHT_PEL_SQRT_INV;
	Qpp_33x1[WZ_33] *= WEIGHT_PEL_SQRT_INV;
	Qpp_33x1[WST_33] *= WEIGHT_WST_SQRT_INV;
	
	//------------------ check constraints
	RVALS3(_Q_34x1[WST_34], _Qp_33x1[WST_33], WSTpmax, WSTppmax, WSTmin, WSTmax, D2R, &lb_27x1[1], &ub_27x1[1]);
	RVALS3(_Q_34x1[RHY_34], _Qp_33x1[RHY_33], RHYpmax, RHYppmax, RHYmin, RHYmax, D2R, &lb_27x1[2], &ub_27x1[2]);
	RVALS3(_Q_34x1[RHR_34], _Qp_33x1[RHR_33], RHRpmax, RHRppmax, RHRmin, RHRmax, D2R, &lb_27x1[3], &ub_27x1[3]);
	RVALS3(_Q_34x1[RHP_34], _Qp_33x1[RHP_33], RHPpmax, RHPppmax, RHPmin, RHPmax, D2R, &lb_27x1[4], &ub_27x1[4]);
	RVALS3(_Q_34x1[RKN_34], _Qp_33x1[RKN_33], RKNpmax, RKNppmax, RKNmin, RKNmax, D2R, &lb_27x1[5], &ub_27x1[5]);
	RVALS3(_Q_34x1[RAP_34], _Qp_33x1[RAP_33], RAPpmax, RAPppmax, RAPmin, RAPmax, D2R, &lb_27x1[6], &ub_27x1[6]);
	RVALS3(_Q_34x1[RAR_34], _Qp_33x1[RAR_33], RARpmax, RARppmax, RARmin, RARmax, D2R, &lb_27x1[7], &ub_27x1[7]);
	RVALS3(_Q_34x1[LHY_34], _Qp_33x1[LHY_33], LHYpmax, LHYppmax, LHYmin, LHYmax, D2R, &lb_27x1[8], &ub_27x1[8]);
	RVALS3(_Q_34x1[LHR_34], _Qp_33x1[LHR_33], LHRpmax, LHRppmax, LHRmin, LHRmax, D2R, &lb_27x1[9], &ub_27x1[9]);
	RVALS3(_Q_34x1[LHP_34], _Qp_33x1[LHP_33], LHPpmax, LHPppmax, LHPmin, LHPmax, D2R, &lb_27x1[10], &ub_27x1[10]);
	RVALS3(_Q_34x1[LKN_34], _Qp_33x1[LKN_33], LKNpmax, LKNppmax, LKNmin, LKNmax, D2R, &lb_27x1[11], &ub_27x1[11]);
	RVALS3(_Q_34x1[LAP_34], _Qp_33x1[LAP_33], LAPpmax, LAPppmax, LAPmin, LAPmax, D2R, &lb_27x1[12], &ub_27x1[12]);
	RVALS3(_Q_34x1[LAR_34], _Qp_33x1[LAR_33], LARpmax, LARppmax, LARmin, LARmax, D2R, &lb_27x1[13], &ub_27x1[13]);
	RVALS3(_Q_34x1[RSP_34], _Qp_33x1[RSP_33], RSPpmax, RSPppmax, RSPmin, RSPmax, D2R, &lb_27x1[14], &ub_27x1[14]);
	RVALS3(_Q_34x1[RSR_34], _Qp_33x1[RSR_33], RSRpmax, RSRppmax, RSRmin, RSRmax, D2R, &lb_27x1[15], &ub_27x1[15]);
	RVALS3(_Q_34x1[RSY_34], _Qp_33x1[RSY_33], RSYpmax, RSYppmax, RSYmin, RSYmax, D2R, &lb_27x1[16], &ub_27x1[16]);
	RVALS3(_Q_34x1[REB_34], _Qp_33x1[REB_33], REBpmax, REBppmax, REBmin, REBmax, D2R, &lb_27x1[17], &ub_27x1[17]);
	RVALS3(_Q_34x1[RWY_34], _Qp_33x1[RWY_33], RWYpmax, RWYppmax, RWYmin, RWYmax, D2R, &lb_27x1[18], &ub_27x1[18]);
	RVALS3(_Q_34x1[RWP_34], _Qp_33x1[RWP_33], RWPpmax, RWPppmax, RWPmin, RWPmax, D2R, &lb_27x1[19], &ub_27x1[19]);	
	RVALS3(_Q_34x1[LSP_34], _Qp_33x1[LSP_33], LSPpmax, LSPppmax, LSPmin, LSPmax, D2R, &lb_27x1[20], &ub_27x1[20]);
	RVALS3(_Q_34x1[LSR_34], _Qp_33x1[LSR_33], LSRpmax, LSRppmax, LSRmin, LSRmax, D2R, &lb_27x1[21], &ub_27x1[21]);
	RVALS3(_Q_34x1[LSY_34], _Qp_33x1[LSY_33], LSYpmax, LSYppmax, LSYmin, LSYmax, D2R, &lb_27x1[22], &ub_27x1[22]);
	RVALS3(_Q_34x1[LEB_34], _Qp_33x1[LEB_33], LEBpmax, LEBppmax, LEBmin, LEBmax, D2R, &lb_27x1[23], &ub_27x1[23]);
	RVALS3(_Q_34x1[LWY_34], _Qp_33x1[LWY_33], LWYpmax, LWYppmax, LWYmin, LWYmax, D2R, &lb_27x1[24], &ub_27x1[24]);
	RVALS3(_Q_34x1[LWP_34], _Qp_33x1[LWP_33], LWPpmax, LWPppmax, LWPmin, LWPmax, D2R, &lb_27x1[25], &ub_27x1[25]);
	RVALS3(_Q_34x1[RWY2_34], _Qp_33x1[RWY2_33], RWY2pmax, RWY2ppmax, RWY2min, RWY2max, D2R, &lb_27x1[26], &ub_27x1[26]);
	RVALS3(_Q_34x1[LWY2_34], _Qp_33x1[LWY2_33], LWY2pmax, LWY2ppmax, LWY2min, LWY2max, D2R, &lb_27x1[27], &ub_27x1[27]);

//printf("%f %f %f %f %f\n",LKNpmax, LKNppmax, LKNmin, LKNmax,D2R);
//  printf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",lb_27x1[1],lb_27x1[2],lb_27x1[3],lb_27x1[4],lb_27x1[5],lb_27x1[6],lb_27x1[7],lb_27x1[8],lb_27x1[9],lb_27x1[10],lb_27x1[11],lb_27x1[12],lb_27x1[13],lb_27x1[14],lb_27x1[15],lb_27x1[16],lb_27x1[17],lb_27x1[18],lb_27x1[19],lb_27x1[20],lb_27x1[21],lb_27x1[22],lb_27x1[23],lb_27x1[24],lb_27x1[25],lb_27x1[26],lb_27x1[27]);
//for(i=1;i<=27;i++)
//{ printf(" %f %f %f %f %f %f\n",R2D,Qpp_33x1[i+6]*R2D, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D,lb_27x1[i],ub_27x1[i]);}
	do
	{
		isLimited = 0;
		if(ps_torque_mode_flag == 0 || ps_torque_mode_flag == 3)
			for(i=1;i<=27;i++)
			{
				index_limited[i] = 0;
				qpp_limited[i] = 0.f;
				if(Qpp_33x1[i+6] >= ub_27x1[i])//>=
				{
					isLimited = 1;
					index_limited[i] = 1;
					qpp_limited[i] = ub_27x1[i];
				}
				else if(Qpp_33x1[i+6] <= lb_27x1[i])//<=
				{
					isLimited = 1;
					index_limited[i] = 1;
					qpp_limited[i] = lb_27x1[i];
				}

				if(isLimited==1)
				{
					printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
					printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);			
					return -2;
				}

				if(fabs(_Qp_33x1[i+6])>=QP_MAX)
				{
					printf("\n i: %d, joint: %d, p: %.4fdeg, v: %.4fdeg/s, a: %.4fdeg/s^2", _offline_traj.i, i+6, _Q_34x1[i+7]*R2D, _Qp_33x1[i+6]*R2D, Qpp_33x1[i+6]*R2D);
					printf("\n ub: %.4fdeg/s^2, lb: %.4fdeg/s^2", ub_27x1[i]*R2D, lb_27x1[i]*R2D);			
					return -2;
				}
			}
		
		if(isLimited==1)
		{
			// will be filled			
		}
	} while(isLimited==1);
	
	_Q_34x1[1] += _Qp_33x1[1]*DT + 0.5f*Qpp_33x1[1]*DT*DT;
	_Q_34x1[2] += _Qp_33x1[2]*DT + 0.5f*Qpp_33x1[2]*DT*DT;
	_Q_34x1[3] += _Qp_33x1[3]*DT + 0.5f*Qpp_33x1[3]*DT*DT;

	//------------------ update the pelvis quaternion 
	Wq(0, &_Q_34x1[3], _TEMP1_34x34);
	trans2(1.f, _TEMP1_34x34,3,4);
	mult_smv(0.5f, (const float**)_TEMP1_34x34,4,3, &Qpp_33x1[3], (float*)_TEMP2_34x34[1]); // qtpp

	Qq(0, &_Q_34x1[3], _TEMP1_34x34);
	_TEMP1_34x34[5][1] = 0.f;
	_TEMP1_34x34[5][2] = _Qp_33x1[4];
	_TEMP1_34x34[5][3] = _Qp_33x1[5];
	_TEMP1_34x34[5][4] = _Qp_33x1[6];
	mult_smv(0.5f, (const float**)_TEMP1_34x34,4,4, (const float*)_TEMP1_34x34[5], (float*)_TEMP3_34x34[1]); // qtp

	_Q_34x1[4] += _TEMP3_34x34[1][1]*DT + 0.5f*DT*DT*_TEMP2_34x34[1][1];
	_Q_34x1[5] += _TEMP3_34x34[1][2]*DT + 0.5f*DT*DT*_TEMP2_34x34[1][2];
	_Q_34x1[6] += _TEMP3_34x34[1][3]*DT + 0.5f*DT*DT*_TEMP2_34x34[1][3];
	_Q_34x1[7] += _TEMP3_34x34[1][4]*DT + 0.5f*DT*DT*_TEMP2_34x34[1][4];

	ftemp1_7x1[0] = (float)sqrt(_Q_34x1[4]*_Q_34x1[4] + _Q_34x1[5]*_Q_34x1[5] + _Q_34x1[6]*_Q_34x1[6] + _Q_34x1[7]*_Q_34x1[7]);
	_Q_34x1[4] /= ftemp1_7x1[0];
	_Q_34x1[5] /= ftemp1_7x1[0];
	_Q_34x1[6] /= ftemp1_7x1[0];
	_Q_34x1[7] /= ftemp1_7x1[0];
	//----------------------------

	_Q_34x1[WST_34] += _Qp_33x1[WST_33]*DT + 0.5f*Qpp_33x1[7]*DT*DT;
	_Q_34x1[RHY_34] += _Qp_33x1[RHY_33]*DT + 0.5f*Qpp_33x1[8]*DT*DT;
	_Q_34x1[RHR_34] += _Qp_33x1[RHR_33]*DT + 0.5f*Qpp_33x1[9]*DT*DT;
	_Q_34x1[RHP_34] += _Qp_33x1[RHP_33]*DT + 0.5f*Qpp_33x1[10]*DT*DT;
	_Q_34x1[RKN_34] += _Qp_33x1[RKN_33]*DT + 0.5f*Qpp_33x1[11]*DT*DT;
	_Q_34x1[RAP_34] += _Qp_33x1[RAP_33]*DT + 0.5f*Qpp_33x1[12]*DT*DT;
	_Q_34x1[RAR_34] += _Qp_33x1[RAR_33]*DT + 0.5f*Qpp_33x1[13]*DT*DT;
	_Q_34x1[LHY_34] += _Qp_33x1[LHY_33]*DT + 0.5f*Qpp_33x1[14]*DT*DT;
	_Q_34x1[LHR_34] += _Qp_33x1[LHR_33]*DT + 0.5f*Qpp_33x1[15]*DT*DT;
	_Q_34x1[LHP_34] += _Qp_33x1[LHP_33]*DT + 0.5f*Qpp_33x1[16]*DT*DT;
	_Q_34x1[LKN_34] += _Qp_33x1[LKN_33]*DT + 0.5f*Qpp_33x1[17]*DT*DT;
	_Q_34x1[LAP_34] += _Qp_33x1[LAP_33]*DT + 0.5f*Qpp_33x1[18]*DT*DT;
	_Q_34x1[LAR_34] += _Qp_33x1[LAR_33]*DT + 0.5f*Qpp_33x1[19]*DT*DT;	
	_Q_34x1[RSP_34] += _Qp_33x1[RSP_33]*DT + 0.5f*Qpp_33x1[20]*DT*DT;
	_Q_34x1[RSR_34] += _Qp_33x1[RSR_33]*DT + 0.5f*Qpp_33x1[21]*DT*DT;
	_Q_34x1[RSY_34] += _Qp_33x1[RSY_33]*DT + 0.5f*Qpp_33x1[22]*DT*DT;
	_Q_34x1[REB_34] += _Qp_33x1[REB_33]*DT + 0.5f*Qpp_33x1[23]*DT*DT;
	_Q_34x1[RWY_34] += _Qp_33x1[RWY_33]*DT + 0.5f*Qpp_33x1[24]*DT*DT;
	_Q_34x1[RWP_34] += _Qp_33x1[RWP_33]*DT + 0.5f*Qpp_33x1[25]*DT*DT;
	_Q_34x1[LSP_34] += _Qp_33x1[LSP_33]*DT + 0.5f*Qpp_33x1[26]*DT*DT;
	_Q_34x1[LSR_34] += _Qp_33x1[LSR_33]*DT + 0.5f*Qpp_33x1[27]*DT*DT;
	_Q_34x1[LSY_34] += _Qp_33x1[LSY_33]*DT + 0.5f*Qpp_33x1[28]*DT*DT;
	_Q_34x1[LEB_34] += _Qp_33x1[LEB_33]*DT + 0.5f*Qpp_33x1[29]*DT*DT;
	_Q_34x1[LWY_34] += _Qp_33x1[LWY_33]*DT + 0.5f*Qpp_33x1[30]*DT*DT;
	_Q_34x1[LWP_34] += _Qp_33x1[LWP_33]*DT + 0.5f*Qpp_33x1[31]*DT*DT;
	_Q_34x1[RWY2_34] += _Qp_33x1[RWY2_33]*DT + 0.5f*Qpp_33x1[32]*DT*DT;
	_Q_34x1[LWY2_34] += _Qp_33x1[LWY2_33]*DT + 0.5f*Qpp_33x1[33]*DT*DT;
	
	_Qp_33x1[1] += DT*Qpp_33x1[1];
	_Qp_33x1[2] += DT*Qpp_33x1[2];
	_Qp_33x1[3] += DT*Qpp_33x1[3];
	_Qp_33x1[4] += DT*Qpp_33x1[4];
	_Qp_33x1[5] += DT*Qpp_33x1[5];
	_Qp_33x1[6] += DT*Qpp_33x1[6];
	_Qp_33x1[WST_33] += DT*Qpp_33x1[7];
	_Qp_33x1[RHY_33] += DT*Qpp_33x1[8];
	_Qp_33x1[RHR_33] += DT*Qpp_33x1[9];
	_Qp_33x1[RHP_33] += DT*Qpp_33x1[10];
	_Qp_33x1[RKN_33] += DT*Qpp_33x1[11];
	_Qp_33x1[RAP_33] += DT*Qpp_33x1[12];
	_Qp_33x1[RAR_33] += DT*Qpp_33x1[13];
	_Qp_33x1[LHY_33] += DT*Qpp_33x1[14];
	_Qp_33x1[LHR_33] += DT*Qpp_33x1[15];
	_Qp_33x1[LHP_33] += DT*Qpp_33x1[16];
	_Qp_33x1[LKN_33] += DT*Qpp_33x1[17];
	_Qp_33x1[LAP_33] += DT*Qpp_33x1[18];
	_Qp_33x1[LAR_33] += DT*Qpp_33x1[19];	
	_Qp_33x1[RSP_33] += DT*Qpp_33x1[20];
	_Qp_33x1[RSR_33] += DT*Qpp_33x1[21];
	_Qp_33x1[RSY_33] += DT*Qpp_33x1[22];
	_Qp_33x1[REB_33] += DT*Qpp_33x1[23];
	_Qp_33x1[RWY_33] += DT*Qpp_33x1[24];
	_Qp_33x1[RWP_33] += DT*Qpp_33x1[25];
	_Qp_33x1[LSP_33] += DT*Qpp_33x1[26];
	_Qp_33x1[LSR_33] += DT*Qpp_33x1[27];
	_Qp_33x1[LSY_33] += DT*Qpp_33x1[28];
	_Qp_33x1[LEB_33] += DT*Qpp_33x1[29];
	_Qp_33x1[LWY_33] += DT*Qpp_33x1[30];
	_Qp_33x1[LWP_33] += DT*Qpp_33x1[31];
	_Qp_33x1[RWY2_33] += DT*Qpp_33x1[32];
	_Qp_33x1[LWY2_33] += DT*Qpp_33x1[33];

	WSTRefAngleCurrent = _Q_34x1[WST_34]*R2D;

	RHYRefAngleCurrent = (_Q_34x1[RHY_34]+rhy_compen)*R2D;
	RHRRefAngleCurrent = (_Q_34x1[RHR_34]+rhr_compen)*R2D;
	RHPRefAngleCurrent = _Q_34x1[RHP_34]*R2D;
	RKNRefAngleCurrent = _Q_34x1[RKN_34]*R2D;
	RAPRefAngleCurrent = _Q_34x1[RAP_34]*R2D;
	RARRefAngleCurrent = _Q_34x1[RAR_34]*R2D;
	LHYRefAngleCurrent = (_Q_34x1[LHY_34]+lhy_compen)*R2D;
	LHRRefAngleCurrent = (_Q_34x1[LHR_34]+lhr_compen)*R2D;
	LHPRefAngleCurrent = _Q_34x1[LHP_34]*R2D;
	LKNRefAngleCurrent = _Q_34x1[LKN_34]*R2D;
	LAPRefAngleCurrent = _Q_34x1[LAP_34]*R2D;
	LARRefAngleCurrent = _Q_34x1[LAR_34]*R2D;

	RSPRefAngleCurrent = _Q_34x1[RSP_34]*R2D;
	RSRRefAngleCurrent = _Q_34x1[RSR_34]*R2D - OFFSET_RSR;
	RSYRefAngleCurrent = _Q_34x1[RSY_34]*R2D;
	REBRefAngleCurrent = _Q_34x1[REB_34]*R2D - OFFSET_REB;
	RWYRefAngleCurrent = _Q_34x1[RWY_34]*R2D;
	RWPRefAngleCurrent = _Q_34x1[RWP_34]*R2D;
	RWY2RefAngleCurrent = _Q_34x1[RWY2_34]*R2D;
	LSPRefAngleCurrent = _Q_34x1[LSP_34]*R2D;
	LSRRefAngleCurrent = _Q_34x1[LSR_34]*R2D - OFFSET_LSR;
        LSYRefAngleCurrent = _Q_34x1[LSY_34]*R2D;
	LEBRefAngleCurrent = _Q_34x1[LEB_34]*R2D - OFFSET_LEB;	
	LWYRefAngleCurrent = _Q_34x1[LWY_34]*R2D;
	LWPRefAngleCurrent = _Q_34x1[LWP_34]*R2D;
	LWY2RefAngleCurrent = _Q_34x1[LWY2_34]*R2D;
        /*
	pSharedMemory->disp_pCOM[0] = _pCOM_3x1[1];
	pSharedMemory->disp_pCOM[1] = _pCOM_3x1[2];

	pSharedMemory->disp_pRF[0] = _pRF_3x1[1];
	pSharedMemory->disp_pRF[1] = _pRF_3x1[2];
	pSharedMemory->disp_pRF[2] = _pRF_3x1[3];

	pSharedMemory->disp_pLF[0] = _pLF_3x1[1];
	pSharedMemory->disp_pLF[1] = _pLF_3x1[2];
	pSharedMemory->disp_pLF[2] = _pLF_3x1[3];

	pSharedMemory->disp_qRF[0] = _qRF_4x1[1];
	pSharedMemory->disp_qRF[1] = _qRF_4x1[2];
	pSharedMemory->disp_qRF[2] = _qRF_4x1[3];
	pSharedMemory->disp_qRF[3] = _qRF_4x1[4];

	pSharedMemory->disp_qLF[0] = _qLF_4x1[1];
	pSharedMemory->disp_qLF[1] = _qLF_4x1[2];
	pSharedMemory->disp_qLF[2] = _qLF_4x1[3];
	pSharedMemory->disp_qLF[3] = _qLF_4x1[4];
	
	pSharedMemory->disp_pRH[0] = _pRH_3x1[1];
	pSharedMemory->disp_pRH[1] = _pRH_3x1[2];
	pSharedMemory->disp_pRH[2] = _pRH_3x1[3];

	pSharedMemory->disp_pLH[0] = _pLH_3x1[1];
	pSharedMemory->disp_pLH[1] = _pLH_3x1[2];
	pSharedMemory->disp_pLH[2] = _pLH_3x1[3];

	pSharedMemory->disp_qRH[0] = _qRH_4x1[1];
	pSharedMemory->disp_qRH[1] = _qRH_4x1[2];
	pSharedMemory->disp_qRH[2] = _qRH_4x1[3];
	pSharedMemory->disp_qRH[3] = _qRH_4x1[4];

	pSharedMemory->disp_qLH[0] = _qLH_4x1[1];
	pSharedMemory->disp_qLH[1] = _qLH_4x1[2];
	pSharedMemory->disp_qLH[2] = _qLH_4x1[3];
	pSharedMemory->disp_qLH[3] = _qLH_4x1[4];

	for(i=1;i<=34;i++)
		pSharedMemory->disp_Q_34x1[i] = Qpp_33x1[i];
        */
	//----------------------------------------------------------------- 100Hz
	if(half_flag != 0)
	{
		half_flag = 0;
		return 0;
	}
	else
		half_flag = 1;
	
	diff_mm((const float**)_jRH_6x33,6,33, (const float**)_jRH_old2_6x33, _jRHp_6x33);
	subs_m((const float**)_jRH_old_6x33,6,33, _jRH_old2_6x33);
	subs_m((const float**)_jRH_6x33,6,33, _jRH_old_6x33);
	mult_sm((const float**)_jRHp_6x33,6,33, 1.f/(2.f*2.f*DT), _jRHp_6x33);
	
	diff_mm((const float**)_jLH_6x33,6,33, (const float**)_jLH_old2_6x33, _jLHp_6x33);
	subs_m((const float**)_jLH_old_6x33,6,33, _jLH_old2_6x33);
	subs_m((const float**)_jLH_6x33,6,33, _jLH_old_6x33);
	mult_sm((const float**)_jLHp_6x33,6,33, 1.f/(2.f*2.f*DT), _jLHp_6x33);


	for(i=1; i<=34; i++)
		meas_Q_34x1[i] = _Q_34x1[i];

	meas_Q_34x1[RSP_34] = ((float)RSPEncoderValue)*D2R;
	meas_Q_34x1[RSR_34] = ((float)RSREncoderValue)*D2R + OFFSET_RSR*D2R;
	meas_Q_34x1[RSY_34] = ((float)RSYEncoderValue)*D2R;
	meas_Q_34x1[REB_34] = ((float)REBEncoderValue)*D2R + OFFSET_REB*D2R;
	meas_Q_34x1[RWY_34] = ((float)RWYEncoderValue)*D2R;
	meas_Q_34x1[RWP_34] = ((float)RWPEncoderValue)*D2R;
	meas_Q_34x1[RWY2_34] = ((float)RWY2EncoderValue)*D2R;

	meas_Q_34x1[LSP_34] = ((float)LSPEncoderValue)*D2R;
	meas_Q_34x1[LSR_34] = ((float)LSREncoderValue)*D2R + OFFSET_LSR*D2R;
	meas_Q_34x1[LSY_34] = ((float)LSYEncoderValue)*D2R;
	meas_Q_34x1[LEB_34] = ((float)LEBEncoderValue)*D2R + OFFSET_LEB*D2R;
	meas_Q_34x1[LWY_34] = ((float)LWYEncoderValue)*D2R;
	meas_Q_34x1[LWP_34] = ((float)LWPEncoderValue)*D2R;
	meas_Q_34x1[LWY2_34] = ((float)LWY2EncoderValue)*D2R;

	meas_Qp_33x1[RSP_33] = (meas_Q_34x1[RSP_34]-Q_old2_34x1[RSP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RSR_33] = (meas_Q_34x1[RSR_34]-Q_old2_34x1[RSR_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RSY_33] = (meas_Q_34x1[RSY_34]-Q_old2_34x1[RSY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[REB_33] = (meas_Q_34x1[REB_34]-Q_old2_34x1[REB_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RWY_33] = (meas_Q_34x1[RWY_34]-Q_old2_34x1[RWY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RWP_33] = (meas_Q_34x1[RWP_34]-Q_old2_34x1[RWP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[RWY2_33] = (meas_Q_34x1[RWY2_34]-Q_old2_34x1[RWY2_34])/(2.f*2.f*DT);

	meas_Qp_33x1[LSP_33] = (meas_Q_34x1[LSP_34]-Q_old2_34x1[LSP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LSR_33] = (meas_Q_34x1[LSR_34]-Q_old2_34x1[LSR_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LSY_33] = (meas_Q_34x1[LSY_34]-Q_old2_34x1[LSY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LEB_33] = (meas_Q_34x1[LEB_34]-Q_old2_34x1[LEB_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWY_33] = (meas_Q_34x1[LWY_34]-Q_old2_34x1[LWY_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWP_33] = (meas_Q_34x1[LWP_34]-Q_old2_34x1[LWP_34])/(2.f*2.f*DT);
	meas_Qp_33x1[LWY2_33] = (meas_Q_34x1[LWY2_34]-Q_old2_34x1[LWY2_34])/(2.f*2.f*DT);

/*
	//------------------------ joint limits
	for(i=20;i<=33;i++)
		if(fabs(meas_Qp_33x1[i])> QP_MAX)
		{
			RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, 0, 0, 0);

			RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, 0, 0, 0);
			RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, 0, 0, 0);

			printf("\n velocity limit error! - n: %d, joint: %d, p: %.4fdeg, p_old: %.4fdeg, p_old2: %.4fdeg, v: %.4frad/s", n, i, meas_Q_34x1[i+1]*R2D, Q_old_34x1[i+1]*R2D, Q_old2_34x1[i+1]*R2D, meas_Qp_33x1[i]);
			printf("\n %f",_Q_34x1[i+1]*R2D);
			return -2;
		}
	//-------------------------------
*/
	Q_old2_34x1[RSP_34] = Q_old_34x1[RSP_34];
	Q_old2_34x1[RSR_34] = Q_old_34x1[RSR_34];
	Q_old2_34x1[RSY_34] = Q_old_34x1[RSY_34];
	Q_old2_34x1[REB_34] = Q_old_34x1[REB_34];
	Q_old2_34x1[RWY_34] = Q_old_34x1[RWY_34];
	Q_old2_34x1[RWP_34] = Q_old_34x1[RWP_34];
	Q_old2_34x1[RWY2_34] = Q_old_34x1[RWY2_34];

	Q_old2_34x1[LSP_34] = Q_old_34x1[LSP_34];
	Q_old2_34x1[LSR_34] = Q_old_34x1[LSR_34];
	Q_old2_34x1[LSY_34] = Q_old_34x1[LSY_34];
	Q_old2_34x1[LEB_34] = Q_old_34x1[LEB_34];
	Q_old2_34x1[LWY_34] = Q_old_34x1[LWY_34];
	Q_old2_34x1[LWP_34] = Q_old_34x1[LWP_34];
	Q_old2_34x1[LWY2_34] = Q_old_34x1[LWY2_34];

	Q_old_34x1[RSP_34] = meas_Q_34x1[RSP_34];
	Q_old_34x1[RSR_34] = meas_Q_34x1[RSR_34];
	Q_old_34x1[RSY_34] = meas_Q_34x1[RSY_34];
	Q_old_34x1[REB_34] = meas_Q_34x1[REB_34];
	Q_old_34x1[RWY_34] = meas_Q_34x1[RWY_34];
	Q_old_34x1[RWP_34] = meas_Q_34x1[RWP_34];
	Q_old_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

	Q_old_34x1[LSP_34] = meas_Q_34x1[LSP_34];
	Q_old_34x1[LSR_34] = meas_Q_34x1[LSR_34];
	Q_old_34x1[LSY_34] = meas_Q_34x1[LSY_34];
	Q_old_34x1[LEB_34] = meas_Q_34x1[LEB_34];
	Q_old_34x1[LWY_34] = meas_Q_34x1[LWY_34];
	Q_old_34x1[LWP_34] = meas_Q_34x1[LWP_34];
	Q_old_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];


	if(ps_torque_mode_flag == 0)
	{
		for(i=1;i<=3;i++)
		{
			pRH2_3x1[i] = _pRH_3x1[i];
			pLH2_3x1[i] = _pLH_3x1[i];
			qRH2_4x1[i] = _qRH_4x1[i];
			qLH2_4x1[i] = _qLH_4x1[i];
		}
		qRH2_4x1[4] = _qRH_4x1[4];
		qLH2_4x1[4] = _qLH_4x1[4];

		pre_gain = 0.f;
		count = 0;
	}
	else if(ps_torque_mode_flag == 1 || ps_torque_mode_flag == 2)
	{
		for(i=1;i<=3;i++)
		{
			pRH1_3x1[i] = _pRH_3x1[i];
			pLH1_3x1[i] = _pLH_3x1[i];
			qRH1_4x1[i] = _qRH_4x1[i];
			qLH1_4x1[i] = _qLH_4x1[i];
		}
		qRH1_4x1[4] = _qRH_4x1[4];
		qLH1_4x1[4] = _qLH_4x1[4];
	
		if(ps_torque_mode_flag == 1)
		{
			pre_gain = (float)count/20.f;
			count++;
			if(count>20)
			{
				ps_torque_mode_flag = 2;
				printf("\n torque mode");
			}
		}
		else
		{
			pre_gain = 1.f;
			count = 0;
		}
		
	}
	else if(ps_torque_mode_flag == 3)
	{	
		_Q_34x1[RSP_34] = meas_Q_34x1[RSP_34];
		_Q_34x1[RSR_34] = meas_Q_34x1[RSR_34];
		_Q_34x1[RSY_34] = meas_Q_34x1[RSY_34];
		_Q_34x1[REB_34] = meas_Q_34x1[REB_34];
		_Q_34x1[RWY_34] = meas_Q_34x1[RWY_34];
		_Q_34x1[RWP_34] = meas_Q_34x1[RWP_34];
		_Q_34x1[RWY2_34] = meas_Q_34x1[RWY2_34];

		_Q_34x1[LSP_34] = meas_Q_34x1[LSP_34];
		_Q_34x1[LSR_34] = meas_Q_34x1[LSR_34];
		_Q_34x1[LSY_34] = meas_Q_34x1[LSY_34];
		_Q_34x1[LEB_34] = meas_Q_34x1[LEB_34];
		_Q_34x1[LWY_34] = meas_Q_34x1[LWY_34];
		_Q_34x1[LWP_34] = meas_Q_34x1[LWP_34];
		_Q_34x1[LWY2_34] = meas_Q_34x1[LWY2_34];

		_Qp_33x1[RSP_33] = meas_Qp_33x1[RSP_33];
		_Qp_33x1[RSR_33] = meas_Qp_33x1[RSR_33];
		_Qp_33x1[RSY_33] = meas_Qp_33x1[RSY_33];
		_Qp_33x1[REB_33] = meas_Qp_33x1[REB_33];
		_Qp_33x1[RWY_33] = meas_Qp_33x1[RWY_33];
		_Qp_33x1[RWP_33] = meas_Qp_33x1[RWP_33];
		_Qp_33x1[RWY2_33] = meas_Qp_33x1[RWY2_33];

		_Qp_33x1[LSP_33] = meas_Qp_33x1[LSP_33];
		_Qp_33x1[LSR_33] = meas_Qp_33x1[LSR_33];
		_Qp_33x1[LSY_33] = meas_Qp_33x1[LSY_33];
		_Qp_33x1[LEB_33] = meas_Qp_33x1[LEB_33];
		_Qp_33x1[LWY_33] = meas_Qp_33x1[LWY_33];
		_Qp_33x1[LWP_33] = meas_Qp_33x1[LWP_33];
		_Qp_33x1[LWY2_33] = meas_Qp_33x1[LWY2_33];

		RSPRefAngleCurrent = _Q_34x1[RSP_34]*R2D;
		RSRRefAngleCurrent = _Q_34x1[RSR_34]*R2D - OFFSET_RSR;
		RSYRefAngleCurrent = _Q_34x1[RSY_34]*R2D;
		REBRefAngleCurrent = _Q_34x1[REB_34]*R2D - OFFSET_REB;
		RWYRefAngleCurrent = _Q_34x1[RWY_34]*R2D;
		RWPRefAngleCurrent = _Q_34x1[RWP_34]*R2D;
		RWY2RefAngleCurrent = _Q_34x1[RWY2_34]*R2D;
		LSPRefAngleCurrent = _Q_34x1[LSP_34]*R2D;
		LSRRefAngleCurrent = _Q_34x1[LSR_34]*R2D - OFFSET_LSR;
		LSYRefAngleCurrent = _Q_34x1[LSY_34]*R2D;
		LEBRefAngleCurrent = _Q_34x1[LEB_34]*R2D - OFFSET_LEB;	
		LWYRefAngleCurrent = _Q_34x1[LWY_34]*R2D;
		LWPRefAngleCurrent = _Q_34x1[LWP_34]*R2D;
		LWY2RefAngleCurrent = _Q_34x1[LWY2_34]*R2D;
		/*
		SendRunStopCMD(Joint[LSP].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[LSY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[LWY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[LWY2].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RSP].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RSY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RWY].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, 0, 0, 0);

		SendRunStopCMD(Joint[RWY2].JMC, 0x01);
		RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, 0, 0, 0);
	        */
		pre_gain = 0.f;
		pSharedMemory->torque_mode_flag = 0;
		printf("\n position mode");
	}

	_Q_34x1[RSP_34] =  pre_gain*meas_Q_34x1[RSP_34] + (1.f-pre_gain)*_Q_34x1[RSP_34];
	_Q_34x1[RSR_34] =  pre_gain*meas_Q_34x1[RSR_34] + (1.f-pre_gain)*_Q_34x1[RSR_34];
	_Q_34x1[RSY_34] =  pre_gain*meas_Q_34x1[RSY_34] + (1.f-pre_gain)*_Q_34x1[RSY_34];
	_Q_34x1[REB_34] =  pre_gain*meas_Q_34x1[REB_34] + (1.f-pre_gain)*_Q_34x1[REB_34];
	_Q_34x1[RWY_34] =  pre_gain*meas_Q_34x1[RWY_34] + (1.f-pre_gain)*_Q_34x1[RWY_34];
	_Q_34x1[RWP_34] =  pre_gain*meas_Q_34x1[RWP_34] + (1.f-pre_gain)*_Q_34x1[RWP_34];
	_Q_34x1[RWY2_34] =  pre_gain*meas_Q_34x1[RWY2_34] + (1.f-pre_gain)*_Q_34x1[RWY2_34];
	
	_Q_34x1[LSP_34] =  pre_gain*meas_Q_34x1[LSP_34] + (1.f-pre_gain)*_Q_34x1[LSP_34];
	_Q_34x1[LSR_34] =  pre_gain*meas_Q_34x1[LSR_34] + (1.f-pre_gain)*_Q_34x1[LSR_34];
	_Q_34x1[LSY_34] =  pre_gain*meas_Q_34x1[LSY_34] + (1.f-pre_gain)*_Q_34x1[LSY_34];
	_Q_34x1[LEB_34] =  pre_gain*meas_Q_34x1[LEB_34] + (1.f-pre_gain)*_Q_34x1[LEB_34];
	_Q_34x1[LWY_34] =  pre_gain*meas_Q_34x1[LWY_34] + (1.f-pre_gain)*_Q_34x1[LWY_34];
	_Q_34x1[LWP_34] =  pre_gain*meas_Q_34x1[LWP_34] + (1.f-pre_gain)*_Q_34x1[LWP_34];
	_Q_34x1[LWY2_34] =  pre_gain*meas_Q_34x1[LWY2_34] + (1.f-pre_gain)*_Q_34x1[LWY2_34];

	_Qp_33x1[RSP_33] =  pre_gain*meas_Qp_33x1[RSP_33] + (1.f-pre_gain)*_Qp_33x1[RSP_33];
	_Qp_33x1[RSR_33] =  pre_gain*meas_Qp_33x1[RSR_33] + (1.f-pre_gain)*_Qp_33x1[RSR_33];
	_Qp_33x1[RSY_33] =  pre_gain*meas_Qp_33x1[RSY_33] + (1.f-pre_gain)*_Qp_33x1[RSY_33];
	_Qp_33x1[REB_33] =  pre_gain*meas_Qp_33x1[REB_33] + (1.f-pre_gain)*_Qp_33x1[REB_33];
	_Qp_33x1[RWY_33] =  pre_gain*meas_Qp_33x1[RWY_33] + (1.f-pre_gain)*_Qp_33x1[RWY_33];
	_Qp_33x1[RWP_33] =  pre_gain*meas_Qp_33x1[RWP_33] + (1.f-pre_gain)*_Qp_33x1[RWP_33];
	_Qp_33x1[RWY2_33] =  pre_gain*meas_Qp_33x1[RWY2_33] + (1.f-pre_gain)*_Qp_33x1[RWY2_33];
	
	_Qp_33x1[LSP_33] =  pre_gain*meas_Qp_33x1[LSP_33] + (1.f-pre_gain)*_Qp_33x1[LSP_33];
	_Qp_33x1[LSR_33] =  pre_gain*meas_Qp_33x1[LSR_33] + (1.f-pre_gain)*_Qp_33x1[LSR_33];
	_Qp_33x1[LSY_33] =  pre_gain*meas_Qp_33x1[LSY_33] + (1.f-pre_gain)*_Qp_33x1[LSY_33];
	_Qp_33x1[LEB_33] =  pre_gain*meas_Qp_33x1[LEB_33] + (1.f-pre_gain)*_Qp_33x1[LEB_33];
	_Qp_33x1[LWY_33] =  pre_gain*meas_Qp_33x1[LWY_33] + (1.f-pre_gain)*_Qp_33x1[LWY_33];
	_Qp_33x1[LWP_33] =  pre_gain*meas_Qp_33x1[LWP_33] + (1.f-pre_gain)*_Qp_33x1[LWP_33];
	_Qp_33x1[LWY2_33] =  pre_gain*meas_Qp_33x1[LWY2_33] + (1.f-pre_gain)*_Qp_33x1[LWY2_33];

	if(ps_torque_mode_flag == 1 || ps_torque_mode_flag == 2)
	{
		for(i=1;i<=33;i++)
			duty_joint_limit_33x1[i] = 0.f;
		/*getDuty4JointLimit(_Q_34x1, duty_joint_limit_33x1);
		getFricCompen(_Qp_33x1, viscous_33x1);
		getGravityTorque(_Q_34x1, gravity_33x1);
		pSharedMemory->disp_grav_33x1[RSP_33] = gravity_33x1[RSP_33];
		pSharedMemory->disp_grav_33x1[RSR_33] = gravity_33x1[RSR_33];
		pSharedMemory->disp_grav_33x1[RSY_33] = gravity_33x1[RSY_33];
		pSharedMemory->disp_grav_33x1[REB_33] = gravity_33x1[REB_33];
		pSharedMemory->disp_grav_33x1[RWY_33] = gravity_33x1[RWY_33];
		pSharedMemory->disp_grav_33x1[RWP_33] = gravity_33x1[RWP_33];
		pSharedMemory->disp_grav_33x1[RWY2_33] = gravity_33x1[RWY2_33];

		pSharedMemory->disp_grav_33x1[LSP_33] = gravity_33x1[LSP_33];
		pSharedMemory->disp_grav_33x1[LSR_33] = gravity_33x1[LSR_33];
		pSharedMemory->disp_grav_33x1[LSY_33] = gravity_33x1[LSY_33];
		pSharedMemory->disp_grav_33x1[LEB_33] = gravity_33x1[LEB_33];
		pSharedMemory->disp_grav_33x1[LWY_33] = gravity_33x1[LWY_33];
		pSharedMemory->disp_grav_33x1[LWP_33] = gravity_33x1[LWP_33];
		pSharedMemory->disp_grav_33x1[LWY2_33] = gravity_33x1[LWY2_33];*/

		for(i=1;i<=6;i++)
			for(j=1;j<=33;j++)
			{
				_jT1_33x33[i][j] = _jRH_6x33[i][j];			
				_jT1_33x33[i+6][j] = _jLH_6x33[i][j];			
			}

		for(i=1; i<=12; i++)
		{
			_jT1_33x33[i][1] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][2] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][3] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][4] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][5] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][6] = 0.f;  // exclude pelvis from the solution
			_jT1_33x33[i][7] = 0.f;  // exclude WST from the solution
		}

		if(ps_ladder_demo == 0)
		{
			neutral_Q_34x1[RSP_34] = _Q_34x1[RSP_34];
			neutral_Q_34x1[RSR_34] = _Q_34x1[RSR_34];
			neutral_Q_34x1[RSY_34] = _Q_34x1[RSY_34];
			neutral_Q_34x1[REB_34] = _Q_34x1[REB_34];
			neutral_Q_34x1[RWY_34] = _Q_34x1[RWY_34];
			neutral_Q_34x1[RWP_34] = _Q_34x1[RWP_34];
			neutral_Q_34x1[RWY2_34] = _Q_34x1[RWY2_34];
			
			neutral_Q_34x1[LSP_34] = _Q_34x1[LSP_34];
			neutral_Q_34x1[LSR_34] = _Q_34x1[LSR_34];
			neutral_Q_34x1[LSY_34] = _Q_34x1[LSY_34];
			neutral_Q_34x1[LEB_34] = _Q_34x1[LEB_34];
			neutral_Q_34x1[LWY_34] = _Q_34x1[LWY_34];
			neutral_Q_34x1[LWP_34] = _Q_34x1[LWP_34];
			neutral_Q_34x1[LWY2_34] = _Q_34x1[LWY2_34];

			pRH2_3x1[1] = _pRH_3x1[1];
			pRH2_3x1[2] = _pRH_3x1[2];
			pRH2_3x1[3] = _pRH_3x1[3];
			qRH2_4x1[1] = _qRH_4x1[1];
			qRH2_4x1[2] = _qRH_4x1[2];
			qRH2_4x1[3] = _qRH_4x1[3];
			qRH2_4x1[4] = _qRH_4x1[4];
							
			pLH2_3x1[1] = _pLH_3x1[1];
			pLH2_3x1[2] = _pLH_3x1[2];
			pLH2_3x1[3] = _pLH_3x1[3];
			qLH2_4x1[1] = _qLH_4x1[1];
			qLH2_4x1[2] = _qLH_4x1[2];
			qLH2_4x1[3] = _qLH_4x1[3];
			qLH2_4x1[4] = _qLH_4x1[4];

			_TEMP3_34x34[1][1] = Xrh_6x1[1] = 0.f;
			_TEMP3_34x34[1][2] = Xrh_6x1[2] = 0.f;
			_TEMP3_34x34[1][3] = Xrh_6x1[3] = 0.f;
			_TEMP3_34x34[1][4] = Xrh_6x1[4] = 0.f;
			_TEMP3_34x34[1][5] = Xrh_6x1[5] = 0.f;
			_TEMP3_34x34[1][6] = Xrh_6x1[6] = 0.f;
			_TEMP3_34x34[1][7] = Xlh_6x1[1] = 0.f;
			_TEMP3_34x34[1][8] = Xlh_6x1[2] = 0.f;
			_TEMP3_34x34[1][9] = Xlh_6x1[3] = 0.f;
			_TEMP3_34x34[1][10] = Xlh_6x1[4] = 0.f;
			_TEMP3_34x34[1][11] = Xlh_6x1[5] = 0.f;
			_TEMP3_34x34[1][12] = Xlh_6x1[6] = 0.f;
		}
		else if(ps_ladder_demo == 1)
		{
			des_pRH_3x1[1] = pRH2_3x1[1];
			des_pRH_3x1[2] = pRH2_3x1[2];
			des_pRH_3x1[3] = pRH2_3x1[3];
			des_qRH_4x1[1] = qRH2_4x1[1];
			des_qRH_4x1[2] = qRH2_4x1[2];
			des_qRH_4x1[3] = qRH2_4x1[3];
			des_qRH_4x1[4] = qRH2_4x1[4];
			des_vRH_3x1[1] = 0.f;
			des_vRH_3x1[2] = 0.f;
			des_vRH_3x1[3] = 0.f;
			des_wRH_3x1[1] = 0.f;
			des_wRH_3x1[2] = 0.f;
			des_wRH_3x1[3] = 0.f;
				
			des_pLH_3x1[1] = pLH2_3x1[1];
			des_pLH_3x1[2] = pLH2_3x1[2];
			des_pLH_3x1[3] = pLH2_3x1[3];
			des_qLH_4x1[1] = qLH2_4x1[1];
			des_qLH_4x1[2] = qLH2_4x1[2];
			des_qLH_4x1[3] = qLH2_4x1[3];
			des_qLH_4x1[4] = qLH2_4x1[4];
			des_vLH_3x1[1] = 0.f;
			des_vLH_3x1[2] = 0.f;
			des_vLH_3x1[3] = 0.f;
			des_wLH_3x1[1] = 0.f;
			des_wLH_3x1[2] = 0.f;
			des_wLH_3x1[3] = 0.f;

			diff_vv(des_pRH_3x1,3, _pRH_3x1, ftemp3_7x1);
			mult_mv((const float**)_jRH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
			diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
			Xrh_6x1[1] = ps_kp[0]*ftemp3_7x1[1] + ps_kd[0]*ftemp1_7x1[1];
			Xrh_6x1[2] = ps_kp[1]*ftemp3_7x1[2] + ps_kd[1]*ftemp1_7x1[2];
			Xrh_6x1[3] = ps_kp[2]*ftemp3_7x1[3] + ps_kd[2]*ftemp1_7x1[3];
			QTdel(des_qRH_4x1, _qRH_4x1, ftemp3_7x1);
			diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
			sum_svsv(ps_kp[3],ftemp3_7x1,3, ps_kd[3],ftemp4_7x1, &Xrh_6x1[3]);

			diff_vv(des_pLH_3x1,3, _pLH_3x1, ftemp3_7x1);
			mult_mv((const float**)_jLH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
			diff_vv(des_vLH_3x1,3, ftemp2_7x1, ftemp1_7x1);
			Xlh_6x1[1] = ps_kp[4]*ftemp3_7x1[1] + ps_kd[4]*ftemp1_7x1[1];
			Xlh_6x1[2] = ps_kp[5]*ftemp3_7x1[2] + ps_kd[5]*ftemp1_7x1[2];
			Xlh_6x1[3] = ps_kp[6]*ftemp3_7x1[3] + ps_kd[6]*ftemp1_7x1[3];
			QTdel(des_qLH_4x1, _qLH_4x1, ftemp3_7x1);
			diff_vv(des_wLH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
			sum_svsv(ps_kp[7],ftemp3_7x1,3, ps_kd[7],ftemp4_7x1, &Xlh_6x1[3]);

			_TEMP3_34x34[1][1] = Xrh_6x1[1];
			_TEMP3_34x34[1][2] = Xrh_6x1[2];
			_TEMP3_34x34[1][3] = ps_ref_fRH_3x1[3];
			_TEMP3_34x34[1][4] = Xrh_6x1[4];
			_TEMP3_34x34[1][5] = Xrh_6x1[5];
			_TEMP3_34x34[1][6] = Xrh_6x1[6];
			_TEMP3_34x34[1][7] = Xlh_6x1[1]; //pSharedMemory->ref_fLH_3x1[1];
			_TEMP3_34x34[1][8] = Xlh_6x1[2]; //pSharedMemory->ref_fLH_3x1[2];
			_TEMP3_34x34[1][9] = ps_ref_fLH_3x1[3]; //pSharedMemory->ref_fLH_3x1[3];
			_TEMP3_34x34[1][10] = Xlh_6x1[4];
			_TEMP3_34x34[1][11] = Xlh_6x1[5];
			_TEMP3_34x34[1][12] = Xlh_6x1[6];
		}
		
		else if(ps_ladder_demo == 2)
		{
			neutral_Q_34x1[RSP_34] = _Q_34x1[RSP_34];
			neutral_Q_34x1[RSR_34] = _Q_34x1[RSR_34];
			neutral_Q_34x1[RSY_34] = _Q_34x1[RSY_34];
			neutral_Q_34x1[REB_34] = _Q_34x1[REB_34];
			neutral_Q_34x1[RWY_34] = _Q_34x1[RWY_34];
			neutral_Q_34x1[RWP_34] = _Q_34x1[RWP_34];
			neutral_Q_34x1[RWY2_34] = _Q_34x1[RWY2_34];
					
			pRH2_3x1[1] = _pRH_3x1[1];
			pRH2_3x1[2] = _pRH_3x1[2];
			pRH2_3x1[3] = _pRH_3x1[3];
			qRH2_4x1[1] = _qRH_4x1[1];
			qRH2_4x1[2] = _qRH_4x1[2];
			qRH2_4x1[3] = _qRH_4x1[3];
			qRH2_4x1[4] = _qRH_4x1[4];

			des_pLH_3x1[1] = pLH2_3x1[1];
			des_pLH_3x1[2] = pLH2_3x1[2];
			des_pLH_3x1[3] = pLH2_3x1[3];
			des_qLH_4x1[1] = qLH2_4x1[1];
			des_qLH_4x1[2] = qLH2_4x1[2];
			des_qLH_4x1[3] = qLH2_4x1[3];
			des_qLH_4x1[4] = qLH2_4x1[4];
			des_vLH_3x1[1] = 0.f;
			des_vLH_3x1[2] = 0.f;
			des_vLH_3x1[3] = 0.f;
			des_wLH_3x1[1] = 0.f;
			des_wLH_3x1[2] = 0.f;
			des_wLH_3x1[3] = 0.f;
		
			diff_vv(des_pLH_3x1,3, _pLH_3x1, ftemp3_7x1);
			mult_mv((const float**)_jLH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
			diff_vv(des_vLH_3x1,3, ftemp2_7x1, ftemp1_7x1);
			Xlh_6x1[1] = ps_kp[4]*ftemp3_7x1[1] + ps_kd[4]*ftemp1_7x1[1];
			Xlh_6x1[2] = ps_kp[5]*ftemp3_7x1[2] + ps_kd[5]*ftemp1_7x1[2];
			Xlh_6x1[3] = ps_kp[6]*ftemp3_7x1[3] + ps_kd[6]*ftemp1_7x1[3];
			QTdel(des_qLH_4x1, _qLH_4x1, ftemp3_7x1);
			diff_vv(des_wLH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
			sum_svsv(ps_kp[7],ftemp3_7x1,3, ps_kd[7],ftemp4_7x1, &Xlh_6x1[3]);

	        _TEMP3_34x34[1][1] = Xrh_6x1[1]= 0.f;
			_TEMP3_34x34[1][2] = Xrh_6x1[2]= 0.f;
			_TEMP3_34x34[1][3] = Xrh_6x1[3]= 0.f;
			_TEMP3_34x34[1][4] = Xrh_6x1[4]= 0.f;
			_TEMP3_34x34[1][5] = Xrh_6x1[5]= 0.f;
			_TEMP3_34x34[1][6] = Xrh_6x1[6]= 0.f;
			_TEMP3_34x34[1][7] = Xlh_6x1[1] ;
			_TEMP3_34x34[1][8] = Xlh_6x1[2] ;
			_TEMP3_34x34[1][9] = Xlh_6x1[3] ;
			_TEMP3_34x34[1][10] = Xlh_6x1[4] ;
			_TEMP3_34x34[1][11] = Xlh_6x1[5] ;
			_TEMP3_34x34[1][12] = Xlh_6x1[6] ;
		}
		/*
		else if(pSharedMemory->ladder_demo == 2)
		{
			neutral_Q_34x1[LSP_34] = _Q_34x1[LSP_34];
			neutral_Q_34x1[LSR_34] = _Q_34x1[LSR_34];
			neutral_Q_34x1[LSY_34] = _Q_34x1[LSY_34];
			neutral_Q_34x1[LEB_34] = _Q_34x1[LEB_34];
			neutral_Q_34x1[LWY_34] = _Q_34x1[LWY_34];
			neutral_Q_34x1[LWP_34] = _Q_34x1[LWP_34];
			neutral_Q_34x1[LWY2_34] = _Q_34x1[LWY2_34];
					
			pLH2_3x1[1] = _pLH_3x1[1];
			pLH2_3x1[2] = _pLH_3x1[2];
			pLH2_3x1[3] = _pLH_3x1[3];
			qLH2_4x1[1] = _qLH_4x1[1];
			qLH2_4x1[2] = _qLH_4x1[2];
			qLH2_4x1[3] = _qLH_4x1[3];
			qLH2_4x1[4] = _qLH_4x1[4];

			des_pRH_3x1[1] = pRH2_3x1[1];
			des_pRH_3x1[2] = pRH2_3x1[2];
			des_pRH_3x1[3] = pRH2_3x1[3];
			des_qRH_4x1[1] = qRH2_4x1[1];
			des_qRH_4x1[2] = qRH2_4x1[2];
			des_qRH_4x1[3] = qRH2_4x1[3];
			des_qRH_4x1[4] = qRH2_4x1[4];
			des_vRH_3x1[1] = 0.f;
			des_vRH_3x1[2] = 0.f;
			des_vRH_3x1[3] = 0.f;
			des_wRH_3x1[1] = 0.f;
			des_wRH_3x1[2] = 0.f;
			des_wRH_3x1[3] = 0.f;
		
			diff_vv(des_pRH_3x1,3, _pRH_3x1, ftemp3_7x1);
			mult_mv((const float**)_jRH_6x33,6,33, _Qp_33x1, ftemp2_7x1);
			diff_vv(des_vRH_3x1,3, ftemp2_7x1, ftemp1_7x1);
			Xrh_6x1[1] = pSharedMemory->kp[0]*ftemp3_7x1[1] + pSharedMemory->kd[0]*ftemp1_7x1[1];
			Xrh_6x1[2] = pSharedMemory->kp[1]*ftemp3_7x1[2] + pSharedMemory->kd[1]*ftemp1_7x1[2];
			Xrh_6x1[3] = pSharedMemory->kp[2]*ftemp3_7x1[3] + pSharedMemory->kd[2]*ftemp1_7x1[3];
			QTdel(des_qRH_4x1, _qRH_4x1, ftemp3_7x1);
			diff_vv(des_wRH_3x1,3, &ftemp2_7x1[3], ftemp4_7x1);
			sum_svsv(pSharedMemory->kp[3],ftemp3_7x1,3, pSharedMemory->kd[3],ftemp4_7x1, &Xrh_6x1[3]);

			_TEMP3_34x34[1][1] = Xrh_6x1[1];
			_TEMP3_34x34[1][2] = Xrh_6x1[2];
			_TEMP3_34x34[1][3] = Xrh_6x1[3];
			_TEMP3_34x34[1][4] = Xrh_6x1[4];
			_TEMP3_34x34[1][5] = Xrh_6x1[5];
			_TEMP3_34x34[1][6] = Xrh_6x1[6];
			_TEMP3_34x34[1][7] = Xlh_6x1[1] = 0.f;
			_TEMP3_34x34[1][8] = Xlh_6x1[2] = 0.f;
			_TEMP3_34x34[1][9] = Xlh_6x1[3] = 0.f;
			_TEMP3_34x34[1][10] = Xlh_6x1[4] = 0.f;
			_TEMP3_34x34[1][11] = Xlh_6x1[5] = 0.f;
			_TEMP3_34x34[1][12] = Xlh_6x1[6] = 0.f;
		}		
		*/
		trans(1.f, (const float**)_jT1_33x33,12,33, _TEMP2_34x34);
		mult_mv((const float**)_TEMP2_34x34,33,12, _TEMP3_34x34[1], ct_33x1);

		if(pinv_SR((const float**)_jT1_33x33, 12, 33, (float)1.e-10, _jT1inv_33x33) != 0)
		{
			printf("\n pinv error!");
			return -2; // singularity occurred
		}
		mult_mm((const float**)_jT1inv_33x33,33,12, (const float**)_jT1_33x33, 33, _TEMP1_34x34);
		diff_mm((const float**)_EYE_33, 33,33, (const float**)_TEMP1_34x34, _N1_33x33);


		for(i=1;i<=33;i++)
			_TEMP3_34x34[1][i] = 0.f;
		_TEMP3_34x34[1][20] = ps_kd[8]*(-_Qp_33x1[RSP_33]) + ps_kp[8]*(neutral_Q_34x1[RSP_34]-_Q_34x1[RSP_34]);
		_TEMP3_34x34[1][21] = ps_kd[8]*(-_Qp_33x1[RSR_33]) + ps_kp[8]*(neutral_Q_34x1[RSR_34]-_Q_34x1[RSR_34]);
		_TEMP3_34x34[1][22] = ps_kd[8]*(-_Qp_33x1[RSY_33]) + ps_kp[8]*(neutral_Q_34x1[RSY_34]-_Q_34x1[RSY_34]);
		_TEMP3_34x34[1][23] = ps_kd[8]*(-_Qp_33x1[REB_33]) + ps_kp[8]*(neutral_Q_34x1[REB_34]-_Q_34x1[REB_34]);
		_TEMP3_34x34[1][24] = ps_kd[8]*(-_Qp_33x1[RWY_33]) + ps_kp[8]*(neutral_Q_34x1[RWY_34]-_Q_34x1[RWY_34]);
		_TEMP3_34x34[1][25] = ps_kd[8]*(-_Qp_33x1[RWP_33]) + ps_kp[8]*(neutral_Q_34x1[RWP_34]-_Q_34x1[RWP_34]);
		_TEMP3_34x34[1][26] = ps_kd[8]*(-_Qp_33x1[LSP_33]) + ps_kp[8]*(neutral_Q_34x1[LSP_34]-_Q_34x1[LSP_34]);
		_TEMP3_34x34[1][27] = ps_kd[8]*(-_Qp_33x1[LSR_33]) + ps_kp[8]*(neutral_Q_34x1[LSR_34]-_Q_34x1[LSR_34]);
		_TEMP3_34x34[1][28] = ps_kd[8]*(-_Qp_33x1[LSY_33]) + ps_kp[8]*(neutral_Q_34x1[LSY_34]-_Q_34x1[LSY_34]);
		_TEMP3_34x34[1][29] = ps_kd[8]*(-_Qp_33x1[LEB_33]) + ps_kp[8]*(neutral_Q_34x1[LEB_34]-_Q_34x1[LEB_34]);
		_TEMP3_34x34[1][30] = ps_kd[8]*(-_Qp_33x1[LWY_33]) + ps_kp[8]*(neutral_Q_34x1[LWY_34]-_Q_34x1[LWY_34]);
		_TEMP3_34x34[1][31] = ps_kd[8]*(-_Qp_33x1[LWP_33]) + ps_kp[8]*(neutral_Q_34x1[LWP_34]-_Q_34x1[LWP_34]);
		_TEMP3_34x34[1][32] = ps_kd[8]*(-_Qp_33x1[RWY2_33]) + ps_kp[8]*(neutral_Q_34x1[RWY2_34]-_Q_34x1[RWY2_34]);
		_TEMP3_34x34[1][33] = ps_kd[8]*(-_Qp_33x1[LWY2_33]) + ps_kp[8]*(neutral_Q_34x1[LWY2_34]-_Q_34x1[LWY2_34]);

		if(ps_ladder_demo==1 || ps_ladder_demo==2)
		{
			if(ps_ladder_demo==1)
			{
				_TEMP3_34x34[1][20] = 0.f;
				_TEMP3_34x34[1][21] = 0.f;
				_TEMP3_34x34[1][22] = 0.f;
				_TEMP3_34x34[1][23] = 0.f;
				//_TEMP3_34x34[1][24] = 0.f;
				_TEMP3_34x34[1][25] = 0.f;
				_TEMP3_34x34[1][26] = 0.f;
				_TEMP3_34x34[1][27] = 0.f;
				_TEMP3_34x34[1][28] = 0.f;
				_TEMP3_34x34[1][29] = 0.f;
				//_TEMP3_34x34[1][30] = 0.f;
				_TEMP3_34x34[1][31] = 0.f;
				_TEMP3_34x34[1][32] = 0.f;
				_TEMP3_34x34[1][33] = 0.f;
			}
			else if(ps_ladder_demo==2)
			{
				_TEMP3_34x34[1][20] = 0.f;
				_TEMP3_34x34[1][21] = 0.f;
				_TEMP3_34x34[1][22] = 0.f;
				_TEMP3_34x34[1][23] = 0.f;
				_TEMP3_34x34[1][24] = 0.f;
				_TEMP3_34x34[1][25] = 0.f;
				_TEMP3_34x34[1][32] = 0.f;
			}
			/*
			else if(ps_ladder_demo==2)
			{
				_TEMP3_34x34[1][26] = 0.f;
				_TEMP3_34x34[1][27] = 0.f;
				_TEMP3_34x34[1][28] = 0.f;
				_TEMP3_34x34[1][29] = 0.f;
				_TEMP3_34x34[1][30] = 0.f;
				_TEMP3_34x34[1][31] = 0.f;
				_TEMP3_34x34[1][33] = 0.f;
			}*/
			mult_mv((const float**)_N1_33x33,33,33, _TEMP3_34x34[1], _TEMP1_34x34[1]);
			sum_vv(ct_33x1,33, (const float*)_TEMP1_34x34[1], ct_33x1);
		}

		ct_33x1[RSP_33] = _gain_task[RSP_33]*ct_33x1[RSP_33]+_gain_gravity[RSP_33]*gravity_33x1[RSP_33];
		ct_33x1[RSR_33] = _gain_task[RSR_33]*ct_33x1[RSR_33]+_gain_gravity[RSR_33]*gravity_33x1[RSR_33];
		ct_33x1[RSY_33] = _gain_task[RSY_33]*ct_33x1[RSY_33]+_gain_gravity[RSY_33]*gravity_33x1[RSY_33];
		ct_33x1[REB_33] = _gain_task[REB_33]*ct_33x1[REB_33]+_gain_gravity[REB_33]*gravity_33x1[REB_33];
		ct_33x1[RWY_33] = _gain_task[RWY_33]*ct_33x1[RWY_33]+_gain_gravity[RWY_33]*gravity_33x1[RWY_33];
		ct_33x1[RWP_33] = _gain_task[RWP_33]*ct_33x1[RWP_33]+_gain_gravity[RWP_33]*gravity_33x1[RWP_33];
		ct_33x1[RWY2_33] = _gain_task[RWY2_33]*ct_33x1[RWY2_33]+_gain_gravity[RWY2_33]*gravity_33x1[RWY2_33];

		ct_33x1[LSP_33] = _gain_task[LSP_33]*ct_33x1[LSP_33]+_gain_gravity[LSP_33]*gravity_33x1[LSP_33];
		ct_33x1[LSR_33] = _gain_task[LSR_33]*ct_33x1[LSR_33]+_gain_gravity[LSR_33]*gravity_33x1[LSR_33];
		ct_33x1[LSY_33] = _gain_task[LSY_33]*ct_33x1[LSY_33]+_gain_gravity[LSY_33]*gravity_33x1[LSY_33];
		ct_33x1[LEB_33] = _gain_task[LEB_33]*ct_33x1[LEB_33]+_gain_gravity[LEB_33]*gravity_33x1[LEB_33];
		ct_33x1[LWY_33] = _gain_task[LWY_33]*ct_33x1[LWY_33]+_gain_gravity[LWY_33]*gravity_33x1[LWY_33];
		ct_33x1[LWP_33] = _gain_task[LWP_33]*ct_33x1[LWP_33]+_gain_gravity[LWP_33]*gravity_33x1[LWP_33];
		ct_33x1[LWY2_33] = _gain_task[LWY2_33]*ct_33x1[LWY2_33]+_gain_gravity[LWY2_33]*gravity_33x1[LWY2_33];
                /*
		pSharedMemory->disp_ct_33x1[RSP_33] = ct_33x1[RSP_33];
		pSharedMemory->disp_ct_33x1[RSR_33] = ct_33x1[RSR_33];
		pSharedMemory->disp_ct_33x1[RSY_33] = ct_33x1[RSY_33];
		pSharedMemory->disp_ct_33x1[REB_33] = ct_33x1[REB_33];
		pSharedMemory->disp_ct_33x1[RWY_33] = ct_33x1[RWY_33];
		pSharedMemory->disp_ct_33x1[RWP_33] = ct_33x1[RWP_33];
		pSharedMemory->disp_ct_33x1[RWY2_33] = ct_33x1[RWY2_33];
		
		pSharedMemory->disp_ct_33x1[LSP_33] = ct_33x1[LSP_34];
		pSharedMemory->disp_ct_33x1[LSR_33] = ct_33x1[LSR_34];
		pSharedMemory->disp_ct_33x1[LSY_33] = ct_33x1[LSY_34];
		pSharedMemory->disp_ct_33x1[LEB_33] = ct_33x1[LEB_34];
		pSharedMemory->disp_ct_33x1[LWY_33] = ct_33x1[LWY_34];
		pSharedMemory->disp_ct_33x1[LWP_33] = ct_33x1[LWP_34];
		pSharedMemory->disp_ct_33x1[LWY2_33] = ct_33x1[LWY2_34];

		
		pSharedMemory->disp_duty_33x1[RSP_33] = duty_33x1[RSP_33] = limitDuty(0.4f, torque2duty(RSP, pre_gain*ct_33x1[RSP_33]) + pre_gain*viscous_33x1[RSP_33] + duty_joint_limit_33x1[RSP_33]);
		pSharedMemory->disp_duty_33x1[RSR_33] = duty_33x1[RSR_33] = limitDuty(0.4f, torque2duty(RSR, pre_gain*ct_33x1[RSR_33]) + pre_gain*viscous_33x1[RSR_33] + duty_joint_limit_33x1[RSR_33]);
		pSharedMemory->disp_duty_33x1[RSY_33] = duty_33x1[RSY_33] = limitDuty(0.4f, torque2duty(RSY, pre_gain*ct_33x1[RSY_33]) + pre_gain*viscous_33x1[RSY_33] + duty_joint_limit_33x1[RSY_33]);
		pSharedMemory->disp_duty_33x1[REB_33] = duty_33x1[REB_33] = limitDuty(0.4f, torque2duty(REB, pre_gain*ct_33x1[REB_33]) + pre_gain*viscous_33x1[REB_33] + duty_joint_limit_33x1[REB_33]);
		pSharedMemory->disp_duty_33x1[RWY_33] = duty_33x1[RWY_33] = limitDuty(0.4f, torque2duty(RWY, pre_gain*ct_33x1[RWY_33]) + pre_gain*viscous_33x1[RWY_33] + duty_joint_limit_33x1[RWY_33]);
		pSharedMemory->disp_duty_33x1[RWP_33] = duty_33x1[RWP_33] = limitDuty(0.4f, torque2duty(RWP, pre_gain*ct_33x1[RWP_33]) + pre_gain*viscous_33x1[RWP_33] + duty_joint_limit_33x1[RWP_33]);
		pSharedMemory->disp_duty_33x1[RWY2_33] = duty_33x1[RWY2_33] = limitDuty(0.4f, torque2duty(RWY2, pre_gain*ct_33x1[RWY2_33]) + pre_gain*viscous_33x1[RWY2_33] + duty_joint_limit_33x1[RWY2_33]);

		pSharedMemory->disp_duty_33x1[LSP_33] = duty_33x1[LSP_33] = limitDuty(0.4f, torque2duty(LSP, pre_gain*ct_33x1[LSP_33]) + pre_gain*viscous_33x1[LSP_33] + duty_joint_limit_33x1[LSP_33]);
		pSharedMemory->disp_duty_33x1[LSR_33] = duty_33x1[LSR_33] = limitDuty(0.4f, torque2duty(LSR, pre_gain*ct_33x1[LSR_33]) + pre_gain*viscous_33x1[LSR_33] + duty_joint_limit_33x1[LSR_33]);
		pSharedMemory->disp_duty_33x1[LSY_33] = duty_33x1[LSY_33] = limitDuty(0.4f, torque2duty(LSY, pre_gain*ct_33x1[LSY_33]) + pre_gain*viscous_33x1[LSY_33] + duty_joint_limit_33x1[LSY_33]);
		pSharedMemory->disp_duty_33x1[LEB_33] = duty_33x1[LEB_33] = limitDuty(0.4f, torque2duty(LEB, pre_gain*ct_33x1[LEB_33]) + pre_gain*viscous_33x1[LEB_33] + duty_joint_limit_33x1[LEB_33]);
		pSharedMemory->disp_duty_33x1[LWY_33] = duty_33x1[LWY_33] = limitDuty(0.4f, torque2duty(LWY, pre_gain*ct_33x1[LWY_33]) + pre_gain*viscous_33x1[LWY_33] + duty_joint_limit_33x1[LWY_33]);
		pSharedMemory->disp_duty_33x1[LWP_33] = duty_33x1[LWP_33] = limitDuty(0.4f, torque2duty(LWP, pre_gain*ct_33x1[LWP_33]) + pre_gain*viscous_33x1[LWP_33] + duty_joint_limit_33x1[LWP_33]);
		pSharedMemory->disp_duty_33x1[LWY2_33] = duty_33x1[LWY2_33] = limitDuty(0.4f, torque2duty(LWY2, pre_gain*ct_33x1[LWY2_33]) + pre_gain*viscous_33x1[LWY2_33] + duty_joint_limit_33x1[LWY2_33]);
		
		RBpwmCommandHR2ch(CAN1, Joint[RSP].JMC, (int)(PWM_SIGN_RSP*1000.f*duty_33x1[RSP_33]), (int)(PWM_SIGN_RSR*1000.f*duty_33x1[RSR_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[RSY].JMC, (int)(PWM_SIGN_RSY*1000.f*duty_33x1[RSY_33]), (int)(PWM_SIGN_REB*1000.f*duty_33x1[REB_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[RWY].JMC, (int)(PWM_SIGN_RWY*1000.f*duty_33x1[RWY_33]), (int)(PWM_SIGN_RWP*1000.f*duty_33x1[RWP_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[RWY2].JMC, (int)(PWM_SIGN_RWY2*1000.f*duty_33x1[RWY2_33]), 0, 1);

		RBpwmCommandHR2ch(CAN1, Joint[LSP].JMC, (int)(PWM_SIGN_LSP*1000.f*duty_33x1[LSP_33]), (int)(PWM_SIGN_LSR*1000.f*duty_33x1[LSR_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[LSY].JMC, (int)(PWM_SIGN_LSY*1000.f*duty_33x1[LSY_33]), (int)(PWM_SIGN_LEB*1000.f*duty_33x1[LEB_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[LWY].JMC, (int)(PWM_SIGN_LWY*1000.f*duty_33x1[LWY_33]), (int)(PWM_SIGN_LWP*1000.f*duty_33x1[LWP_33]), 1);
		RBpwmCommandHR2ch(CAN1, Joint[LWY2].JMC, (int)(PWM_SIGN_LWY2*1000.f*duty_33x1[LWY2_33]), 0, 1);*/
		
	}
	
	return 0;	
}


int main()
{

ps_position_mode_flag = 0;
ps_torque_mode_flag = 0;
ps_ladder_demo = 0;
ps_move_sec=0; 
_PassiveUpdatedFlag = 0;


InitGlobalMotionVariables();

RHYRefAngleCurrent=0;//*DEG2RAD;
RHRRefAngleCurrent=0;//*DEG2RAD;
RHPRefAngleCurrent=-10;//*DEG2RAD;
RKNRefAngleCurrent=20;//*DEG2RAD;
RAPRefAngleCurrent=-10;//*DEG2RAD;
RARRefAngleCurrent=0;//*DEG2RAD;

LHYRefAngleCurrent=0;//*DEG2RAD;
LHRRefAngleCurrent=0;//*DEG2RAD;
LHPRefAngleCurrent=-10;//*DEG2RAD;
LKNRefAngleCurrent=20;//*DEG2RAD;
LAPRefAngleCurrent=-10;//*DEG2RAD;
LARRefAngleCurrent=0;//*DEG2RAD;

WSTRefAngleCurrent=0;//*DEG2RAD;

RSPRefAngleCurrent=20;//*DEG2RAD;
RSRRefAngleCurrent=-45;//*DEG2RAD;
RSYRefAngleCurrent=0;//*DEG2RAD;
REBRefAngleCurrent=-90;//*DEG2RAD;
RWYRefAngleCurrent=0;//*DEG2RAD;
RWPRefAngleCurrent=-20;//*DEG2RAD;
RWY2RefAngleCurrent=-45;//*DEG2RAD;

LSPRefAngleCurrent=20;//*DEG2RAD;
LSRRefAngleCurrent=45;//*DEG2RAD;
LSYRefAngleCurrent=0;//*DEG2RAD;
LEBRefAngleCurrent=-90;//*DEG2RAD;
LWYRefAngleCurrent=0;//*DEG2RAD;
LWPRefAngleCurrent=-20;//*DEG2RAD;
LWY2RefAngleCurrent=45;//*DEG2RAD;

 RHYEncoderValue=RHYRefAngleCurrent;///DEG2RAD;
 RHREncoderValue=RHRRefAngleCurrent;///DEG2RAD;
 RHPEncoderValue=RHPRefAngleCurrent;///DEG2RAD;
 RKNEncoderValue=RKNRefAngleCurrent;///DEG2RAD;
 RAPEncoderValue=RAPRefAngleCurrent;///DEG2RAD;
 RAREncoderValue=RARRefAngleCurrent;///DEG2RAD;
 LHYEncoderValue=LHYRefAngleCurrent;///DEG2RAD;
 LHREncoderValue=LHRRefAngleCurrent;///DEG2RAD;
 LHPEncoderValue=LHPRefAngleCurrent;///DEG2RAD;
 LKNEncoderValue=LKNRefAngleCurrent;///DEG2RAD;
 LAPEncoderValue=LAPRefAngleCurrent;///DEG2RAD;
 LAREncoderValue=LARRefAngleCurrent;///DEG2RAD;

 WSTEncoderValue=WSTRefAngleCurrent;///DEG2RAD;

 RSPEncoderValue=RSPRefAngleCurrent;///DEG2RAD;
 RSREncoderValue=RSRRefAngleCurrent- OFFSET_RSR;///DEG2RAD- OFFSET_RSR;
 RSYEncoderValue=RSYRefAngleCurrent;///DEG2RAD;
 REBEncoderValue=REBRefAngleCurrent- OFFSET_REB;///DEG2RAD- OFFSET_REB;
 RWYEncoderValue=RWYRefAngleCurrent;///DEG2RAD;
 RWPEncoderValue=RWPRefAngleCurrent;///DEG2RAD;
 RWY2EncoderValue=RWY2RefAngleCurrent;///DEG2RAD;

 LSPEncoderValue=LSPRefAngleCurrent;///DEG2RAD;
 LSREncoderValue=LSRRefAngleCurrent- OFFSET_LSR;///DEG2RAD- OFFSET_LSR;
 LSYEncoderValue=LSYRefAngleCurrent;///DEG2RAD;
 LEBEncoderValue=LEBRefAngleCurrent- OFFSET_LEB;///DEG2RAD- OFFSET_LEB;
 LWYEncoderValue=LWYRefAngleCurrent;///DEG2RAD;
 LWPEncoderValue=LWPRefAngleCurrent;///DEG2RAD;
 LWY2EncoderValue=LWY2RefAngleCurrent;///DEG2RAD;

ps_kp[0]=1500,	ps_kd[0]=20;       // RH px
ps_kp[1]=1500,	ps_kd[1]=20	;// RH py
ps_kp[2]=1500,	ps_kd[2]=20	;// RH pz
ps_kp[3]=20,	ps_kd[3]=0.8	;// RH att
ps_kp[4]=1500,	ps_kd[4]=20	;// LH px

ps_kp[5]=1500,	ps_kd[5]=20	;// LH py
ps_kp[6]=1500,	ps_kd[6]=20	;// LH pz
ps_kp[7]=20,	ps_kd[7]=0.8	;// LH att
ps_kp[8]=200,	ps_kd[8]=0.8	;// redundant task, in force control mode
ps_kp[9]=800,	ps_kd[9]=60	;// Xpel, in kinematics
ps_kp[10]=400,	ps_kd[10]=40	;// Xwst, in kinematics

ps_kp[11]=400,	ps_kd[11]=40	;// Xpelz, in kinematics
ps_kp[12]=900,	ps_kd[12]=60	;// Xcom, in kinematics
ps_kp[13]=900,	ps_kd[13]=60	;// Xfoot, in kinematics
ps_kp[14]=900,	ps_kd[14]=60	;// Xhand, in kinematics
ps_kp[15]=100,	ps_kd[15]=20	;// redundant task, in kinematics0

ps_kp[16]=0,	ps_kd[16]=0	;//

ps_drvRF[3]=0;ps_drvRF[2]=0;ps_drvRF[1]=0;ps_drvRF[0]=0;
ps_drvLF[3]=0;ps_drvLF[2]=0;ps_drvLF[1]=0;ps_drvLF[0]=0;
ps_dpRF[2]=0;ps_dpRF[1]=0;ps_dpRF[0]=0;
ps_dpLF[2]=0;ps_dpLF[1]=0;ps_dpLF[0]=0;
ps_drvRH[3]=0;ps_drvRH[2]=0;ps_drvRH[1]=0;ps_drvRH[0]=0;
ps_drvLH[3]=0;ps_drvLH[2]=0;ps_drvLH[1]=0;ps_drvLH[0]=0; 
ps_dpRH[2]=0;ps_dpRH[1]=0;ps_dpRH[0]=0;
ps_dpLH[2]=0;ps_dpLH[1]=0;ps_dpLH[0]=0;
ps_dpCOM[2]=0;ps_dpCOM[1]=0;ps_dpCOM[0]=0; 
ps_dpPELz=0; 
ps_drvPEL[3]=0;ps_drvPEL[2]=0;ps_drvPEL[1]=0;ps_drvPEL[0]=0;


float ref[27];
if(_PassiveUpdatedFlag==0)
UpdateGlobalMotionVariables();

printf("\n Starting ladder climbing or quad test...");	

_OfflineCount = 1;
_FKineUpdatedFlag = 0;
int i_temp;

//offline count =1 and 2 and 3
for(int n=1;n<=3;n++){


if(n==3){
ps_position_mode_flag = 0;
}

if(_PassiveUpdatedFlag==1)
_FKineUpdatedFlag = FKine_Whole();



i_temp = WBIK_DRC_ladder_climbing(_OfflineCount);

printf("\n1 %d  %d %d\n\n",_PassiveUpdatedFlag,i_temp,_OfflineCount);

_OfflineCount++;

 ref[0] = RHYEncoderValue=RHYRefAngleCurrent;///DEG2RAD;
 ref[1] = RHREncoderValue=RHRRefAngleCurrent;///DEG2RAD;
 ref[2] = RHPEncoderValue=RHPRefAngleCurrent;///DEG2RAD;
 ref[3] = RKNEncoderValue=RKNRefAngleCurrent;///DEG2RAD;
 ref[4] = RAPEncoderValue=RAPRefAngleCurrent;///DEG2RAD;
 ref[5] = RAREncoderValue=RARRefAngleCurrent;///DEG2RAD;
 ref[6] = LHYEncoderValue=LHYRefAngleCurrent;///DEG2RAD;
 ref[7] = LHREncoderValue=LHRRefAngleCurrent;///DEG2RAD;
 ref[8] = LHPEncoderValue=LHPRefAngleCurrent;///DEG2RAD;
 ref[9] = LKNEncoderValue=LKNRefAngleCurrent;///DEG2RAD;
 ref[10] = LAPEncoderValue=LAPRefAngleCurrent;///DEG2RAD;
 ref[11] = LAREncoderValue=LARRefAngleCurrent;///DEG2RAD;

 ref[12] = WSTEncoderValue=WSTRefAngleCurrent;///DEG2RAD;
 ref[13] = RSPEncoderValue=RSPRefAngleCurrent;///DEG2RAD;
 ref[14] = RSREncoderValue=RSRRefAngleCurrent- OFFSET_RSR;///DEG2RAD- OFFSET_RSR;
 ref[15] = RSYEncoderValue=RSYRefAngleCurrent;///DEG2RAD;
 ref[16] = REBEncoderValue=REBRefAngleCurrent- OFFSET_REB;///DEG2RAD- OFFSET_REB;
 ref[17] = RWYEncoderValue=RWYRefAngleCurrent;///DEG2RAD;
 ref[18] = RWPEncoderValue=RWPRefAngleCurrent;///DEG2RAD;
 ref[25] = RWY2EncoderValue=RWY2RefAngleCurrent;///DEG2RAD;

 ref[19] = LSPEncoderValue=LSPRefAngleCurrent;///DEG2RAD;
 ref[20] = LSREncoderValue=LSRRefAngleCurrent- OFFSET_LSR;///DEG2RAD- OFFSET_LSR;
 ref[21] = LSYEncoderValue=LSYRefAngleCurrent;///DEG2RAD;
 ref[22] = LEBEncoderValue=LEBRefAngleCurrent- OFFSET_LEB;///DEG2RAD- OFFSET_LEB;
 ref[23] = LWYEncoderValue=LWYRefAngleCurrent;///DEG2RAD;
 ref[24] = LWPEncoderValue=LWPRefAngleCurrent;///DEG2RAD;
 ref[26] = LWY2EncoderValue=LWY2RefAngleCurrent;///DEG2RAD;

int iii;


//fprintf(fp3, "%f %f %f %f %f %f %f %f %f %f %f %f    %f     %f %f %f %f %f %f %f %f %f %f %f %f %f %f",ref[0],ref[1],ref[2],ref[3],ref[4],ref[5],ref[6],ref[7],ref[8],ref[9],ref[10],ref[11],ref[12],ref[13],ref[14],ref[15],ref[16],ref[17],ref[18],ref[25],ref[19],ref[20],ref[21],ref[22],ref[23],ref[24],ref[26]);
//fprintf(fp3,"\n");
}


/*
ps_dpRF[0] = -0.0;
ps_dpRF[1] = -0.0;
ps_dpRF[2] = 0.0;
ps_drvRF[0]=10.0f*D2R;
ps_drvRF[1]=0;
ps_drvRF[2]=0;
ps_drvRF[3]=1;
ps_dpLF[0] = -0.0;
ps_dpLF[1] = -0.0;
ps_dpLF[2] = 0.0;
ps_drvLF[0]=10.0f*D2R;
ps_drvLF[1]=0;
ps_drvLF[2]=0;
ps_drvLF[3]=1;

ps_dpLF[0] = -0.0;
ps_dpLF[1] = -0.0;
ps_dpLF[2] = 0.0;
ps_move_sec = 5.0f;
ps_position_mode_flag = 2;
*/
/*
ps_dpRH[0] = 0.0;
ps_dpRH[1] = 0.0;
ps_dpRH[2] = 0.0;
ps_dpLH[0] = 0;
ps_dpLH[1] = 0;
ps_dpLH[2] = 0.0;
ps_move_sec = 2.0f;
ps_position_mode_flag = 1;
*/
/*
ps_dpPELz = -0.1f;
ps_move_sec = 1.0f;
ps_position_mode_flag = 4;
*/
/*
ps_dpCOM[0] = 0.0f;
ps_dpCOM[1] = -0.07;
ps_move_sec = 3.0f;
ps_position_mode_flag = 3;
*/
/*
ps_dpPELz = -0.1f;
ps_dpCOM[0] = 0.00679f;
ps_dpCOM[1] = -0.08631;
ps_move_sec = 5.0f;
ps_position_mode_flag = 6;
*/  



//offline count = 4


FILE *fp4;
fp4 =fopen( "arm1.txt","w");


ps_dpRH[0] = 0.0;
ps_dpRH[1] = 0.0;
ps_dpRH[2] = 0.0;
ps_dpLH[0] = 0.1;
ps_dpLH[1] = -0.1;
ps_dpLH[2] = 0.3;
ps_move_sec = 2.0f;
ps_position_mode_flag = 1;

int init_p = 0;

while(ps_position_mode_flag != 0){
if(_PassiveUpdatedFlag==1)
_FKineUpdatedFlag = FKine_Whole();

i_temp = WBIK_DRC_ladder_climbing(_OfflineCount);

if(init_p==0){
printf("\n4 %d %d %d\n\n",_PassiveUpdatedFlag,i_temp,_OfflineCount);
init_p = 1;}

_OfflineCount++;


 ref[0] = RHYEncoderValue=RHYRefAngleCurrent;///DEG2RAD;
 ref[1] = RHREncoderValue=RHRRefAngleCurrent;///DEG2RAD;
 ref[2] = RHPEncoderValue=RHPRefAngleCurrent;///DEG2RAD;
 ref[3] = RKNEncoderValue=RKNRefAngleCurrent;///DEG2RAD;
 ref[4] = RAPEncoderValue=RAPRefAngleCurrent;///DEG2RAD;
 ref[5] = RAREncoderValue=RARRefAngleCurrent;///DEG2RAD;
 ref[6] = LHYEncoderValue=LHYRefAngleCurrent;///DEG2RAD;
 ref[7] = LHREncoderValue=LHRRefAngleCurrent;///DEG2RAD;
 ref[8] = LHPEncoderValue=LHPRefAngleCurrent;///DEG2RAD;
 ref[9] = LKNEncoderValue=LKNRefAngleCurrent;///DEG2RAD;
 ref[10] = LAPEncoderValue=LAPRefAngleCurrent;///DEG2RAD;
 ref[11] = LAREncoderValue=LARRefAngleCurrent;///DEG2RAD;

 ref[12] = WSTEncoderValue=WSTRefAngleCurrent;///DEG2RAD;
 ref[13] = RSPEncoderValue=RSPRefAngleCurrent;///DEG2RAD;
 ref[14] = RSREncoderValue=RSRRefAngleCurrent- OFFSET_RSR;///DEG2RAD- OFFSET_RSR;
 ref[15] = RSYEncoderValue=RSYRefAngleCurrent;///DEG2RAD;
 ref[16] = REBEncoderValue=REBRefAngleCurrent- OFFSET_REB;///DEG2RAD- OFFSET_REB;
 ref[17] = RWYEncoderValue=RWYRefAngleCurrent;///DEG2RAD;
 ref[18] = RWPEncoderValue=RWPRefAngleCurrent;///DEG2RAD;
 ref[25] = RWY2EncoderValue=RWY2RefAngleCurrent;///DEG2RAD;

 ref[19] = LSPEncoderValue=LSPRefAngleCurrent;///DEG2RAD;
 ref[20] = LSREncoderValue=LSRRefAngleCurrent- OFFSET_LSR;///DEG2RAD- OFFSET_LSR;
 ref[21] = LSYEncoderValue=LSYRefAngleCurrent;///DEG2RAD;
 ref[22] = LEBEncoderValue=LEBRefAngleCurrent- OFFSET_LEB;///DEG2RAD- OFFSET_LEB;
 ref[23] = LWYEncoderValue=LWYRefAngleCurrent;///DEG2RAD;
 ref[24] = LWPEncoderValue=LWPRefAngleCurrent;///DEG2RAD;
 ref[26] = LWY2EncoderValue=LWY2RefAngleCurrent;///DEG2RAD;


fprintf(fp4, "%f %f %f %f %f %f %f %f %f %f %f %f    %f     %f %f %f %f %f %f %f %f %f %f %f %f %f %f",ref[0],ref[1],ref[2],ref[3],ref[4],ref[5],ref[6],ref[7],ref[8],ref[9],ref[10],ref[11],ref[12],ref[13],ref[14],ref[15],ref[16],ref[17],ref[18],ref[25],ref[19],ref[20],ref[21],ref[22],ref[23],ref[24],ref[26]);
fprintf(fp4,"\n");

}


//offline count = 5

FILE *fp5;
fp5 =fopen( "joint1.txt","w");

FILE *fp;
if ( (fp =fopen( "offline_ingress_1.txt","r")) == NULL )
{printf("Data file offline_ingress_1.txt was not found\n");		
return -2;}
				
fscanf(fp,"%d ", &ps_off_traj_length);
  
if(ps_off_traj_length > MAX_OFF_TRAJ)
{fclose(fp);
printf("MAX_OFF_TRAJ error!\n");
}
else
{
        
	for (int k=0; k < ps_off_traj_length; k++)
	//fscanf(fp,"%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e", 
        {fscanf(fp,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ", 
						    &ps_off_traj_dpPELz[k], &ps_off_traj_dpCOM[k][0], &ps_off_traj_dpCOM[k][1], 
							&ps_off_traj_dpRH[k][0], &ps_off_traj_dpRH[k][1], &ps_off_traj_dpRH[k][2],
							&ps_off_traj_dpLH[k][0], &ps_off_traj_dpLH[k][1], &ps_off_traj_dpLH[k][2],
							&ps_off_traj_dpRF[k][0], &ps_off_traj_dpRF[k][1], &ps_off_traj_dpRF[k][2],
							&ps_off_traj_dpLF[k][0], &ps_off_traj_dpLF[k][1], &ps_off_traj_dpLF[k][2],
							&ps_off_traj_drvRH[k][0], &ps_off_traj_drvRH[k][1], &ps_off_traj_drvRH[k][2], &ps_off_traj_drvRH[k][3],
							&ps_off_traj_drvLH[k][0], &ps_off_traj_drvLH[k][1], &ps_off_traj_drvLH[k][2], &ps_off_traj_drvLH[k][3],
							&ps_off_traj_drvRF[k][0], &ps_off_traj_drvRF[k][1], &ps_off_traj_drvRF[k][2], &ps_off_traj_drvRF[k][3],
							&ps_off_traj_drvLF[k][0], &ps_off_traj_drvLF[k][1], &ps_off_traj_drvLF[k][2], &ps_off_traj_drvLF[k][3]);
			ps_off_traj_dpRH[k][0]=ps_off_traj_dpCOM[k][0];  ps_off_traj_dpRH[k][1]=ps_off_traj_dpCOM[k][1]; ps_off_traj_dpRH[k][2]=ps_off_traj_dpPELz[k];	}
	ps_off_traj_count = 0;
	fclose(fp);		
	ps_position_mode_flag = 5;
}	

init_p=0;

while(ps_position_mode_flag != 0){
if(_PassiveUpdatedFlag==1)
_FKineUpdatedFlag = FKine_Whole();

i_temp = WBIK_DRC_ladder_climbing(_OfflineCount);

if(init_p==0){
printf("\n5 %d %d %d\n\n",_PassiveUpdatedFlag,i_temp,_OfflineCount);
init_p = 1;}

_OfflineCount++;


 ref[0] = RHYEncoderValue=RHYRefAngleCurrent;///DEG2RAD;
 ref[1] = RHREncoderValue=RHRRefAngleCurrent;///DEG2RAD;
 ref[2] = RHPEncoderValue=RHPRefAngleCurrent;///DEG2RAD;
 ref[3] = RKNEncoderValue=RKNRefAngleCurrent;///DEG2RAD;
 ref[4] = RAPEncoderValue=RAPRefAngleCurrent;///DEG2RAD;
 ref[5] = RAREncoderValue=RARRefAngleCurrent;///DEG2RAD;
 ref[6] = LHYEncoderValue=LHYRefAngleCurrent;///DEG2RAD;
 ref[7] = LHREncoderValue=LHRRefAngleCurrent;///DEG2RAD;
 ref[8] = LHPEncoderValue=LHPRefAngleCurrent;///DEG2RAD;
 ref[9] = LKNEncoderValue=LKNRefAngleCurrent;///DEG2RAD;
 ref[10] = LAPEncoderValue=LAPRefAngleCurrent;///DEG2RAD;
 ref[11] = LAREncoderValue=LARRefAngleCurrent;///DEG2RAD;

 ref[12] = WSTEncoderValue=WSTRefAngleCurrent;///DEG2RAD;
 ref[13] = RSPEncoderValue=RSPRefAngleCurrent;///DEG2RAD;
 ref[14] = RSREncoderValue=RSRRefAngleCurrent- OFFSET_RSR;///DEG2RAD- OFFSET_RSR;
 ref[15] = RSYEncoderValue=RSYRefAngleCurrent;///DEG2RAD;
 ref[16] = REBEncoderValue=REBRefAngleCurrent- OFFSET_REB;///DEG2RAD- OFFSET_REB;
 ref[17] = RWYEncoderValue=RWYRefAngleCurrent;///DEG2RAD;
 ref[18] = RWPEncoderValue=RWPRefAngleCurrent;///DEG2RAD;
 ref[25] = RWY2EncoderValue=RWY2RefAngleCurrent;///DEG2RAD;

 ref[19] = LSPEncoderValue=LSPRefAngleCurrent;///DEG2RAD;
 ref[20] = LSREncoderValue=LSRRefAngleCurrent- OFFSET_LSR;///DEG2RAD- OFFSET_LSR;
 ref[21] = LSYEncoderValue=LSYRefAngleCurrent;///DEG2RAD;
 ref[22] = LEBEncoderValue=LEBRefAngleCurrent- OFFSET_LEB;///DEG2RAD- OFFSET_LEB;
 ref[23] = LWYEncoderValue=LWYRefAngleCurrent;///DEG2RAD;
 ref[24] = LWPEncoderValue=LWPRefAngleCurrent;///DEG2RAD;
 ref[26] = LWY2EncoderValue=LWY2RefAngleCurrent;///DEG2RAD;


fprintf(fp5, "%f %f %f %f %f %f %f %f %f %f %f %f    %f     %f %f %f %f %f %f %f %f %f %f %f %f %f %f",ref[0],ref[1],ref[2],ref[3],ref[4],ref[5],ref[6],ref[7],ref[8],ref[9],ref[10],ref[11],ref[12],ref[13],ref[14],ref[15],ref[16],ref[17],ref[18],ref[25],ref[19],ref[20],ref[21],ref[22],ref[23],ref[24],ref[26]);
fprintf(fp5,"\n");

}



fclose(fp4);
fclose(fp5);

return 0;


}


