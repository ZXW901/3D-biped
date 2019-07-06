#ifndef CONFIG_H_
#define CONFIG_H_

#define PI 3.14159265
#define G	9.81

#define L0	0
#define L1	500		//大腿长度（mm）
#define L2	500		//小腿长度（mm）

#define	KS_LEG	2950	//Leg rotational spring constant (N*m/rad)
#define M_TORSO	22.2	//Torso mass (kg)
#define M_LEG	20.15	//Leg mass (kg)


#define U_LIM	600			//Motor torque limit (N*m)
#define KP_LEG	5000		//Leg motor proportional gain (N*m/rad)
#define KD_LEG	15		//Leg motor differential gain (N*m*s/rad)
#define KP_HIP	2000		//Hip motor proportional gain (N*m/rad)
#define KD_HIP	20			//Hip motor differential gain (N*m*s/rad)
#define S_TORSO	1			//Scale leg actuator gains for torso stabilization
#define S_LEG	0.5			//Scale leg actuator gains for swing phase
#define THRES_LO	10		//Lower spring torque threshold (N*m)
#define THRES_HI	50		//Upper spring torque threshold (N*m)
#define TAU_C	0.12		//Filter time constant
#define T_STEP	0.35		//Step duration (s)
#define L0_LEG	0.78692		//Nominal leg length (m)
#define L_RET	0.2			//Leg retraction (m)
#define L_EXT_GAIN	0.03	//Leg extension gain

#define DX_GAIN	0.18		//X velocity feed forward gain
#define DX_ERR_P_GAIN	0.1	//X velocity error P gain
#define DX_ERR_D_GAIN	0.1	//X velocity error D gain
#define Y0_OFFSET	0.1		//Y hip offset (m)
#define Y0_GAIN	0.03		//Y hip offset gain
#define DY_GAIN	0.19		//Y velocity feed forward gain
#define DY_ERR_P_GAIN	0.1	//Y velocity error P gain
#define DY_ERR_D_GAIN	0.1	//Y velocity error D gain
#endif // !CONFIG_H_

