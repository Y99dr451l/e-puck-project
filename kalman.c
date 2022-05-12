#include "ch.h"
#include "hal.h"
#include <math.h>

#include "pi_regulator.h"
#include "kalman.h"

#define NB_PARAM 3
#define SIGMA_QK_X 1	// mm, the lower, the more certain we are about the prediction on sonar
#define SIGMA_QK_Y 1
#define SIGMA_QK_THETA 1
#define SIGMA_RK_X 1	// mm, the lower, the more certain we are about the prediction on odometry
#define SIGMA_RK_Y 1
#define SIGMA_RK_THETA 1

static float alpha;
normalise_alpha(alpha, side);

//send array of matrix dimensions to fill a diagonal
void arr_set_to_diagonal(float32_t * id_arr, float32_t diagonal, int size) {
	for (int i = 0 ; i < size ; i++)
		for (int j = size*i ; j < size*(i+1) ; j++) {
			if (j == i * size + i) id_arr[j] = diagonal[i];
			else id_arr[j] = 0;
		}
}

//update control manually, usual input would be v_x, v_y, omega
void set_control(float32_t speed_arr) {
	for (int i = 0 ; i < NB_PARAM ; i++) control_mat.pData[i] = pos[i];
}

//void kalman_init() {
//	arm_mat_init_f32(old_pos, NB_PARAM, 1, old_pos_arr);
//	arm_mat_init_f32(old_pos_gauss, NB_PARAM, NB_PARAM, old_pos_gauss_arr);
//	arm_mat_init_f32(sonar_pos, NB_PARAM, 1, sonar_pos_arr);
//	float32_t rk_diag = {SIGMA_RK_X, SIGMA_RK_Y, SIGMA_RK_THETA};
//	arr_set_to_diagonal(sonar_pos_gauss_arr, rk_diag, NB_PARAM);
//	arm_mat_init_f32(sonar_pos_gauss, NB_PARAM, NB_PARAM, sonar_pos_gauss_arr);
//	arm_mat_init_f32(kalman_gain, NB_PARAM, NB_PARAM, kalman_gain_arr);
//	float32_t qk_diag = {SIGMA_QK_X, SIGMA_QK_Y, SIGMA_QK_THETA};
//	arr_set_to_diagonal(gaussian_qk_arr, qk_diag, NB_PARAM);
//	arm_mat_init_f32(gaussian_qk, NB_PARAM, NB_PARAM, gaussian_qk_arr);
//}

static THD_WORKING_AREA(waKalman, 2048);
static THD_FUNCTION(Kalman, arg) {
	static arm_matrix_instance_f32 old_pos; //vector position
	static float32_t old_pos_arr[NB_PARAM] = { 0 }; //initial position at origin
	static arm_matrix_instance_f32 old_pos_gauss;
	static float32_t old_pos_gauss_arr[NB_PARAM*NB_PARAM] = { 0 }; //all coefficients to zero=>we know exactly where we are
	static arm_matrix_instance_f32 sonar_pos; //position given by sonar
	static float32_t sonar_pos_arr[NB_PARAM] = { 0 };
	static arm_matrix_instance_f32 sonar_pos_gauss;
	static float32_t sonar_pos_gauss_arr[NB_PARAM * NB_PARAM];
	static arm_matrix_instance_f32 kalman_gain;
	static float32_t kalman_gain_arr[NB_PARAM * NB_PARAM] = { 0 };
	static arm_matrix_instance_f32 control_mat;
	static float32_t control_mat_arr[NB_PARAM] = { 0 };
	static float time_step; // control matrix B
	static arm_matrix_instance_f32 gaussian_qk;
	static float32_t gaussian_qk_arr[NB_PARAM * NB_PARAM];
	arm_mat_init_f32(old_pos, NB_PARAM, 1, old_pos_arr);
	arm_mat_init_f32(old_pos_gauss, NB_PARAM, NB_PARAM, old_pos_gauss_arr);
	arm_mat_init_f32(sonar_pos, NB_PARAM, 1, sonar_pos_arr);
	float32_t rk_diag = {SIGMA_RK_X, SIGMA_RK_Y, SIGMA_RK_THETA};
	arr_set_to_diagonal(sonar_pos_gauss_arr, rk_diag, NB_PARAM);
	arm_mat_init_f32(sonar_pos_gauss, NB_PARAM, NB_PARAM, sonar_pos_gauss_arr);
	arm_mat_init_f32(kalman_gain, NB_PARAM, NB_PARAM, kalman_gain_arr);
	float32_t qk_diag = {SIGMA_QK_X, SIGMA_QK_Y, SIGMA_QK_THETA};
	arr_set_to_diagonal(gaussian_qk_arr, qk_diag, NB_PARAM);
	arm_mat_init_f32(gaussian_qk, NB_PARAM, NB_PARAM, gaussian_qk_arr);
	while(1) {
		time = chVTGetSystemTime();
	//first predictions
		//on position
		chRegSetThreadName(__FUNCTION__); (void)arg;
		arm_matrix_instance_f32 tmp_mat;
		float32_t tmp_mat_arr[NB_PARAM];
		arm_mat_init_f32(tmp_mat, NB_PARAM, 1, tmp_mat_arr);
		arm_mat_scale_f32(control_mat, time_step, tmp_mat); //B*u
		arm_mat_add_f32(tmp_mat, old_pos, old_pos);	//x = FxF^T + B*u; F = I
		//on normal distribution from odometry
		arm_mat_add_f32(old_pos_gauss, gaussian_qk, old_pos_gauss);	//P = FPF^T + Q_k; F = I
	//Kalman Gain calculation
		arm_matrix_instance_f32 tmp_mat;
		float32_t tmp_mat_arr[NB_PARAM * NB_PARAM];
		arm_mat_init_f32(tmp_mat, NB_PARAM, NB_PARAM, tmp_mat_arr);
		arm_mat_add_f32(old_pos_gauss, sonar_pos_gauss, tmp_mat); //(P_k + R_k)
		arm_mat_inverse_f32(tmp_mat, tmp_mat); //(P_k + R_k)^-1
		arm_mat_mult_f32(old_pos_gauss, tmp_mat, kalman_gain); //K = P_k * (P_k + R_k)^-1
	//update predictions
		//on position
		arm_matrix_instance_f32 tmp_mat;
		float32_t tmp_mat_arr[NB_PARAM];
		arm_mat_init_f32(tmp_mat, NB_PARAM, 1, tmp_mat_arr);
		arm_mat_sub_f32(sonar_pos_gauss, old_pos_gauss, tmp_mat); //z-x
		arm_mat_mult_f32(kalman_gain, tmp_mat, tmp_mat); //K*(z-x)
		arm_mat_add_f32(old_pos, tmp_mat, old_pos); //x = x + K*(z-x)
		//on normal distribution of position
		arm_matrix_instance_f32 tmp_mat;
		float32_t tmp_mat_arr[NB_PARAM * NB_PARAM];
		arm_mat_init_f32(tmp_mat, NB_PARAM, NB_PARAM, tmp_mat_arr);
		arm_mat_mult_f32(kalman_gain, old_pos_gauss, tmp_mat); //K*P
		arm_mat_sub_f32(old_pos_gauss, tmp_mat, old_pos_gauss); //P = P - K*P, new position is in old_post_gauss
		chThdSleepUntilWindowed(time, time + MS2ST(100)); // 10Hz
	}
}

void kalman_start(void){chThdCreateStatic(waKalman, sizeof(waKalman), NORMALPRIO, Kalman, NULL);}
