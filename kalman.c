#include "ch.h"
#include "hal.h"
#include <math.h>

#include "pi_regulator.h"
#include "kalman.h"

<<<<<<< Updated upstream
#define NB_PARAM 3
#define SIGMA_QK_X 1	// mm, the lower, the more certain we are about the prediction on sonar
#define SIGMA_QK_Y 1
#define SIGMA_QK_THETA 1
#define SIGMA_RK_X 1	// mm, the lower, the more certain we are about the prediction on odometry
#define SIGMA_RK_Y 1
#define SIGMA_RK_THETA 1

static float alpha;
normalise_alpha(alpha, side);
=======
#define RAD2DEG (180./M_PI)
#define DEG2RAD (M_PI/180.)
#define FFT_SIZE 1024
#define NB_PARAM 3
#define BATCH_SIZE 20
#define PHYSIC_FACTOR_RL 1.4353 // speed_of_sound/(distance_between_mics*2*pi*frequency)
#define PHYSIC_FACTOR_BF 1.3647 // same
#define INTENSITY_TRESHOLD 300
#define RL_MAX (1/(PHYSIC_FACTOR_RL * PHYSIC_FACTOR_RL)) // inverse of theoretical maximal phase_diff, squared for abs-value
#define BF_MAX (1/(PHYSIC_FACTOR_BF * PHYSIC_FACTOR_BF)) // same
#define NB_MIC 4
#define PP_OFFSET 90
#define PP_SLOPE -3.6
#define PN_OFFSET -45
#define PN_SLOPE 2.2
#define NP_OFFSET 90
#define NP_SLOPE -2.25
#define NN_OFFSET -180
#define NN_SLOPE -2.25
#define NN_OFFSET2 90
#define NN_SLOPE2 4.5
#define LU_TABLE_NB_VALUES 8
#define LU_TABLE_DIST_STEP 55
#define LU_TABLE_DIST_OFFSET 8
#define FREQIND 86 // 650Hz

static BSEMAPHORE_DECL(samples_ready_sem, TRUE);
static float32_t intensity_to_distance_arr[4][8] = {{30000, 7800, 4600, 3000, 1600, 1200, 900, 700}, //order : BLFR
													{30000, 10000, 5000, 3000, 1700, 1300, 1000, 900},
													{30000, 7600, 3740, 2820, 2430, 1900, 1680, 1380},
													{30000, 8900, 5100, 3000, 1600, 1100, 900, 800},
												   };
static float32_t mag_freq_index_bff[NB_MIC];
static float32_t mag_freq_index[NB_MIC];
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
static uint16_t filled = 0;
static uint8_t passes = 0;
static int cntr1 = 0;
static int cntr2 = 0;
static float RmL = 0.;
static float BmF = 0.;
static float alpha_origin;
static float x_origin;
static float y_origin;
static float alpha_final;
float32_t old_pos_arr[NB_PARAM] = { 0 }; //initial position at origin
static bool CALIBRATION = 1;
>>>>>>> Stashed changes

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

<<<<<<< Updated upstream
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
=======
void audio_processing(int16_t *data, uint16_t num_samples) {
	if (num_samples != 640) return;
	int16_t samples = (filled < FFT_SIZE - 160) ? 160 : FFT_SIZE - filled;
	if (samples > 0) {
		for (uint16_t i = 0; i < samples; i++) {
			micRight_cmplx_input[(filled + i) << 1] = data[(i << 2)];
			micLeft_cmplx_input[(filled + i) << 1]  = data[(i << 2) + 1];
			micBack_cmplx_input[(filled + i) << 1]  = data[(i << 2) + 2];
			micFront_cmplx_input[(filled + i) << 1] = data[(i << 2) + 3];
			micRight_cmplx_input[((filled + i) << 1) + 1] = 0;
			micLeft_cmplx_input[((filled + i) << 1) + 1]  = 0;
			micBack_cmplx_input[((filled + i) << 1) + 1]  = 0;
			micFront_cmplx_input[((filled + i) << 1) + 1] = 0;
		} filled += samples;
	}
	if (filled >= FFT_SIZE) {
		filled = 0;
		doFFT_optimized(micRight_cmplx_input);
		doFFT_optimized(micLeft_cmplx_input);
		doFFT_optimized(micBack_cmplx_input);
		doFFT_optimized(micFront_cmplx_input);
		// correction coeffs to
		float tmp1 = phase_diff(micRight_cmplx_input[FREQIND], micRight_cmplx_input[FREQIND+1], micLeft_cmplx_input[FREQIND], micLeft_cmplx_input[FREQIND+1]);
		float tmp2 = phase_diff(micBack_cmplx_input[FREQIND], micBack_cmplx_input[FREQIND+1], micFront_cmplx_input[FREQIND], micFront_cmplx_input[FREQIND+1]);
		if (tmp1 * tmp1 < RL_MAX) {RmL += tmp1; ++cntr1;} // noisy samples are discarded so they don't break inverse trig functions
		if (tmp2 * tmp2 < BF_MAX) {BmF += tmp2; ++cntr2;}
		passes++;
		float source_intensity = sqrt(micBack_cmplx_input[FREQIND] * micBack_cmplx_input[FREQIND] - micBack_cmplx_input[FREQIND+1] * micBack_cmplx_input[FREQIND]);
		if ((passes >= BATCH_SIZE) && (source_intensity > INTENSITY_TRESHOLD) && ((cntr1 > 0) && (cntr2 > 0))) { // double buffering
			mag_freq_index_bff[B_] = sqrt(micBack_cmplx_input[FREQIND] * micBack_cmplx_input[FREQIND] - micBack_cmplx_input[FREQIND+1] * micBack_cmplx_input[FREQIND]);
			mag_freq_index_bff[L_] = sqrt(micLeft_cmplx_input[FREQIND] * micLeft_cmplx_input[FREQIND] - micLeft_cmplx_input[FREQIND+1] * micLeft_cmplx_input[FREQIND+1]);
			mag_freq_index_bff[F_] = sqrt(micFront_cmplx_input[FREQIND] * micFront_cmplx_input[FREQIND] - micFront_cmplx_input[FREQIND+1] * micFront_cmplx_input[FREQIND+1]);
			mag_freq_index_bff[R_] = sqrt(micRight_cmplx_input[FREQIND] * micRight_cmplx_input[FREQIND] - micRight_cmplx_input[FREQIND+1] * micRight_cmplx_input[FREQIND+1]);
			chBSemSignal(&samples_ready_sem);
		}
	}
}
>>>>>>> Stashed changes

static THD_WORKING_AREA(waKalman, 2048);
static THD_FUNCTION(Kalman, arg) {
<<<<<<< Updated upstream
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
=======
	chRegSetThreadName(__FUNCTION__); (void)arg;
	float int_from_activ_mic;
	uint8_t min_alpha_ind = 0;
	float distance = 0; //data available only for 40dB source
	arm_matrix_instance_f32 old_pos; //vector position
	arm_matrix_instance_f32 old_pos_gauss;
	float32_t old_pos_gauss_arr[NB_PARAM*NB_PARAM] = { 0 };
	arm_matrix_instance_f32 sonar_pos; //position given by sonar
	float32_t sonar_pos_arr[NB_PARAM] = { 0 };
	arm_matrix_instance_f32 sonar_pos_gauss;
	float32_t sonar_pos_gauss_arr[NB_PARAM * NB_PARAM];
	arm_matrix_instance_f32 kalman_gain;
	float32_t kalman_gain_arr[NB_PARAM * NB_PARAM] = { 0 };
	arm_matrix_instance_f32 control_mat;
	float32_t control_mat_arr[NB_PARAM] = { 0 };
	arm_matrix_instance_f32 gaussian_qk;
	float32_t gaussian_qk_arr[NB_PARAM * NB_PARAM];
	arm_mat_init_f32(&old_pos, NB_PARAM, 1, &old_pos_arr);
	arm_mat_init_f32(&old_pos_gauss, NB_PARAM, NB_PARAM, old_pos_gauss_arr);
	arm_mat_init_f32(&sonar_pos, NB_PARAM, 1, sonar_pos_arr);
	{
		float32_t rk_diag = {1., 1., 1.};
		arr_set_to_diagonal(sonar_pos_gauss_arr, &rk_diag, NB_PARAM);
	}
	arm_mat_init_f32(&sonar_pos_gauss, NB_PARAM, NB_PARAM, sonar_pos_gauss_arr);
	arm_mat_init_f32(&kalman_gain, NB_PARAM, NB_PARAM, kalman_gain_arr);
	{
		float32_t qk_diag = {1., 1., 1.};
		arr_set_to_diagonal(gaussian_qk_arr, &qk_diag, NB_PARAM);
	}
	arm_mat_init_f32(&control_mat, NB_PARAM, 1, control_mat_arr);
	arm_mat_init_f32(&gaussian_qk, NB_PARAM, NB_PARAM, gaussian_qk_arr);
	GPTD12.tim->CNT = 0;
	while (1) {
		chBSemWait(&samples_ready_sem);
		for (int i = 0 ; i < NB_MIC ; i++) mag_freq_index[i] = mag_freq_index_bff[i]; // double buffering
		{
			float alpha_rl, alpha_bf;
			passes = 0;
			alpha_rl = asin(PHYSIC_FACTOR_RL*RmL/cntr1); // angles between source and 2 pairs of microphones
			alpha_bf = asin(PHYSIC_FACTOR_BF*BmF/cntr2);
			cntr1 = 0; cntr2 = 0; RmL = 0; BmF = 0;
			// slope & offset are chosen by experience with a source going around the device, to optimally fit reality
			if (alpha_rl > 0) {
				if (alpha_bf > -10) alpha_final = PP_OFFSET + alpha_rl * PP_SLOPE;
				else alpha_final = PN_OFFSET - alpha_rl * PN_SLOPE;
			} else {
				if (alpha_bf > 0) alpha_final = NP_OFFSET + alpha_rl * NP_SLOPE;
				else if (alpha_rl < -20) alpha_final = NN_OFFSET - alpha_bf * NN_SLOPE;
				else alpha_final = NN_OFFSET2 - alpha_bf * NN_SLOPE2;
			}
		}
		//distance to source calculation :
		{ //find microphone closest to the source
			float32_t min_alpha_arr[NB_MIC + 1] = {0., 0., 0., 0., 0.}; //4 microphones, but we must check +pi and -pi which both count for the back microphone
			//compare alpha_final, to the angle associated to each microphone (back_m = -pi/+pi, left_m = +pi/2, right_m = -pi/2, front_m = 0
			for (int i = 0; i < NB_MIC + 1 ; i++) min_alpha_arr[i] = alpha_final + (-M_PI) + 2 * M_PI/NB_MIC * i;
			float min_alpha = 2 * M_PI; //maximum value obtainable
			//find microphone that minimizes it's angle to alpha_final
			for (int i = 0; i < NB_MIC + 1 ; i++)
				if (min_alpha_arr[i] * min_alpha_arr[i] < min_alpha * min_alpha) {
					min_alpha = min_alpha_arr[i];
					min_alpha_ind = i;
					int_from_activ_mic = mag_freq_index[i];
				}
		}
		//evaluate distance by framing measured intensity with 2 values from reference table
		for (int i = 0 ; i < LU_TABLE_NB_VALUES - 1 ; i++) {
			if ((int_from_activ_mic < intensity_to_distance_arr[min_alpha_ind][i]) && (int_from_activ_mic > intensity_to_distance_arr[min_alpha_ind][i+1])) {
				//fixed path between each table element is 55mm with a starting offset of 8mm, obtained by experiencing
				distance = i * LU_TABLE_DIST_STEP + LU_TABLE_DIST_OFFSET; break;
			}
		}
		if (CALIBRATION) {
			alpha_origin = alpha_final;
			x_origin = distance * cos(alpha_final);
			y_origin = distance * sin(alpha_final);
			CALIBRATION = 0;
			continue;
		}
		//Start Kalman
		{ //update from sonar data
			float *pos = get_position();
			*(sonar_pos.pData + X_) = x_origin - distance * cos(alpha_final + pos[T_]);
			*(sonar_pos.pData + Y_) = y_origin - distance * sin(alpha_final + pos[T_]);
			*(sonar_pos.pData + T_) = alpha_final;
		}
		{ //update control
			int16_t *speeds = get_speeds();
			int16_t *speedrot = get_speedrot();
			*(control_mat.pData + X_) = speeds[0] * MM_PER_STEP;
			*(control_mat.pData + Y_) = speeds[1] * MM_PER_STEP;
			*(control_mat.pData + T_) = (2 * speedrot[1]) * MM_PER_STEP * INV_ROBOT_WIDTH; //diff vitesse roues (droite - gauche) / rayon bot => ?
		}
		{
			arm_matrix_instance_f32 tmp_mat;
			float32_t tmp_mat_arr[NB_PARAM];
			arm_mat_init_f32(&tmp_mat, NB_PARAM, 1, tmp_mat_arr);
			arm_mat_scale_f32(&control_mat, (GPTD12.tim->CNT)*1e-5, &tmp_mat); //B*u
			GPTD12.tim->CNT = 0;
			arm_mat_add_f32(&tmp_mat, &old_pos, &old_pos);	//x = FxF^T + B*u; F = I
		}
		//on normal distribution from odometry:
		arm_mat_add_f32(&old_pos_gauss, &gaussian_qk, &old_pos_gauss);	//P = FPF^T + Q_k; F = I
		{//Kalman Gain calculation:
			arm_matrix_instance_f32 tmp_mat;
			float32_t tmp_mat_arr[NB_PARAM * NB_PARAM];
			arm_mat_init_f32(&tmp_mat, NB_PARAM, NB_PARAM, tmp_mat_arr);
			arm_mat_add_f32(&old_pos_gauss, &sonar_pos_gauss, &tmp_mat); //(P_k + R_k)
			arm_mat_inverse_f32(&tmp_mat, &tmp_mat); //(P_k + R_k)^-1
			arm_mat_mult_f32(&old_pos_gauss, &tmp_mat, &kalman_gain); //K = P_k * (P_k + R_k)^-1
		}
		//update predictions:
		{ //on position:
			arm_matrix_instance_f32 tmp_mat;
			float32_t tmp_mat_arr[NB_PARAM];
			arm_mat_init_f32(&tmp_mat, NB_PARAM, 1, tmp_mat_arr);

			arm_mat_sub_f32(&sonar_pos, &old_pos, &tmp_mat); //z-x
			arm_mat_mult_f32(&kalman_gain, &tmp_mat, &tmp_mat); //K*(z-x)
			arm_mat_add_f32(&old_pos, &tmp_mat, &old_pos); //x = x + K*(z-x)
		}
		{ //on normal distribution of position:
			arm_matrix_instance_f32 tmp_mat;
			float32_t tmp_mat_arr[NB_PARAM * NB_PARAM];
			arm_mat_init_f32(&tmp_mat, NB_PARAM, NB_PARAM, tmp_mat_arr);
			arm_mat_mult_f32(&kalman_gain, &old_pos_gauss, &tmp_mat); //K*P
			arm_mat_sub_f32(&old_pos_gauss, &tmp_mat, &old_pos_gauss); //P = P - K*P
		}
>>>>>>> Stashed changes
	}
}

void kalman_start(void){chThdCreateStatic(waKalman, sizeof(waKalman), NORMALPRIO, Kalman, NULL);}
