#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

/*
 * 1 for inverse dynamics
 * 2 for feed-forward only (all joints)
 */
int CONTROL_MODE = 2;

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
//float offset_Enc2_rad = -0.37;
//float offset_Enc3_rad = 0.27;

float offset_Enc2_rad = -0.427257;
float offset_Enc3_rad = 0.230558;


// Your global variables.

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

#pragma DATA_SECTION(theta3array, ".my_arrs")
float theta3array[100];


long arrayindex = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float x_pos = 0;
float y_pos = 0;
float z_pos = 0;

float fric_on = 0.0;

// inverse kinematics
float theta_1 = 0;
float theta_2 = 0;
float theta_3 = 0;

float motor_theta_1 = 0;
float motor_theta_2 = 0;
float motor_theta_3 = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

//Controller Parameters
//Proportional
float kp[3] = {110, 130, 55};

//Derivative
float kd[3] = {2,2,1.7};

//Intergral
float ki[3] = {550,600,185};

// velocity filtering
float Theta_old[3] = {0,0,0};
float Omega_old1[3] = {0,0,0};
float Omega_old2[3] = {0,0,0};
float Omega[3] = {0,0,0};

//Integral Estimation
float Ik[3] = {0,0,0};
float Ik_old[3] = {0,0,0};
float e_old[3] = {0,0,0};

float integral_threshold = 0.1;


// current positions
float theta_motor[3] = {0,0,0};

//current tau
float t[3] = {0,0,0};

//feedfoward control
float theta_d[3] = {0,0,0};
float theta_dot_d[3] = {0,0,0};
float theta_ddot_d[3] = {0,0,0};

float J[3] = {0.0167, 0.03, 0.0128};

// friction compensation
float minimum_velocity[3] = {0.05, 0.05, 0.05};
float u_fric[3] = {0,0,0};
float Viscous_positive[3] = {0.130,0.2500,0.21};
float Viscous_negative[3] = {0.074,0.21,0.33};
float Coulomb_positive[3] = {0.300,0.25,0.4};
float Coulomb_negative[3] = {-0.390,-0.6,-0.6};
float slope_between_minimums[3] = {3.6,3.6,3.6};

//Controller Parameters
//Proportional
float kp_inv[3] = {0, 10000, 10000};

//Derivative
float kd_inv[3] = {0,200,200};

//alpha values
float a_theta[3] = {0, 0, 0};

float mystep = 0.25;





//This function calculates the motor positions given a desired x,y,and z position
void inverse_kinematics(float x, float y, float z){

    //The DH angles are calculated using a geometric analysis of the possible configurations of the robot
    theta_1 = atan2(y,x);                                                       //The base DH angle is the inverse tangent between the x and y coordinates
    theta_3 = 3.14159 - acos(-(pow(x,2)+pow(y,2)+pow(z-10,2)-200)/200);         //The elbow DH angle is calculated using the law of cosines
    theta_2 = -(atan2(z-10,sqrt(pow(x,2)+pow(y,2))) + theta_3/2);               //The shoulder DH angle is calculated using Pythagoras' Theorem and the half angle formula

    //The DH angles must be converted to motor theta angles using appropriate transformation
    theta_d[0] = theta_1;
    theta_d[1] = theta_2 + PI/2;
    theta_d[2] = theta_3 + theta_2;
}

void filter_velocity() {
    int i;
    for(i = 0; i < 3; i++) {
        Omega[i] = (theta_motor[i] - Theta_old[i])/0.001;
        Omega[i] = (Omega[i] + Omega_old1[i] + Omega_old2[i])/3.0;
        Theta_old[i] = theta_motor[i];

        Omega_old2[i] = Omega_old1[i];
        Omega_old1[i] = Omega[i];
    }
}

void estimate_integral() {
    int i;
    for(i=0;i<3;i++){
        float e_k = theta_d[i] - theta_motor[i];
        Ik[i] = Ik_old[i] + (e_old[i] + e_k)/2*0.001;
        Ik_old[i] = Ik[i];
        e_old[i] = e_k;
    }
}



void pid_control() {
    int i = 0;
    for (i = 0; i < 3; i++){
        t[i] = kp[i]*(theta_d[i]-theta_motor[i]) - kd[i]*Omega[i];
        if (fabs(theta_d[i] - theta_motor[i]) < integral_threshold){
            estimate_integral();
            t[i] = t[i] + ki[i]*Ik[i];
        } else {
            Ik[i] = 0;
            Ik_old[i] = 0;
        }


        if (t[i] >= 5) {
            t[i] = 5;
            Ik[i] = Ik_old[i];
        }
        else if (t[i] <= -5) {
            t[i] = -5;
            Ik[i] = Ik_old[i];
        }
    }
}

//void trajectory(float time){
//    //lemniscate
//    float x_d = 5*sqrt(2)*cos(PI*time)*sin(PI*time)/(sin(PI*time)*sin(PI*time) + 1) + 14;
//    float y_d = 5*sqrt(2)*cos(PI*time)/(sin(PI*time)*sin(PI*time) + 1);
//    float z_d = 3*sin(PI*time)+ 10;
//
//
//    inverse_kinematics(x_d,y_d,z_d);
//}

//void get_trajectory(float time) {
//    float i = 0;
//    for (i = 0; i < 3; i++){
//        if((time >= 0) && (time < 1)) {
//            theta_d[i] = 1.5*pow(time,2) -pow(time,3);
//            theta_dot_d[i] = 3*time - 3*pow(time,2);
//            theta_ddot_d[i] = 3 - 6*time;
//        }
//        else if ((time >= 1) && (time <=2 )) {
//            theta_d[i] = -2 + 6*time - 4.5*pow(time,2) + pow(time,3);
//            theta_dot_d[i] = 6 - 9*time +3*pow(time,2);
//            theta_ddot_d[i] = -9 +6*time;
//        }
//        else {
//            theta_d[i] = 0;
//            theta_dot_d[i] = 0;
//            theta_ddot_d[i] = 0;
//        }
//    }
//
//}

void feedforward_control() {

    int joint_limit = 1;

    if(CONTROL_MODE == 1) {
        joint_limit = 1;
    }
    else if(CONTROL_MODE == 2) {
        joint_limit = 3;
    }

    int i = 0;
    for (i = 0; i < joint_limit; i++){ // ONLY OPERATE ON JOINT 1 WHILE USING INVERSE DYNAMICS CONTROL LOOPS
        t[i] = kp[i]*(theta_d[i]-theta_motor[i]) + kd[i]*(theta_dot_d[i] - Omega[i]) + J[i]*theta_ddot_d[i];
        if (fabs(theta_d[i] - theta_motor[i]) < 0.05){
            estimate_integral();
            t[i] = t[i] + ki[i]*Ik[i];
        } else {
            Ik[i] = 0;
            Ik_old[i] = 0;
        }
        if (t[i] >= 5) {
            t[i] = 5;
            Ik[i] = Ik_old[i];
        }
        else if (t[i] <= -5) {
            t[i] = -5;
            Ik[i] = Ik_old[i];
        }
    }

}

void friction_compensation() {
    int i = 0;
    for(i = 0; i < 3; i++) {
        if (Omega[i] > minimum_velocity[i]) {
            u_fric[i] = Viscous_positive[i]*Omega[i] + Coulomb_positive[i];
        } else if (Omega[i] < -minimum_velocity[i]) {
            u_fric[i] = Viscous_negative[i]*Omega[i] + Coulomb_negative[i];
        } else {
            u_fric[i] = slope_between_minimums[i]*Omega[i];
        }
    }
}


void inverse_dynamics_outer_loop() {
    int joint = 1; // only implement on Joint 2 and 3
    for(joint = 1; joint < 3; joint++) {
        a_theta[joint] = theta_ddot_d[joint]
                         + kp_inv[joint]*(theta_d[joint]-theta_motor[joint])
                         + kd_inv[joint]*(theta_dot_d[joint]-Omega[joint]);
    }
}

void inverse_dynamics_inner_loop() {
    // tau = D(theta)a_theta + C(theta, theta_dot)theta_dot + G(theta)

    float sintheta32 = sin(theta_motor[2] - theta_motor[1]);
    float costheta32 = cos(theta_motor[2] - theta_motor[1]);

    float p1 = 0.0466;
    float p2 = 0.0388;
    float p3 = 0.0284;
    float p4 = 0.1405;
    float p5 = 0.1298;



//    t[1] = a_theta[1]*(p1-p3*sintheta32/*Omega[1]*/)
//            + Omega[1]*p3*costheta32*Omega[1]
//            - p4*9.81*sin(theta_motor[1]);
//
//    t[2] = a_theta[2]*(p2-p3*sintheta32)
//            - Omega[2]*p3*costheta32*Omega[2]
//            - p5*9.81*cos(theta_motor[2]);

        t[1] = p1*a_theta[1] - p3*sintheta32*a_theta[2]
                - Omega[2]*p3*costheta32*Omega[2]
                - p4*9.81*sin(theta_motor[1]);

        t[2] = -p3*sintheta32*a_theta[1] + p2*a_theta[2]
                + Omega[1]*p3*costheta32*Omega[1]
                - p5*9.81*cos(theta_motor[2]);

}



//This function calculates the forward kinematics of the manipulator given the motor positions

void forward_kinematics(float motor1, float motor2, float motor3){

    //The forward kinematics function uses the translational vector from the full DH matrix calculated in Robotica
    x_pos = 10*cos(motor1)*(cos(motor3)+sin(motor2));
    y_pos = 10*sin(motor1)*(cos(motor3)+sin(motor2));
    z_pos = 10*(1+cos(motor2)-sin(motor3));

}


////////////////////

typedef struct steptraj_s {
    long double b[7];
    long double a[7];
    long double xk[7];
    long double yk[7];
    float qd_old;
    float qddot_old;
    int size;
} steptraj_t;

steptraj_t trajectory = {1.0417207161648879e-11L,6.2503242969893271e-11L,1.5625810742473319e-10L,2.0834414323297759e-10L,1.5625810742473319e-10L,6.2503242969893271e-11L,1.0417207161648879e-11L,
                        1.0000000000000000e+00L,-5.8226600985221673e+00L,1.4126404426217572e+01L,-1.8278500308471997e+01L,1.3303686800870629e+01L,-5.1641897532443632e+00L,8.3525893381702754e-01L,
                        0,0,0,0,0,0,0,
                        0,0,0,0,0,0,0,
                        0,
                        0,
                        7};

// this function must be called every 1ms.
void implement_discrete_tf(steptraj_t *traj, float step, float *qd, float *qd_dot, float *qd_ddot) {
    int i = 0;

    traj->xk[0] = step;
    traj->yk[0] = traj->b[0]*traj->xk[0];
    for (i = 1;i<traj->size;i++) {
        traj->yk[0] = traj->yk[0] + traj->b[i]*traj->xk[i] - traj->a[i]*traj->yk[i];
    }

    for (i = (traj->size-1);i>0;i--) {
        traj->xk[i] = traj->xk[i-1];
        traj->yk[i] = traj->yk[i-1];
    }

    *qd = traj->yk[0];
    *qd_dot = (*qd - traj->qd_old)*1000;  //0.001 sample period
    *qd_ddot = (*qd_dot - traj->qddot_old)*1000;

    traj->qd_old = *qd;
    traj->qddot_old = *qd_dot;
}

// to call this function create a variable that steps to the new positions you want to go to, pass this var to step
// pass a reference to your qd variable your qd_dot variable and your qd_double_dot variable
// for example
//  implement_discrete_tf(&trajectory, mystep, &qd, &dot, &ddot);

//////////////







// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

//    theta_motor = {theta1motor, theta2motor, theta3motor};
    theta_motor[0] = theta1motor;
    theta_motor[1] = theta2motor;
    theta_motor[2] = theta3motor;

    filter_velocity();


//    position_d((mycount%2000)/1000.0);
//
//    feedforward_control();

//    float time = (mycount%2000)/1000.0;


    // 1) Calculate the desired trajectory
//    trajectory(time);

    if((mycount % 4000)==0) {
        mystep = 0.75;
    }

    if((mycount % 8000)==0) {
        mystep = 0.25;
    }

    float qd;
    float dot;
    float ddot;

    implement_discrete_tf(&trajectory, mystep, &qd, &dot, &ddot);

    int joint_idx = 0;
    for(joint_idx = 0; joint_idx < 3; joint_idx++) {
        theta_d[joint_idx] = qd;
        theta_dot_d[joint_idx] = dot;
        theta_ddot_d[joint_idx] = ddot;
    }

    // 2) Given measured thetas, calculate actual states, error, error_dot, theta_dot


    // 3) Calculate the outer loop control to come up with values for a_theta_2 and a_theta_3
    inverse_dynamics_outer_loop();

    // 4) Calculate the inner loop control to find control effort to apply to joint 2 and 3
    inverse_dynamics_inner_loop();


    // 5) Calculate Lab 2 feed-forward control for joint 1 to find control effort to apply.
    feedforward_control(); // currently only active on joint 1

    *tau1 = t[0];
    *tau2 = t[1];
    *tau3 = t[2];

    // 6) Calculate friction compensation control effort given the velocities of joint 1,2, and 3
    friction_compensation();

    // 7) Add the friction compensation to the control efforts calculated in 3 and 4 above

    *tau1 = *tau1 + fric_on*u_fric[0];
    *tau2 = *tau2 + fric_on*u_fric[1];
    *tau3 = *tau3 + fric_on*u_fric[2];

    // 8) Write control efforts to PWM outputs to drive each joint



	Simulink_PlotVar1 = qd;
	Simulink_PlotVar2 = theta_motor[0];
	Simulink_PlotVar3 = theta_motor[1];
	Simulink_PlotVar4 = theta_motor[2];


	//Motor torque limitation(Max: 5 Min: -5)

	// save past states
	if ((mycount%50)==0) {

		theta1array[arrayindex] = theta1motor;
		theta2array[arrayindex] = theta2motor;

		if (arrayindex >= 100) {
			arrayindex = 0;
		} else {
			arrayindex++;
		}

	}

	/*
	 * Forward Kinematics
	 *
	 * based on motor angles
	 */

	forward_kinematics(theta1motor, theta2motor, theta3motor);

	/*
	 * Inverse Kinematics
	 *
	 * based on {x,y,z} pos calculated above
	 */

	inverse_kinematics(x_pos, y_pos, z_pos);


	if ((mycount%500)==0) {
		if (whattoprint > 0.5) {
			serial_printf(&SerialA, "I love robotics\n\r");
		} else {
			printtheta1motor = theta1motor;
			printtheta2motor = theta2motor;
			printtheta3motor = theta3motor;


			SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
		}
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
		GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
	}


	mycount++;
}

void printing(void){
//	serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor,printtheta2motor,printtheta3motor);
//	serial_printf(&SerialA, "x: %.2f,   y: %.2f,  z: %.2f   \n\r",x_pos,y_pos,z_pos);
//	//serial_printf(&SerialA, "Estimated IK solution: theta1: %.2f,   theta2: %.2f,  theta3: %.2f   \n\r",theta_1,theta_2,theta_3);
//    serial_printf(&SerialA, "Estimated IK solution: motor_theta1: %.2f,   motor_theta2: %.2f,  motor_theta3: %.2f   \n\r",motor_theta_1,motor_theta_2,motor_theta_3);

//    serial_printf(&SerialA, "theta1motor:  %.2f \n\r",theta1motor);
//    serial_printf(&SerialA, "theta_motor[0]:  %.2f \n\r",theta_motor[0]);
}