#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"


#define PI 3.14159

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


long arrayindex = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float x_pos = 0;
float y_pos = 0;
float z_pos = 0;

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


//This function calculates the motor positions given a desired x,y,and z position
void inverse_kinematics(float x, float y, float z){

    //The DH angles are calculated using a geometric analysis of the possible configurations of the robot
    theta_1 = atan2(y,x);                                                       //The base DH angle is the inverse tangent between the x and y coordinates
    theta_3 = 3.14159 - acos(-(pow(x,2)+pow(y,2)+pow(z-10,2)-200)/200);         //The elbow DH angle is calculated using the law of cosines
    theta_2 = -(atan2(z-10,sqrt(pow(x,2)+pow(y,2))) + theta_3/2);               //The shoulder DH angle is calculated using Pythagoras' Theorem and the half angle formula

    //The DH angles must be converted to motor theta angles using appropriate transformation
    motor_theta_1 = theta_1;
    motor_theta_2 = theta_2 + PI/2;
    motor_theta_3 = theta_3 + theta_2;
}

//This function calculates the forward kinematics of the manipulator given the motor positions

void forward_kinematics(float motor1, float motor2, float motor3){

    //The forward kinematics function uses the translational vector from the full DH matrix calculated in Robotica
    x_pos = 10*cos(motor1)*(cos(motor3)+sin(motor2));
    y_pos = 10*sin(motor1)*(cos(motor3)+sin(motor2));
    z_pos = 10*(1+cos(motor2)-sin(motor3));

    }

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {


	*tau1 = 0;
	*tau2 = 0;
	*tau3 = 0;

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

	Simulink_PlotVar1 = theta1motor;
	Simulink_PlotVar2 = theta2motor;
	Simulink_PlotVar3 = theta3motor;
	Simulink_PlotVar4 = 0;

	mycount++;
}

void printing(void){
	serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor,printtheta2motor,printtheta3motor);
	serial_printf(&SerialA, "x: %.2f,   y: %.2f,  z: %.2f   \n\r",x_pos,y_pos,z_pos);
	//serial_printf(&SerialA, "Estimated IK solution: theta1: %.2f,   theta2: %.2f,  theta3: %.2f   \n\r",theta_1,theta_2,theta_3);
    serial_printf(&SerialA, "Estimated IK solution: motor_theta1: %.2f,   motor_theta2: %.2f,  motor_theta3: %.2f   \n\r",motor_theta_1,motor_theta_2,motor_theta_3);
}

