#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"


// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
//float offset_Enc2_rad = -0.37;
//float offset_Enc3_rad = 0.27;

float offset_Enc2_rad = -0.427257;
float offset_Enc3_rad = 0.230558;

const float DEG2RAD = 0.0174533;

typedef struct goal goal;
struct goal {
    float position[3];
    float rot;
    float stiffness[3];
    float total_time; // travel time in seconds
};

//Initialize relevant trajectory points
goal start = {.position[0] = 5.61, .position[1] = 0.0, .position[2] = 16.78, .rot = 0, .stiffness[0] = 0.5, .stiffness[1] = 0.5, .stiffness[2] = 0.5, .total_time = 1};
goal home = {.position[0] = 10.0, .position[1] = 0.0, .position[2] = 20.0, .rot = 0, .stiffness[0] = 0.5, .stiffness[1] = 0.5, .stiffness[2] = 0.5, .total_time = 1};
goal above_hole = {.position[0] = 1.48, .position[1] = 13.86, .position[2] = 7.17, .rot = 0, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 1, .total_time = 1};
goal in_hole = {.position[0] = 1.48, .position[1] = 13.86, .position[2] = 4.90, .rot = 0, .stiffness[0] = 0.15, .stiffness[1] = 0.15, .stiffness[2] = 1, .total_time = 1};
goal avoid_box = {.position[0] = 11.46, .position[1] = 4.16, .position[2] = 8.0, .rot = 0, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 1, .total_time = 1};
goal zig_start = {.position[0] = 14.88, .position[1] = 4.16, .position[2] = 7.72, .rot = 1, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 1, .total_time = 1};
goal pre_turn1 = {.position[0] = 16.21, .position[1] = 2.19, .position[2] = 7.72, .rot = 999, .stiffness[0] = 1, .stiffness[1] = 0.15, .stiffness[2] = 1, .total_time = 1};
goal post_turn1 = {.position[0] = 15.41, .position[1] = 1.54, .position[2] = 7.72, .rot = 999, .stiffness[0] = 0.999, .stiffness[1] = 0.999, .stiffness[2] = 1, .total_time = 1};
goal pre_turn2 = {.position[0] = 13.3, .position[1] = 1,93, .position[2] = 7.72, .rot = 999, .stiffness[0] = 1, .stiffness[1] = 0.15, .stiffness[2] = 1, .total_time = 1};
goal post_turn2 = {.position[0] = 12.74, .position[1] = 1.11, .position[2] = 7.72, .rot = 999, .stiffness[0] = 0.999, .stiffness[1] = 0.999, .stiffness[2] = 0.999, .total_time = 1};
goal zag_end = {.position[0] = 15.65, .position[1] = -2.24, .position[2] = 7.72, .rot = 999, .stiffness[0] = 1, .stiffness[1] = 0.15, .stiffness[2] = 1, .total_time = 1};
goal zag_up = {.position[0] = 15.65, .position[1] = -2.24, .position[2] = 15.0, .rot = 0, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 1, .total_time = 1};
goal above_egg = {.position[0] = 14.78, .position[1] = -5.61, .position[2] = 15.0, .rot = 0, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 1, .total_time = 1};
goal down_egg = {.position[0] = 14.78, .position[1] = -5.61, .position[2] = 13.07, .rot = 0, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 0.75, .total_time = 1};



//Create trajectory
int num_goals = 18;
goal trajectory[18] = {
                      {.position[0] = 5.61, .position[1] = 0.0, .position[2] = 16.78, .rot = 0, .stiffness[0] = 0.5, .stiffness[1] = 0.5, .stiffness[2] = 0.5, .total_time = 0.1},
                      {.position[0] = 10.0, .position[1] = 0.0, .position[2] = 20.0, .rot = 0, .stiffness[0] = 0.75, .stiffness[1] = 0.75, .stiffness[2] = 0.75, .total_time = 0.5},
                      {.position[0] = 1.48, .position[1] = 13.86, .position[2] = 7.5, .rot = 0, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 1, .total_time = 0.5},
                      {.position[0] = 1.48, .position[1] = 13.86, .position[2] = 4.90, .rot = 0, .stiffness[0] = 0.15, .stiffness[1] = 0.15, .stiffness[2] = 1, .total_time = 1},
                      {.position[0] = 1.48, .position[1] = 13.86, .position[2] = 7.5, .rot = 0, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 1, .total_time = 1},
                      {.position[0] = 11.46, .position[1] = 4.16, .position[2] = 8.0, .rot = 0, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 1, .total_time = 0.5},
                      {.position[0] = 14.88, .position[1] = 4.16, .position[2] = 7.72, .rot = 0, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 1, .total_time = 0.5},
                      {.position[0] = 16.21, .position[1] = 2.19, .position[2] = 7.72, .rot = -1.0472, .stiffness[0] = 0.85, .stiffness[1] = 0.15, .stiffness[2] = 1, .total_time = 1},
                      {.position[0] = 15.41, .position[1] = 1.54, .position[2] = 7.72, .rot = 0.610865, .stiffness[0] = 0.85, .stiffness[1] = 0.15, .stiffness[2] = 1, .total_time = 1},
                      {.position[0] = 13.3, .position[1] = 1,93, .position[2] = 7.72, .rot = -0.261799, .stiffness[0] = 1, .stiffness[1] = 0.15, .stiffness[2] = 1, .total_time = 1},
                      {.position[0] = 12.74, .position[1] = 1.11, .position[2] = 7.72, .rot = 0.610865, .stiffness[0] = 0.85, .stiffness[1] = 0.15, .stiffness[2] = 1, .total_time = 1},
                      {.position[0] = 15.65, .position[1] = -2.24, .position[2] = 7.72, .rot = -1.0472, .stiffness[0] = 1, .stiffness[1] = 0.15, .stiffness[2] = 1, .total_time = 1},
                      {.position[0] = 15.65, .position[1] = -2.24, .position[2] = 15.0, .rot = 0, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 1, .total_time = 0.5},
                      {.position[0] = 14.78, .position[1] = -5.61, .position[2] = 15.0, .rot = 0, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 1, .total_time = 0.5},
                      {.position[0] = 14.78, .position[1] = -5.61, .position[2] = 13.07, .rot = 0, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 0.85, .total_time = 0.5},
                      {.position[0] = 14.78, .position[1] = -5.61, .position[2] = 15.0, .rot = 0, .stiffness[0] = 1, .stiffness[1] = 1, .stiffness[2] = 1, .total_time = 0.5},
                      {.position[0] = 10.0, .position[1] = 0.0, .position[2] = 20.0, .rot = 0, .stiffness[0] = 0.5, .stiffness[1] = 0.5, .stiffness[2] = 0.5, .total_time = 0.5},
                      {.position[0] = 10.0, .position[1] = 0.0, .position[2] = 20.0, .rot = 0, .stiffness[0] = 0.5, .stiffness[1] = 0.5, .stiffness[2] = 0.5, .total_time = 10}
                     };


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

float fric_on = 1.0;

long arrayindex = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

// current positions
float theta_motor[3] = {0,0,0};

// velocity filtering
float Theta_old[3] = {0,0,0};
float Omega_old1[3] = {0,0,0};
float Omega_old2[3] = {0,0,0};
float Omega[3] = {0,0,0};

//current tau
float t[3] = {0,0,0};

// friction compensation
float minimum_velocity[3] = {0.05, 0.05, 0.05};
float u_fric[3] = {0,0,0};
float Viscous_positive[3] = {0.130,0.2500,0.21};
float Viscous_negative[3] = {0.074,0.21,0.33};
float Coulomb_positive[3] = {0.300,0.25,0.4};
float Coulomb_negative[3] = {-0.390,-0.6,-0.6};
float slope_between_minimums[3] = {3.6,3.6,3.6};

//Task Space Globals
float KP_task_const[3] = {3.5, 1.4, 2.5};
float KD_task_const[3] = {0.035, 0.025, 0.04};

float KP_task[3] = {.35, .14, .25};
float KD_task[3] = {0.0035, 0.0025, 0.004};

float Velocity[3] = {0,0,0};
float Velocity_old1[3] = {0,0,0};
float Velocity_old2[3] = {0,0,0};

float Velocity_d[3] = {0,0,0};

float Position[3] = {0,0,0};
float Position_old[3] = {0,0,0};

float Position_d[3] = {10,10,10};

//Axis of Weakness
float Rot[3] = {0,0,0};

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

//This function calculates the forward kinematics of the manipulator given the motor positions

void forward_kinematics(float motor1, float motor2, float motor3){

    //The forward kinematics function uses the translational vector from the full DH matrix calculated in Robotica
    Position[0] = 10*cos(motor1)*(cos(motor3)+sin(motor2));
    Position[1] = 10*sin(motor1)*(cos(motor3)+sin(motor2));
    Position[2] = 10*(1+cos(motor2)-sin(motor3));

}


//////////////
float position_start[3] = {5.61, 0.0, 16.78};
float position_home[3] = {10.0, 0.0, 20.0};
float delta[3] = {0, 0, 0};
float t_total = 2.0;
float t_start = 0;
int goal_curr = 0;
int goal_prev = 0;

void task_space_trajectory(float t) {
    float time = (t-t_start)/t_total;
    int i = 0;
    for (i = 0; i < 3; i++) {
        Position_d[i] = delta[i]*time + position_start[i];
    }

    if(t-t_start == t_total) {
        t_start = t;
        goal_curr++;


        int i = 0;
        for(i = 0; i < 3; i++) {
            position_start[i] = Position_d[i];
        }
    }
}

void set_goal(float t) {
    if(goal_curr > num_goals) {
        // set to home position
        int i = 0;
        for(i = 0; i < 3; i++) {
            delta[i] = 0;
            t_total = 1;
            KP_task[i] = KP_task_const[i];
            KD_task[i] = KD_task_const[i];
        }
        Rot[2] = 0;
    }

    if(goal_curr > goal_prev) {
        goal_prev = goal_curr;
        t_total = trajectory[goal_curr].total_time;

        // step goal
        int i = 0;
        for(i = 0; i < 3; i++) {
            delta[i] = trajectory[goal_curr].position[i] - position_start[i];
            KP_task[i] = trajectory[goal_curr].stiffness[i] * KP_task_const[i];
            KD_task[i] = trajectory[goal_curr].stiffness[i] * KD_task_const[i];
        }

        Rot[2] = trajectory[goal_curr].rot; // z-axis rotation
    }
    else if(goal_curr == 0) {
        int i = 0;
        for(i = 0; i < 3; i++) {
            delta[i] = trajectory[goal_curr].position[i] - position_start[i];
            KP_task[i] = trajectory[goal_curr].stiffness[i] * KP_task_const[i];
            KD_task[i] = trajectory[goal_curr].stiffness[i] * KD_task_const[i];
        }
    }

    task_space_trajectory(t);
}


////////////////////
void filter_velocity_task() {
    int i;
    for(i = 0; i < 3; i++) {
        Velocity[i] = (Position[i] - Position_old[i])/0.001;
        Velocity[i] = (Velocity[i] + Velocity_old1[i] + Velocity_old2[i])/3.0;
        Position_old[i] = Position[i];

        Velocity_old2[i] = Velocity_old1[i];
        Velocity_old1[i] = Velocity[i];
    }
}

void task_space_control() {

    float sin_M1 = sin(theta_motor[0]);
    float cos_M1 = cos(theta_motor[0]);
    float sin_M2 = sin(theta_motor[1]);
    float cos_M2 = cos(theta_motor[1]);
    float sin_M3 = sin(theta_motor[2]);
    float cos_M3 = cos(theta_motor[2]);


    float J_t[3][3] = {{-10*sin_M1*(cos_M3+sin_M2),  10*cos_M1*(cos_M3+sin_M2),  0},
                       { 10*cos_M1*(cos_M2-sin_M3),  10*sin_M1*(cos_M2-sin_M3),  -10*(cos_M3+sin_M2)},
                       {-10*cos_M1*sin_M3,           -10*sin_M1*sin_M3,          -10*cos_M3}};

    float sin_x = sin(Rot[0]);
    float cos_x = cos(Rot[0]);
    float sin_y = sin(Rot[1]);
    float cos_y = cos(Rot[1]);
    float sin_z = sin(Rot[2]);
    float cos_z = cos(Rot[2]);



    float R_WN[3][3] = { {cos_z*cos_y - sin_z*sin_x*sin_y, -sin_z*cos_x, cos_z*sin_y+sin_z*sin_x*cos_y},
                        {sin_z*cos_y + cos_z*sin_x*sin_y, cos_z*cos_x, sin_z*sin_y-cos_z*sin_x*cos_y},
                        {-cos_x*sin_y, sin_x, cos_x*cos_y}};

    float R_NW[3][3] = {{R_WN[0][0], R_WN[1][0], R_WN[2][0]},
                        {R_WN[0][1], R_WN[1][1], R_WN[2][1]},
                        {R_WN[0][2], R_WN[1][2], R_WN[2][2]}};


    float f_p[3] = {0,0,0};
    float f_d[3] = {0,0,0};
    float F_N[3] = {0,0,0};
    float F_W[3] = {0,0,0};

    int i = 0;
    for(i = 0; i < 3; i++) {
        f_p[i] = -KP_task[i]*R_NW[i][0]*(Position[0]-Position_d[0]) + -KP_task[i]*R_NW[i][1]*(Position[1]-Position_d[1]) + -KP_task[i]*R_NW[i][2]*(Position[2]-Position_d[2]);
        f_d[i] = -KD_task[i]*R_NW[i][0]*(Velocity[0]-Velocity_d[0]) + -KD_task[i]*R_NW[i][1]*(Velocity[1]-Velocity_d[1]) + -KD_task[i]*R_NW[i][2]*(Velocity[2]-Velocity_d[2]);
        F_N[i] = f_p[i] + f_d[i];
    }

    for(i = 0; i < 3; i++) {
        F_W[i] = R_WN[i][0]*F_N[0] + R_WN[i][1]*F_N[1] + R_WN[i][2]*F_N[2];
    }

    /*
     * Calculate control efforts (tau)
     */
    for(i = 0; i < 3; i++) {
        t[i] = J_t[i][0]*F_W[0] + J_t[i][1]*F_W[1] + J_t[i][2]*F_W[2];
    }


}

////////////////////
// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

    theta_motor[0] = theta1motor;
    theta_motor[1] = theta2motor;
    theta_motor[2] = theta3motor;

//    filter_velocity();

    float time = (mycount)/1000.0;

    set_goal(time);

    forward_kinematics(theta_motor[0], theta_motor[1], theta_motor[2]);

    filter_velocity_task();

    task_space_control();

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



    Simulink_PlotVar1 = Position_d[0];
    Simulink_PlotVar2 = Position[0];
    Simulink_PlotVar3 = Position[1];
    Simulink_PlotVar4 = Position[2];


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
//  serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor,printtheta2motor,printtheta3motor);
    serial_printf(&SerialA, "x: %.2f,   y: %.2f,  z: %.2f   \n\r",Position[0], Position[1], Position[2]);
    serial_printf(&SerialA, "Waypoint: %d\n\r", goal_curr);
//  //serial_printf(&SerialA, "Estimated IK solution: theta1: %.2f,   theta2: %.2f,  theta3: %.2f   \n\r",theta_1,theta_2,theta_3);
//    serial_printf(&SerialA, "Estimated IK solution: motor_theta1: %.2f,   motor_theta2: %.2f,  motor_theta3: %.2f   \n\r",motor_theta_1,motor_theta_2,motor_theta_3);

//    serial_printf(&SerialA, "theta1motor:  %.2f \n\r",theta1motor);
//    serial_printf(&SerialA, "theta_motor[0]:  %.2f \n\r",theta_motor[0]);
}

