/***************************************************************************************/
/* Dynamic Simulation of a Planar 2DoF Robotic Arm, Copyright Atsushi Kakogawa         */
/* Created April 23, 2020, Department of Robotics, Ritsumeikan University, Japan       */
/***************************************************************************************/

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <GL/glut.h>
#include <math.h>
#include <string.h>
#include "function.h"

int width = 800, height = 800;

void init(void) {

	// Total Time
	t = 0, T = 5;
	// Sampling Time for Dynamics
	dt = 0.0001;

	// Target Potision of the Arm End
	xend = 0;									// m
	yend = 0.5;

	//xend = len[1] * cos(the[1]) + len[2] * cos(the[1] + the[2]);
	//yend = len[1] * sin(the[1]) + len[2] * sin(the[1] + the[2]);

	xd = 0;
	yd = 0.5;// m
	theend[1] = pi / 4;
	theend[2] = pi / 6;

	//thed[1] = pi / 4;
	//thed[2] = pi / 6;
	//
	// Weight and Inertia		//各パラメータ
	m[1] = 1;									// kg
	m[2] = 1;									// kg
	len[1] = 0.3;								// m
	lenG[1] = len[1] / 2;						// m
	len[2] = 0.3;								// m
	lenG[2] = len[2] / 2;						// m
	I[1] = 1 / 12 * m[1] * lenG[1] * lenG[1];	// kg*m^2
	I[2] = 1 / 12 * m[2] * lenG[2] * lenG[2];	// kg*m^2

	//逆運動学による手先位置PTP制御
	theta_rev_ptp[1] = (atan2(yd, xd) - acos(((len[1] * len[1]) - (len[2] * len[2]) + (xd * xd) + (yd * yd)) / (2 * len[1] * sqrt((xd * xd) + (yd * yd)))));
	theta_rev_ptp[2] = pi - acos(((len[1] * len[1]) + (len[2] * len[2]) - ((xd * xd) + (yd * yd))) / (2 * len[1] * len[2]));
	printf("theta1 = %f\n", theta_rev_ptp[1]);

	// Gain
	kpx = 500;
	kpy = 500;
	kd[1] = 10;
	kd[2] = 10;
	kix = 100;
	kiy = 100;

	// Initial parameters		//初期条件
	//the[1] = - pi / 3;
	//the[2] = pi / 6;
	the[1] = 0;
	the[2] = 0;
	dthe[1] = 0;
	dthe[2] = 0;
	ddthe[1] = 0;
	ddthe[2] = 0;
	x[0] = 0;
	y[0] = 0;
	x[1] = len[1] * cos(the[1]);
	y[1] = len[1] * sin(the[1]);
	x[2] = len[1] * cos(the[1]) + len[2] * cos(the[1] + the[2]);
	y[2] = len[1] * sin(the[1]) + len[2] * sin(the[1] + the[2]);
	xini = len[1] * cos(the[1]) + len[2] * cos(the[1] + the[2]);
	yini = len[1] * sin(the[1]) + len[2] * sin(the[1] + the[2]);
	theini[1] = 0;
	theini[2] = 0;

} 

int main(int argc, char** argv){

	init();

	errno_t error_XY, error_T;
	error_XY = fopen_s(&DATA_XY, "xy.dat", "w");
	fprintf(DATA_XY, "t [s]	x2 [Nm]	y2 [Nm]\n");
	fprintf(DATA_XY, "%.3f	%lf	%lf\n", t, x[2], y[2]);
	error_T = fopen_s(&DATA_T, "tau.dat", "w");
	fprintf(DATA_T, "t [s]	tau1 [Nm]	tau2 [Nm]\n");
	fprintf(DATA_T, "%.3f	%lf	%lf\n", t, tau[1], tau[2]);

	// init gult
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(width, height);
	glutCreateWindow("2-DoF Robotic Arm");

	// Call-back
	glutDisplayFunc(display);
	glutIdleFunc(idle);

	// 2D-setup
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluOrtho2D( -0.3, 1.2, -0.75, 0.75 );

	glutMainLoop();

	fclose(DATA_XY);
	fclose(DATA_T);

	return 0;
}