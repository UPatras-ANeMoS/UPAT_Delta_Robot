/****************************************************************
 *  phidget.cpp							*
 *  								*
 *  Created by Georgios Ntekoumes on December 2016		*
 *  Copyrights © 2016 Georgios Ntekoumes. All rights reserved.  *
 *							        *
 ****************************************************************/

#include <stdio.h>
#include <iostream>
#include <phidget21.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h>
#include "ros/console.h"
#include <time.h>
#include <unistd.h>
#include <ios>
#include <fstream>

using namespace std;

#define pi 3.14159265358979323846

double feedback[3]={-1,-1,-1};

int choice;
double X,Y,Z;

struct timespec loop_start, loop_end, start, end;

double sB = 0.3; double sP = 0.05; double L = 0.2; double l = 0.51;



//---------------------------------------FKin--------------------------------------------//
void ForwardKinematics(double &x, double &y, double &z, double &theta1, double &theta2, double &theta3)
{
	// Diastaseis rompot
	double wB = (sqrt(3) / 6)*sB; double uB = (sqrt(3) / 3)*sB; double wP = (sqrt(3) / 6)*sP; double uP = (sqrt(3) / 3)*sP;
	// Shmeia Av
	double x1 = 0; double y1 = -wB - L*cos(theta1*pi / 180) + uP; double z1 = -L*sin(theta1*pi / 180);
	double x2 = (sqrt(3) / 2)*(wB + L*cos(theta2*pi / 180)) - sP / 2; double y2 = (wB + L*cos(theta2*pi / 180)) / 2 - wP; double z2 = -L*sin(theta2*pi / 180);
	double x3 = -(sqrt(3) / 2)*(wB + L*cos(theta3*pi / 180)) + sP / 2; double y3 = (wB + L*cos(theta3*pi / 180)) / 2 - wP; double z3 = -L*sin(theta3*pi / 180);

	if (z1 == z2 && z1 == z3){
		double a = 2 * (x3 - x1); double b = 2 * (y3 - y1);
		double c = -pow(x1, 2) - pow(y1, 2) + pow(x3, 2) + pow(y3, 2);
		double d = 2 * (x3 - x2); double e = 2 * (y3 - y2);
		double f = -pow(x2, 2) - pow(y2, 2) + pow(x3, 2) + pow(y3, 2);

		x = (c*e - b*f) / (a*e - b*d);
		y = (a*f - c*d) / (a*e - b*d);

		double A = 1; double B = -2 * z1;
		double C = pow(z1, 2) - pow(l, 2) + pow(x - x1, 2) + pow(y - y1, 2);
		double z_1 = (-B + sqrt(pow(B, 2) - 4 * A*C)) / (2 * A);
		double z_2 = (-B - sqrt(pow(B, 2) - 4 * A*C)) / (2 * A);

		if (z_1 < z_2){ z = z_1; }
		else { z = z_2; }

	}

	else if (z1 == z3 && z1 != z2){
		double ze = z3;
		double a11 = 2 * (x3 - x1); double a12 = 2 * (y3 - y1);
		double a21 = 2 * (x3 - x2); double a22 = 2 * (y3 - y2); double a23 = 2 * (ze - z2);
		double b1 = -pow(x1, 2) - pow(y1, 2) + pow(x3, 2) + pow(y3, 2);
		double b2 = -pow(x2, 2) - pow(y2, 2) - pow(z2, 2) + pow(x3, 2) + pow(y3, 2) + pow(ze, 2);

		double a1 = (b2 / a21) - (b1 / a11); double a2 = (a22 / a21) - (a12 / a11); 
		double a3 = (a23 / a21); double a4 = -a3 / a2; double a5 = a1 / a2;
		double a6 = (-a12*a4) / a11; double a7 = (b1 - a12*a5) / a11;

		double a = pow(a4, 2) + pow(a6, 2) + 1;
		double b = 2 * a6*(a7 - x1) + 2 * a4*(a5 - y1) - 2 * ze;
		double c = a7*(a7 - 2 * x1) + a5*(a5 - 2 * y1) + pow(x1, 2) + pow(y1, 2) + pow(ze, 2) - pow(l, 2);

		double z_1 = (-b + sqrt(pow(b, 2) - 4 * a*c)) / (2 * a);
		double z_2 = (-b - sqrt(pow(b, 2) - 4 * a*c)) / (2 * a);

		if (z_1 < z_2){ z = z_1; }
		else { z = z_2; }

		x = a6*z + a7;
		y = a4*z + a5;

	}

	else if (z2 == z3 && z1 != z2){
		double ze = z3;
		double a11 = 2 * (x3 - x1); double a12 = 2 * (y3 - y1); double a13 = 2 * (ze - z1);
		double a21 = 2 * (x3 - x2); double a22 = 2 * (y3 - y2);
		double b1 = -pow(x1, 2) - pow(y1, 2) - pow(z1, 2) + pow(x3, 2) + pow(y3, 2) + pow(ze, 2);
		double b2 = -pow(x2, 2) - pow(y2, 2) + pow(x3, 2) + pow(y3, 2);

		double a1 = (a22 / a21) - (a12 / a11); double a2 = (a13 / a11); 
		double a3 = (b2 / a21) - (b1 / a11); double a4 = a2 / a1; double a5 = a3 / a1;
		double a6 = (-a22*a4) / a21; double a7 = (b2 - a22*a5) / a21;

		double a = pow(a4, 2) + pow(a6, 2) + 1;
		double b = 2 * a6*(a7 - x1) + 2 * a4*(a5 - y1) - 2 * z1;
		double c = a7*(a7 - 2 * x1) + a5*(a5 - 2 * y1) + pow(x1, 2) + pow(y1, 2) + pow(z1, 2) - pow(l, 2);

		double z_1 = (-b + sqrt(pow(b, 2) - 4 * a*c)) / (2 * a);
		double z_2 = (-b - sqrt(pow(b, 2) - 4 * a*c)) / (2 * a);

		if (z_1 < z_2){ z = z_1; }
		else { z = z_2; }

		x = a6*z + a7;
		y = a4*z + a5;

	}

	else {
		double a11 = 2 * (x3 - x1); double a12 = 2 * (y3 - y1); double a13 = 2 * (z3 - z1);
		double a21 = 2 * (x3 - x2); double a22 = 2 * (y3 - y2); double a23 = 2 * (z3 - z2);
		double b1 = -pow(x1, 2) - pow(y1, 2) - pow(z1, 2) + pow(x3, 2) + pow(y3, 2) + pow(z3, 2);
		double b2 = -pow(x2, 2) - pow(y2, 2) - pow(z2, 2) + pow(x3, 2) + pow(y3, 2) + pow(z3, 2);

		double a1 = (a11 / a13) - (a21 / a23); double a2 = (a12 / a13) - (a22 / a23);
		double a3 = (b2 / a23) - (b1 / a13); double a4 = -a2 / a1; double a5 = -a3 / a1;
		double a6 = (-a21*a4 - a22) / a23; double a7 = (b2 - a21*a5) / a23;

		double a = pow(a4, 2) + pow(a6, 2) + 1;
		double b = 2 * a4*(a5 - x1) - 2 * y1 + 2 * a6*(a7 - z1);
		double c = a5*(a5 - 2 * x1) + a7*(a7 - 2 * z1) + pow(x1, 2) + pow(y1, 2) + pow(z1, 2) - pow(l, 2);

		double y_1 = (-b + sqrt(pow(b, 2) - 4 * a*c)) / (2 * a);
		double y_2 = (-b - sqrt(pow(b, 2) - 4 * a*c)) / (2 * a);

		double x_1 = a4*y_1 + a5;
		double x_2 = a4*y_2 + a5;
		double z_1 = a6*y_1 + a7;
		double z_2 = a6*y_2 + a7;

		if (z_1 < z_2){ x = x_1; y = y_1; z = z_1; }
		else { x = x_2; y = y_2; z = z_2; }

	}

}

//---------------------------------------IKin--------------------------------------------//
void InverseKinematics(double &theta1, double &theta2, double &theta3, double &x, double &y, double &z)
{
	double wB = (sqrt(3) / 6)*sB; double uB = (sqrt(3) / 3)*sB; double wP = (sqrt(3) / 6)*sP; double uP = (sqrt(3) / 3)*sP;
	double a = wB - uP; double b = (sP / 2) - (sqrt(3)*wB) / 2; double c = wP - wB / 2;

	double E1 = 2 * L*(y + a);
	double F1 = 2 * z*L;
	double G1 = pow(x,2) + pow(y,2) + pow(z,2) + pow(a,2) + pow(L,2) + 2*y*a - pow(l,2);
	double E2 = -L*(sqrt(3)*(x + b) + y + c);
	double F2 = 2 * z*L;
	double G2 = pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(b, 2) + pow(c, 2) + pow(L, 2) + 2 *(x*b + y*c) - pow(l, 2);
	double E3 = L*(sqrt(3)*(x - b) - y - c);
	double F3 = 2 * z*L;
	double G3 = pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(b, 2) + pow(c, 2) + pow(L, 2) + 2 * (-x*b + y*c) - pow(l, 2);

	double t1_1 = (-F1 + sqrt(pow(E1, 2) + pow(F1, 2) - pow(G1, 2))) / (G1 - E1);
	double t1_2 = (-F1 - sqrt(pow(E1, 2) + pow(F1, 2) - pow(G1, 2))) / (G1 - E1);
	double t2_1 = (-F2 + sqrt(pow(E2, 2) + pow(F2, 2) - pow(G2, 2))) / (G2 - E2);
	double t2_2 = (-F2 - sqrt(pow(E2, 2) + pow(F2, 2) - pow(G2, 2))) / (G2 - E2);
	double t3_1 = (-F3 + sqrt(pow(E3, 2) + pow(F3, 2) - pow(G3, 2))) / (G3 - E3);
	double t3_2 = (-F3 - sqrt(pow(E3, 2) + pow(F3, 2) - pow(G3, 2))) / (G3 - E3);

	double th1_1 = 2 * atan(t1_1); double th1_2 = 2 * atan(t1_2);
	double th2_1 = 2 * atan(t2_1); double th2_2 = 2 * atan(t2_2);
	double th3_1 = 2 * atan(t3_1); double th3_2 = 2 * atan(t3_2);

	if (th1_1 <= (pi / 2) && th1_1 >= -(pi / 2)){ theta1 = (180 * th1_1) / pi; }
	else { theta1 = (180 * th1_2) / pi; }

	if (th2_1 <= (pi / 2) && th2_1 >= -(pi / 2)){ theta2 = (180 * th2_1) / pi; }
	else { theta2 = (180 * th2_2) / pi; }

	if (th3_1 <= (pi / 2) && th3_1 >= -(pi / 2)){ theta3 = (180 * th3_1) / pi; }
	else { theta3 = (180 * th3_2) / pi; }

}

//---------------------------------------PhidgetFunctions------------------------------------------//
int CCONV AttachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
	printf("%s %10d attached!\n", name, serialNo);

	return 0;
}

int CCONV DetachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
	printf("%s %10d detached!\n", name, serialNo);

	return 0;
}

int CCONV ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr, int ErrorCode, const char *Description)
{
	printf("Error handled. %d - %s\n", ErrorCode, Description);
	return 0;
}

int CCONV PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value)
{
	//printf("Motor: %d > Current Position: %f\n", Index, Value);
	return 0;
}

//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
int display_properties(CPhidgetAdvancedServoHandle phid)
{
	int serialNo, version, numMotors;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetAdvancedServo_getMotorCount (phid, &numMotors);

	//printf("%s\n", ptr);
	//printf("Serial Number: %10d\nVersion: %8d\n# Motors: %d\n", serialNo, version, numMotors);

	return 0;
}

void chatterCallback(const std_msgs::Float64MultiArray& msg)
{
  //ROS_INFO("I heard: ", msg.data);
  feedback[0] = msg.data[0]; feedback[1] = msg.data[1]; feedback[2] = msg.data[2];
  //std::cout<<feedback[0]<<"\t"<<feedback[1]<<"\t"<<feedback[2]<<std::endl;
  //std::cout<<msg.data[0]<<"\t"<<msg.data[1]<<"\t"<<msg.data[2]<<std::endl;
  //ROS_INFO("I heard: ", feedback);
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "phidget");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("feedback", 1, chatterCallback);
  usleep(100000);
  while(feedback[0]<0 || feedback[0]<0 || feedback[0]<0){
    ros::spinOnce();
  }
  std::cout<<"Theta 1 = "<<feedback[0]<<"\t"<<"Theta 2 = "<<feedback[1]<<"\t"<<"Theta 3 = "<<feedback[2]<<std::endl;
  //std::cout<<"Press any key to continue..."<<std::endl;
  //getchar();
  
  //arxeio
  //std::ofstream log("/home/george/delta/shmeia.txt", std::ios_base::app | std::ios_base::out);
  //arxeio
  
  // Anoigma servo
  int result;
  double curr_pos;
  double theta1, theta2, theta3;
  double pos1, pos2, pos3;
  const char *err;
  double minAccel, maxVel;

  CPhidgetAdvancedServoHandle servo = 0;
  CPhidgetAdvancedServo_create(&servo);
  CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo, AttachHandler, NULL);
  CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo, DetachHandler, NULL);
  CPhidget_set_OnError_Handler((CPhidgetHandle)servo, ErrorHandler, NULL);
  CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo, PositionChangeHandler, NULL);
  CPhidget_open((CPhidgetHandle)servo, -1);
  printf("Waiting for Phidget to be attached....\n");
  if ((result = CPhidget_waitForAttachment((CPhidgetHandle)servo, 5000)))
  {
    CPhidget_getErrorDescription(result, &err);
    printf("Problem waiting for attachment: %s\n", err);
    return 0;
  }
  //display_properties(servo);
  printf("Reading.....\n");

  CPhidgetAdvancedServo_getAccelerationMin(servo, 3, &minAccel);
  CPhidgetAdvancedServo_getVelocityMax(servo, 3, &maxVel);
  CPhidgetAdvancedServo_setAcceleration(servo, 3, minAccel * 10);
  CPhidgetAdvancedServo_setVelocityLimit(servo, 3, maxVel/1.5);
  CPhidgetAdvancedServo_setAcceleration(servo, 4, minAccel * 10);
  CPhidgetAdvancedServo_setVelocityLimit(servo, 4, maxVel/1.5);
  CPhidgetAdvancedServo_setAcceleration(servo, 2, minAccel * 10);
  CPhidgetAdvancedServo_setVelocityLimit(servo, 2, maxVel/1.5);
  // end anoigma servo
  
  double Kp = 0.05, Ki = 0.1, Kd = 0.01;						// Kp, Ki,Kd
  double th1d, th2d, th3d, th1f, th2f, th3f, th1r, th2r, th3r;
  double th1is, th2is, th3is, x_is, y_is, z_is;		
  double x,y,z;
  long double time, dt;
  dt = 0.015;
  uint64_t diff;
  
  ros::spinOnce();
  th1is = feedback[0]; th2is = feedback[1]; th3is = feedback[2];
  ForwardKinematics(x_is,y_is,z_is,th1is,th2is,th3is);
  X = x_is; Y = y_is; Z = z_is;
  if (th1is < 0){ th1is = 0; }
  if (th2is < 0){ th2is = 0; }
  if (th3is < 0){ th3is = 0; }
  if (th1is > 90){ th1is = 90; }
  if (th2is > 90){ th2is = 90; }
  if (th3is > 90){ th3is = 90; }
  pos1 = 111 + th1is;
  pos2 = 118 + th2is;
  pos3 = 126 + th3is;
  CPhidgetAdvancedServo_setPosition(servo, 3, pos1);
  CPhidgetAdvancedServo_setPosition(servo, 4, pos2);
  CPhidgetAdvancedServo_setPosition(servo, 2, pos3);
  CPhidgetAdvancedServo_setEngaged(servo, 3, 1);
  CPhidgetAdvancedServo_setEngaged(servo, 4, 1);
  CPhidgetAdvancedServo_setEngaged(servo, 2, 1);
  usleep(500000);
  // engage kai etoimo gia kinhsh
  
  bool i=true;
  while(i){
    std::cout<<"For desired point press 1, for exit press anything else: \n";
    std::cin>>choice;
    switch(choice){
      case 1:{
	std::cout<<"Please insert cartesian coordinates.\n";
	std::cout<<"[-0.1 < x < 0.1](m): x = ";
	std::cin>>X;
	while (X<-0.1 || X>0.1){
	  std::cout<<"must be [-0.1 < x < 0.1](m): x = ";
	  std::cin>>X;
	}
	std::cout<<"[-0.1 < y < 0.1](m): y = ";
	std::cin>>Y;
	while (Y<-0.1 || Y>0.1){
	  std::cout<<"must be [-0.1 < y < 0.1](m): y = ";
	  std::cin>>Y;
	}
	std::cout<<"[-0.62 < z < -0.52](m): z = ";
	std::cin>>Z;
	while (Z<-0.62 || Z>-0.52){
	  std::cout<<"must be [-0.62 < z < -0.52](m): z = ";
	  std::cin>>Z;
	}
	std::cout<<"(x,y,z) = ("<<X<<","<<Y<<","<<Z<<")\n";
	
	//---------------------------------PID loop-----------------------------------------//
	
	InverseKinematics(th1d, th2d, th3d, X, Y, Z);
	ros::spinOnce();
	double proportional[3];
	double integral[3];
	double derivative[3];
	double pre_error[3] = {0, 0, 0};
	double error[3];
	double I_error[3];
	I_error[0] = th1d / Ki;
	I_error[1] = th2d / Ki;
	I_error[2] = th3d / Ki;
	
	clock_gettime(CLOCK_MONOTONIC, &start);			//xronos metakinhshs
	
	while ((!((fabs(th1d - feedback[0])<1) && (fabs(th2d - feedback[1])<1) && (fabs(th3d - feedback[2])<1)))){
	  clock_gettime(CLOCK_MONOTONIC, &loop_start);			//xronos loopas
	  error[0] = th1d - feedback[0]; error[1] = th2d - feedback[1]; error[2] = th3d - feedback[2];
	  I_error[0] = I_error[0] + error[0] * dt;
	  I_error[1] = I_error[1] + error[1] * dt;
	  I_error[2] = I_error[2] + error[2] * dt;
	  proportional[0] = Kp*error[0];
	  proportional[1] = Kp*error[1];
	  proportional[2] = Kp*error[2];
	  integral[0] = Ki*I_error[0];
	  integral[1] = Ki*I_error[1];
	  integral[2] = Ki*I_error[2];
	  derivative[0] = Kd * ( (error[0] - pre_error[0]) / dt );
	  derivative[1] = Kd * ( (error[1] - pre_error[1]) / dt );
	  derivative[2] = Kd * ( (error[2] - pre_error[2]) / dt );
	  pre_error[0] = error[0];
	  pre_error[1] = error[1];
	  pre_error[2] = error[2];
	  theta1 = proportional[0] + integral[0] + derivative[0];
	  theta2 = proportional[1] + integral[1] + derivative[1];
	  theta3 = proportional[2] + integral[2] + derivative[2];
	  
	  if (theta1 < 0){ theta1 = 0; }
	  if (theta2 < 0){ theta2 = 0; }
	  if (theta3 < 0){ theta3 = 0; }
	  if (theta1 > 80){ theta1 = 80; }
	  if (theta2 > 80){ theta2 = 80; }
	  if (theta3 > 80){ theta3 = 80; }
	  
	  pos1 = 111 + theta1;
	  pos2 = 118 + theta2;
	  pos3 = 126 + theta3;
			  
	  CPhidgetAdvancedServo_setPosition(servo, 3, pos1);
	  CPhidgetAdvancedServo_setPosition(servo, 4, pos2);
	  CPhidgetAdvancedServo_setPosition(servo, 2, pos3);
	  usleep(10000);
	  
	  ros::spinOnce();
	  
	  clock_gettime(CLOCK_MONOTONIC, &loop_end);
	  diff = (double)1000000000 * (loop_end.tv_sec - loop_start.tv_sec) + loop_end.tv_nsec - loop_start.tv_nsec;
	  dt = (long long unsigned int)diff/(double)1000000000;
	  std::cout<<"dt = "<<dt<<std::endl;
	  
	  clock_gettime(CLOCK_MONOTONIC, &end);
	  diff = (double)1000000000 * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec;
	  time = (long long unsigned int)diff/(double)1000000000;
	  
	}
	
	std::cout<<"Time elapsed = "<<time<<" seconds."<<std::endl;
	
	//--------------------------------End of PID loop----------------------------------------//
	
//  	double xi,psi,zhta;
//  	ForwardKinematics(xi,psi,zhta,feedback[0],feedback[1],feedback[2]);
//  	log <<X<<"\t"<<Y<<"\t"<<Z<<"\t"<<xi<<"\t"<<psi<<"\t"<<zhta<<th1d<<"\t"<<th2d<<"\t"<<th3d<<"\t"<<feedback[0]<<"\t"<<feedback[1]<<"\t"<<feedback[2]<< "\n";
	
      }
	break;
	
      default:
	std::cout<<"Disengage servos.\n";
	i=false;
	break;
    }
  }
  
  usleep(1000000);
  double r;
  for (unsigned int q2 = 0; q2 < 40; q2++){
    r = (q2 + 1) / 40;
    x = X + r*(x_is - X);
    y = Y + r*(y_is - Y);
    z = Z + r*(z_is - Z);
    InverseKinematics(th1d, th2d, th3d, x, y, z);
    pos1 = 111 + th1d;
    pos2 = 118 + th2d;
    pos3 = 126 + th3d;
    CPhidgetAdvancedServo_setPosition(servo, 3, pos1);
    CPhidgetAdvancedServo_setPosition(servo, 4, pos2);
    CPhidgetAdvancedServo_setPosition(servo, 2, pos3);
    //usleep(100000);
  }
  usleep(1000000);
  
  printf("Closing...\n");
  CPhidgetAdvancedServo_setEngaged(servo, 3, 0);
  CPhidgetAdvancedServo_setEngaged(servo, 4, 0);
  CPhidgetAdvancedServo_setEngaged(servo, 2, 0);
  CPhidget_close((CPhidgetHandle)servo);
  CPhidget_delete((CPhidgetHandle)servo);

  

  return 0;
}







