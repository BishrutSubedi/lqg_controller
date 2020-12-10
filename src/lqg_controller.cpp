#include "R_COM.h"
#include <string>
#include <iostream>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <ctime>
#include <ratio>
#include <chrono>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <fstream>
#include <sys/time.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/NavSatFix.h"
#include <ros/callback_queue.h>

// from remote_initial_Scan_main.cpp

using namespace std;
using namespace std::chrono;
double Setpoint=0, Input, Output;
//using namespace exploringBB;
// Set parameters for the LQG cntroller
double lastW = 0;     
double lastE = 0;     
double Kcp = 4.4721;
double Kcf = 3.1535;
double Kep = 1.7321;
double Kef = 1;
double J = 2;

double kp=2.1;  // THIS VALUE IS HERE TO ROTATE THE MOTOR ONLY, NOT FROM ESTIMATION
double ki=2.6;  // FOR ROTATION, KI GAIN, NOTHING TO DO WITH ESTIMATION


const int sampleTime = 10;  // can change to 20s
double SampleTimeInSec = ((double)sampleTime)/1000;
steady_clock::time_point t2 = steady_clock::now();
steady_clock::time_point t1 = steady_clock::now();
steady_clock::time_point t4 = steady_clock::now();
steady_clock::time_point t3 = steady_clock::now();

steady_clock::time_point time_end,LQG_lastTime,now;
steady_clock::time_point rtime_end,rnow,r_lastTime, com_sent_time;
steady_clock::time_point pre_Azmuth_time =steady_clock::now();

// initial heading*******************************
double offsetTol = 4;

// THESE ARE TARGET VALUE OF AZIMUTH TO GET THE ANTENNA TO USING DC MOTOR ROTATION
// THEY ARE USED LATER AS HARDCORED FOR ESTIMATION USING MATLAB

//int Azmuth[] ={ 165,80,155,80,160,95,190,120,200,145,225,165,255,185,255,175,235,140,215,135,195,100,175,85,145,60,140};//exp1_songwei
//int Azmuth[] ={ 75,150,90,170,220,165,245,290,200,260,170,100,165,80,155,65,135,45,165,100,195,130,215,145,235,140,220};//exp2
//int Azmuth[] ={150,90,150,220,165,245,290,230usleep(200000)0,180,135,220,135,190,115,165,115,200,155,100,165,90,155,105,185,140,210,150}; //exp4
//int Azmuth[] ={ 145,60,155,230,170,225,150,80,140,65,135,45,105,185,115,175,95,30,125,55,145,200,135,205,125,180,90};//exp5
//int Azmuth[] ={ 125,185,130,215,130,215,135,75,145,95,160,115,185,265,200,275,185,245,180,250,170};//exp6
//int Azmuth[] ={ 135,85,140,90,170,125,195,155,210,165,205,150,200,135,180,125,170,110,165,100,155,85,140,80,135,70,115};//exp7

//Experiment for only one
int Azmuth[] ={165}; //exp4

//int Azmuth[] ={ 125,185,130,215,130,75,145,95,160,170};//exp1
//int Azmuth[] ={ 145,60,155,230,170,225,150,80,140,65,135};//exp2
//int Azmuth[] ={145,60,155,230,170};//exp3

//int Azmuth[] ={ 165,80,155,80,160,95,190,120,200,145,225,165,255,185,255,175,235};//exp1_songwei

int Azmuth_index=0;  // IS A GLOBAL VARIABLE
//***********************************************

void setting();     

void PI_Motor() ;
double Get_headingDiff(double Input, double Setpoint);
double PI_Controller(double error);
double integral=0;
readcompass R_compass;  //DECLARING A CLASS I.E. R_compass is object of type readcompass
int Azmuth_num;         // A GLOBAL VARIABLE
//*********************************************

// CREATING STRUCTURE WITH 3 FIELDS,  time, pwm and value heading to write to data files
struct DATA{          

	double during_time;     //TIME STAMP 
  int value_pwm;
  float value_heading;

};

DATA data[10000];   // CREATE ARRAY OF STRUCTURES, I.E. 10000

int data_index=0;   // A GLOBAL VARIABLE

ros::Publisher ros_serial;
ros::Publisher pwm_pub;
ros::Subscriber remote_rssi;
std_msgs::Int16   pwm_msg;


int main(int argc, char **argv)
{
	ros::init(argc,argv,"parameter_estimation");
	ros::NodeHandle n;

	pwm_pub = n.advertise<std_msgs::Int16>("remote_pwm",1);

	setting();

  Azmuth_num=sizeof(Azmuth)/sizeof(Azmuth[0]); 
	cout<<Azmuth_num<<endl;
	
	sleep(2);

  cout<< "time_count" << "," << "Output_pwm" <<" ,"<< "raw_angle" <<" ," << "target_angle" << endl; //printing to screen as well

	while (Azmuth_index < Azmuth_num)     //NOTE: THIS LOOP ONLY EXITS AFTER ALL ANGLE TARGET REACHED AND DO NOT WRITE ANYTHING AS WRITING IS AFTER WHILE LOOP FINISHES.
	{
    Lqg_Motor();
		//PI_Motor();  //Azmuth index is incremented in PI_Motor() // fills global variable

	} //END WHILE LOOP

		pwm_msg.data = 0;    //STOP THE MOTOR
		pwm_pub.publish(pwm_msg);

		return 0;
}


void setting()
{
  //Only have to set compass. pwm and gpio as different hardware
	R_compass.setting_compass();//setting compass
	usleep(200000);
}


void PI_Motor()   //motor control
  {
	 t2 = steady_clock::now();
	duration<double> time_span = duration_cast<duration<double>>(t2 - t1);    // For time stamp and freq

    if ( time_span.count()>SampleTimeInSec)
    {
      float headingRaw =R_compass.c_heading();                              //Get current compass
      //cout<<"heading: "<< headingRaw<<endl;
      float targetHeading = Azmuth[Azmuth_index];                           //Get destination direction
      Input = headingRaw;
      Setpoint = targetHeading;
      float headingDiff = Get_headingDiff(Input, Setpoint);

      if (abs(headingDiff) < offsetTol){    //offsetTot = 1.5
        pwm_msg.data = 0;                   //STOP    
	      pwm_pub.publish(pwm_msg);

    	  if(Azmuth_index < Azmuth_num)
    	   {
           Azmuth_index++;
    	   }
        }

      else
       {
        Output = PI_Controller(headingDiff);
	      t4 = steady_clock::now();
      	duration<double> time_freq = duration_cast<duration<double>>(t4 - t3);
        cout<< time_freq.count() << "," << Output <<" ,"<< Input <<" ," << targetHeading << endl; //printing to screen as well

        if (Output > 0)
        { 
          //Output = (int) Output
        	pwm_msg.data = abs(Output);    //MOTOR ROTATION YAHA MATRA CHANGE GAR H/W
		      pwm_pub.publish(pwm_msg);

        }
        else
        {
          //Output = (int) Output
        	pwm_msg.data = -(abs(Output));    //like double negative is +ve, I don't think this is necessary
		      pwm_pub.publish(pwm_msg);

          }

        }
        t1 = steady_clock::now();
        data[data_index].during_time=time_span.count();
        data[data_index].value_pwm=Output;
        data[data_index].value_heading=Input;       // THIS IS THE COMPASS READING, BETTER TO CALL R_compass.c_heading() for raw
        data_index++;

    }
  }


double Get_headingDiff(double Input, double Setpoint){
    double error1 = Setpoint - Input;
    double error2 = Setpoint + 360 - Input;
    double error3 = Setpoint - Input - 360;
    double error;
    if (abs(error1)  <= abs(error2) && abs(error1) <= abs(error3))
    {
      error = error1;
      }
    else if (abs(error2) <= abs(error1) && abs(error2) <= abs(error3))
    {
      error = error2;
      }
    else
    {
      error = error3;
    }
    return error;
  }


double PI_Controller(double error)
  {

	//integral += error * SampleTimeInSec; //(integral = integral + error * SampleTimeInSec
	Output = error * kp ; //+ integral * ki;

    if (Output > 254){
      Output = 254;
      }

    else if (Output < -254){
        Output = -254;
      }
	
    return (Output / 254)*100;
  }

/* ADDED LQG CONTROLLER */

void Lqg_Motor()   //motor control
  {

    if (duration_cast<duration<double>>(steady_clock::now()- LQG_lastTime).count()>0.02 ) //decide the loop time 50 hz
    {
      float headingRaw =R_compass.c_heading();   //Get current compass

		  // subscribe com
      //***************
      //**publish compass heading
      //***************
      if(duration_cast<duration<double>>(steady_clock::now()- com_sent_time).count()>0.1)
      {
    	  com_sent_time=steady_clock::now();
		    com_msg.data=headingRaw;
    	  com_pub.publish(com_msg);
      }
      //**************************

      float targetHeading;

      //judge if running LQG
      //***************
      if(flag_LQG_motor==1)
      {
    	  targetHeading = Azmuth;
      }
      else
      {
    	  targetHeading = headingRaw;
      }
      //***************

      Input = headingRaw;
      Setpoint = targetHeading;

      float headingDiff = Get_headingDiff(Input, Setpoint);// get the small difference between the current heading and desired heading

      //if heading difference is less than offsettol, the motor will keep quite.
      if (abs(headingDiff) < offsetTol){
    	  pwm_msg.data=0;
    	  pwm_pub.publish(pwm_msg);
        }
       else
       {
        Output = LQG_Controller(headingDiff);
		   pwm_msg.data=-Output;
		   pwm_pub.publish(pwm_msg);

        }
      LQG_lastTime=steady_clock::now();
    }
  }


double LQG_Controller(double error)
  {
    Output = -Kcp * error - Kcf * lastW;
    if (Output > 254){
      Output = 254;
      }
    else if (Output < -254){
        Output = -254;
      }
    lastE = lastW * SampleTimeInSec + Kep * SampleTimeInSec * (error - lastE);
    lastW = Output * SampleTimeInSec /J + Kef * SampleTimeInSec * (error - lastE);
    return Output/254*100;
  }

