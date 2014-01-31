#include <iostream>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "networking.cpp"
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/time.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <math.h>
#include <fstream>

using namespace std;

extern "C" {
  #include "serial_comm.h" //a C header, so lets wrap it in extern "C" 
}







struct hsvRange{

int hmax,hmin,smax,smin,vmax,vmin;

} colorRange;


struct cmdData{

double x,y,z,r,p,ya,a,t_min,distX,distY;

};

std::vector<cmdData> cmd;

typedef std::vector<cmdData>::size_type cmd_sz;


int fps = 60;
//int delay = ((double)(1/(double)(fps)))*1000;
//Prototypes
void readCapParams();
void shutterCB(int pos, void* param);
void onMouse( int event, int x, int y, int, void* );
void writeLog();


///Globals

cv::VideoCapture cap;  


//Theese two are global so that the callback can modify them 
long fcount = 0; 
unsigned long t_us_start; 

//
int main(int argc, char** argv){

//for timing

struct timeval t;
unsigned long t_us_now,t_us_done,t_diff,t2_us_start,t2_us_stop,t2_diff,t2_temp;
double fps_calc =0;
double fps_avg =0;

// HSV range
colorRange.hmin = 38;
colorRange.hmax = 60;
colorRange.smin = 20;
colorRange.smax = 50;
colorRange.vmin = 170;
colorRange.vmax = 205;

// For color/treshold sliders
//cv::Scalar g_tres_min(128,128,128);
//cv::Scalar g_tres_max(170,170,170);

//unsigned int colH = 128,colS = 128,colV
 
networking n;

 
initSerial();

 
cv::Mat frame; 
//std::cout << "Init test3" << std::endl;

cv::namedWindow("Color", CV_WINDOW_AUTOSIZE); //create a window with the name "MyWindow"

cv::namedWindow("Thresholded", CV_WINDOW_AUTOSIZE); //create a window with the name "HSV"
cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
int initShutter = 581;
//int initShutter = 0;

int shutterVal = initShutter;
int cannyMin = 103;

// Shutter slider
cv::createTrackbar("Shutter","Color",&shutterVal,4095,shutterCB,NULL);

// Canny treshold

cv::createTrackbar("Threshold","Color",&cannyMin,255,NULL,NULL);


cv::Mat colorFrame;
cv::Mat tresholdedFrame;
cv::Mat hsvFrame;
cv::Mat grey,tmp;
cv::Mat contourOutput;

// HSV values are H from 0-180 while S and V are from 0-255 
//cv::createTrackbar("tbH","MyWindow",&colH,180,HCB,&g_tres_min);
//cv::createTrackbar("tbS","MyWindow",&colS,255,SCB,NULL);
//cv::createTrackbar("tbV","MyWindow",&colV,255,VCB,NULL);

cv::setMouseCallback("Color",onMouse,&colorFrame);

cap.open(CV_CAP_DC1394); // Open first firewire camera. in 2.3 use CV_CAP, in 2.5 use CV::CAP

//Get al config of camera
//readCapParams();




//std::cout << "Initial FPS: " << cap.get(CV_CAP_PROP_FPS) << std::endl;
// Try setting one

cap.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U,794); // 736
cap.set(CV_CAP_PROP_WHITE_BALANCE_RED_V,437);
cap.set(CV_CAP_PROP_EXPOSURE,initShutter); // "Shutter" in coriander
cap.set(CV_CAP_PROP_FPS,fps);
cap.set(CV_CAP_PROP_GAMMA,0);
cap.set(CV_CAP_PROP_GAIN,30);
 

//initial time
gettimeofday(&t, NULL);
t_us_start = t.tv_sec*(1000000)+t.tv_usec;

t_us_done = t_us_start;


int cnt = -1;
int frameCnt = 0;
bool countingState=false;

cv::RNG rng(12345);


n.startNet();
n.initRobotServer();


n.startInterface();
//n.testInterface("movel([0.22,-0.33,0.34,0,0,0])");





while(1){

 
cap >> frame;


std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;


//Lets get time now
gettimeofday(&t, NULL);
t_us_now = t.tv_sec*(1000000)+t.tv_usec;


t_diff = t_us_now-t_us_done;

fps_calc = 1.0/((double)(t_diff)/1000000.0); // calc fps

fps_avg = fcount/((double)(t_us_now-t_us_start)/1000000.0); // calc avg fps
fcount++;

gettimeofday(&t, NULL);
t_us_done = t.tv_sec*(1000000)+t.tv_usec;

// Get color image, decode bayer BGGR.  
cv::cvtColor(frame,colorFrame,CV_BayerBG2RGB,0);
cv::cvtColor(colorFrame, grey, CV_RGB2GRAY );


// Remove gripper from img
cv::Rect roi = cv::Rect(475,405,165,75);
cv::Mat submatrix = cv::Mat(grey,roi);
submatrix.setTo(cv::Scalar(255));

cv::threshold(grey,tresholdedFrame,cannyMin,255,cv::THRESH_BINARY_INV);



//cv::cvtColor(colorFrame,hsvFrame,CV_RGB2HSV,0);

/* -------- TRESHOLDING ----------- */

//cv::Scalar hsvMax(colorRange.hmax,colorRange.smax,colorRange.vmax);
//cv::Scalar hsvMin(colorRange.hmin,colorRange.smin,colorRange.vmin);

// Apply treshold (HSV)
//cv::inRange(hsvFrame,hsvMin,hsvMax,tresholdedFrame);

//cv::cvtColor( colorFrame, grey, CV_BGR2GRAY );

//blur( grey, tmp, cv::Size(5,5) );

cv::Canny( tresholdedFrame, tmp, cannyMin, cannyMin*2, 3 );


//contourOutput = tresholdedFrame.clone();
//blur( contourOutput, tmp, cv::Size(3,3) );


cv::findContours(tmp,contours,hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

int blablalba = 0 ;

 /// Get the moments
  std::vector<cv::Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false ); }

  ///  Get the mass centers:
  std::vector<cv::Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

 
    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
// Draw some countours, and maybe some bounding box
 
 
cv::Mat drawing = cv::Mat::zeros( tresholdedFrame.size(), CV_8UC3 );


cv::Point centerOfBlock;

  for( int i = 0; i< contours.size(); i++ )
     {

	// Filter out small areas, and filter out contours that are holes 
	// See http://stackoverflow.com/questions/8461612/using-hierarchy-in-findcontours-in-opencv
       if(cv::contourArea(contours[i]) < 200  ){
	//if(1==2){ // To small, ignore || hierarchy[i][3] < 0
	 
	// std::cout << contours.size() << endl;
	 
      }else{
      // cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
    //   boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
    //   cv::Scalar color = cv::Scalar( 255,0,0 );
      // cv::drawContours( drawing, contours, i, color, -1, 8, hierarchy, 0, cv::Point() );
    //    color = cv::Scalar( 0,0,255 );
      // cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
       
       cv::Scalar color = cv::Scalar( 0,255,0 );
	//draw center of mass
      cv::circle( drawing, mc[i], 5, color, -1, 8, 0 );
	
	
	cv::Point xx = cv::Point(boundRect[i].tl().x+(boundRect[i].width/2),boundRect[i].tl().y+(boundRect[i].height/2));
	//cv::circle( drawing, xx, 2, color, -1, 8, 0 );
	 
	centerOfBlock = mc[i];
	
	/*std::cout << "Nu er det contour nr: " << i << ". Hir(0,1,2,3): " << hierarchy[i][0] << "," 
	<< hierarchy[i][1] << "," << hierarchy[i][2] << "," << hierarchy[i][3] << std::endl;*/
	

	
      }
     }
     
     

  /*
   *CALCULATE ERROR
   
   */
  
  
     
   cv::Size s = tresholdedFrame.size(); 
     
   cv::Point centerOfFrame = cv::Point(s.width/2,s.height/2);  
     
   float distX = centerOfFrame.x-centerOfBlock.x;
   
   float distY = centerOfFrame.y-centerOfBlock.y;
   
   if(centerOfBlock.x == 0 || centerOfBlock.y == 0 || (fabs(distX) < 3 && fabs(distY) < 3)){
	    
  

     
  }else{
    
        // std::cout << "Dist(x,y): " << distX << "," << distY << std::endl;
	  double x_out = distX*0.0010;
	  double y_out = distY*-0.0010;
	  double a_out = 1.0;
	  double t_min_out = 0.2;
      
    std::stringstream ss;
  //  ss << "speedl(["<< x_out << "," << y_out << ", 0, 0, 0, 0],"<< a_out << "," << t_min_out <<")";
    ss << "speedj(["<< x_out << ",0, 0, 0, 0, 0],"<< a_out << "," << t_min_out <<")";
    std::string outss = ss.str();
      
  //  std::cout << outss << std::endl;
cmdData tmp;

tmp.x = x_out;
tmp.y = y_out;
tmp.z = 0;
tmp.r = 0;
tmp.p =0;
tmp.ya =0;
tmp.a = a_out;
tmp.t_min = t_min_out;
tmp.distX = distX;
tmp.distY = distY;

cmd.push_back(tmp);

    
    
   n.testInterface(outss);
  }

   
  

   
     

 


if(!frame .data) break;


// For filtered HSV
cv::imshow("Thresholded",tresholdedFrame); // Uncomment this line to see the actual picture. It will give an unsteady FPS


cv::imshow("Color",grey); // Uncomment this line to see the actual picture. It will give an unsteady FPS

cv::imshow( "Contours", drawing );



if(cv::waitKey(1) >= 27){ /*break;*/  } // We wait 1ms - so that the frame can be drawn. break on ESC



}

cv::destroyWindow("Color"); //destroy the window with the name, "MyWindow"
cv::destroyWindow("Thresholded"); 
closeSerial();
writeLog();


n.stopInterface();
n.stopNet();

return 0;
}

void readCapParams(){


double mode = 99;
for(int i = -4;i<=21;i++){


mode = cap.get(i);
std::cout << "Param nr  " << i << " har: " << mode << std::endl;
mode = 99;

}




}

void shutterCB(int pos, void* param){
struct timeval t;

cap.set(CV_CAP_PROP_EXPOSURE,pos);

fcount=0; // Reset frame counter, so we dont have do wait for the avg to "catch" up

gettimeofday(&t, NULL);
t_us_start = t.tv_sec*(1000000)+t.tv_usec;

std::cout << "CALLBACK !!!: pos:  " << pos << "Shutter read: " << cap.get(CV_CAP_PROP_EXPOSURE) << std::endl;
}





void onMouse( int event, int x, int y, int, void* userdata ){
cv::Mat hsvMat;

if( event != cv::EVENT_LBUTTONDOWN ) {return;}
unsigned int R=0,G=0,B=0,H=0,S=0,V=0;

cv::Mat* RGB = (cv::Mat*) userdata;
B = (*RGB).at<cv::Vec3b>(y,x)[0]; 
G = (*RGB).at<cv::Vec3b>(y,x)[1]; 
R = (*RGB).at<cv::Vec3b>(y,x)[2]; 

// Convert to HSV
cv::cvtColor((*RGB),hsvMat,CV_RGB2HSV);

H = hsvMat.at<cv::Vec3b>(y,x)[0]; 
S = hsvMat.at<cv::Vec3b>(y,x)[1]; 
V = hsvMat.at<cv::Vec3b>(y,x)[2]; 


std::cout << "(x,y):" << x << "," << y << " (R,G,B): " << R << "," << G << "," << B << " - (H,S,V) " << H << "," << S << "," << V << std::endl;

}



void writeLog(){

cmd_sz size = cmd.size();

std::ofstream a_file ("cmdLog.txt",std::ios::trunc);


for(std::vector<cmdData>::size_type i = 0; i != size; ++i){

 
  a_file<<  cmd[i].distX << "," << cmd[i].distY << "," << cmd[i].x << "," << cmd[i].y << "," << cmd[i].z << "," 
  << cmd[i].r << "," << cmd[i].p << "," <<cmd[i].ya << ","
  <<cmd[i].a << "," << cmd[i].t_min  << endl;
  // Close the file stream explicitly


}

  a_file.close();



}