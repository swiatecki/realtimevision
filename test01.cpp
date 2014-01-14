#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "test02.cpp"

int main(int argc, char** argv){

//TestClass tc;

//tc.doStuff();
  
cv::VideoCapture cap;  
  
cv::Mat img = cv::imread("moose.jpeg", CV_LOAD_IMAGE_UNCHANGED); 
std::cout << "Hello World" << std::endl;


if(! img.data ) // Check for invalid input
    {
        std::cout << "Could not open or find the image" << std::endl ;
        return -1;
    }


cv::namedWindow("MyWindow", CV_WINDOW_AUTOSIZE); //create a window with the name "MyWindow"
cv::imshow("MyWindow", img); //display the image which is stored in the 'img' in the "MyWindow" window

cv::waitKey(0); //wait infinite time for a keypress

cv::destroyWindow("MyWindow"); //destroy the window with the name, "MyWindow"





return 0;
}