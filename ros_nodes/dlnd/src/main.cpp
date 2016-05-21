#include <ctime>
#include <string>
#include <stdio.h>

#include "iostream"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/background_segm.hpp"
#include <fstream>

#include <stdlib.h>

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Time.h"
#include "dlnd/camMsg.h"

#include <math.h>

using namespace cv;
using namespace std;
using namespace ros;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

////////////////////////////////////////////////////////////////////////////
/// \brief affiche_frame                   display image
/// \param frame                           image to display
/// \param frame_name                      name of the frame
////////////////////////////////////////////////////////////////////////////
void affiche_frame(Mat frame, string frame_name)
{
    imshow(frame_name, frame);
    waitKey(1);

}

////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief diff_image           difference entre 2 images successives
/// \param cap                  flux video de la camera (pour récupérer l'image suivante)
/// \param frame                image actuelle
/// \param frame                image suivante
/// \param diff_frame           difference to return
////////////////////////////////////////////////////////////////////////////////////////////////
void diff_image(VideoCapture cap, Mat frame, Mat frame_suiv, Mat &diff_frame)
{
    // Conversion frame from color to gray
    Mat frame_gray;
    //cvtColor(frame, frame, COLOR_BGR2HSV);
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

    // Image t+1
    cap >> frame_suiv;

    // Conversion frame from color to gray
    Mat frame_gray_suiv;
    //cvtColor(frame_suiv, frame_suiv, COLOR_BGR2HSV);
    cvtColor(frame_suiv, frame_gray_suiv, COLOR_BGR2GRAY);

    // Absolute difference between the two frames
    absdiff(frame_gray, frame_gray_suiv, diff_frame);

    // Display difference
    //affiche_frame(diff_frame, "diff_frame");
}

/////////////////////////////////////////////////////////////////////////////////////
/// \brief remove_noise                     clean up binary image
/// \param frame                            binary image to clean then return
/// \param sensity                          threshold sensity
/////////////////////////////////////////////////////////////////////////////////////
void remove_noise(Mat &frame, double sensity=0)
{
    // treshold the binary image
    threshold(frame, frame, sensity, 255, THRESH_BINARY);
    Mat tresh_diff_frame = frame.clone();
    Mat tresh_frame = Mat::zeros(480,640, CV_8UC1);
    //affiche_frame(frame, "thresh_diff_frame");

    // Morphological closing with 2 different structuring elements

    Mat kernel, kernel2;
    kernel = getStructuringElement(MORPH_ELLIPSE, Size(1, 1));
    kernel2 = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));

    /*dilate(frame, frame, kernel);
    erode(frame, frame, kernel2);*/

    dilate(frame, frame, kernel);
    erode(frame, frame, kernel2);

    //affiche_frame(frame, "first_cleaned_frame");

    dilate(frame, frame, kernel2);

    tresh_diff_frame.copyTo(tresh_frame, frame);
    frame = tresh_frame.clone();

    //affiche_frame(frame, "cleaned_frame");


}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief fill_holes            filling holes in a binary image
/// \param frame                 binary image to fill up then return
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void fill_holes(Mat &frame, Mat current_frame, Mat &fond)
{
    Mat calque_fond;
    Mat dilated_frame;
    Mat mask = Mat::zeros(480,640, CV_8UC1);
    for (int i = 4; i < 480; i+=30)
    {
        for (int j = 4; j < 640; j+=30)
        {
            mask.at<uchar>(i, j) = uchar(255);
        }
    }
    //affiche_frame(mask, "masque");

    Mat kernel, kernel2, kernelmask, kernelcross, framebis;
    kernel = getStructuringElement(MORPH_ELLIPSE, Size(15, 15));
    kernel2 = getStructuringElement(MORPH_ELLIPSE, Size(30, 30));
    kernelcross =  getStructuringElement(MORPH_CROSS, Size(20, 20));
    kernelmask = getStructuringElement(MORPH_ELLIPSE, Size(20, 20));

    //dilate(mask, mask, kernelcross);
    //affiche_frame(mask, "dilated_mask");



    /*dilate(frame, frame, kernelcross, Point(-1, -1), 2);
    //affiche_frame(frame, "dilated_frame+");
    dilate(frame, frame, kernel, Point(-1, -1), 3);
    affiche_frame(frame, "dilated2_frame+");
    erode(frame, frame, kernel2, Point(-1, -1), 2);
    affiche_frame(frame, "filled_frame+");*/


    dilate(frame, dilated_frame, kernel, Point(-1, -1), 6);
    //affiche_frame(frame, "dilated_frame");

    erode(dilated_frame, frame, kernel2, Point(-1, -1), 3);
    //affiche_frame(frame, "filled_frame");

    fond = dilated_frame - frame;
    //affiche_frame(fond, "fond");

    current_frame.copyTo(calque_fond, fond);

    //affiche_frame(calque_fond, "calque_fond");

}
void fill_holes_track(Mat &frame)
{
    Mat kernel0, kernel1, kernel2;
    kernel0 = getStructuringElement(MORPH_ELLIPSE, Size(4, 4));
    kernel1 = getStructuringElement(MORPH_ELLIPSE, Size(25, 25));
    kernel2 = getStructuringElement(MORPH_ELLIPSE, Size(27, 27));

    dilate(frame, frame, kernel0);
    dilate(frame, frame, kernel1);
    //affiche_frame(frame, "dilated_frame");

    erode(frame, frame, kernel2);
    //affiche_frame(frame, "filled_frame");

}
//////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief detection             decide if an object is detected or not
/// \param contours              just to get contours size
/// \param objectDetected        boolean whose value is to set
//////////////////////////////////////////////////////////////////////////////////////////////////
void detection(vector<vector<Point> > contours, bool &objectDetected)
{
    //cout << "contours contains : "<< contours.size() << "contours." << endl;

    // Pas de contour, pas d'objet, pas détection
    if (contours.size() == 0)
    {
        objectDetected = false;
    }
    // Au moins un contour, détection
    else
    {
        objectDetected = true;
    }
}

void extract_color(Mat cop_frame, Mat diff_frame, Rect rect, double &nmbr_computed_points, int &tracked_blue, int &tracked_green, int &tracked_red)
{
    nmbr_computed_points = 0;

    vector<int> blue;
    vector<int> green;
    vector<int> red;




    for (vector<int>::const_iterator i = blue.begin(); i != blue.end(); ++i)
        cout << *i << endl;


    for (int i = rect.x; i < rect.x+rect.width; i+=1)
    {
        for (int j = rect.y; j < rect.y+rect.height; j+=1)
        {
            int pix=(int)diff_frame.at<uchar>(j, i);;
            if (pix != 0)
            {
                Vec3b intensity = cop_frame.at<Vec3b>(j, i);

                nmbr_computed_points+=1;

                blue.push_back(intensity.val[0]);
                green.push_back(intensity.val[1]);
                red.push_back(intensity.val[2]);
            }
        }
    }
    
    nth_element(blue.begin(), blue.begin() + blue.size()/2, blue.end());
    nth_element(green.begin(), green.begin() + green.size()/2, green.end());
    nth_element(red.begin(), red.begin() + red.size()/2, red.end());
    tracked_blue = blue[blue.size()/2];
    tracked_green = green[green.size()/2];
    tracked_red = red[red.size()/2];

    //cout <<"median : " << blue[blue.size()/2] << ", " << green[green.size()/2] << ", " << red[red.size()/2] <<'\n';
}

void extract_background_color(Mat cop_frame, Mat fond, Rect rect, int &background_blue, int &background_green, int &background_red)
{
    vector<int> blue;
    vector<int> green;
    vector<int> red;




    for (vector<int>::const_iterator i = blue.begin(); i != blue.end(); ++i)
        cout << *i << endl;


    for (int i = rect.x; i < rect.x+rect.width; i+=2)
    {
        for (int j = rect.y; j < rect.y+rect.height; j+=2)
        {
            int pix=(int)fond.at<uchar>(j, i);;
            if (pix != 0)
            {
                Vec3b intensity = cop_frame.at<Vec3b>(j, i);

                blue.push_back(intensity.val[0]);
                green.push_back(intensity.val[1]);
                red.push_back(intensity.val[2]);
            }
        }
    }

    nth_element(blue.begin(), blue.begin() + blue.size()/2, blue.end());
    nth_element(green.begin(), green.begin() + green.size()/2, green.end());
    nth_element(red.begin(), red.begin() + red.size()/2, red.end());
    if (blue.size() != 0)
    {
        background_blue = blue[blue.size()/2];
        background_green = green[green.size()/2];
        background_red = red[red.size()/2];
    }
    //cout <<"median : " << blue[blue.size()/2] << ", " << green[green.size()/2] << ", " << red[red.size()/2] <<'\n';
}

void display_target_color(int b, int g, int r, Mat &tracked_color)
{
    for (int i = 0; i < 50; i++)
    {
        for (int j = 0; j < 50; j++)
        {
            tracked_color.at<Vec3b>(i, j) = Vec3b(b, g, r);
            //tracked_color.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
        }
    }
    affiche_frame(tracked_color, "tracked_color");
}

void display_background_color(int b, int g, int r, Mat &tracked_color)
{
    for (int i = 0; i < 50; i++)
    {
        for (int j = 0; j < 50; j++)
        {
            tracked_color.at<Vec3b>(i, j) = Vec3b(b, g, r);
            //tracked_color.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
        }
    }
    affiche_frame(tracked_color, "background_color");
}

void display_cursors(Mat &cursors, int h_var, int s_var, int v_var)
{
    cursors = Mat(25, 100, CV_8UC3, Scalar(255, 255, 255));

    if(h_var<100)
    {
        for (int i = 0; i < 5; i++)
        {
            for (int j = 5; j < h_var; j++)
            {
                cursors.at<Vec3b>(i, j) = Vec3b(0, 255, 255);
            }
        }
    }
    else
    {
        for (int i = 0; i < 5; i++)
        {
            for (int j = 5; j < 100; j++)
            {
                cursors.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
            }
        }
    }
    if(s_var<500)
    {
        for (int i = 10; i < 15; i++)
        {
            for (int j = 1; j < (int)(s_var/5); j++)
            {
                cursors.at<Vec3b>(i, j) = Vec3b(0, 255, 255);
            }
        }
    }
    else
    {
        for (int i = 10; i < 15; i++)
        {
            for (int j = 1; j < 100; j++)
            {
                cursors.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
            }
        }
    }
    if(v_var<500)
    {
        for (int i = 20; i < 25; i++)
        {
            for (int j = 1; j < (int)(v_var/5); j++)
            {
                cursors.at<Vec3b>(i, j) = Vec3b(0, 255, 255);
            }
        }
    }
    else
    {
        for (int i = 20; i < 25; i++)
        {
            for (int j = 1; j < 100; j++)
            {
                cursors.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
            }
        }
    }
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 5; j++)
        {
            cursors.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
        }
    }

    for (int i = 10; i < 15; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            cursors.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
        }
    }

    for (int i = 20; i < 25; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            cursors.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
        }
    }
    affiche_frame(cursors, "cursors");
}

void filter_calque(Mat &calque, Mat background_color_frame, Mat &binary_background)
{
    namedWindow("filtered calque", WINDOW_AUTOSIZE);
    moveWindow("filtered calque", 700,0);
    Mat background_color_frame_HSV;
    cvtColor(background_color_frame, background_color_frame_HSV,COLOR_BGR2HSV);

    int h, s, v;

    Vec3b intensity = background_color_frame_HSV.at<Vec3b>(2, 2);
    h = (intensity.val[0]);
    s = (intensity.val[1]);
    v = (intensity.val[2]);

    Mat calque_HSV;
    cvtColor(calque, calque_HSV,COLOR_BGR2HSV);

    inRange(calque_HSV,Scalar(h-10,s-20,v-25),Scalar(h+10,s+20,v+25),binary_background);

    Mat kernel;
    kernel = getStructuringElement(MORPH_ELLIPSE, Size(4, 4));

    dilate(binary_background, binary_background, kernel);
    erode(binary_background, binary_background, kernel);
    //affiche_frame(binary_background, "binary_background");
}

void enfile(int pile[6], int value)
{
    if (pile[5] != 5)
    {
        pile[5]++;
    }
    else
    {
        pile[5] =1;
    }
    pile[pile[5]-1] = value;
}

void dispersion(int pile[6], int &variance)
{
    double mean = (pile[0] + pile[1] + pile[2] + pile[3] + pile[4])/5;
    double var = (pile[0] - mean)*(pile[0] - mean)
            +(pile[1] - mean)*(pile[1] - mean)
            +(pile[2] - mean)*(pile[2] - mean)
            +(pile[3] - mean)*(pile[3] - mean)
            +(pile[4] - mean)*(pile[4] - mean);
    var = var/5;
    variance = (int)var;
    cout <<"variance : " << variance << endl;
}
void print_file(int pile[6])
{
    for (int i= 0; i<6; i++)
    {
        int value = pile[i];
        cout <<  value << endl;
    }
    cout << "" << endl;
}
void morphOps(Mat &thresh){

    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle

    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);


    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);



}

string intToString(int number){


    std::stringstream ss;
    ss << number;
    return ss.str();
}

void drawObject(int x, int y,Mat &frame){

    //use some of the openCV drawing functions to draw crosshairs
    //on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

    circle(frame,Point(x,y),20,Scalar(0,255,0),2);
    if(y-25>0)
        line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
        line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    if(x-25>0)
        line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
        line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

    putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

}


void extract_size (Mat diff_frame, Rect rect, double &size)
{
    size = 0;

    for (int i = rect.x; i < rect.x+rect.width; i+=1)
    {
        for (int j = rect.y; j < rect.y+rect.height; j+=1)
        {
            int pix=(int)diff_frame.at<uchar>(j, i);;
            if (pix != 0)
            {
                size+=1;
            }
        }
    }
}

void select_target(Mat &frame, Mat diff_frame, Mat &calque, Rect &rect, int &x, int &y, double &size, vector< vector<Point> > contours, bool &right_object_tracked)
{
    right_object_tracked = false;

    double prev_size = size;
    int prev_x = x;
    int prev_y = y;

    bool good_target = false;

    int compteur_contour = contours.size();
    while (!good_target && compteur_contour > 0)
    {
        rect = boundingRect(contours[compteur_contour-1]);

        x = rect.x+(int)(rect.width/2);
        y = rect.y+(int)(rect.height/2);

        // THIS IS WHERE YOU NEED TO PUT AN IFSTATEMENT TO DECIDE WHETHER OR NOR IT IS A GOOD (X,Y)
        // TOMORROW

        extract_size (diff_frame, rect, size);

        compteur_contour--;
        if(size > (prev_size/3) && size < (prev_size*8) && abs(x-prev_x) < 100 && abs(y-prev_y) < 100)
        {
            good_target = true;

            rectangle(frame, rect, Scalar(0,255,0));
            drawObject(x, y,frame);
            rectangle(calque, rect, Scalar(0,255,0));
        }

    }
    if (good_target)
    {
        right_object_tracked = true;
    }
    else
    {
        right_object_tracked = false;
    }
    //right_object_tracked = true;
    //cout << "POIIIINTS : " << (double)size << endl;
}
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed, bool &objectFound){

    Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //find contours of image using openCV findContours function
    findContours(temp,contours,hierarchy,RETR_CCOMP,CHAIN_APPROX_SIMPLE );
    //use moments method to find our filtered object
    double refArea = 0;
    objectFound = false;
    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;

                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
                    x = moment.m10/area;
                    y = moment.m01/area;
                    objectFound = true;
                    refArea = area;
                }else objectFound = false;


            }
            //let user know you found an object
            if(objectFound ==true){
                putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
                //draw object location on screen
                drawObject(x,y,cameraFeed);}

        }else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
    }
    imshow("Original Image",cameraFeed);
}

void set_new_tolerances(int tolerance_h, int tolerance_s, int tolerance_v, int tracked_h, int tracked_s, int tracked_v, int &tracked_h_max, int &tracked_h_min, int &tracked_s_max, int &tracked_s_min, int &tracked_v_max, int &tracked_v_min)
{
    if ((tracked_h-tolerance_h)>0)
    {
        tracked_h_min=tracked_h-tolerance_h;
    }
    else
    {
        tracked_h_min=0;
    }
    if ((tracked_h+tolerance_h)<255)
    {
        tracked_h_max=tracked_h+tolerance_h;
    }
    else
    {
        tracked_h_max=255;
    }

    if ((tracked_s-tolerance_s)>0)
    {
        tracked_s_min=tracked_s-tolerance_s;
    }
    else
    {
        tracked_s_min=0;
    }
    if ((tracked_s+tolerance_s)<255)
    {
        tracked_s_max=tracked_s+tolerance_s;
    }
    else
    {
        tracked_s_max=255;
    }

    if ((tracked_v-tolerance_v)>0)
    {
        tracked_v_min=tracked_v-tolerance_v;
    }
    else
    {
        tracked_v_min=0;
    }
    if ((tracked_v+tolerance_v)<255)
    {
        tracked_v_max=tracked_v+tolerance_v;
    }
    else
    {
        tracked_v_max=255;
    }
}

int main(int argc, char **argv)
{
    Mat frame;
    Mat frame_HSV;
    Mat frame_suiv;
    Mat diff_frame;
    Mat fond;
    Mat calque;
    Mat filtered_calque;
    Mat binary_background;
    Mat cursors;
    Mat tracked_color_frame = Mat::zeros(50,50, CV_8UC3);
    Mat background_color_frame = Mat::zeros(50,50, CV_8UC3);
    //Mat contour_frame;
    double sensity = 15;
    double nmbr_computed_points;
    int tracked_blue;
    int tracked_green;
    int tracked_red;
    int background_blue;
    int background_green;
    int background_red;
    int tracked_h, tracked_s, tracked_v;
    int tracked_h_min, tracked_s_min, tracked_v_min;
    int tracked_h_max, tracked_s_max, tracked_v_max;
    int tolerance_h = 8, tolerance_s = 40, tolerance_v = 15;
    int last_h[6] = {0, 0, 0, 0, 0, 0}, last_s[6] = {0, 0, 0, 0, 0, 0}, last_v[6] = {0, 0, 0, 0, 0, 0};
    int h_variance, s_variance, v_variance;
    int x=0, y=0;
    int index = 0;
    int inputCamera;
    int idCamera;
    double size = 0;
    double psi = 0.54490914;
    double theta = 0.44530104;
    bool right_target_tracked = false;
    bool objectDetected = false;
    bool TrackingMode = false;
    string Mode = "Auto";
    clock_t t, tdt = clock();
    Publisher pub;
    Publisher pubAng;

    init(argc, argv, "talkerErreurPixel");
    NodeHandle n;

    pub = n.advertise<geometry_msgs::Point>("erreurPixel", 1000);
    pubAng = n.advertise<dlnd::camMsg>("cambr", 1000);


    Rate loop_rate(10);


    VideoCapture cap;

    if (n.getParam("inputCamera", inputCamera))
    {
        cap.open(inputCamera);
        cout << "inputCamera : " << inputCamera << endl;
    }
    else
    {
        cap.open(1);
    }

    if (n.getParam("idCamera", idCamera))
    {
        cout << "idCamera : " << idCamera << endl;
    }
    else
    {
        idCamera = 0;
    }


    //if(!cap.isOpened())  // check if we succeeded
    //  cout << "FAIL" << endl;
    //return -1;
    //cap >> frame;


    while(ok())
    {
        fond = Mat::zeros(640,480, CV_8UC1);
        calque = Mat::zeros(640,480, CV_8UC3);
        filtered_calque = Mat::zeros(640,480, CV_8UC3);
        cap >> frame; // get a new frame from camera
        if (Mode.compare("Auto")==0)
        {
            //affiche_frame(frame, "frame");
            if (!TrackingMode)
            {
                ofstream fichier("/home/adrien/Bureau/ProjetIndus/Code/DLND/test.txt", ios::app);
                namedWindow("frame", WINDOW_AUTOSIZE);
                moveWindow("frame", 0,0);



                diff_image(cap, frame, frame_suiv, diff_frame);
                remove_noise(diff_frame, sensity);
                fill_holes(diff_frame, frame, fond);

                Mat contour_frame = diff_frame.clone();
                vector<vector<Point> > contours;
                findContours(contour_frame, contours,RETR_LIST, CHAIN_APPROX_SIMPLE);
                detection(contours, objectDetected);

                if (objectDetected)
                {
                    //cout << "last contour : "<< contours[contours.size()-1] << endl;
                    namedWindow("background_color", WINDOW_AUTOSIZE);
                    moveWindow("background_color", 0,580);
                    namedWindow("tracked_color", WINDOW_AUTOSIZE);
                    moveWindow("tracked_color", 450,580);
                    namedWindow("cursors", WINDOW_AUTOSIZE);
                    moveWindow("cursors", 700,580);

                    Mat cop_frame = frame.clone();
                    Rect rect;
                    rect = boundingRect(contours[contours.size()-1]);
                    rectangle(frame, rect, Scalar(0,0,255));
                    //cout << "rect : "<< rect << endl;

                    Mat background_contour_frame = diff_frame.clone();
                    vector<vector<Point> > background_contours;
                    findContours(background_contour_frame, background_contours,RETR_LIST, CHAIN_APPROX_SIMPLE);
                    Rect background_rect;
                    background_rect = boundingRect(background_contours[background_contours.size()-1]);
		    
                    // color near background
                    extract_background_color(cop_frame, fond, background_rect, background_blue, background_green, background_red);
                    display_background_color(background_blue, background_green, background_red, background_color_frame);

                    cop_frame.copyTo(calque, diff_frame);
                    //affiche_frame(calque, "calque");

                    filter_calque(calque, background_color_frame, binary_background);

                    cop_frame.copyTo(filtered_calque, diff_frame - binary_background);
                    affiche_frame(filtered_calque, "filtered calque");


                    // color object
                    extract_color(cop_frame, diff_frame-binary_background, rect, nmbr_computed_points, tracked_blue, tracked_green, tracked_red);
                    display_target_color(tracked_blue, tracked_green, tracked_red, tracked_color_frame);

                    Mat test_hsv;
                    cvtColor(tracked_color_frame, test_hsv, COLOR_BGR2HSV);
                    Vec3b intensity = test_hsv.at<Vec3b>(2, 2);
                    tracked_h = intensity.val[0];
                    tracked_s = intensity.val[1];
                    tracked_v = intensity.val[2];

                    enfile(last_h, tracked_h);
                    enfile(last_s, tracked_s);
                    enfile(last_v, tracked_v);
                    //print_file(last_blue);
                    dispersion(last_h, h_variance);
                    dispersion(last_s, s_variance);
                    dispersion(last_v, v_variance);
                    cout << "" << endl;
                    x = rect.x+(int)(rect.width/2);
                    y = rect.y+(int)(rect.height/2);
                    drawObject(x, y,frame);
                    display_cursors(cursors, h_variance, s_variance, v_variance);

                    if((h_variance < 5) &
                            (s_variance < 20) &
                            (v_variance < 10))
                    {
                        TrackingMode = true;
                        destroyAllWindows();
                    }

                    index++;
                    fichier << index;
                    fichier << ",";
                    fichier << tracked_h;
                    fichier << ",";
                    fichier << tracked_s;
                    fichier << ",";
                    fichier << tracked_v;
                    fichier << ",";
                    fichier << 320;
                    fichier << ",";
                    fichier << 240;
		    fichier << ',';
		    fichier << '0';
                    fichier << "\n";
                }
                //cout << '\r' << "h : " << (int)h << " s : " << (int)s << " v : " << (int)v << flush;
                affiche_frame(frame, "frame");
                fichier.close();
            }

            else if (TrackingMode)
            {
                ofstream fichier("/home/adrien/workspaceRos/src/dlnd/log/test.txt", ios::app);

                namedWindow("tracked_color", WINDOW_AUTOSIZE);
                moveWindow("tracked_color", 0,580);
                namedWindow("frame", WINDOW_AUTOSIZE);
                moveWindow("frame", 0,0);
                namedWindow("calque", WINDOW_AUTOSIZE);
                moveWindow("calque", 700,0);

                display_target_color(tracked_blue, tracked_green, tracked_red, tracked_color_frame);

                set_new_tolerances(tolerance_h, tolerance_s, tolerance_v, tracked_h, tracked_s, tracked_v, tracked_h_max, tracked_h_min, tracked_s_max, tracked_s_min, tracked_v_max, tracked_v_min);

                cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
                inRange(frame_HSV, Scalar((int)tracked_h_min, (int)tracked_s_min, (int)tracked_v_min), Scalar((int)tracked_h_max, (int)tracked_s_max, (int)tracked_v_max), diff_frame);


                remove_noise(diff_frame);
                fill_holes_track(diff_frame);

                Mat contour_frame = diff_frame.clone();
                vector<vector<Point> > contours;
                findContours(contour_frame, contours,RETR_LIST, CHAIN_APPROX_SIMPLE);
                detection(contours, objectDetected);


                if (objectDetected)
                {
                    //cout << "last contour : "<< contours[contours.size()-1] << endl;
                    
                    size = nmbr_computed_points;
                    //cout << "size : "<< nmbr_computed_points << endl;
                    //cout << "last contour : "<< contours[contours.size()-1] << endl;

                    Mat cop_frame = frame.clone();
                    cop_frame.copyTo(calque, diff_frame);
                    Rect rect;
                    select_target(frame, diff_frame, calque, rect, x, y, size, contours, right_target_tracked);

                    if (right_target_tracked)
		    {
		            affiche_frame(calque, "calque");
		            
		            cop_frame.copyTo(calque, diff_frame);
		            extract_color(cop_frame, diff_frame, rect, nmbr_computed_points, tracked_blue, tracked_green, tracked_red);
		            cout << "POIIIINTS : " << (double)nmbr_computed_points << endl;
		            display_target_color(tracked_blue, tracked_green, tracked_red, tracked_color_frame);

		            Mat test_hsv;
		            cvtColor(tracked_color_frame, test_hsv, COLOR_BGR2HSV);
		            Vec3b intensity = test_hsv.at<Vec3b>(2, 2);
		            tracked_h = (int)((tracked_h + 0.2*intensity.val[0])/1.2);
		            tracked_s = (int)((tracked_s + 0.2*intensity.val[1])/1.2);
		            tracked_v = (int)((tracked_v + 0.2*intensity.val[2])/1.2);

		            geometry_msgs::Point ErreurPixel;
		            dlnd::camMsg cambr;
		            //pos.header.frame_id = "tmp";
		            //pos.header.stamp = Time::now();
		            ErreurPixel.x = (double) (x-(FRAME_WIDTH/2));
		            ErreurPixel.y = (double) (y-(FRAME_HEIGHT/2));

		            cambr.id = idCamera;
			    if ((x-(FRAME_WIDTH/2)) != 0)
			    {
				cambr.etheta = (double) (2*theta)/(x-(FRAME_WIDTH/2));
			    }
			    else 
			    {
				cambr.etheta = 0;
			    }

			    if ((y-(FRAME_HEIGHT/2)) != 0)
			    {
				cambr.epsi = (double) (2*psi)/(y-(FRAME_HEIGHT/2));
			    }
			    else 
			    {
				cambr.epsi = 0;
			    }
		           
		            pub.publish(ErreurPixel);
		            pubAng.publish(cambr);

		            spinOnce();

		            loop_rate.sleep();

		            index++;
		            fichier << index;
		            fichier << ",";
		            fichier << tracked_h;
		            fichier << ",";
		            fichier << tracked_s;
		            fichier << ",";
		            fichier << tracked_v;
		            fichier << ",";
		            fichier << x;
		            fichier << ",";
		            fichier << y;
			    fichier << ",";
		            fichier << (int)nmbr_computed_points;
		            fichier << "\n";

		            affiche_frame(frame, "frame");

                    }
                    else
                    {
                        putText(frame,"Object too small, fast or inexistant",Point(0,50),2,1,Scalar(0,0,255),2);
                        affiche_frame(frame, "frame");
                    }
                }
                fichier.close();



                //cout << '\r' << "total points computed : " << (int)nmbr_computed_points << "    " << (double)1000000/(tdt - t) << " fps   " << flush;


            }

            t = tdt;
            //circle(frame, ², 1.0, Scalar(255,0,0), LINE_8, 0);

            tdt = clock();

            cout << '\r' << (double)1000000/(tdt - t) << " fps   " << flush;
            //cout << "moy_blue : "<< moy_blue << "    "  << flush;
            //cout << "moy_green : "<< moy_green << "     " << flush;
            //cout << "moy_red : "<< moy_red << "     " << flush;
        }


        //circle(frame, Point( 10, 10 ), 1.0, Scalar(255,0,0), LINE_8, 0);
        //Canny(diff_frame, edges, 0, 30, 3);
        //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);


        if(waitKey(1) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
