#include <opencv2/core/core.hpp>
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>

#include "../include/phi_ardrone/phi_locate_circle.h"

#define _USE_MATH_DEFINES

using namespace cv;
using namespace std;


AR_Drone_LocateCircle::AR_Drone_LocateCircle()
{
    _InnerCannyThreshold = 200;
    _CircleDetectionThreshold = 150;
    _min_rad = 20, _max_rad = 100;

    _pi = M_PI;

    _omega  = 55;
    _range = 6;
    _min_saturation = 50;
    _max_saturation = 100;

}

AR_Drone_LocateCircle::~AR_Drone_LocateCircle()
{

}

void AR_Drone_LocateCircle::_CalculateValues()
{
    _red_vector = sin (2*_pi* (float) _omega / 360);
    _green_vector = cos (2*_pi* (float) _omega / 360);
    _color_range = pow ( cos (2* _pi * (float) _range / 360) , 2) ;

}

void AR_Drone_LocateCircle::_LocateCirclePosition()
{

    cout << "Size" << _image.cols << "x" << _image.rows<< endl;
    cout << "Center" << _red_ball_center.x << "and" << _red_ball_center.y << endl;

    if ( (_red_ball_center.x < (_image.cols/3)) && (_red_ball_center.y < (_image.rows / 3) )) _quadrante = 1;
    else if ( (_red_ball_center.x < (2 * _image.cols/3)) && (_red_ball_center.y < (_image.rows / 3) )) _quadrante = 2;
    else if ( (_red_ball_center.x < _image.cols) && (_red_ball_center.y < (_image.rows / 3) )) _quadrante = 3;

    else if ( (_red_ball_center.x < (_image.cols/3)) && (_red_ball_center.y < (2 * _image.rows / 3) )) _quadrante = 4;
    else if ( (_red_ball_center.x < (2 * _image.cols/3)) && (_red_ball_center.y < (2 * _image.rows / 3) )) _quadrante = 5;
    else if ( (_red_ball_center.x < _image.cols) && (_red_ball_center.y < (2 * _image.rows / 3) )) _quadrante = 6;

    else if ( (_red_ball_center.x < (_image.cols/3)) && (_red_ball_center.y < _image.rows )) _quadrante = 7;
    else if ( (_red_ball_center.x < (2 * _image.cols/3)) && (_red_ball_center.y < _image.rows)) _quadrante = 8;
    else if ( (_red_ball_center.x < _image.cols) && (_red_ball_center.y < _image.rows )) _quadrante = 9;

    cout << "Ball Location:" << _quadrante << endl;

}

void AR_Drone_LocateCircle::_FindChromaticityRange()
{
  _image.copyTo(_new_image);

    int x = 0, y = 0;
    for (x = 0; x < _new_image.rows; x++)
        for (y = 0; y < _new_image.cols; y++)
        {
            float total = _new_image.at<Vec3b>(x,y)[0] + _new_image.at<Vec3b>(x,y)[1] + _new_image.at<Vec3b>(x,y)[2] ;
            float red_proportion =  _new_image.at<Vec3b>(x,y)[2] / total;
            float green_proportion = _new_image.at<Vec3b>(x,y)[1] / total;
            float blue_proportion = 1 - red_proportion - green_proportion;
            float point_color_length = red_proportion * red_proportion + green_proportion * green_proportion;

            float m_param = _red_vector * red_proportion +  _green_vector * green_proportion;

            if (( m_param * m_param > _color_range * point_color_length) && ( (float)_min_saturation / 100 * (float) _min_saturation / 100 < point_color_length ) && (((float)_max_saturation / 100) * ((float)_max_saturation/100) > point_color_length))
            {


                _new_image.at<Vec3b>(x,y)[2] = 255;
                _new_image.at<Vec3b>(x,y)[1] = 255;
                _new_image.at<Vec3b>(x,y)[0] = 255;

            }
            else
            {
                _new_image.at<Vec3b>(x,y)[2] = 1;
                _new_image.at<Vec3b>(x,y)[1] = 1;
                _new_image.at<Vec3b>(x,y)[0] = 1;

            }

        }


}

void AR_Drone_LocateCircle::_ShowQuadrants()
{
   int total_cols = _image.cols;
   int total_rows = _image.rows;

   //cout << "Dimensão:" << _image.cols << "e" << _image.rows << endl;

    line( _image, Point(total_cols/3, 0) , Point(total_cols /3, total_rows), Scalar( 0, 0, 0 ), 2 , 8 );

    line( _image, Point(2* total_cols/ 3, 0 ) , Point(2 *  total_cols /3, total_rows), Scalar( 0, 0, 0 ), 2 , 8 );

    line( _image, Point(0, total_rows/3) , Point(total_cols, total_rows/3), Scalar( 0, 0, 0 ), 2 , 8 );

    line( _image, Point(0, 2 * total_rows/3) , Point( total_cols, 2 * total_rows/3), Scalar( 0, 0, 0 ), 2 , 8 );

}

void AR_Drone_LocateCircle::_DrawVector()
{
    int line_size = 5;


    switch (_quadrante)
    {
    case 1:
        line( _image, Point(5 * _image.cols / 12, 5 * _image.rows /12) , Point(_image.cols /6, _image.rows/6), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(_image.cols /6, _image.rows/6) , Point(_image.cols /6 + 15, _image.rows/6 ), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(_image.cols /6, _image.rows/6) , Point(_image.cols /6, _image.rows/6 + 15), Scalar( 0, 0, 0 ), line_size , 8 );
        break;
    case 2:
        line( _image, Point(_image.cols / 2, 5 * _image.rows / 12 ) , Point(_image.cols / 2, _image.rows/6), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(_image.cols / 2, _image.rows/6) , Point(_image.cols / 2  , _image.rows/6 ), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(_image.cols / 2, _image.rows/6) , Point(_image.cols / 2 , _image.rows/6  ), Scalar( 0, 0, 0 ), line_size , 8 );
        break;
    case 3:
        line( _image, Point(7 * _image.cols / 12, 5 * _image.rows /12) , Point(5 * _image.cols /6, _image.rows/6), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(5 * _image.cols /6, _image.rows/6) , Point(5 * _image.cols /6 - 15, _image.rows/6), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(5 * _image.cols /6, _image.rows/6) , Point(5 * _image.cols /6, _image.rows/6 + 15 ), Scalar( 0, 0, 0 ), line_size , 8 );
        break;
    case 4:
        line( _image, Point(5 * _image.cols / 12, _image.rows / 2) , Point(_image.cols /6, _image.rows/2), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(_image.cols /6, _image.rows/2), Point(_image.cols /6, _image.rows/2), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(_image.cols /6, _image.rows/2) , Point(_image.cols /6, _image.rows/2), Scalar( 0, 0, 0 ), line_size , 8 );
        break;
    case 6:
        line( _image, Point(7 * _image.cols / 12,  _image.rows / 2) , Point(5 * _image.cols /6, _image.rows/2), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(5 * _image.cols /6, _image.rows/2) , Point(5 * _image.cols /6, _image.rows/2), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(5 * _image.cols /6, _image.rows/2) , Point(5 * _image.cols /6, _image.rows/2), Scalar( 0, 0, 0 ), line_size , 8 );
        break;
    case 7:
        line( _image, Point(5 * _image.cols / 12, 7 * _image.rows /12) , Point(_image.cols /6, 5 * _image.rows/6), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(_image.cols /6, 5 * _image.rows/6) , Point(_image.cols /6, 5 * _image.rows/6), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(_image.cols /6, 5 * _image.rows/6), Point(_image.cols /6, 5 * _image.rows/6), Scalar( 0, 0, 0 ), line_size , 8 );
        break;
    case 8:
        line( _image, Point( _image.cols / 2, 7 * _image.rows /12) , Point(  _image.cols /2, 5 * _image.rows/6), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(  _image.cols /2, 5 * _image.rows/6) , Point(  _image.cols /2, 5 * _image.rows/6), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(  _image.cols /2, 5 * _image.rows/6) , Point(  _image.cols /2, 5 * _image.rows/6), Scalar( 0, 0, 0 ), line_size , 8 );
        break;
    case 9:
        line( _image, Point(7 * _image.cols / 12, 7 * _image.rows /12) , Point(5* _image.cols /6, 5 * _image.rows/6), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(5* _image.cols /6, 5 * _image.rows/6) , Point(5* _image.cols /6, 5 * _image.rows/6), Scalar( 0, 0, 0 ), line_size , 8 );
        line( _image, Point(5* _image.cols /6, 5 * _image.rows/6) , Point(5* _image.cols /6, 5 * _image.rows/6), Scalar( 0, 0, 0 ), line_size , 8 );
        break;

    default: break;
    }

}

void AR_Drone_LocateCircle::_DrawCircles ( vector<Vec3f> vetor)
{



    for( int i = 0; i < vetor.size(); i++ )
    {
        Point center(cvRound(vetor[i][0]), cvRound(vetor[i][1]));

        int radius = cvRound(vetor[i][2]);

        //circle radius
        circle(_image, center, 3, Scalar(255,255,255), -1, 10, 0 );

        // circle outline
        circle(_image, center, radius, Scalar(255,255,255), 3, 15, 0 );

        //_LocateCirclePosition();
       //_DrawVector();

    }


}

void AR_Drone_LocateCircle::_SelectedChromaticity()
{
    _shown_color_range = imread("color_range.jpg", CV_LOAD_IMAGE_COLOR);

    int x = 0, y = 0;
    float total;

    for (x = 0; x < _shown_color_range.rows; x++)
        for (y = 0; y < _shown_color_range.cols; y++)
        {
            float local_total = _shown_color_range.at<Vec3b>(x,y)[0] + _shown_color_range.at<Vec3b>(x,y)[1] + _shown_color_range.at<Vec3b>(x,y)[2] ;
            float red_proportion =  _shown_color_range.at<Vec3b>(x,y)[2] / local_total;
            float green_proportion = _shown_color_range.at<Vec3b>(x,y)[1] / local_total;
            float blue_proportion = 1 - red_proportion - green_proportion;
            float point_color_lenght = red_proportion * red_proportion + green_proportion * green_proportion;

            float local_m_param = _red_vector * red_proportion +  _green_vector * green_proportion;

            if (( local_m_param * local_m_param > _color_range * point_color_lenght) && ( (float)_min_saturation / 100 * (float) _min_saturation / 100 < point_color_lenght ) && (((float)_max_saturation / 100) * ((float)_max_saturation/100) > point_color_lenght))
            {
                /*
                _shown_color_range.at<Vec3b>(x,y)[2] = 255;
                _shown_color_range.at<Vec3b>(x,y)[1] = 255;
                _shown_color_range.at<Vec3b>(x,y)[0] = 255;*/

            }
            else
            {
                _shown_color_range.at<Vec3b>(x,y)[2] = 1;
                _shown_color_range.at<Vec3b>(x,y)[1] = 1;
                _shown_color_range.at<Vec3b>(x,y)[0] = 1;

            }

        }



}

void AR_Drone_LocateCircle::_onTrackChange(int value, void *b)
{
    AR_Drone_LocateCircle* object = static_cast<AR_Drone_LocateCircle*>( b );

    object->_CalculateValues();
    object->_SelectedChromaticity();
}

void AR_Drone_LocateCircle::_CreateTrackbarsAndWindows()
{

// Windows
namedWindow( "Display window", CV_WINDOW_AUTOSIZE);
namedWindow( "Control Bars", CV_WINDOW_AUTOSIZE );
namedWindow( "Selected Range", CV_WINDOW_AUTOSIZE );
namedWindow( "Found Colors", CV_WINDOW_AUTOSIZE );

// Trackbars creation


// Hough circles Bars
createTrackbar("Hough Cirles Parameter 1", "Control Bars", &_InnerCannyThreshold, 255);
createTrackbar("Hough Cirles Parameter 2", "Control Bars", &_CircleDetectionThreshold, 255);
createTrackbar("Minimum Ball Radius", "Control Bars", &_min_rad, 255);
createTrackbar("Maximum Ball Radius", "Control Bars", &_max_rad, 255);

// Chromaticity Bars

createTrackbar("Omega","Control Bars",&_omega,360,_onTrackChange, this);
createTrackbar("Range","Control Bars",&_range,100,_onTrackChange, this);
createTrackbar("Min Saturation","Control Bars",&_min_saturation,255,_onTrackChange, this);
createTrackbar("Max Saturation","Control Bars",&_max_saturation,255,_onTrackChange, this);



}

cv::Point AR_Drone_LocateCircle::_FindWithHough()
{

    Point center;

    Mat src_gray;

    cvtColor (_new_image, src_gray, CV_BGR2GRAY );

   blur( src_gray, src_gray, Size(10,10) );

    HoughCircles(src_gray, _circles, CV_HOUGH_GRADIENT, 2, 20, _InnerCannyThreshold, _CircleDetectionThreshold, _min_rad, _max_rad);


    for( int i = 0; i < _circles.size(); i++ )
    {
       center.x = cvRound(_circles[i][0]);
       center.y = cvRound(_circles[i][1]);

    }


    _DrawCircles(_circles);

    return center;

}

cv::Point AR_Drone_LocateCircle::_FindWithMoment()
{
    Point center;

    Mat src_gray;

    cvtColor(_new_image, src_gray, CV_BGR2GRAY );

    blur( src_gray, src_gray, Size(10,10) );

    Moments main_moment = moments(src_gray, 0 );

    double moment10 = main_moment.m10;
    double moment01 = main_moment.m01;
    double area = main_moment.m00;

    //if(area>400000)
    {

         center.x = moment10/area;
         center.y = moment01/area;
         line(_image, Point(center.x,center.y), Point(center.x, center.y), Scalar( 150, 255, 0 ), 10, 8);

    }

    return center;



}

cv::Point AR_Drone_LocateCircle::_Iteration()
{
    Point center_Iteration;

    _FindChromaticityRange();

    center_Iteration = _FindWithHough();

    _ShowQuadrants();

    imshow("Display window", _image);
    imshow("Found Colors", _new_image);

    imshow("Selected Range", _shown_color_range);

    return center_Iteration;

}

void AR_Drone_LocateCircle::_OnStart()
{
    _shown_color_range = imread("_color_range.jpg", CV_LOAD_IMAGE_COLOR);
    _CreateTrackbarsAndWindows();

     _CalculateValues();


}


