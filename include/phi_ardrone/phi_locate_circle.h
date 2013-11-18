#ifndef PHI_LOCATE_CIRCLE_H
#define PHI_LOCATE_CIRCLE_H
#include "../include/phi_ardrone/phi_locate_circle.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/image_encodings.h"
#include <iostream>

using namespace cv;

class AR_Drone_LocateCircle
{
public:

    AR_Drone_LocateCircle();
    ~AR_Drone_LocateCircle();
	cv::Point Iteration(Mat &frame);

private:
    void _CalculateValues();
    void _LocateCirclePosition();
    void _FindChromaticityRange();
    void _ShowQuadrants();
    void _DrawVector();
    void _DrawCircles ( vector<Vec3f> vetor);
    void _SelectedChromaticity();

    // Callback
    static void _onTrackChange(int value, void *b);

    void _CreateTrackbarsAndWindows();
    cv::Point _FindWithHough();
    cv::Point _FindWithMoment();

    void _OnStart();

private:

    cv::Mat _image,_new_image, _shown_color_range,
    _color_range_triangle;

    vector<Vec3f> _circles;
    vector<Mat> _hsv_split;
    vector<Mat> _rgb_split;

    Point _found_circle;
    Point _red_ball_center;

    double _pi;

    float _red_vector, _green_vector, _color_range;

    int _quadrante;

    int _InnerCannyThreshold;
    int _CircleDetectionThreshold;
    int _min_rad, _max_rad;

    int _omega;
    int _range;
    int _min_saturation;
    int _max_saturation;





};

#endif
