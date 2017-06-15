#ifndef AYBFONKSIYONLAR_H_INCLUDED
#define AYBFONKSIYONLAR_H_INCLUDED


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "aybClasslar.h"
#include <math.h>
using namespace cv;
using namespace std;

void show(string str1, Mat &image)
{
    imshow(str1, image);
    waitKey(1);
}

void showWait(string str1, Mat &image)
{
    imshow(str1, image);
    waitKey(0);
}

double areaOfConvexHull(vector<Point> &contour)
{
    vector<Point> hull;
    convexHull(contour, hull);
    return contourArea(hull);
}

double areaOfIt(vector<Point> &contour)
{
    return contourArea(contour);
}



double circularityOfConvexHull(vector<Point> &contour)
{
    vector<Point> hull;
    convexHull(contour, hull);
    return 4.0*3.1416*contourArea(hull)/pow(arcLength(hull, true),2);
}

double circularityOfIt(vector<Point> &contour)
{
    return 4.0*3.1416*contourArea(contour)/pow(arcLength(contour, true),2);
}

void drawCircles(Mat &image,vector<Circle> circles, Scalar color=Scalar(0,0,255))
{

    for (vector<Circle>::iterator it = circles.begin(); it!=circles.end(); it++)
    {
        circle(image, it->center, it->radius, color, -1);
        circle(image, it->center, it->radius, Scalar(0,0,0), 2);


    }

}
void drawCircles(Mat &image, Circle circ, Scalar color=Scalar(0,0,255))
{

    circle(image, circ.center, circ.radius, color, -1);
    circle(image, circ.center, circ.radius, Scalar(0,0,0), 2);

}



vector<Circle> minCircles(vector<vector<Point> > &contours)
{

    vector<Circle> circles(0);
    for (vector<vector<Point> >::iterator it = contours.begin(); it!=contours.end();it++)
    {
        Point2f center;
        float radius;
        minEnclosingCircle(*it, center, radius);
        Circle circ(center,radius);
        circles.push_back(circ);
    }
    return circles;
}

double dist(Circle circle1, Circle circle2)
{
    return pow(pow(circle1.center.x-circle2.center.x,2)+pow(circle1.center.y-circle2.center.y,2),0.5);
}

void filtDuplicates(vector<Circle> &circles)
{
    for (vector<Circle>::iterator it = circles.begin(); it!=circles.end()-1;)
    {
        if (dist(*it,*(it+1))<0.00013020833*240*320)
        {
            it=circles.erase(it);
        }
        else
            ++it;
    }
}

void filtIrrelevant(vector<Circle> &circles)
{
    for (vector<Circle>::iterator it = circles.begin(); it!=circles.end()-2;)
    {
        if (std::min(dist(*it,*(it+1)),dist(*it,*(it+2)))>0.00065104166*240*320)
        {
            it=circles.erase(it);
        }
        else
            ++it;
    }

}

void kanalDetect(const Mat &canny, vector<Circle> &dst, int resH, int resW)
{
    vector<vector<Point> > kanalContours;
    vector<Vec4i> hierarchy;
    Mat cannyKanal;
    canny.copyTo(cannyKanal);
    double KANAL_AREA=0.001172*resH*resW;


    findContours(cannyKanal, kanalContours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    for (vector<vector<Point> >::iterator it = kanalContours.begin(); it!=kanalContours.end();)
        {
            double area, circ;
            circ=circularityOfIt(*it);
            area=areaOfIt(*it);
            if (area>KANAL_AREA*1.3||area<KANAL_AREA/1.5|| circ<0.75)
            {
                it=kanalContours.erase(it);
            }
            else
                ++it;
        }

        if (kanalContours.size()<3)
        {
            dst = vector<Circle>(0);
            return;

        }

        vector<Circle> kanalCircles=minCircles(kanalContours);
        filtDuplicates(kanalCircles);
        sort(kanalCircles.begin(), kanalCircles.end());
        filtIrrelevant(kanalCircles);


        dst = kanalCircles;
        return;

}

void pinponDetect(const Mat &canny, Circle &dst, int resH, int resW)
{

    vector<vector<Point> > pinponContours;
    vector<Vec4i> hierarchy;
    Mat cannyPinpon;
    canny.copyTo(cannyPinpon);
    morphologyEx(cannyPinpon, cannyPinpon, MORPH_CLOSE,
                 getStructuringElement(MORPH_RECT,Size(5,5)),Point(-1,1),1);
    findContours(cannyPinpon, pinponContours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    double PINPON_AREA=0.01758*resH*resW;
    for (vector<vector<Point> >::iterator it = pinponContours.begin(); it!=pinponContours.end();)
        {
            double area, circ;
            circ=circularityOfIt(*it);
            area=areaOfIt(*it);
            if (area>PINPON_AREA*1.3||area<PINPON_AREA/1.3|| circ<0.85)
            {
                it=pinponContours.erase(it);
            }
            else
                ++it;
        }

        if (pinponContours.size()<1)
        {
            dst = Circle(Point2f(0,0),0.1);
            return;
        }



        vector<Circle> pinponCircles=minCircles(pinponContours);
        filtDuplicates(pinponCircles);
        sort(pinponCircles.begin(), pinponCircles.end());
        dst = pinponCircles[0];

        return;
}




#endif // AYBFONKSIYONLAR_H_INCLUDED












