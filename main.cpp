#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>
#include "aybFonksiyonlar.h"
#include "aybClasslar.h"
#include <chrono>

#include <ctime>
#include <time.h>

using namespace cv;
using namespace std;


int main()
{
    Motor motorSag(0,2,3), motorSol(12,13,14);
    const double PINPON_RADIUS=2.0;
    Webcam cap(0, 240, 320);
    cap.start();
    double iK2R=0, iCenter=0, iK2B=0;
    double HEDEF_KANAL2ROBOT=0.30*240, HEDEF_CENTER=0.66*320, HEDEF_KANAL2BARGE=5;
    double lastErrK2R, lastErrCenter, lastErrK2B;
    double baseSpeed=15;
    double KP_K2R=320*240*0.0000003, KI_K2R=0, KD_K2R=0;
    //double KP_center=320*240*0.0000025, KI_center=0, KD_center=0;
    double KP_K2B=5, KI_K2B=0, KD_K2B=0;

    Mat kernel=getStructuringElement( MORPH_RECT, Size(1,1));
    struct timespec start, finish;
    double elapsed;




    for (int n=0;;n++)
    {
        clock_gettime(CLOCK_MONOTONIC, &start);

        Mat image;
        cap.read(image);
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray,gray, Size(5,5), 0);
        Mat canny;
        Canny(gray, canny, 20, 200);
        morphologyEx(canny, canny, MORPH_CLOSE, kernel);
        show("captured",image);




        vector<Circle> kanalCircles;
        Circle pinponCircle;
        std::thread kanalDetectThread(kanalDetect, canny, std::ref(kanalCircles), 240, 320);
        std::thread pinponDetectThread(pinponDetect, canny, std::ref(pinponCircle), 240, 320);
        kanalDetectThread.join();
        pinponDetectThread.join();
        if (pinponCircle.radius<1)
        {
            cout<<"top yok"<<endl;
            motorSag.setSpeed(0);
            motorSol.setSpeed(0);
            continue;
        }

        if (kanalCircles.size()==0)
        {
            cout<<"kanal yok"<<endl;
            motorSag.setSpeed(0);
            motorSol.setSpeed(0);
            continue;
        }




        double scaleFactor=PINPON_RADIUS/pinponCircle.radius;
        double minDist=999.9;
        double dist1;
        for (vector<Circle>::iterator it = kanalCircles.begin(); it!=kanalCircles.end(); it++)
        {
            dist1=dist(*it, pinponCircle);
            if (dist1<minDist) {minDist=dist1;}

        }
        double kanalToBarge=(minDist-pinponCircle.radius)*scaleFactor;


        minDist=999.9;
        Circle closest;
        for (vector<Circle>::iterator it = kanalCircles.begin(); it!=kanalCircles.end(); it++)
        {
            dist1=240-it->center.y;
            if (dist1<minDist)
            {
                minDist=dist1;
                closest=*it;
            }
            if (it->center.x>(0.30*320)){break;}
        }

        drawCircles(image, pinponCircle);
        drawCircles(image, kanalCircles);
        drawCircles(image, closest ,Scalar(255,255,255));
        double kanalToRobot=minDist;



        cout<< "Kanal to barge= "<<kanalToBarge<<" cm."<<endl;
        cout<< "Kanal to robot= "<<kanalToRobot<<" cm."<<endl;
        double errK2R=kanalToRobot-HEDEF_KANAL2ROBOT;
        double pK2R=errK2R;
        double dK2R=errK2R-lastErrK2R;
               iK2R=iK2R+lastErrK2R;
               lastErrK2R=errK2R;

        //double errCenter=pinponCircle.center.x-HEDEF_CENTER;
        //double pCenter=errCenter;
       // double dCenter=errCenter-lastErrCenter;
               //iCenter=iCenter+lastErrCenter;
               //=errCenter;

        double errK2B=kanalToBarge-HEDEF_KANAL2BARGE;
        double pK2B=errK2B;
        double dK2B=errK2B-lastErrK2B;
               iK2B=iK2B+lastErrK2B;
               lastErrK2B=errK2B;

        double speed;
//        speed=baseSpeed-(KP_center*pCenter+KD_center*dCenter+KI_center*iCenter);

        if (pinponCircle.center.x<100)
        {
            speed=40;
        }
        else
        {
            speed=baseSpeed+(KP_K2B*pK2B+KD_K2B*dK2B+KI_K2B*iK2B);
        }

        double speedSol=speed+speed*(KP_K2R*pK2R+KD_K2R*dK2R+KI_K2R*iK2R);
        double speedSag=speed-speed*(KP_K2R*pK2R+KD_K2R*dK2R+KI_K2R*iK2R);


        motorSag.setSpeed(speedSag);
        motorSol.setSpeed(speedSol);
        cout<<speedSag<<"% duty cycle sag"<<endl;
        cout<<speedSol<<"% duty cycle sol"<<endl;

        show("result", image);

        clock_gettime(CLOCK_MONOTONIC, &finish);

        elapsed = (finish.tv_sec - start.tv_sec);
        elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
        cout<<"main fps="<<(int)(1.0/elapsed)<<endl;

    }


    cap.release();
    return 0;
    }
















