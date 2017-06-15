#ifndef AYBFONKSIYONVECLASSLAR_INCLUDED
#define AYBFONKSIYONVECLASSLAR_INCLUDED

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <thread>
#include <wiringPi.h>
#include <softPwm.h>
using namespace cv;
using namespace std;

class Circle
{
public:
    Point2f center;
    float radius;
    Circle(Point2f center= Point2f(0,0), float radius=0)
    {
        this->center=center;
        this->radius=radius;
    };

    bool operator<(Circle circle)
    {
        return this->center.x < circle.center.x;
    }
};


 class Webcam
 {
 public:
     VideoCapture stream;
     Mat frame;
     bool stopped;
     bool grabbed;
     std::thread loop;
     Webcam(int source=0, int height=240, int width=320):loop()
     {
         stream=VideoCapture(source);
         stream.set(CV_CAP_PROP_FRAME_HEIGHT,height);
         stream.set(CV_CAP_PROP_FRAME_WIDTH,width);
         stopped=false;
         Mat image;
         grabbed=stream.read(frame);


     }
     ~Webcam()
      {
          stopped=true;
          if (loop.joinable())
          loop.join();
      }
      void start()
      {
          loop=std::thread(&Webcam::update, this);
      }
      void release()
      {
          stopped=true;
          stream.release();
      }
      void read(Mat& frm)
      {
          frame.copyTo(frm);
      }

      void update()
      {
          while(true)
          {
            if (stopped)
                return;
            stream.read(frame);
          }

     }
  };

class Motor
{
public:
    Motor(int pwmPin, int yonPin1, int yonPin2)
    {
        this->pwmPin=pwmPin;
        this->yonPin1=yonPin1;
        this->yonPin2=yonPin2;
        wiringPiSetup ();
        pinMode (yonPin1, OUTPUT); //0
        pinMode (yonPin2, OUTPUT);
        pinMode (pwmPin, OUTPUT);
        digitalWrite (yonPin1, HIGH);
        digitalWrite (yonPin2, LOW);
        softPwmCreate (pwmPin, 0, 100);

    }
    ~Motor()
    {
        digitalWrite (pwmPin, LOW);
    }

    void setSpeed(double hizD)
    {
        int hiz= (int) hizD;

        if (hiz<0)
        {
            speed=0;
        }
        else if (hiz<100)
        {
            speed=hiz;
        }
        else
        {
            speed=100;
        }

        softPwmWrite (pwmPin, speed);
    }
private:
    int pwmPin;
    int yonPin1;
    int yonPin2;
    double speed;
};




#endif // AYBFONKSIYONVECLASSLAR_INCLUDED
