
#include <math.h>
#include <time.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <sstream>
#include <opencv2/imgproc/imgproc_c.h>
#include <iostream>
//#include <QString>
//#include <QDir>
//#include <QStringList>
#include <algorithm>


//#include <ariac.h>


#include <ctime>
#include "linenav/kimread.h"
#include "navigation.h"
#include "linenav/dispnav.h"
#include "RobotInterface.h"
#include "ImagesOffline.h"

using namespace std;

void usage(int argc, char** argv){
    std::cout<<"Usage: "<<argv[0]<<" keyframepath "<<"Imgpath " <<std::endl;
    std::cout<<"Usage: "<<argv[0]<<" ../data/offlinetest/kfls ../data/offlinetest/imgs" <<std::endl;
}




int main(int argc, char** argv)
{
    int ret = -1;
    int dispflag = 0;
    if(argc<3){
        usage(argc,argv);
        return ret;
    }
    if(argc>4)
        dispflag = 1;

    /* Read repeat image sequence */
    std::string imdir(argv[2]);
    imdir += "/";

    RobotInterface *robot;
    robot = new ImagesOffline(imdir);
    robot->openCamera(0);


     cv::Mat K = robot->getK();
     std::cout<<K;

     cv::Mat cim;
     robot->getCurrImage(cim);

   /* Read teach reference images */
     kimRead kf(argv[1]);


    /* Setup image display */
    dispNav *d;
    d=new dispNav;
    d->saveimage(true);
    d->displayimage(true);
    d->setDisptime(100);

    /* setup navigation */
    navigation nav(d,K);


    /* Global localisation in Reference Image List */

      nav.initlocalisation(cim,kf);

    /* Perform Succesive Localisation */

     int flag;
     int count = 0;
     int islast;

     /* Robot velocities */
     const double vb = 0.15; //base forward velocity
     const double sp = 0.3; // factor by which forward velocity reduced when turning

     double v = 0.05;   //forward velocity
     double w = 0.0;    //rotational velocity

     std::cout<<"\n ---------------------------------------------"<<std::endl;

     while(1)
     {
      clock_t tStart = clock();

        robot->getCurrImage(cim);

        if(cim.channels()==3)
            cv::cvtColor(cim,cim,cv::COLOR_BGR2GRAY);

        flag = nav.step(cim);
        //std::cout<<flag<<std::endl;

        if(flag==0) count = 0;
        else if(flag == 1) count++;
        else if(flag <0) { v = 0.0; w = 0.0; robot->setBaseVelocities(v,w); continue;}
        else continue;

        w = nav.getRotVel();

        std::cout<<"Rot Vel = "<<w<<std::endl;

        if(fabs(w)>0.08) v = vb*sp;
        else v = vb;

        robot->setBaseVelocities(v,w);

        std::cout<<"---------------------------------------------"<<std::endl;
        if(count>2){
            islast = nav.SwitchtoNewKeyImages(kf);
            if(islast){
                std::cout<<"end of topological navigation"<<std::endl;
                break;
            }
        }
     std::cout<<"\n exec time = "<<((double)(clock() - tStart)/CLOCKS_PER_SEC)<<std::endl;
     std::cout<<"---------------------------------------------"<<std::endl;
     }


}




