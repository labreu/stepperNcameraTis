#include <string.h>
#include <iostream>
#include <fstream>
#include <wiringPi.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include "v4ldevice.h"
#include <ctime>

//
//  main.cpp
//
//  Created by Lucas Ribeiro de Abreu on 16/07/2015.
//  Copyright (c) 2015 Lucas Ribeiro de Abreu. All rights reserved.
//


#define ID 123
#define ACC 200
#define DELAY 4000
#define VMAX 0.6 ///velocidade maxima = 60% do delay

using namespace std;
using namespace cv;

void digitalWrite(int pin, int x);
void delayMicroseconds(int d);
string timeDateNow();
void handleFiles();
void eot();
int E;
int G;
void onTrackbar_Gchanged(int, void*);
void onTrackbar_Echanged(int, void*);
bool debug = true;
bool lastDirX = false;
bool lastDirY = false;
int order = 3;

int steps[4][4] = {
    {1, 0, 0, 1},
    {1, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 1}};
int pins1[4] = { 6, 13, 19, 26};
int pins2[4] = {12, 16, 20, 21};

class Camera{
    
public:
    int n;
    int wKey;
    IplImage* raw;
    IplImage* colorPhoto;
    char video[20];
    Size ImageSize;
    Size photoSize;
    unsigned char* ImageBuffer;
    char winName[10];
    char photoName[20];
    Mat currentImg;
    
    Camera();
    
    void getLive(){
        
        ImageBuffer = snapFrame();
        if( ImageBuffer != NULL )
        {
            raw->imageData = (char*)ImageBuffer;
            currentImg = Mat(raw);                           	//convert IplImage to Mat
            cvtColor(currentImg, currentImg, CV_BayerGR2RGB);  	//converte do formato raw para RGB
            imshow(winName, currentImg);
            wKey = cvWaitKey(60);
        }
    }
    
    void getPhoto(){
        
        ImageBuffer = snapFrame();
        ImageBuffer = snapFrame();
        ImageBuffer = snapFrame();
        ImageBuffer = snapFrame();     //buggy driver paranoia
        ImageBuffer = snapFrame();
        ImageBuffer = snapFrame();     //solves shutter problem
        ImageBuffer = snapFrame();
        ImageBuffer = snapFrame();
        ImageBuffer = snapFrame();
        if( ImageBuffer != NULL )
        {
            colorPhoto->imageData = (char*)ImageBuffer;
            currentImg = Mat(colorPhoto);
            sprintf(photoName, "raw%d-E%d-G%d.png", n, E, G);
            imwrite(photoName, currentImg);
            n++;
        }
    }
    
    int convertImg(){
        
        printf("Convertendo imagens\n");
        for(int i = 0; i< n ; i++)
        {
            sprintf(photoName, "raw%d-E%d-G%d.png", i, E, G);
            Mat img = imread(photoName, 0); //read gray
            if(img.empty())
                return 0;
            cvtColor(img, img, CV_BayerGR2RGB);
            sprintf(photoName, "%d.png", i);
            imwrite(photoName, img);
        }
        return 1;
    }
    
    void initCamera(int mode){
        delayMicroseconds((unsigned int)100000);
        if (mode==1) {
            open_device((char*)video);
            init_device(ImageSize.width, ImageSize.height);
            start_capturing();
        }
        if (mode==2){
            open_device((char*)video);
            init_device(photoSize.width, photoSize.height);//inicia com resolucao da foto
            start_capturing();
        }
    }
    void stopCamera(){
        stop_capturing();
        uninit_device();
        close_device();
    }
};

Camera::Camera(void)
{
    //camera initial parameters
    system("v4l2-ctl --set-ctrl=gain=12");
    system("v4l2-ctl --set-ctrl=exposure_absolute=1000");
    n=0;
    wKey = -1;
    strcpy(video,"/dev/video0");
    ImageBuffer = NULL;
    strcpy(winName,"Live");
    photoSize.width = 1920;		//resolucao foto
    photoSize.height = 1080;
    ImageSize.width = 640;		//resolucao img live
    ImageSize.height = 480;
    raw = cvCreateImage(ImageSize , IPL_DEPTH_8U, 1 );
    colorPhoto = cvCreateImage(photoSize , IPL_DEPTH_8U, 1 );
    
}

class Motors{
    
public:
    int tdelay ;
    int acceleration;
    int posM1;
    int posM2;
    
    Motors();
    
    void step(int n, int motor){
        bool neg = false;
        unsigned int velocity = tdelay + acceleration;
        if (n < 0) {
            n = - n;
            neg = true;
        }
        if (motor == 1) {
            
            for (int i = 0; i < n; i++) {
                if (i == (n/5)) {
                    acceleration = 0;
                }
                if (i == ((4 * n)/5)) {
                    acceleration = ACC;
                }
                //cout << "M1\n";
                if (!neg) {
                    posM1++;
                    if (posM1 == 4) {
                        posM1 = 0;
                    }
                }
                if (neg) {
                    posM1--;
                    if (posM1 == - 1) {
                        posM1 = 3;
                    }
                }
                velocity = velocity + acceleration;
                if (velocity <= VMAX * DELAY) {
                    velocity = VMAX * DELAY;
                }
                if (velocity >= DELAY) {
                    velocity = DELAY;
                }
                
                digitalWrite(pins1[0], steps[posM1][0]);
                delayMicroseconds(velocity);
                digitalWrite(pins1[1], steps[posM1][1]);
                delayMicroseconds(velocity);
                digitalWrite(pins1[2], steps[posM1][2]);
                delayMicroseconds(velocity);
                digitalWrite(pins1[3], steps[posM1][3]);
                delayMicroseconds(velocity);
            }
            acceleration = - ACC;
        }
        else{//motor 2
            
            for (int i = 0; i < n; i++) {
                if (i == (n /5)) {
                    acceleration = 0;
                }
                if (i == ((4 * n)/5)) {
                    acceleration = ACC;
                }
                //cout << "M2\n";
                if (!neg) {
                    posM2++;
                    if (posM2 == 4) {
                        posM2 = 0;
                    }
                }
                if (neg) {
                    posM2--;
                    if (posM2 == - 1) {
                        posM2 = 3;
                    }
                }
                velocity = velocity + acceleration;
                if (velocity <= VMAX * DELAY) {
                    velocity = VMAX * DELAY;
                }
                if (velocity >= DELAY) {
                    velocity = DELAY;
                }
                
                digitalWrite(pins2[0], steps[posM2][0]);
                delayMicroseconds(velocity);
                digitalWrite(pins2[1], steps[posM2][1]);
                delayMicroseconds(velocity);
                digitalWrite(pins2[2], steps[posM2][2]);
                delayMicroseconds(velocity);
                digitalWrite(pins2[3], steps[posM2][3]);
                delayMicroseconds(velocity);
            }
            acceleration = - ACC;
        }
    }
};

Motors::Motors(void){
    printf("WiringPi %d\n",wiringPiSetupGpio()); //0 = OK
    for (int i =0 ; i< 4; i++) {
        pinMode(pins1[i], OUTPUT);
        pinMode(pins2[i], OUTPUT);
    }
    posM1 = 0;
    posM2 = 0;
    acceleration = - ACC;
    tdelay = DELAY;
    
    cout << "Motores inicializados"<<endl;
}


int main(int argc, const char * argv[]) {
    
    
    char winName[10] = "Live";
    
    unsigned int dl = 200000;
    cout << timeDateNow()<<endl;
    
    //window
    cvNamedWindow(winName, 1);
    createTrackbar("Exposure", winName, &E, 3000, onTrackbar_Echanged);
    createTrackbar("Gain", winName, &G, 63, onTrackbar_Gchanged);
    printf("Press: W-A-S-D to navigate\nPress: ESC to exit\n");
    
    Motors m;
    Camera c;
    c.initCamera(1);
    cout << "Camera inicializada"<<endl;
    
    while (1) {
        c.getLive();
        
        switch (c.wKey) {
            case 103:{
                c.stopCamera();
                c.initCamera(2);
                for(int j=0; j< order; j++){
                    for(int i=0; i< order; i++){
                        c.getPhoto();
                        if(i<order-1){
                            if((j%2) == 0){
                                m.step(25,1);
                                delayMicroseconds(dl);
                                if(debug)
                                    printf("Direita\n");
                            }
                            if((j%2) != 0){
                                m.step(-25,1);
                                delayMicroseconds(dl);
                                if(debug)
                                    printf("Esquerda\n");
                            }
                        }
                        
                    }
                    m.step(25,2);
                    delayMicroseconds(dl);
                    if(debug)
                        printf("Desceu\n");
                    if((j%2)==0){
                        m.step(-25,1);
                        if(debug)
                            printf("Compensou esquerda\n");
                    }
                    if((j%2)!=0){
                        m.step(25,1);
                        if(debug)
                            printf("Compensou direita\n");
                    }
                    
                }
                c.convertImg();
                handleFiles();
                c.stopCamera();
                c.initCamera(1);
            }break;
                
            case 27: {	//esc
                c.stopCamera();
                cvDestroyWindow( winName );
                break;
            }break;
                
            case 100: {//W A S D
                if(lastDirX){
                    m.step(28, 1);
                    if(debug)
                        printf("compensou Right\n");
                }
                lastDirX = false;
                m.step(1, 1);
                if(debug)
                    printf("Right\n");
            }break;
                
            case 97: {
                if(!lastDirX){
                    m.step(-28, 1);
                    if(debug)
                        printf("compensou left\n");
                }
                lastDirX = true;
                m.step(-1, 1);
                if(debug)
                    printf("Left\n");
            }break;
                
            case 115: {
                if(lastDirY){
                    m.step(28, 2);
                    if(debug)
                        printf("compensou back\n");
                }
                lastDirY = false;
                m.step(1, 2);
                if(debug)
                    printf("Back\n");
            }break;
                
            case 119: {
                if(!lastDirY){
                    m.step(-28, 2);
                    if(debug)
                        printf("compensou foward\n");
                }
                lastDirY = true;
                m.step(-1,2);
                if(debug)
                    printf("Foward\n");
            }break;
            default:
                break;
        }
    }
    
    return 0;
}

void handleFiles(){
    
    char cmd[40];
    eot();
    sprintf(cmd, "rm -f *raw*");
    system(cmd);
    sprintf(cmd, "mv *.png //home//pi//sync");
    system(cmd);
    sprintf(cmd, "mv *eot* //home//pi//sync");
    system(cmd);
}

void onTrackbar_Echanged(int, void*){
    
    char cmd[50];
    if(E<1)
        E=1; //exp minimo
    sprintf(cmd, "v4l2-ctl --set-ctrl=exposure_absolute=%d &", E);
    system(cmd);
    usleep(100);
}

void onTrackbar_Gchanged(int, void*){
    
    char cmd[50];
    if(G<4)
        G=4; //ganho minimo
    sprintf(cmd, "v4l2-ctl --set-ctrl=gain=%d &", G);
    system(cmd);
    usleep(100);
}

void eot(){
    ofstream outfile;
    outfile.open("eot.txt");
    outfile << ID << endl;
    outfile << order*order<< endl;
    outfile.close();
}

string timeDateNow(){
    
    time_t now = time(0); // current date/time based on current system
    tm *ltm = localtime(&now);
    string date;
    string time;
    //date = to_string(ltm->tm_mday)+"/"+ to_string(ltm->tm_mon) + "/" + to_string(1900 + ltm->tm_year)+"\n";
    
    int hour = (ltm->tm_hour);
    int min = (1 + ltm->tm_min);
    int sec = (1 + ltm->tm_sec);
    //time = to_string(hour) + ":" + to_string(min) + ":" + to_string( sec)+"\n";
    return date+time;
}
