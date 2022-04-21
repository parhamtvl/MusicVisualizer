// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Small example how to use the library.
// For more examples, look at demo-main.cc
//
// This code is public domain
// (but note, that the led-matrix library this depends on is GPL v2)

#include "led-matrix.h"
#include "threaded-canvas-manipulator.h"
#include <bits/stdc++.h>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <signal.h>
#include <iostream>
#include <string>
#include <sstream>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>


using rgb_matrix::GPIO;
using rgb_matrix::RGBMatrix;
using rgb_matrix::Canvas;
using namespace std;
void HSVtoRGB(float H, float S,float V, float &R, float &G, float &B);

int serial_port = 0;
volatile bool interrupt_received = false;

static void InterruptHandler(int signo) {
  interrupt_received = true;
}

string buffer;
double values[32];

int set_interface_attribs (int fd, int speed)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

         tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
         tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
          tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
          tty.c_cflag |= CS8; // 8 bits per byte (most common)
          tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
          tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

          //tty.c_lflag &= ~ICANON;
          tty.c_lflag |= ICANON;
          tty.c_lflag &= ~ECHO; // Disable echo
          tty.c_lflag &= ~ECHOE; // Disable erasure
          tty.c_lflag &= ~ECHONL; // Disable new-line echo
          tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
          tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
          tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

          tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
          tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
          // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
          // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

          tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
          tty.c_cc[VMIN] = 0;



        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}


string getSingleLine()
{
    // clear the buffer
    /*for(int i=0; i<1; ++i)
    {
        // read small chunk
        char readbuffer[512] = {0};
        int length = read(serial_port, &readbuffer, sizeof(readbuffer)-1);
        if (length <= 0)
            break;
    }*/

    buffer = "";
    while(interrupt_received == false)
    {
        // read small chunk
        char readbuffer[512] = {0};
        int length = read(serial_port, &readbuffer, sizeof(readbuffer)-1);
        if (length > 0)
        {
          // append to the main buffer
          //printf("[%s]", readbuffer);
          //fflush(stdout);
          buffer += readbuffer;

          // scan for newline
          size_t newLinePos = buffer.find("\n");
          if (newLinePos != string::npos)
          {
              string firstLine = buffer.substr(0, newLinePos);
              string remaining = buffer.substr(newLinePos + 1);
              buffer = remaining;
              return firstLine;
          }
        }
        else if (length < 0) // reset the port
        {
            printf("%d error", length);
            fflush(stdout);
        }
        //usleep(10 * 1000);
    }
    exit(0);
}

void DrawLine(Canvas *canvas, int x1, int y1, int x2, int y2, int r, int g, int b)
{

    float fn, gn;
    int bigN = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    for(int i=0; i<bigN; i++)
    {
        fn = x1 + ((x2-x1)*1.0/bigN)*i;
        gn = y1 + ((y2-y1)*1.0/bigN)*i;
        canvas->SetPixel(fn, gn, r, g, b);
    }

}




void DrawOnCanvas(Canvas *canvas)
{
    char patterns;
    int n1;

    printf("Enter a number corresponding to a pattern:\n"
            "1. Spectrum Analyzer\n"
            "2. Rotating Spectrum\n"
            "3. Butterfly Effect\n"
            "4. Exit\n");
    scanf("%d", &n1);


    serial_port = open("/dev/ttyS0", O_RDWR);
    set_interface_attribs(serial_port, 115200);
    int r = 0, g = 0, b = 0;
    float t = 0;

    while(interrupt_received == false)
    {
        canvas->Fill(0, 0, 0);
        string line = getSingleLine();

        // line parsing
        printf("[%s]\n", line.c_str());
        fflush(stdout);

        t = t + 0.05;

        // parse the numbers into our "values" array
        stringstream ss(line);
        for(int i=0; i<32; ++i)
        {
            ss >> values[i];
            if (ss.peek() == ',')
                ss.ignore();
        }
        /*switch(patterns)
        {
            case 1:


        }*/
        for(int i=0; i<31; ++i)
        {
            // truncating cast
            int value = 5*log(values[i]);
            int capped = std::min(31, value);
            int com;

            int nxt = 5*log(values[i+1]);
            int nxtcap = std::min(31, nxt);

            // linear interpolatio
            float R=0,G=0,B=0;
            HSVtoRGB((360/33)*i, 90, std::min(100, 3*value), R, G, B);
            for (int z = 0; z < 10; ++z)
            {
                float bet = capped + (nxtcap - capped)*z / 10.0;
                //com = capped + bet;
                int x1t = 16;
                int y1t = 16;
                int x2t = x1t + bet*cos(2*3.1416*(i+z/10.0)/32);
                int y2t = y1t + bet*sin(2*3.1416*(i+z/10.0)/32);
                DrawLine(canvas, x1t, y1t, x2t, y2t, R, G, B);
            }
            //printf("%f %f %f\n", R,G,B);

            //DrawLine(canvas, i, 0, i, capped, R, G, B);

            /*int x1t = 16 + cos(t-1.57)*(i/3.0);
            int y1t = 16 + sin(t-1.57)*(0/3.0);
            int x2t = 16 + cos(t)*(i/3.0);
            int y2t = 16 + sin(t)*(capped/3.0);*/

            /*int x1t = 16 + cos(t-1.57)*(i/2.0);
            int y1t = 16 + sin(t-1.57)*(i/2.0);
            int x2t = x1t + cos(t)*(capped/2.0);
            int y2t = y1t + sin(t)*(capped/2.0);*/

            /*int x1t = 16;
            int y1t = 16;
            int x2t = x1t + capped*cos(2*3.1416*i/320);
            int y2t = y1t + capped*sin(2*3.1416*i/320);*/

            /*for )
            {
                canvas->SetPixel(z, 6, R, G, B);
            }*/


            //printf("%d\n%d\n%d\n%d\n", x1t, y1t, x2t, y2t);
             /*float r;
             for(int i=0; i<10;i++)
             {
                 r = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
             canvas->SetPixel(fn, gn, r, g, b);
             } */
            //DrawLine(canvas, x1t, y1t, x2t, y2t, R, G, B);
             //DrawLine(canvas, 16, 16, x2t, y2t, R, G, B);


            /*for(int j=0; j<capped; ++j)
            {
                canvas->SetPixel(i, j, R, G, B);
            }*/

        }



        //usleep(50 * 1000);

        // output to matrix
        usleep(20 * 1000);
    }



  /*int center_x = canvas->width() / 2;
  int center_y = canvas->height() / 2;
  float radius_max = canvas->width() / 2;
  float angle_step = 1.0 / 360;
  for (float a = 0, r = 0; r < radius_max; a += angle_step, r += angle_step)
  {
    if (interrupt_received)
      return;
    float dot_x = cos(a * 2 * M_PI) * r;
    float dot_y = sin(a * 2 * M_PI) * r;
    canvas->SetPixel(center_x + dot_x, center_y + dot_y, 255, 0, 0);
    usleep(1 * 1000);  // wait a little to slow down things.
  }*/
}

void HSVtoRGB(float H, float S,float V, float &R, float &G, float &B){
    if(H>360 || H<0 || S>100 || S<0 || V>100 || V<0){
        return;
    }
    float s = S/100;
    float v = V/100;
    float C = s*v;
    float X = C*(1-abs(fmod(H/60.0, 2)-1));
    float m = v-C;
    float r,g,b;
    if(H >= 0 && H < 60){
        r = C,g = X,b = 0;
    }
    else if(H >= 60 && H < 120){
        r = X,g = C,b = 0;
    }
    else if(H >= 120 && H < 180){
        r = 0,g = C,b = X;
    }
    else if(H >= 180 && H < 240){
        r = 0,g = X,b = C;
    }
    else if(H >= 240 && H < 300){
        r = X,g = 0,b = C;
    }
    else{
        r = C,g = 0,b = X;
    }

    R = (r+m)*255;
    G = (g+m)*255;
    B = (b+m)*255;

}
//while(serial_port > 0)
//{
//   read(serial_port, &data, sizeof(data));
//    //printf("%s", data);
//}

int main(int argc, char *argv[])
{
  RGBMatrix::Options defaults;
  defaults.hardware_mapping = "adafruit-hat";  // or e.g. "adafruit-hat"
  defaults.rows = 32;
  defaults.chain_length = 1;
  //defaults.brightness = 50;
  defaults.parallel = 1;
  defaults.show_refresh_rate = false;
  Canvas *canvas = rgb_matrix::CreateMatrixFromFlags(&argc, &argv, &defaults);
  if (canvas == NULL)
    return 1;

  // It is always good to set up a signal handler to cleanly exit when we
  // receive a CTRL-C for instance. The DrawOnCanvas() routine is looking
  // for that.
  signal(SIGTERM, InterruptHandler);
  signal(SIGINT, InterruptHandler);
  DrawOnCanvas(canvas);    // Using the canvas.

  // Animation finished. Shut down the RGB matrix.
  canvas->Clear();
  delete canvas;
  close(serial_port);
  return 0;
}
