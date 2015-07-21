#pragma once

#include <opencv2\opencv.hpp>
#include <stdio.h>

using namespace std;
using namespace cv;

inline Scalar COLOR(int ID) 
{
	int a = 255-(5*ID)%256;
	int b = 255-(57*ID)%256;
	int c = 255-(1035*ID)%256;
	return Scalar(a,b,c);
}

#define OVERLAP 0

inline double getRectDist(Rect r1, Rect r2, int type)
{
	Rect op = r1 & r2; //Lấy phần giống nhau
	//Tính tỷ lên với phần khác nhau
	return 1 - (double)op.area()/(double)(r1.area() + r2.area() - op.area());
}

inline int string2int(const char* s)
{
	int i;
	if(sscanf(s, "%d", &i) == EOF)
	{
		cout << "Error reading integer" << endl;
	}

	return i;
}

inline float string2float(const char* s)
{
	return (float)atof(s);
}

inline double _string2double(const string s)
{
	return atof(s.c_str());
}

inline string _double2string(double d)
{
	ostringstream s;
	s << d;
	return s.str();
}

inline int _char_p2int(const char* s)
{
	return atoi(s);
}

inline Rect scaleWin(Rect win, double scale)
{
	return Rect(
		(int)(win.x + 0.5*(1-scale)*win.width),
		(int)(win.y + 0.5*(1-scale)*win.height),
		(int)(win.width*scale),
		(int)(win.height*scale));
}