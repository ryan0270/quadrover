#include <fstream>
#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

int main(int argv, char* argc[])
{
	cout << "start chadding" << endl;

	string imgDir = "../images";

	int keypress = 0;
	namedWindow("chad",1);
	moveWindow("chad",0,0);
	int imgId = 200;
	stringstream ss;
	Mat img;
	while(keypress != (int)'q')
	{
		ss.str("");
		ss << "img_" << imgId++ << ".bmp";
		img = imread(imgDir+"/"+ss.str());
		cout << imgDir << "/" << ss.str();
		if(img.data != NULL)
			imshow("chad",img);
		else
			cout << "*";
		cout << endl;
		keypress = cv::waitKey() % 256;
	}

    return 0;
}


