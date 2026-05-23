#ifndef DepthImage_h
#define DepthImage_h

#include <iostream>
#include <fstream>
#include <cmath>
using namespace std;

class DepthImage
{
private:
    int width, height;
    float* data;

public:
    DepthImage(int width, int height) : width(width), height(height)
    {
        data = new float[width * height];
    }

    void setPixel(int x, int y, float depth)
    {
        data[y * width + x] = depth;
    }

    void writeRaw(const char* path)
    {
        ofstream file(path, ios::binary);
        file.write((char*)data, sizeof(float) * width * height);
        file.close();
    }
};

#endif
