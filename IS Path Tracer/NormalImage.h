#ifndef NormalImage_h
#define NormalImage_h

#include <fstream>

#include "glm/vec3.hpp"
using namespace std;

class NormalImage {
private:
    int width, height;
    float *data;

public:
    NormalImage(int width, int height) : width(width), height(height) {
        data = new float[3 * width * height];
    }

    void setPixel(int x, int y, glm::vec3 normal) {
        for (int i = 0; i < 3; i++) {
            data[3 * (y * width + x) + i] = normal[i];
        }
    }

    void writeRaw(const char *path) {
        ofstream file(path, ios::binary);
        file.write((char *) data, sizeof(float) * 3 * width * height);
        file.close();
    }
};

#endif
