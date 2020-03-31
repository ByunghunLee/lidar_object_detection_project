#pragma once

#ifndef LIDAR_OBSTACLE_DETECTION_TYPES_H
#define LIDAR_OBSTACLE_DETECTION_TYPES_H
struct Color
{

    float r, g, b;

    Color(float setR, float setG, float setB)
            : r(setR), g(setG), b(setB)
    {}
};

enum CameraAngle
{
    XY, TopDown, Side, FPS
};

struct Vect3
{

    double x, y, z;

    Vect3(double setX, double setY, double setZ)
            : x(setX), y(setY), z(setZ)
    {}

    Vect3 operator+(const Vect3& vec)
    {
        Vect3 result(x+vec.x,y+vec.y,z+vec.z);
        return result;
    }
};


#endif //LIDAR_OBSTACLE_DETECTION_TYPES_H
