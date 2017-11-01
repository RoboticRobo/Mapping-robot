#include <iostream>
#include <math.h>

using namespace std;

#define PI 3.14159265

double end_world_x = 0;
double end_world_y = 0;

double roundDown(double value)
{
    return value < 0.0001 && value > -0.0001 ? 0 : value;
}

void convert_to_world_frame(double posx, double posy, double angle, double walkFront, double walkSide)
{
    end_world_x = roundDown(posx + (walkFront * sin(angle) + walkSide * cos(PI + angle)));
    end_world_y = roundDown(posy + (walkFront * cos(angle) + walkSide * sin(PI + angle)));
}
int main()
{
    /* moveX means robot move forward
	moveY means robot turn positive for left, negative for right */
    double currentPositionX, currentPositionY, angle, moveX, moveY;
    cin >> currentPositionX >> currentPositionY >> angle >> moveX >> moveY;
    angle = angle * PI / 180;
    convert_to_world_frame(currentPositionX, currentPositionY, angle, moveX, moveY);
    cout << "y: " << end_world_y << endl;
    cout << "x: " << end_world_x << endl;
    return 0;
}
