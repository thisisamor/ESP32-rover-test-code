#include <math.h>
#include <iostream>

double range(double angle)
{
    if ( angle < 0 )
    {
        angle += 360; 
    }
}

int main()
{
    double x, y; 
    std::cout << "current position: "; 
    std::cin >> x >> y; 
    double angle; 
    std::cout << "current angle: "; 
    std::cin >> angle; 
    double Y = range(atan(-y/(3600.0+x))*180*M_1_PI); 
    double B = range(-atan((2400.0+y)/(3600.0-x))*180*M_1_PI); 
    double R = range(-90.0-atan(x/(2400.0+y))*180*M_1_PI); 
    if ( angle >= Y )
    {
        std::cout << "angle_Y = " << angle-Y << std::endl; 
        std::cout << "angle_B = " << angle-B << std::endl;
        std::cout << "angle_R = " << angle-R << std::endl; 
    }
    else if ( angle >= B )
    {
        std::cout << "angle_Y = " << 360-angle-Y << std::endl; 
        std::cout << "angle_B = " << angle-B << std::endl;
        std::cout << "angle_R = " << angle-R << std::endl;    
    }
    else if ( angle >= R )
    {
        std::cout << "angle_Y = " << 360+angle-Y << std::endl; 
        std::cout << "angle_B = " << 360+angle-B << std::endl;
        std::cout << "angle_R = " << angle-R << std::endl;    
    }
    else
    {
        std::cout << "angle_Y = " << 360+angle-Y << std::endl; 
        std::cout << "angle_B = " << 360+angle-B << std::endl;
        std::cout << "angle_R = " << 360+angle-R << std::endl;    
    }
}