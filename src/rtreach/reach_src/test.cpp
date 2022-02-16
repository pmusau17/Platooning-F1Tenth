#include <iostream>

extern "C"
{
     double getSimulatedSafeTime(double start[4],double heading_input,double throttle);
}

int main(void)
{
    
    
    double startState[4] = {0.0, 0.0, 0.0, 0.0};
    double control_input[2] = {0.0,0.0};

    double delta = control_input[1];
    double u = control_input[0];

    getSimulatedSafeTime(startState,delta,u);

    return 0;
}