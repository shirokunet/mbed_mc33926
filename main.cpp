#include "mbed.h"
#include "DualMC33926MotorShield.h"

Serial pc(USBTX, USBRX, 115200);
DualMC33926MotorShield md(D7, D9, A0, D8, D10, A1, D4, D12);

int main() 
{
    md.setM1Speed(400);
    while(1)
    {
        pc.printf("%3.6f\r\n", md.getM1CurrentMilliamps());
        wait_ms(10);
    }
}
