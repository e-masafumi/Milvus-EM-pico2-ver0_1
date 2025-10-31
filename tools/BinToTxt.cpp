#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

struct sensorsData{
	uint32_t timeBuff_32=0;
	double outTemp=0.0;
	double outPress=0.0;
	double xAccel=0.0;
	double yAccel=0.0;
	double zAccel=0.0;
	double xMag=0.0;
	double yMag=0.0;
	double zMag=0.0;
};

struct sensorsData logData;

int main(void){

    FILE *file;

//    file = fopen("test.dat", "rb");
 //   fread(&logData, sizeof(logData), 1, file);
 //   fclose(file);

    printf("uint32_t: %lu\n", sizeof(logData.timeBuff_32));
    printf("double: %lu\n", sizeof(logData.outTemp));
    printf("logData: %lu\n", sizeof(logData));

    return 0;
}
