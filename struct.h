#ifndef FUNC_STRUCT_H
#define FUNC_STRUCT_H

#include <cstdint>

struct str_sensorsData{
	uint64_t timeBuff_64=0;
	double mainVol=0.0;
	double mainCur=0.0;
	double mainPow=0.0;
	double outTemp=0.0;
	double outPress=0.0;
	double xAccel=0.0;
	double yAccel=0.0;
	double zAccel=0.0;
	double xMag=0.0;
	double yMag=0.0;
	double zMag=0.0;
};

struct str_NMEA_GGA{
	char type[6];
	int hours;
	int minutes;
	double seconds;
	double time;
	double latitude_DM;
	double latitude_D;
	char nOrS;
	double longitude_DM;
	double longitude_D;
	char eOrW;
	int qual;
	int sats;
	double hdop;
	double altitudeASL;
	char altitudeASL_Unit;
	double altitudeGeoid;
	char altitudeGeoid_Unit;
	double age;
	int id;
	char checkSum[3];
};

struct str_ULSA{
	int id;
	int active;
	int direction;
	double absoluteSpeed;
	double noseSpeed;
	double soundSpeed;
	double virtualTemp;
};
#endif
