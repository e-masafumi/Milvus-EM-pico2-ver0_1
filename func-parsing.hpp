#ifndef PARSE_CSV_ULSA_HPP
#define PARSE_CSV_ULSA_HPP

#include "struct.h"

// 関数プロトタイプ宣言
void parseCsvULSA(const char* line, str_ULSA& ulsabuf);
bool checkNMEA(const char* line);
bool parseNMEA_GGA(const char* line, str_NMEA_GGA& ggabuf);

#endif // PARSE_CSV_ULSA_HPP

