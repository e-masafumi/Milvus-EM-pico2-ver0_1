#include "func-parsing.hpp"
#include <cstring>
#include <cstdio>
#include <cstdlib>

struct str_ULSA decodedULSA;

void parseCsvULSA(const char* line) {
  // もし '#' で始まっていたらスキップするだけ
	if (line[0] == '#') line++;

  // 一時バッファを作る（strtokは元の文字列を書き換えるのでコピーする）
  char buf[256];
  strncpy(buf, line, sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';

  char* token = strtok(buf, ",");
  int   index = 0;

  while (token != nullptr) {
		if (index == 0) {
			decodedULSA.id = atoi(token);
      printf("id = %d\n", decodedULSA.id);
    } 
		else if (index == 1) {
			decodedULSA.active = atoi(token);
      printf("active = %d\n", decodedULSA.active);
    } 
		else if (index == 2) {
      decodedULSA.direction = atoi(token);
      printf("direction = %d\n", decodedULSA.direction);
    } 
		else if (index == 3) {
      decodedULSA.absoluteSpeed = atof(token);
      printf("absoluteSpeed = %f\n", decodedULSA.absoluteSpeed);
		}
		else if (index == 4) {
      decodedULSA.noseSpeed = atof(token);
      printf("noseSpeed = %f\n", decodedULSA.noseSpeed);
		}
		else if (index == 5) {
      decodedULSA.soundSpeed = atof(token);
      printf("soundSpeed = %f\n", decodedULSA.soundSpeed);
		}
		else if (index == 6) {
      decodedULSA.virtualTemp = atof(token);
      printf("virtualTemp = %f\n", decodedULSA.virtualTemp);
		}

    index++;
    token = strtok(nullptr, ",");  // 次のカンマ区切りへ
  }
}
