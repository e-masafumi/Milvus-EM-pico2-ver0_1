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
//      printf("id = %d\n", decodedULSA.id);
    } 
		else if (index == 1) {
			decodedULSA.active = atoi(token);
//      printf("active = %d\n", decodedULSA.active);
    } 
		else if (index == 2) {
      decodedULSA.direction = atoi(token);
//      printf("direction = %d\n", decodedULSA.direction);
    } 
		else if (index == 3) {
      decodedULSA.absoluteSpeed = atof(token);
//      printf("absoluteSpeed = %f\n", decodedULSA.absoluteSpeed);
		}
		else if (index == 4) {
      decodedULSA.noseSpeed = atof(token);
//      printf("noseSpeed = %f\n", decodedULSA.noseSpeed);
		}
		else if (index == 5) {
      decodedULSA.soundSpeed = atof(token);
//      printf("soundSpeed = %f\n", decodedULSA.soundSpeed);
		}
		else if (index == 6) {
      decodedULSA.virtualTemp = atof(token);
//      printf("virtualTemp = %f\n", decodedULSA.virtualTemp);
		}

    index++;
    token = strtok(nullptr, ",");  // 次のカンマ区切りへ
  }
}

bool parseNMEA(const char* line){
	if(!line){		//空データならfalse返す
		return false;
	}

	while(*line == ' ' || *line == '\t'){ //空白があったらスキップ
		++line;
	}

	if (*line != '$'){	//先頭が$じゃなかったらfalse返す
		return false;
	}

	const char* p = line + 1;	//$の次から読む
	uint8_t sum = 0;	//XORの積算
	const char* asterisk = nullptr;	//アスタリスク判別用

	for (; *p; ++p) {	//アスタリスクの前までを順にXORしていく
    if (*p == '*'){ 
			asterisk = p;
			break; 
		}
    sum ^= static_cast<uint8_t>(*p);
  }

	if (!asterisk){	//アスタリスクがない(ぬるぽのまま)なら
		return false;
	}	
	
  auto hexval = [](char c) -> int {	//Oct.→Dec.変換用のラムダ式
		if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
  };

	int hi = hexval(asterisk[1]);	//チェックサム上位1桁
	int lo = hexval(asterisk[2]);	//チェックサム下位1桁
	if (hi < 0 || lo < 0) return false;	//どちらかが異常な値ならfalse返す

	uint8_t recv = static_cast<uint8_t>((hi << 4) | lo);	//チェックサムをまとめる
  if (recv != sum) return false;	//計算した値と違ったらfalseを返す

  const char* tail = asterisk + 3;	//
  while (*tail == '\r' || *tail == '\n') ++tail;
  return (*tail == '\0');
}
