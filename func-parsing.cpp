#include "func-parsing.hpp"
#include <cstring>
#include <cstdio>
#include <cmath>
#include <cstdlib>


//struct str_ULSA decodedULSA;
//struct str_NMEA decodedNMEA;

void parseCsvULSA(const char* line, str_ULSA& ulsabuf) {
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
			ulsabuf.id = atoi(token);
//      printf("id = %d\n", decodedULSA.id);
    } 
		else if (index == 1) {
			ulsabuf.active = atoi(token);
//      printf("active = %d\n", decodedULSA.active);
    } 
		else if (index == 2) {
      ulsabuf.direction = atoi(token);
//      printf("direction = %d\n", decodedULSA.direction);
    } 
		else if (index == 3) {
      ulsabuf.absoluteSpeed = atof(token);
//      printf("absoluteSpeed = %f\n", decodedULSA.absoluteSpeed);
		}
		else if (index == 4) {
      ulsabuf.noseSpeed = atof(token);
//      printf("noseSpeed = %f\n", decodedULSA.noseSpeed);
		}
		else if (index == 5) {
      ulsabuf.soundSpeed = atof(token);
//      printf("soundSpeed = %f\n", decodedULSA.soundSpeed);
		}
		else if (index == 6) {
      ulsabuf.virtualTemp = atof(token);
//      printf("virtualTemp = %f\n", decodedULSA.virtualTemp);
		}

    index++;
    token = strtok(nullptr, ",");  // 次のカンマ区切りへ
  }
}

bool checkNMEA(const char* line){
	char buf[256];
	
	if(!line || *line == '\0'){		//空データならfalse返す
		return false;
	}

  strncpy(buf, line, sizeof(buf));
	buf[sizeof(buf) - 1] = '\0';	//終端保証
	
	char* p = buf;	//$の次から読む
	
	while(*p == ' ' || *p == '\t'){ //空白があったらスキップ
		++p;
	}

	if (*p != '$'){	//先頭が$じゃなかったらfalse返す
		return false;
	}

	p++;	//$の次から読む
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
	
  auto hexval = [](char c) -> int {	//Hex.→Dec.変換用のラムダ式
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

  const char* tail = asterisk + 3;	//行末の改行コード確認(\nは\0に置換済み)
  while (*tail == '\r' || *tail == '\n') ++tail;
  return (*tail == '\0');
}


bool parseNMEA_GGA(const char* line, str_NMEA_GGA& ggabuf) {
	char buf[256];
  strncpy(buf, line, sizeof(buf));

	char *p_buf = buf;
	char *token = strsep(&p_buf, ",");

//  char* token = strtok(buf, ",");
  int   index = 0;
  
	if (token && strlen(token) >= 6 && strcmp(token + 3, "GGA") == 0) {
		ggabuf.type[0] = token[0];
		ggabuf.type[1] = token[1];
		ggabuf.type[2] = token[2];
		ggabuf.type[3] = token[3];
		ggabuf.type[4] = token[4];
		ggabuf.type[5] = token[5];
		while (token != nullptr) {
			switch(index){
				case 0:
					ggabuf.time = atof(token);
					ggabuf.hours = static_cast<int>(ggabuf.time) / 10000;
					ggabuf.minutes = (static_cast<int>(ggabuf.time) / 100) % 100;
					ggabuf.seconds = fmod(ggabuf.time, 100.0);
					break;
				case 1:
					ggabuf.latitude_DM = atof(token);
					ggabuf.latitude_D = floor(ggabuf.latitude_DM / 100.0) + (ggabuf.latitude_DM - floor(ggabuf.latitude_DM / 100.0) * 100.0) / 60.0;
					break;
				case 2:
					ggabuf.nOrS = *token;
					break;
				case 3:
					ggabuf.longitude_DM = atof(token);
					ggabuf.longitude_D = floor(ggabuf.longitude_DM / 100.0) + (ggabuf.longitude_DM - floor(ggabuf.longitude_DM / 100.0) * 100.0) / 60.0;
					break;
				case 4:
					ggabuf.eOrW = *token;
					break;
				case 5:
					ggabuf.qual = atoi(token);
					break;
				case 6:
					ggabuf.sats = atoi(token);
					break;
				case 7:
					ggabuf.hdop = atof(token);
					break;
				case 8:
					ggabuf.altitudeASL = atof(token);
					break;
				case 9:
					ggabuf.altitudeASL_Unit = *token;
					break;
				case 10:
					ggabuf.altitudeGeoid = atof(token);
					break;
				case 11:
					ggabuf.altitudeGeoid_Unit = *token;
					break;
				case 12:
					ggabuf.age = atof(token);
					break;
				case 13:
					ggabuf.id = atoi(token);
					break;
				case 14:
					ggabuf.checkSum[0] = token[0];
					ggabuf.checkSum[1] = token[1];
					ggabuf.checkSum[2] = token[2];
					break;
				default:
					break;
			}
			
			index++; 
//			token = strtok(nullptr, ",");  // 次のカンマ区切りへ
			token = strsep(&p_buf, ",");  // 次のカンマ区切りへ
		}
	}
	else{
		return false;
	}
	return true;
}
