#ifndef VARIABLE_EXTERN_H
#define VARIABLE_EXTERN_H

//extern std::vector<std::string> splitNMEA;

extern bool messageStartFlag;
extern bool messageFinishFlag;
extern bool uart0DataInFlag;
extern char uart0ReadBuff;
extern char readNMEA[30][15];

#endif
