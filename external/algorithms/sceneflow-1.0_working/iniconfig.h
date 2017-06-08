#ifndef __INICONFIG_H
#define __INICONFIG_H

extern int PrintConfig(FILE *f);
extern int ReadConfig(char *fname);

#define INI_SEC(section,comment)
#define INI_STR(name,default,comment) extern char *name
#define INI_INT(name,default,comment) extern int name
#define INI_DBL(name,default,comment) extern double name
#include "inifile.h"
#undef INI_SEC
#undef INI_STR
#undef INI_INT
#undef INI_DBL

#endif
