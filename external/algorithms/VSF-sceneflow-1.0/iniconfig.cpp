#include <string.h>
#include "iniparser.h"
#include "iniconfig.h"
#ifdef _MSC_VER
#define snprintf _snprintf
#endif

#define INI_SEC(section,comment)
#define INI_STR(name,default,comment) char *name = default
#define INI_INT(name,default,comment) int name = default
#define INI_DBL(name,default,comment) double name = default
#include "inifile.h"
#undef INI_SEC
#undef INI_STR
#undef INI_INT
#undef INI_DBL

int ReadConfig(char *fname)
{
    dictionary *d;
    char path[1024];
    char sec[1024];
    
    if (fname == NULL) {
#define INI_SEC(section,comment) {}
#define INI_STR(variable,default,comment) { const char *d = default; if (d != NULL) variable = strdup(d); else variable = strdup("(null)");}
#define INI_INT(variable,default,comment) { variable = default; }
#define INI_DBL(variable,default,comment) { variable = default; }
#include "inifile.h"
#undef INI_SEC
#undef INI_STR
#undef INI_INT
#undef INI_DBL
        return -1;
    }
    d = iniparser_load(fname);
    if (d == NULL)
        return 1;
#define INI_SEC(section,comment) { strncpy(sec,#section,sizeof(sec)); }
#define INI_STR(variable,default,comment) { snprintf(path, sizeof(path), "%s:%s", sec, #variable); variable = iniparser_getstring(d, path, default); if (variable != NULL) variable = strdup(variable); }
#define INI_INT(variable,default,comment) { snprintf(path, sizeof(path), "%s:%s", sec, #variable); variable = iniparser_getint(d, path, default); }
#define INI_DBL(variable,default,comment) { snprintf(path, sizeof(path), "%s:%s", sec, #variable); variable = iniparser_getdouble(d, path, default); }        
#include "inifile.h"
#undef INI_SEC
#undef INI_STR
#undef INI_INT
#undef INI_DBL
    iniparser_freedict(d);
    return 0;
}


int PrintConfig(FILE *f)
{
#define INI_SEC(section,comment) fprintf(f,"\n[%s] ; %s\n", #section, comment)
#define INI_STR(name,default,comment) if (name) fprintf(f,"%s = %s ; %s\n", #name, name, comment)
#define INI_INT(name,default,comment) fprintf(f,"%s = %d ; %s\n", #name, name, comment)
#define INI_DBL(name,default,comment) fprintf(f,"%s = %g ; %s\n", #name, name, comment)
    if (f == NULL)
        return -1;
#include "inifile.h"
    return 0;
#undef INI_SEC
#undef INI_STR
#undef INI_INT
#undef INI_DBL
}
