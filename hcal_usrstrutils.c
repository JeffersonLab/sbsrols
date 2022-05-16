#ifndef _USRSTRUTILS_INCLUDED
#define _USRSTRUTILS_INCLUDED
#include <ctype.h>
/*#include <limits.h>*/
#define LONG_MAX 0x7FFFFFFF
#define MAX_CONFIG_STRING 1024
#define COMMENT_CHAR ';'

#define _USRSTRUTILS_DEBUG

/* usrstrutils

   utilities to extract information from configuration string passed
   to ROC in the *.config file in rcDatabase.

   The config line can be of the form

   keyword[=value][,keyword[=value]] ...

   CRL code can use the following three routines to look for keywords and
   the associated values.

   int getflag(char *s) - Return 0 if s not present as a keyword
                                 1 if keyword is present with no value
				 2 if keyword is present with a value
   int getint(char *s) - If keyword present, interpret value as an integer.
                         Value assumed deximal, unless preceeded by 0x for hex
			 Return 0 if keyword not present or has no value.

   char *getstr(char *s) - Return ptr to string value associated with
                           the keyword.  Return null if keyword not present.
			   return null string if keyword has no value.
			   Caller must free the string.

  2020-03-05 Juan Carlos Cornejo
  - Allow multiple lines and comments apply only per line

*/
/* Define some common keywords as symbols, so we have just one place to
   change them*/
/*#define FLAG_FILE "file.flags"*/
#define FLAG_FILE "ffile"
#define COMMENT_CHAR ';'


#ifndef INTERNAL_FLAGS
#define INTERNAL_FLAGS ""
#endif

char *internal_configusrstr=0;
char *file_configusrstr=0;

/* For internal use. Returns ptr to keyword and ptr to value */
void getflagpos(char *s,char **pos_ret,char **val_ret);
int getflag(char *s)
{
  char *pos,*val;

  getflagpos(s,&pos,&val);
  if(!pos) return(0);
  if(!val) return(1);
  return(2);
}
char *getstr(char *s){
  char *pos,*val;
  char *end;
  char *ret;
  int slen;

  getflagpos(s,&pos,&val);
  if(!val){
    return(0);
  }
  end = strchr(val,',');	/* value string ends at next keyword */
  if(end)
    slen = end - val;
  else				/* No more keywords, value is rest of string */
    slen = strlen(val);

  ret = (char *) malloc(slen+1);
  strncpy(ret,val,slen);
  ret[slen] = '\0';
  return(ret);
}
unsigned int getint(char *s)
{
  char *sval;
  int retval;
  sval = getstr(s);
  if(!sval) return(0);		/* Just return zero if no value string */
  retval = strtol(sval,0,0);
  if(retval == LONG_MAX && (sval[1]=='x' || sval[1]=='X')) {/* Probably hex */
     sscanf(sval,"%x",&retval);
   }
  free(sval);
  return(retval);
}
void getflagpos_instring(char *constr, char *s,char **pos_ret,char **val_ret)
{
  int slen;
  char *pos,*val;

  slen=strlen(s);
  pos = constr;
  while(pos) {
    pos = strstr(pos,s);
    if(pos) {			/* Make sure it is really the keyword */
      /* Make sure it is an isolated keyword */
      if((pos != constr && pos[-1] != ',') ||
	 (pos[slen] != '=' && pos[slen] != ',' && pos[slen] != '\0')) {
	pos += 1;	continue;
      } else break;		/* It's good */
    }
  }
  *pos_ret = pos;
  if(pos) {
    if(pos[slen] == '=') {
      *val_ret = pos + slen + 1;
    } else
      *val_ret = 0;
  } else
    *val_ret = 0;
  return;
}

void getflagpos(char *s,char **pos_ret,char **val_ret)
{
  /* Regexp would be nice
     Look for string s in file_configusrstr, config.usrString, and then
     internal_configusrstr, using the first file it is found in.

     s must occur after a ",", or
     at the start of config.usrString.  s must be followed by "," or "=" or
     null.  (No spaces are allowed in config strings.)
     */
  getflagpos_instring(file_configusrstr,s,pos_ret,val_ret);
  if(*pos_ret) return;
  getflagpos_instring(rol->usrString,s,pos_ret,val_ret);
  if(*pos_ret) return;
  getflagpos_instring(internal_configusrstr,s,pos_ret,val_ret);
  return;
}

void init_strings()
     /* Load/reload config line from user flag file. */
{
  char *ffile_name;
  FILE *fd;
  char s[MAX_CONFIG_STRING];
  char *confStr0;
  char confLn[MAX_CONFIG_STRING];
  int  il = 0;

  if(!internal_configusrstr) {	/* Internal flags not loaded */
    internal_configusrstr = (char *) malloc(strlen(INTERNAL_FLAGS)+1);
    strcpy(internal_configusrstr,INTERNAL_FLAGS);
  }
#ifdef _USRSTRUTILS_DEBUG
  printf("Internal Config: %s\n",internal_configusrstr);
  printf("rcDatabase Conf: %s\n",rol->usrString);
#endif

  ffile_name = getstr(FLAG_FILE);
  printf("About to try to open  %s\n",FLAG_FILE);
/* check that filename exists */
  fd = fopen(ffile_name,"r");
  if(!fd) {
#ifdef _USRSTRUTILS_DEBUG
    printf("Failed to open usr flag file %s\n",ffile_name);
#endif
    free(ffile_name);
    if(file_configusrstr) free(file_configusrstr); /* Remove old line */
    file_configusrstr = (char *) malloc(1);
    file_configusrstr[0] = '\0';
  } else {
    /* Read till an uncommented line is found */
    if(file_configusrstr) free(file_configusrstr); /* Remove old line */
    file_configusrstr = 0;
    while(fgets(confLn,MAX_CONFIG_STRING-1,fd)){
      char *arg = confLn;
      /* Get rid of any whitespace */
      il=0;
      s[0] = '\0';
      while(*arg) {
        if(!isspace(*arg)) {
          s[il++] = *arg;
          s[il] = '\0';
        }
        arg++;
      }
      arg = strchr(s,COMMENT_CHAR);
      if(arg) *arg = '\0'; /* Blow away comments */
      arg = s;
      /* Copy any remaining text to the config string */
      if(strlen(arg)>0) {
        if(!file_configusrstr) {
          file_configusrstr = (char*)malloc(strlen(arg)+1);
          strcpy(file_configusrstr,arg);
        } else {
          confStr0 = &(file_configusrstr[0]);
          file_configusrstr = (char*)malloc(strlen(confStr0)+strlen(arg)+1);
          strcpy(file_configusrstr,confStr0);
          strcpy(&file_configusrstr[strlen(confStr0)],arg);
          free(confStr0);
        }
      }
    }
    if(!file_configusrstr) {
      file_configusrstr = (char *) malloc(1);
      file_configusrstr[0] = '\0';
    }
    fclose(fd);
    free(ffile_name);
  }
#ifdef _USRSTRUTILS_DEBUG
  // daLogMsg("Run time Config: %s\n",file_configusrstr);
#endif
}

#endif /* _USRSTRUTILS_INCLUDED */
