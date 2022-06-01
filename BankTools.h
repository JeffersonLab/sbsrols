/* CBOPEN  == OPEN  BANK <bank_name> of <bank_type>  */
/* CBCLOSE == CLOSE BANK <bank_len>  of <bank_type>  */

/* - BankTools.h ---------------------------------------------------------- */
#ifndef _Bank_Tools_h
#define _Bank_Tools_h

/* Event types : */
#define EV_SYNC     0xffd4
#define EV_PRESTART 0xffd1
#define EV_GO       0xffd2
#define EV_PAUSE    0xffd3
#define EV_END      0xffd4


#define EV_BANK_HDR  0x00000100
#define EV_BAD       0x10000000

#define PHYS_BANK_HDR(t,e) (unsigned int)((((t)&0xf)<<16) | \
					     ((e)&0xff) | \
					     EV_BANK_HDR)

#define CTL_BANK_HDR(t) (unsigned int)((((t)&0xffff)<<16) | \
					0x000001CC)
     
#define IS_BANK(b) (((unsigned int) (b) && EV_BANK_HDR)==EV_BANK_HDR)
     
#define DECODE_BANK_HEADER(b,t,e) { t = (b[1]>>16)&0xffff;\
				      e = b[1]&0xff;}    
     
#define EV_BANK_ID 0xc0010100
#define EV_HDR_LEN 4


/* Event type source */
static int EVENT_type;


/* Param : */

#define BT_BANK_ty 0x10
#define BT_SEG_ty  0x20
#define BT_UC1_ty  0x03
#define BT_UB1_ty  0x07
#define BT_UI2_ty  0x05
#define BT_UI4_ty  0x01

#define BC_RUNCONTROL 0xCC
#define CURRENT_TIME time(0)
#define RUN_NUMBER rol->runNumber
#define RUN_TYPE   rol->runType
#define EVENT_NUMBER *(rol->nevents)

#define MAX_DEPTH__ 32 

unsigned int *StartOfEvent[MAX_DEPTH__],event_depth__, *StartOfUEvent;

/* Macro : */
#ifdef VXWORKS
#define LOGIT logMsg
#else
#define LOGIT printf
#endif

#define NEWEVENT {if(__the_event__ == (DANODE *) 0 && rol->dabufp == NULL) \
		    { \
			partGetItem(rol->pool,__the_event__); \
			if(__the_event__ == (DANODE *) 0) { \
			     LOGIT("TRIG ERROR: no pool buffer available\n"); \
                              return; \
			   } \
			rol->dabufp = (unsigned int *) &__the_event__->length; \
			if (input_event__) { \
			  __the_event__->nevent = input_event__->nevent; \
		        } else { \
			  __the_event__->nevent = *(rol->nevents); \
			} \
		    } \
		}

#define COPYEVENT {if(__the_event__ == (DANODE *) 0) \
		     { \
			partGetItem(rol->pool,__the_event__); \
			rol->dabufp = (unsigned int *) &__the_event__->length; \
			if(input_event__) \
			 { \
			     int jj,len; \
			     len = *(rol->dabufpi); \
			     __the_event__->nevent = input_event__->nevent; \
			     for(jj=0;jj<=len;jj++) \
			         *(rol->dabufp)++ = *(rol->dabufpi)++; \
			 } else { \
			     __the_event__->nevent = *(rol->nevents); \
			 } \
		     } \
		 }


#define PASSEVENT {if(input_event__) \
		     { \
			 if (rol->output) { \
			    listAdd(&(rol->output->list),input_event__); \
			 } else {  \
			    partFreeItem(input_event__);\
				 } \
			 input_event__ = (DANODE *) 0; \
		     } \
		 }


#define USEREVENT {if(__user_event__ == (DANODE *) 0) \
		    { \
			partGetItem(rol->pool,__user_event__); \
			if(__user_event__ == (DANODE *) 0) { \
			     LOGIT("TRIG ERROR: no pool buffer available\n"); \
                              return; \
			   } \
			rol->dabufp = (unsigned int *) &__user_event__->length; \
			__user_event__->nevent = -1; \
		    } \
		}


/* - cbopen ---------------------------------------------------------- 
   crl	: open bank <bank_name> of <bank_type>
   - <bank_name> : converted in bnum through Bank_Dic_ID 
   - <bank_type> : UI2 , UI4, UB1 
   
   example	: open bank 0x1234 of UI2 
   
   call	: cbopen (int bnum, int btype); 
   
   Function: 
   open a CODA bank with CODA Bank Header Format 
   leaves (rol->dabufp) pointing to ready next address 
   keep pointer to length of bank in GblTopBp for length adjustment 
   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#define CBOPEN(bnum, btype, code) {\
				     unsigned int *StartOfBank; \
				     StartOfBank = (rol->dabufp); \
				     *((unsigned int *)++(rol->dabufp)) = (((bnum) << 16) | (btype##_ty) << 8) | (code); \
				       ((rol->dabufp))++;
				   
#define CEOPEN(bnum, btype, nev) {		 \
			       NEWEVENT; \
			       StartOfEvent[event_depth__++] = (int *)(rol->dabufp); \
			       if(input_event__) {\
			         *(++(rol->dabufp)) = ((bnum) << 16) | ((btype##_ty) << 8) | (0xff & (input_event__->nevent));\
			       } else {\
			         *(++(rol->dabufp)) = (syncFlag<<28) | ((bnum) << 16) | ((btype##_ty) << 8) | (nev);\
			       }\
			       ((rol->dabufp))++;}


#define UEOPEN(bnum, btype, code) {\
			       USEREVENT; \
			       StartOfUEvent = (rol->dabufp); \
			       *(++(rol->dabufp)) = (((bnum) << 16) | (btype##_ty) << 8) | (0xff & code);\
			       ((rol->dabufp))++;}
				     
/* - cbclose --------------------------------------------------------- 
crl	: close bank <buff_len> of <bank_type> 
- <bank_type> : UI2 , UI4, UB1 
- <buff_len>  : number of "bank_type" word written 

example	: close bank

Call	: cbclose (btype, &buflen); 

Function: 
Close a CODA bank created by "cbopen" 
leaves the (rol->dabufp) pointing to next empty int after bank 
returns the actual length of that bank 
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#define CBCLOSE \
         *StartOfBank = (unsigned int) (((char *) (rol->dabufp)) - ((char *) StartOfBank));\
	 if ((*StartOfBank & 1) != 0) {\
                        (rol->dabufp) = ((unsigned int *)((char *) (rol->dabufp))+1);\
                        *StartOfBank += 1;\
                      };\
         if ((*StartOfBank & 2) !=0) {\
                        *StartOfBank = *StartOfBank + 2;\
                        (rol->dabufp) = ((unsigned int *)((short *) (rol->dabufp))+1);;\
                      };\
	 *StartOfBank = ( (*StartOfBank) >> 2) - 1;};

#define CECLOSE {event_depth__--;					\
    *StartOfEvent[event_depth__] = (unsigned int) (((char *) (rol->dabufp)) - ((char *) StartOfEvent[event_depth__])); \
    if ((*StartOfEvent[event_depth__] & 1) != 0) {			\
      (rol->dabufp) = ((unsigned int *)((char *) (rol->dabufp))+1);		\
      *StartOfEvent[event_depth__] += 1;				\
    };									\
    if ((*StartOfEvent[event_depth__] & 2) !=0) {			\
      *StartOfEvent[event_depth__] = *StartOfEvent[event_depth__] + 2;	\
      (rol->dabufp) = ((unsigned int *)((short *) (rol->dabufp))+1);;		\
    };									\
    *StartOfEvent[event_depth__] = ( (*StartOfEvent[event_depth__]) >> 2) - 1;};

#define UECLOSE { *StartOfUEvent = (unsigned int) (((char *) (rol->dabufp)) - ((char *) StartOfUEvent));\
	 if ((*StartOfUEvent & 1) != 0) {\
                        (rol->dabufp) = ((unsigned int *)((char *) (rol->dabufp))+1);\
                        *StartOfUEvent += 1;\
                      };\
         if ((*StartOfUEvent & 2) !=0) {\
                        *StartOfUEvent = *StartOfUEvent + 2;\
                        (rol->dabufp) = ((unsigned int *)((short *) (rol->dabufp))+1);;\
                      };\
	 *StartOfUEvent = ( (*StartOfUEvent) >> 2) - 1;\
  /* NOW write the Event onto the Output Queue */ \
	 if (rol->output) { \
	    listAdd(&(rol->output->list),__user_event__); \
	 } else {  \
	    partFreeItem(__user_event__);\
	 } \
	 __user_event__ = (DANODE *) 0; \
	 };


#define CBWRITE32(dat) {*(rol->dabufp)++ = (dat);}
#define CBWRITE16(dat) {*((short *) rol->dabufp)++ = (dat);}
#define CBWRITE8(dat)  {*((char  *)rol->dabufp)++ = (dat);}

#define CBPOINTER (rol->dabufp)
#endif
