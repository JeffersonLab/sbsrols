/******************************************************************************
*
* header file for use with VTP readout lists (CODA 3.0)
*
*  - B. Moffit  October 2019
*
*******************************************************************************/
#ifndef __VTP_ROL__
#define __VTP_ROL__

#define ROL_NAME__ "VTP"
#define POLLING___
#define POLLING_MODE

#define FPGA_MODE

#include <rol.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "remexLib.h"
#include "vtpLib.h"

/* Library to pipe stdout to daLogMsg */
#include "dalmaRolLib.h"


extern void daLogMsg(char *severity, char *fmt,...);
int vtpUploadAll(char *string, int length);
extern int vtpConfig(char *fname);
extern void vtpInitGlobals();

#define VTP_READ_CONF_FILE {				\
    vtpInitGlobals();					\
    vtpConfig("");					\
    if(rol->usrConfig)					\
      vtpConfig(rol->usrConfig);			\
  }

/* Prototypes for user defined routines */
void rocDownload();
void rocPrestart();
void rocPause();
void rocGo();
void rocEnd();
void rocTrigger(int EVTYPE);
void rocTrigger_done();
void rocReset();
void rocLoad();
void rocCleanup();

static int VTP_handlers, VTPflag;
static int VTP_isAsync;
static unsigned int VTP_prescale = 1;
static unsigned int VTP_count = 0;

static void
vtptinit(int code)
{
  VTP_count = 0;
  VTP_prescale = 1;
  vtpRocReset(0);    /* Reset the Hardware ROC - disables all data sources */
}

static void
vtptenable(int code, int val)
{
  vtpRocEnable(val);   /* Enable specified ROC data sources */
  if((val&2) == 0) {
    VTPflag=0;   /* If no Synchronous event polling then disable the vtptest() routine */
    printf("vtptenable: Disabling synchronous trigger polling\n");
  }else{
    VTPflag=1;
    printf("vtptenable: Synchronous trigger polling enabled!!\n");
  }
}

static void
vtptdisable(int code, int val)
{
  VTPflag = 0;
  vtpRocEnable(5);    /* Disable the Synchronous data source */
}

static void
vtptack(int code, unsigned int intMask)
{
  /* if(code == 0)
     vtpTiAck(); */
}

static unsigned int
vtpttype(int code)
{
  return (1);
}

static int
vtpttest(int code)
{
  int val=0;

  if(VTPflag)
    {
      //val = vtpBReady();
      val = vtpRocPoll();   // Returns the number of triggers currently needed to be acknowledged

      if(val > 0)
	{
	  syncFlag = 0; /* No way to get Sync information yet */
	  return(1);
	} else {
	  /*usleep(1000);*/
	  syncFlag = 0;
	  return(0);
	}

      return(0);
    } else {
      return(0);
    }

}


/* define CODA macros needed by trigger_dispatch.h */
#define VTP_TEST  vtpttest
#define VTP_INIT { VTP_handlers =0;VTP_isAsync = 0;VTPflag = 0;vtptinit(1);}
#define VTP_ASYNC(code,id)  {printf("No Async mode is available for VTP\n"); \
                              printf("linking sync VTP trigger to id %d \n",id); \
			       VTP_handlers = (id);VTP_isAsync = 0;}
#define VTP_SYNC(code,id)   {printf("linking sync VTP trigger to id %d \n",id); \
			       VTP_handlers = (id);VTP_isAsync = 0;}
#define VTP_ENA(code,val) vtptenable(code,val);
#define VTP_DIS(code,val) vtptdisable(code,val);
#define VTP_GETID(code) VTP_handlers
#define VTP_TTYPE vtpttype
#define VTP_START(val)	 {;}
#define VTP_STOP(val)	 {;}
#define VTP_ACK(code,val) vtptack(code,val);

static void
__download()
{
  daLogMsg("INFO", "Readout list compiled %s", DAYTIME);
  rol->poll = 1;
  *(rol->async_roc) = 1;	/* Async ROC - do not send control events via the software ROC*/
  daLogMsg("INFO", "Async ROC mode - No software Control Events will be sent to EB\n");
  {				/* begin user */

    daLogMsg("INFO", "******* Entering User Download *******\n");
    rocDownload();
    daLogMsg("INFO", "******* User Download Executed *******");

  }				/* end user */
}				/*end download */

static void
__prestart()
{
  CTRIGINIT;
  *(rol->nevents) = 0;
  {				/* begin user */

    CDOINIT(VTP);
    CTRIGRSS(VTP, 1, usrtrig, usrtrig_done);
    CRTTYPE(1, VTP, 1);

    daLogMsg("INFO", "******* Entering User Prestart *******\n");
    rocPrestart();
    daLogMsg("INFO", "******* User Prestart Executed *******");

  }				/* end user */
  *(rol->nevents) = 0;
  rol->recNb = 0;
}				/*end prestart */

static void
__end()
{
  {				/* begin user */

    daLogMsg("INFO", "******* Entering User End *******\n");
    rocEnd();
    daLogMsg("INFO", "******* User End Executed *******");

  }				/* end user */
}				/* end end block */

static void
__pause()
{
  {				/* begin user */

    daLogMsg("INFO", "******* Entering User Pause *******\n");
    rocPause();
    daLogMsg("INFO", "******* User Pause Executed *******");
  }				/* end user */
}				/*end pause */

static void
__go()
{

  {				/* begin user */
    daLogMsg("INFO", "******* Entering User Go *******\n");
    rocGo();
    daLogMsg("INFO", "******* User Go Executed *******");
  }				/* end user */
}

void
usrtrig(unsigned int EVTYPE, unsigned int EVSOURCE)
{
  {				/* begin user */
    rocTrigger(EVTYPE);
  }				/* end user */
}				/*end trigger */

void
usrtrig_done()
{
  {				/* begin user */
    rocTrigger_done();
  }				/* end user */
}				/*end done */

void
__done()
{
  poolEmpty = 0;		/* global Done, Buffers have been freed */
  {				/* begin user */

  }				/* end user */
}				/*end done */

void
__reset()
{
  {
    rocReset();
  }
}

__attribute__((constructor)) void start (void)
{
  static int started=0;

  if(started==0)
    {
      daLogMsg("INFO","ROC Load");

      rocLoad();
      started=1;

      dalmaInit(1);
    }

}

/* This routine is automatically executed just before the shared libary
   is unloaded.

   Clean up memory that was allocated
*/
__attribute__((destructor)) void end (void)
{
  static int ended=0;

  if(ended==0)
    {
      printf("ROC Cleanup\n");
      rocCleanup();
      dalmaClose();

      ended=1;
    }

}

#endif
