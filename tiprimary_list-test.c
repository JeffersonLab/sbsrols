/*****************************************************************************
 *
 * tiprimary_list.c - "Primary" Readout list routines for TI Master or Slave
 *
 * Usage:
 *
 *    #include "tiprimary_list.c"
 *
 *  then define the following routines:
 *
 *    void rocDownload();
 *    void rocPrestart();
 *    void rocGo();
 *    void rocEnd();
 *    void rocTrigger(int arg);
 *    void rocCleanup()
 */

#define ROL_NAME__ "TIPRIMARY"

#ifndef MAX_EVENT_LENGTH
/* Check if an older definition is used */
#ifdef MAX_SIZE_EVENTS
#define MAX_EVENT_LENGTH MAX_SIZE_EVENTS
#else
#define MAX_EVENT_LENGTH 10240
#endif /* MAX_SIZE_EVENTS */
#endif /* MAX_EVENT_LENGTH */
#ifndef MAX_EVENT_POOL
/* Check if an older definition is used */
#ifdef MAX_NUM_EVENTS
#define MAX_EVENT_POOL MAX_NUM_EVENTS
#else
#define MAX_EVENT_POOL   400
#endif /* MAX_NUM_EVENTS */
#endif /* MAX_EVENT_POOL */

/* POLLING_MODE */
#define POLLING___
#define POLLING_MODE

/* INIT_NAME should be defined with readlist filename base at compilation time */
#ifndef INIT_NAME
#error "INIT_NAME undefined. Set to readout list filename base with gcc flag -DINIT_NAME"
#endif

#include <stdio.h>
#include <rol.h>
#include "jvme.h"
#include <TIPRIMARY_source.h>
#include "tiLib.h"
extern void daLogMsg(char *severity, char *fmt,...);
extern int bigendian_out;

extern int tiNeedAck; /* Request for acknowledge, declared in tiLib */
extern DMANODE *the_event; /* node pointer for event buffer obtained from GETEVENT, declared in dmaPList */
extern unsigned int *dma_dabufp; /* event buffer pointer obtained from GETEVENT, declared in dmaPList */

/* Redefine tsCrate according to TI_MASTER or TI_SLAVE */
#ifdef TI_SLAVE
static int tsCrate=0;
#else
#ifdef TI_MASTER
static int tsCrate=1;
#endif
#endif

int emptyCount = 0;   /* Count the number of times event buffers are empty */
int errCount = 0;     /* Count the number of times no buffer available from vmeIN */

#define ISR_INTLOCK INTLOCK
#define ISR_INTUNLOCK INTUNLOCK

/* Readout Acknowledge condition variable and mutex */
pthread_mutex_t ack_mutex=PTHREAD_MUTEX_INITIALIZER;
#define ACKLOCK {				\
    if(pthread_mutex_lock(&ack_mutex)<0)	\
      perror("pthread_mutex_lock");		\
  }
#define ACKUNLOCK {				\
    if(pthread_mutex_unlock(&ack_mutex)<0)	\
      perror("pthread_mutex_unlock");		\
  }
pthread_cond_t ack_cv = PTHREAD_COND_INITIALIZER;
#define ACKWAIT {					\
    if(pthread_cond_wait(&ack_cv, &ack_mutex)<0)	\
      perror("pthread_cond_wait");			\
  }
#define ACKSIGNAL {					\
    if(pthread_cond_signal(&ack_cv)<0)			\
      perror("pthread_cond_signal");			\
  }
int ack_runend=0;

/* End of run condition variable */
pthread_cond_t endrun_cv = PTHREAD_COND_INITIALIZER;
struct timespec endrun_waittime;
int endrun_timedwait_ret=0;
#define ENDRUN_TIMEDWAIT(__x) {						\
    clock_gettime(CLOCK_REALTIME, &endrun_waittime);			\
    endrun_waittime.tv_sec += __x;					\
    endrun_timedwait_ret = pthread_cond_timedwait(&endrun_cv, &ack_mutex, &endrun_waittime); \
    if(endrun_timedwait_ret<0)						\
      perror("pthread_cond_timedwait");					\
  }
#define ENDRUN_SIGNAL {					\
    if(pthread_cond_signal(&endrun_cv)<0)		\
      perror("pthread_cond_signal");			\
  }

/* ROC Function prototypes defined by the user */
void rocDownload();
void rocPrestart();
void rocGo();
void rocEnd();
void rocTrigger(int arg);
void rocCleanup();

/* Routines to get in/out queue counts */
int  getOutQueueCount();
int  getInQueueCount();

/* Asynchronous (to tiprimary rol) trigger routine, connects to rocTrigger */
void asyncTrigger();

/* Input and Output Partitions for VME Readout */
DMA_MEM_ID vmeIN, vmeOUT;

/**
 *  DOWNLOAD
 */
static void __download()
{
  int status;

  daLogMsg("INFO","Readout list compiled %s", DAYTIME);
#ifdef POLLING___
  rol->poll = 1;
#endif
  *(rol->async_roc) = 0; /* Normal ROC */

  bigendian_out=1;

  pthread_mutex_init(&ack_mutex, NULL);
  pthread_cond_init(&ack_cv,NULL);
  pthread_cond_init(&endrun_cv,NULL);

#ifdef USE_SLAVE_WINDOW
  /* Open Slave Window for SFI Initiated Block transfers */
  printf("Using Slave Window for SFI Block Transfers\n");
  vmeOpenSlaveA32(0x18000000,0x00400000);
  dmaPUseSlaveWindow(1);
#endif

  /* Initialize memory partition library */
  dmaPartInit();

  /* Setup Buffer memory to store events */
  dmaPFreeAll();
  vmeIN  = dmaPCreate("vmeIN",MAX_EVENT_LENGTH,MAX_EVENT_POOL,0);
  vmeOUT = dmaPCreate("vmeOUT",0,0,0);

  if(vmeIN == 0)
    daLogMsg("ERROR", "Unable to allocate memory for event buffers");

  /* Reinitialize the Buffer memory */
  dmaPReInitAll();
  dmaPStatsAll();

  /* Initialize Fiber Latency offset */
  tiSetFiberLatencyOffset_preInit(FIBER_LATENCY_OFFSET);

  /* Set crate ID */
  tiSetCrateID_preInit(ROCID);

#ifndef TI_ADDR
#define TI_ADDR 0
#endif
#ifndef TI_READOUT
#warning "TI_READOUT undefined.  Using TI_REAODUT_EXT_POLL"
#define TI_READOUT TI_READOUT_EXT_POLL
#endif
#ifndef TI_FLAG
#define TI_FLAG 0
#endif

  status = tiInit(TI_ADDR,TI_READOUT,TI_FLAG);
  if(status == -1) daLogMsg("ERROR","Unable to initialize TI board");

  /* Set timestamp format 48 bits */
  tiSetEventFormat(3);

  /* Execute User defined download */
  rocDownload();

  daLogMsg("INFO","Download Executed");

  tiDisableVXSSignals();

} /*end download */

/**
 *  PRESTART
 */
static void __prestart()
{
  ACKLOCK;
  ack_runend=0;
  ACKUNLOCK;
  CTRIGINIT;
  *(rol->nevents) = 0;

  daLogMsg("INFO","Entering Prestart");


  TIPRIMARY_INIT;
  CTRIGRSS(TIPRIMARY,1,usrtrig,usrtrig_done);
  CRTTYPE(1,TIPRIMARY,1);

  /* If the TI Master, send a Clock and Trig Link Reset */
  if(tsCrate)
    {
      tiClockReset();
      taskDelay(2);
      tiTrigLinkReset();
    }

  tiEnableVXSSignals();

  /* Check the health of the vmeBus Mutex.. re-init if necessary */
  vmeCheckMutexHealth(10);

  /* Execute User defined prestart */
  rocPrestart();

  /* If the TI Master, send a Sync Reset */
  if(tsCrate)
    {
      printf("%s: Sending sync as TI master\n",__FUNCTION__);
      taskDelay(2);
      tiSyncReset(1); /* set the Block Level */
      taskDelay(2);
      tiSetBlockLevel(blockLevel);
    }

  /* Connect User Trigger Routine */
  tiIntConnect(TI_INT_VEC,asyncTrigger,0);

  daLogMsg("INFO","Prestart Executed");

  if (__the_event__) WRITE_EVENT_;
  *(rol->nevents) = 0;
  rol->recNb = 0;
} /*end prestart */

/**
 *  PAUSE
 */
static void __pause()
{
  CDODISABLE(TIPRIMARY,1,0);
  daLogMsg("INFO","Pause Executed");

  if (__the_event__) WRITE_EVENT_;
} /*end pause */

/**
 *  GO
 */
static void __go()
{
  daLogMsg("INFO","Entering Go");
  ACKLOCK;
  ack_runend=0;
  ACKUNLOCK;

  emptyCount=0;
  errCount=0;

  CDOENABLE(TIPRIMARY,1,1);
  rocGo();

  tiIntEnable(1);

  if (__the_event__) WRITE_EVENT_;
}

static void __end()
{
  unsigned int blockstatus=0;

  /* Stop triggers on the TI-master */
  if(tsCrate)
    {
      tiDisableTriggerSource(1);
    }

  blockstatus = tiBlockStatus(0,0);

  ACKLOCK;
  ack_runend=1;
  if(blockstatus)
    {
      printf("%s: Clearing data from TI (blockstatus = 0x%x)\n",__FUNCTION__, blockstatus);
      ENDRUN_TIMEDWAIT(30);
      printf("%s: endrun_timedwait_ret = %d   blockstatus = 0x%x\n",
	     __FUNCTION__,endrun_timedwait_ret,tiBlockStatus(0,0));
    }
  ACKUNLOCK;

  INTLOCK;
  INTUNLOCK;

  tiIntDisable();
  tiIntDisconnect();

  /* Execute User defined end */
  rocEnd();

  CDODISABLE(TIPRIMARY,1,0);

  dmaPStatsAll();

  daLogMsg("INFO","End Executed");

  if (__the_event__) WRITE_EVENT_;
} /* end end block */

void usrtrig(unsigned long EVTYPE,unsigned long EVSOURCE)
{
  int ii, len;
  int syncFlag=0;
  unsigned int event_number=0;
  DMANODE *outEvent;
  unsigned int blockstatus = 0;
  int bready = 0;

  outEvent = dmaPGetItem(vmeOUT);
  if(outEvent != NULL)
    {
      len = outEvent->length;
      syncFlag = outEvent->type;
      event_number = outEvent->nevent;

      CEOPEN(ROCID, BT_BANK, blockLevel);

      if(rol->dabufp != NULL)
	{
	  for(ii=0;ii<len;ii++)
	    {
	      *rol->dabufp++ = outEvent->data[ii];
	    }
	}
      else
	{
	  printf("tiprimary_list: ERROR rol->dabufp is NULL -- Event lost\n");
	}

      CECLOSE;

      ACKLOCK;

      dmaPFreeItem(outEvent);

      if(tiNeedAck>0)
	{
	  tiNeedAck=0;
	  ACKSIGNAL;
	}

      if(ack_runend)
	{
	  /* Check for available blocks in the TI */
	  blockstatus = tiBlockStatus(0,0);
	  bready = tiBReady();
	  printf("tiBlockStatus = 0x%x   tiBReady() = %d\n",
		 blockstatus,bready);

	  if((blockstatus == 0) && (bready == 0))
	    ENDRUN_SIGNAL;
	}

      ACKUNLOCK;
    }
  else
    {
      logMsg("Error: no Event in vmeOUT queue\n",0,0,0,0,0,0);
    }

} /*end trigger */

void asyncTrigger()
{
  int intCount=0;
  int length,size;

  intCount = tiGetIntCount();
  syncFlag = tiGetSyncEventFlag();

  /* grap a buffer from the queue */
  GETEVENT(vmeIN,intCount);
  if(the_event == NULL)
    {
      if(errCount == 0)
	daLogMsg("ERROR","asyncTrigger: No DMA Buffer Available. Events could be out of sync!");
      printf("asyncTrigger:ERROR: No buffer available!\n");
      errCount++;
      return;
    }
  if(the_event->length!=0)
    {
      printf("asyncTrigger: ERROR: Interrupt Count = %d the_event->length = %ld\t",intCount, the_event->length);
    }

  /* Store Sync Flag status for this event */
  the_event->type = syncFlag;

  /* Execute user defined Trigger Routine */
  rocTrigger(intCount);

  /* Put this event's buffer into the OUT queue. */
  ACKLOCK;
  PUTEVENT(vmeOUT);

  /* Check if the event length is larger than expected */
  length = (((long)(dma_dabufp) - (long)(&the_event->length))) - 4;
  size = the_event->part->size - sizeof(DMANODE);

  if(length>size)
    {
      printf("rocLib: ERROR: Event length > Buffer size (%d > %d).  Event %ld\n",
	     length,size,the_event->nevent);
      daLogMsg("WARN", "Event buffer overflow");
    }

  if(dmaPEmpty(vmeIN))
    {
      emptyCount++;
      /*printf("WARN: vmeIN out of event buffers (intCount = %d).\n",intCount);*/

      if((ack_runend == 0) || (tiBReady() > 0))
	{
	  /* Set the NeedAck for Ack after a buffer is freed */
	  tiNeedAck = 1;

	  /* Wait for the signal indicating that a buffer has been freed */
	  ACKWAIT;
	}

    }

  ACKUNLOCK;

}

void usrtrig_done()
{
} /*end done */

void __done()
{
  poolEmpty = 0; /* global Done, Buffers have been freed */
} /*end done */

static void __reset()
{
  int iemp=0;

  tiIntDisable();
  tiIntDisconnect();

  /* Empty the vmeOUT queue */
  while(!dmaPEmpty(vmeOUT))
    {
      iemp++;
      dmaPFreeItem(dmaPGetItem(vmeOUT));
      if(iemp>=MAX_EVENT_POOL) break;
    }

  printf(" **Reset Called** \n");

} /* end reset */

int
getOutQueueCount()
{
  if(vmeOUT)
    return(dmaPNodeCount(vmeOUT));
  else
    return(0);
}

int
getInQueueCount()
{
  if(vmeIN)
    return(dmaPNodeCount(vmeIN));
  else
    return(0);
}

/* This routine is automatically executed just before the shared libary
   is unloaded.

   Clean up memory that was allocated
*/
__attribute__((destructor)) void end (void)
{
  static int ended=0;
  extern volatile struct TI_A24RegStruct  *TIp;

  if(ended==0)
    {
      printf("ROC Cleanup\n");
      rocCleanup();

      TIp = NULL;

      dmaPFreeAll();
      vmeCloseA32Slave();
      ended=1;
    }

}

int
tsLive(int sflag)
{
  return tiLive(sflag);
}
