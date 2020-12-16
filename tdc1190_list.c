/*************************************************************************
 *
 *  tdc1190_list.c - Library of routines for readout and buffering of
 *                events using a JLAB Trigger Interface V3 (TI) with
 *                a Linux VME controller from a CAEN 1190/1290 TDC
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     100
#define MAX_EVENT_LENGTH   1024*800	/* Size in Bytes */

#define TI_MASTER

#ifdef TI_MASTER
#define TI_READOUT TI_READOUT_EXT_POLL
#endif

#ifdef TI_SLAVE
#define TI_READOUT TI_READOUT_TS_POLL
#endif


/* GEO slot 21 */
#define TI_ADDR    (21 << 19)

#define FIBER_LATENCY_OFFSET 0x10	/* measured longest fiber length */
#include "tiprimary_list.c"	/* source required for CODA */
#include "dmaBankTools.h"
#include "c1190Lib.h"

/* Default block level */
unsigned int BLOCKLEVEL = 1;
#define BUFFERLEVEL 1

/* function prototype */
void rocTrigger(int arg);

/* CAEN 1190/1290 specific definitions */
#define NUM_V1190 2
#define C1190_BANKID 1190

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,2);

#ifdef TI_MASTER
  /*****************
   *   TI SETUP
   *****************/
/* #define INTRANDOMPULSER */
#if (defined (INTFIXEDPULSER) | defined(INTRANDOMPULSER))
  tiSetTriggerSource(TI_TRIGGER_PULSER); /* TS Inputs enabled */
#else
  tiSetTriggerSource(TI_TRIGGER_TSINPUTS);

  /* Set needed TS input bits */
  tiEnableTSInput(TI_TSINPUT_1);
#endif

  /* Load the trigger table that associates
     pins 21/22 | 23/24 | 25/26 : trigger1
     pins 29/30 | 31/32 | 33/34 : trigger2
   */
  tiLoadTriggerTable(0);

  tiSetTriggerHoldoff(1, 10, 1);
  tiSetTriggerHoldoff(2, 10, 1);

  /* Set number of events per block */
  tiSetBlockLevel(BLOCKLEVEL);

#endif

  tiSetBlockBufferLevel(BUFFERLEVEL);

  tiStatus(0);

  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{
  int itdc;

  /* INIT C1190/C1290 - Must be A32 for 2eSST */
  UINT32 list[NUM_V1190] = {0x100000,0x180000};

  tdc1190InitList(list,NUM_V1190,2);

  tdc1190ConfigureGReadout(C1190_CBLT);

  for(itdc=0; itdc<NUM_V1190; itdc++)
    {
      tdc1190SetTriggerMatchingMode(itdc);
      tdc1190SetEdgeResolution(itdc,100);
    }

  tdc1190GStatus(1);

  tiStatus(0);

  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{

  /* Print out the Run Number and Run Type (config id) */
  printf("rocGo: Activating Run Number %d, Config id = %d\n",
	 rol->runNumber,rol->runType);

  /* Get the current Block Level */
  blockLevel = tiGetCurrentBlockLevel();
  printf("rocGo: Block Level set to %d\n",blockLevel);

  /* Enable/Set Block Level on modules, if needed, here */
  tdc1190GSetBLTEventNumber(blockLevel);

#ifdef TI_MASTER
#if (defined (INTFIXEDPULSER) | defined(INTRANDOMPULSER))
  printf("************************************************************\n");
  printf("%s: TI Configured for Internal Pulser Triggers\n",
	 __func__);
  printf("************************************************************\n");
#endif

  /* Example: How to start internal pulser trigger */
#ifdef INTRANDOMPULSER
  /* Enable Random at rate 500kHz/(2^7) = ~3.9kHz */
  tiSetRandomTrigger(1,0x7);
#elif defined (INTFIXEDPULSER)
  /*
    Enable fixed rate with period (ns)
    120 +30*700*(1024^0) = 21.1 us (~47.4 kHz)
     - arg2 = 0xffff - Continuous
     - arg2 < 0xffff = arg2 times
  */
  tiSoftTrig(1,0xffff,700,0);
#endif
#endif

}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

#ifdef TI_MASTER
#ifdef INTRANDOMPULSER
  /* Disable random pulser */
  tiDisableRandomTrigger();
#endif
#endif

  tdc1190GStatus(0);

  tiStatus(0);

  printf("rocEnd: Ended after %d blocks\n", tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int dCnt, ii;
  unsigned int datascan;

  /* Set TI output 1 high for diagnostics */
  tiSetOutputPort(1,0,0,0);

  /* Readout the trigger block from the TI
     Trigger Block MUST be readout first */
  dCnt = tiReadTriggerBlock(dma_dabufp);

  if(dCnt<=0)
    {
      printf("No TI Trigger data or error.  dCnt = %d\n",dCnt);
    }
  else
    { /* TI Data is already in a bank structure.  Bump the pointer */
      dma_dabufp += dCnt;
    }

  /* EXAMPLE: How to open a bank (name=5, type=ui4) and add data words by hand */
  BANKOPEN(C1190_BANKID,BT_UI4,blockLevel);
  /* Check for valid data here */
  for(ii=0;ii<100;ii++)
    {
      datascan = tdc1190Dready(0);
      if (datascan>0)
	{
	  break;
	}
    }

  if(datascan>0)
    {
      /* Get the TDC data from all modules... rflag=2 for Linked List DMA
	 "64" is ignored in Linux */
      dCnt = tdc1190ReadBlock(0,dma_dabufp,64,2);

      if(dCnt < 0)
	{
	  printf("ERROR: in transfer (event = %d), status = 0x%x\n", tiGetIntCount(),dCnt);
	}
      else
	{
	  dma_dabufp += dCnt;
	}
    }
  else
    {
      printf("ERROR: Data not ready in event %d\n",tiGetIntCount());
    }

  BANKCLOSE;

  /* Set TI output 0 low */
  tiSetOutputPort(0,0,0,0);

}

void
rocCleanup()
{

#ifdef TI_MASTER
  tiResetSlaveConfig();
#endif
}



/*
  Local Variables:
  compile-command: "make -k -B tdc1190_list.so"
  End:
 */
