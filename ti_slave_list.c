/*************************************************************************
 *
 *  ti_slave_list.c - Library of routines for readout and buffering of
 *                    events using a JLAB Trigger Interface V3 (TI) with
 *                    a Linux VME controller in CODA 3.0.
 *
 *                    This is for a TI in Slave Mode controlled by a
 *                    Master TI or Trigger Supervisor
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     100
#define MAX_EVENT_LENGTH   1024*40      /* Size in Bytes */

/* Number of data words in the event */
#define MAX_WORDS   2700

/* Define TI Type (TI_MASTER or TI_SLAVE) */
#define TI_SLAVE
/* TS Fiber Link trigger source (from TI Master, TD, or TS), POLL for available data */
#define TI_READOUT TI_READOUT_TS_POLL
/* TI VME address, or 0 for Auto Initialize (search for TI by slot) */
#define TI_ADDR  0

/* Measured longest fiber length in system */
#define FIBER_LATENCY_OFFSET 0x4A

#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */
#include "sdLib.h"

/* Define buffering level */
#define BUFFERLEVEL 10

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int stat;

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */

  vmeDmaConfig(2,5,1);

  /* Define BLock Level variable to a default */
  blockLevel = 1;


  /*****************
   *   TI SETUP
   *****************/

  /* Init the SD library so we can get status info */
  stat = sdInit(0);
  if(stat==0)
    {
      sdSetActiveVmeSlots(0);
      sdStatus(0);
    }

  tiStatus(0);

  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{

  tiStatus(0);

  /* EXAMPLE: User bank of banks added to prestart event */
  UEOPEN(500,BT_BANK,0);

  /* EXAMPLE: Bank of data in User Bank 500 */
  CBOPEN(1,BT_UI4,0);
  *rol->dabufp++ = 0x11112222;
  *rol->dabufp++ = 0x55556666;
  *rol->dabufp++ = 0xaabbccdd;
  CBCLOSE;

  UECLOSE;

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

  /* Get the broadcasted Block and Buffer Levels from TS or TI Master */
  blockLevel = tiGetCurrentBlockLevel();
  bufferLevel = tiGetBroadcastBlockBufferLevel();
  printf("rocGo: Block Level set to %d  Buffer Level set to %d\n",blockLevel,bufferLevel);

  /* In case of slave, set TI busy to be enabled for full buffer level */
  tiUseBroadcastBufferLevel(1);

  /* Enable/Set Block Level on modules, if needed, here */

}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

  tiStatus(0);

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int evntno)
{
  int ii;
  int stat, dCnt, idata;

  /* Set TI output 1 high for diagnostics */
  tiSetOutputPort(1,0,0,0);

  /* Check if this is a Sync Event */
  stat = tiGetSyncEventFlag();
  if(stat) {
    printf("rocTrigger: Got Sync Event!! Block # = %d\n",evntno);
  }

  /* Readout the trigger block from the TI
     Trigger Block MUST be readout first */
  dCnt = tiReadTriggerBlock(dma_dabufp);
  if(dCnt<=0)
    {
      printf("No data or error.  dCnt = %d\n",dCnt);
    }
  else
    { /* TI Data is already in a bank structure.  Bump the pointer */
      dma_dabufp += dCnt;
    }


  /* EXAMPLE: How to open a bank (name=5, type=ui4) and add data words by hand */
  BANKOPEN(5,BT_UI4,blockLevel);
  *dma_dabufp++ = tiGetIntCount();
  *dma_dabufp++ = 0xdead;
  *dma_dabufp++ = 0xcebaf111;
  *dma_dabufp++ = 0xcebaf222;

  for (ii=1;ii<=MAX_WORDS;ii++) {
    *dma_dabufp++ = ii;
  }
  BANKCLOSE;

  /* Check for sync Event */
  if(tiGetSyncEventFlag()) {
    /* Set new block level if it has changed */
    idata = tiGetCurrentBlockLevel();
    if((idata != blockLevel)&&(idata<255)) {
      blockLevel = idata;
      printf("rocTrigger: Block Level changed to %d\n",blockLevel);
    }

    /* Clear/Update Modules here */

  }



  /* Set TI output 0 low */
  tiSetOutputPort(0,0,0,0);

}

void
rocCleanup()
{

  printf("%s: Reset all Modules\n",__FUNCTION__);

}
