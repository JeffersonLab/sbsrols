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
#define FIBER_LATENCY_OFFSET 0x50

#include <unistd.h>
#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */
#include "sdLib.h"

/* Library to pipe stdout to daLogMsg */
#include "dalmaRolLib.h"

#include "usrstrutils.c"

/* Read the user flags/configuration file. */
typedef struct
{
  int slave1;
  int slave2;
  int slave3;
  int slave4;
  int vtp;
} TI_SLAVE_CONFIG;

TI_SLAVE_CONFIG tiSlaveConfig = {0,0,0,0,0};

void
readUserFlags()
{
  int i;

  printf("%s: Reading user flags file.",
	 __func__);
  init_strings();

  tiSlaveConfig.slave1 = getflag("slave1");
  if(tiSlaveConfig.slave1 == 2)
    {
      tiSlaveConfig.slave1 = getint("slave1");
    }

  tiSlaveConfig.slave2 = getflag("slave2");
  if(tiSlaveConfig.slave2 == 2)
    {
      tiSlaveConfig.slave2 = getint("slave2");
    }

  tiSlaveConfig.slave3 = getflag("slave3");
  if(tiSlaveConfig.slave3 == 2)
    {
      tiSlaveConfig.slave3 = getint("slave3");
    }

  tiSlaveConfig.slave4 = getflag("slave4");
  if(tiSlaveConfig.slave4 == 2)
    {
      tiSlaveConfig.slave4 = getint("slave4");
    }

  tiSlaveConfig.vtp = getflag("vtp");
  if(tiSlaveConfig.vtp == 2)
    {
      tiSlaveConfig.vtp = getint("vtp");
    }

  printf("%s:  tiSlaveConfig = { %d, %d, %d, %d, %d }\n",
	 __func__,
	 tiSlaveConfig.slave1,
	 tiSlaveConfig.slave2,
	 tiSlaveConfig.slave3,
	 tiSlaveConfig.slave4,
	 tiSlaveConfig.vtp);

}

/* Define buffering level */
#define BUFFERLEVEL 1

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int stat;

  readUserFlags();

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */

  if(vmeDmaConfig(2,5,1) < 0)
    vmeDmaConfig(2,3,0);


  /*****************
   *   TI SETUP
   *****************/

  /* Init the SD library so we can get status info */
  stat = sdInit(0);
  if(stat==0)
    {
      sdSetActiveVmeSlots(0);

      // do this to reset VXSQSFP modules
      printf("Resetting QSFP->VXS payload modules...\n");
      sdTestSetStatBitBMask(0xFFFF);
      usleep(250000);
      sdTestSetStatBitBMask(0x0000);

      sdStatus(0);
    }

  tiSetTriggerPulse(1,0,25,0);

  /*
    Increase the OT#2 width for the MPD input trigger
    8 -> (8 + 2) * 4ns = 40ns
  */
  /* Set prompt output width (100 + 2) * 4 = 408 ns */
  tiSetPromptTriggerWidth(127);

  if(tiSlaveConfig.vtp > 0)
    {
      printf("%s: Adding VTP\n",
	     __func__);
      /* Add Crate1 as a slave */
      tiRocEnable(2);
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

  int bufferLevel = 0;
  /* Get the current buffering settings (blockLevel, bufferLevel) */
  blockLevel = tiGetCurrentBlockLevel();
  bufferLevel = tiGetBroadcastBlockBufferLevel();
  printf("%s: Block Level = %d,  Buffer Level (broadcasted) = %d (%d)\n",
	 __func__,
	 blockLevel,
	 tiGetBlockBufferLevel(),
	 bufferLevel);

#ifdef TI_SLAVE
  /* In case of slave, set TI busy to be enabled for full buffer level */

  /* Check first for valid blockLevel and bufferLevel */
  if((bufferLevel > 10) || (blockLevel > 1))
    {
      daLogMsg("ERROR","Invalid blockLevel / bufferLevel received: %d / %d",
	       blockLevel, bufferLevel);
      tiUseBroadcastBufferLevel(0);
      tiSetBlockBufferLevel(1);

      /* Cannot help the TI blockLevel with the current library.
	 modules can be spared, though
      */
      blockLevel = 1;
    }
  else
    {
      tiUseBroadcastBufferLevel(1);
    }
#endif

  DALMAGO;
  tiStatus(0);
  tiTriggerStatus(1);
  DALMASTOP;

  /* Enable/Set Block Level on modules, if needed, here */

}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{
  DALMAGO;
  tiStatus(0);
  tiTriggerStatus(1);
  DALMASTOP;
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



  /* Check for sync Event */
  if(tiGetSyncEventFlag())
    {
      printf("%s: Got Sync Event!! Block # = %d\n",
	     __func__,evntno);
      /* Clear/Update Modules here */

    }


  /* Set TI output 0 low */
  tiSetOutputPort(0,0,0,0);

}

void
rocLoad()
{
  dalmaInit(1);
}
void
rocCleanup()
{
  dalmaClose();
  printf("%s: Reset all Modules\n",__FUNCTION__);

}
