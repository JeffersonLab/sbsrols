/*************************************************************************
 *
 *  ti_bridge_list.c - Library of routines for readout and buffering of
 *                     events using a JLAB Trigger Interface V3 (TI) with
 *                     a Linux VME controller in CODA 3.0.
 *
 *                     This for a TI in Bridge Mode, bridging multiple ROCs
 *                     to a separate TI-master or TS
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*64      /* Size in Bytes */

/* Define TI Type (Bridge treated as as Slave) */
#define TI_SLAVE
/* Bridge port trigger source (e.g. front panel ECL input), POLL for available data */
#define TI_READOUT TI_READOUT_BRIDGE_POLL
/* TI VME address, or 0 for Auto Initialize (search for TI by slot) */
#define TI_ADDR  0

/* Measured longest fiber length in system */
#define FIBER_LATENCY_OFFSET 0x50

#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */
#include "sdLib.h"


#include "usrstrutils.c"

/* experimental tool to pipe stdout to dalogMsg  */
#include "dalmaRolLib.h"


/* Default blocklevel, updated by TI-master / TS (in rocGo) */
#define BLOCKLEVEL 1

/*

  Read the user flags/configuration file.

*/
typedef struct
{
  int slave1;
  int slave2;
  int slave3;
  int slave4;
} TI_SLAVE_CONFIG;

TI_SLAVE_CONFIG tiSlaveConfig = {0,0,0,0};

void
readUserFlags()
{
  int i;

  printf("%s: Reading user flags file.",
	 __func__);
  init_strings();

  // support args : slave1(=1), slave1=n
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

  printf("%s:  tiSlaveConfig = { %d, %d, %d, %d }\n",
	 __func__,
	 tiSlaveConfig.slave1,
	 tiSlaveConfig.slave2,
	 tiSlaveConfig.slave3,
	 tiSlaveConfig.slave4);

}

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int stat;

  /* Read in the init flags using usrstrutils.c */
  readUserFlags();

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1);

  /* Define BLock Level */
  blockLevel = BLOCKLEVEL;


  /*********************
   *   TI Bridge SETUP
   *********************/

  /* Init the SD library so we can get status info */
  stat = sdInit(0);
  if(stat==0)
    {
      sdSetActiveVmeSlots(0);
      sdStatus(0);
    }

  /* Add slave TIs configured with usrstrutils.c */
  if(tiSlaveConfig.slave1)
    {
      printf("%s: Adding Slave to port 1 (%d)\n",
	     __func__, tiSlaveConfig.slave1);
      tiAddSlave(1);
    }
  if(tiSlaveConfig.slave2)
    {
      printf("%s: Adding Slave to port 2 (%d)\n",
	     __func__, tiSlaveConfig.slave2);
      tiAddSlave(2);
    }
  if(tiSlaveConfig.slave3)
    {
      printf("%s: Adding Slave to port 3 (%d)\n",
	     __func__, tiSlaveConfig.slave3);
      tiAddSlave(3);
    }
  if(tiSlaveConfig.slave4)
    {
      printf("%s: Adding Slave to port 4 (%d)\n",
	     __func__, tiSlaveConfig.slave4);
      tiAddSlave(4);
    }

  /* Set prompt output width (127 + 2) * 4 = 516 ns */
  tiSetPromptTriggerWidth(127);

  DALMAGO;
  tiStatus(0);
  DALMASTOP;

  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{

  DALMAGO;
  tiStatus(0);
  DALMASTOP;

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

  DALMAGO;
  tiStatus(0);
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
  DALMASTOP;

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int dCnt;

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

  if(tiGetSyncEventFlag())
    {
      printf("%s: Got Sync Event!! Block # = %d\n",
	     __func__,arg);
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

  printf("%s: Reset all Modules\n",__FUNCTION__);
  tiResetSlaveConfig();
  dalmaClose();

}

/*
  Local Variables:
  compile-command: "make -k ti_bridge_list.so"
  End:
 */
