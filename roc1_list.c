/*************************************************************************
 *
 *  v767_list.c - Library of routines for readout and buffering of 
 *                events using a JLAB Trigger Interface V3 (TI) with 
 *                a Linux VME controller.
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*60      /* Size in Bytes */

/* Define Interrupt source and address */
#define TI_MASTER
#define TI_READOUT TI_READOUT_EXT_POLL  /* Poll for available data, external triggers */
#define TI_ADDR    (21<<19)          /* GEO slot 21 */

/* Decision on whether or not to readout the TI for each block 
   - Comment out to disable readout 
*/
#define TI_DATA_READOUT

#define FIBER_LATENCY_OFFSET 0x4A  /* measured longest fiber length */

#include "dmaBankTools.h"
#include "tiprimary_list.c" /* source required for CODA */
#include "v767Lib.h"

/* Default block level */
unsigned int BLOCKLEVEL=1;
#define BUFFERLEVEL 3

/* Redefine tsCrate according to TI_MASTER or TI_SLAVE */
#ifdef TI_SLAVE
int tsCrate=0;
#else
#ifdef TI_MASTER
int tsCrate=1;
#endif
#endif

/* V767 Parameters */
int TDC_ID1 = 0;
int V767_NMOD = 1;
/* V767_ADR1 is the HW address for the leftmost v767 in the crate */
unsigned int V767_ADR1 = 0x100000;
/* V767_OFF is the increment for each subsequent v767 (ie. next module would be 0x180000) */
unsigned int V767_OFF = 0x080000;

/* trigger window settings (in nanosecs) */
int V767_TW_OFF = 2500;
int V767_TW_WID = 2500;


/* function prototype */
void rocTrigger(int arg);

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
  vmeDmaConfig(2,5,1); 

  /*****************
   *   TI SETUP
   *****************/
  int overall_offset=0x80;

#ifndef TI_DATA_READOUT
  /* Disable data readout */
  tiDisableDataReadout();
  /* Disable A32... where that data would have been stored on the TI */
  tiDisableA32();
#endif

  /* Set crate ID */
  tiSetCrateID(0x01); /* ROC 1 */

  tiSetTriggerSource(TI_TRIGGER_TSINPUTS);

  /* Set needed TS input bits */
  tiEnableTSInput( TI_TSINPUT_1 );

  /* Load the trigger table that associates 
     pins 21/22 | 23/24 | 25/26 : trigger1
     pins 29/30 | 31/32 | 33/34 : trigger2
  */
  tiLoadTriggerTable(0);

  tiSetTriggerHoldoff(1,10,0);
  tiSetTriggerHoldoff(2,10,0);

/*   /\* Set the sync delay width to 0x40*32 = 2.048us *\/ */
  tiSetSyncDelayWidth(0x54, 0x40, 1);

  /* Set the busy source to non-default value (no Switch Slot B busy) */
  tiSetBusySource(TI_BUSY_LOOPBACK,1);

/*   tiSetFiberDelay(10,0xcf); */

#ifdef TI_MASTER
  /* Set number of events per block */
  tiSetBlockLevel(BLOCKLEVEL);
#endif

  tiSetEventFormat(1);

  tiSetBlockBufferLevel(BUFFERLEVEL);

  /*****************
   *  V767 SETUP
   *****************/
  int itdc;
  v767Init(V767_ADR1,V767_OFF,V767_NMOD,0);

  for (itdc=TDC_ID1;itdc<TDC_ID1+V767_NMOD;itdc++) 
    {
      /* select stop trigger matching, substraction of trigger time,
	 all channels on, autoload on, and save configuration */
  
      /*     v767SetAcqMode(itdc,0); */
      /*     v767SetDReadyMode(itdc,0); */
      /*     v767BusErrEnable(itdc);    */
      /*     v767SetBLKEndMode(itdc, 1); */
      /*     v767AutoloadDisable(itdc); */
      /*     v767AddrRotary(itdc); */
      /*     v767SetEdgeDetectionMode(itdc,0);  /\* 0=time off leading edge *\/ */
      /*     v767OverlapTrigDisable(itdc); */
      v767TriggerConfig(itdc,V767_TW_WID,-V767_TW_OFF,0);
      /*     v767ChannelMask(itdc,channelmask);  */
      v767Status(itdc, 0, 0); 
      v767TriggerConfig(itdc,0,0,0); /* Print current trigger config */
      /*     v767SaveConfig(itdc); */
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
  int i;

  for(i=TDC_ID1;i<TDC_ID1+V767_NMOD;i++) 
    {
      v767Clear(i);
      v767Status(i, 0, 0); 
    }

  tiStatus(0);

  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{
  int i;
  /* Enable modules, if needed, here */
  for (i=TDC_ID1;i<TDC_ID1+V767_NMOD;i++) 
    {
      v767Clear(i);
    }


  /* Get the current block level */
  BLOCKLEVEL = tiGetCurrentBlockLevel();
  printf("%s: Current Block Level = %d\n",
	 __FUNCTION__,BLOCKLEVEL);

  /* Use this info to change block level is all modules */

}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

  int i;

  for (i=TDC_ID1;i<TDC_ID1+V767_NMOD;i++) 
    {
      v767Status(i, 0, 0); 
    }

  tiStatus(0);

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());
  
}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int ii, islot;
  int stat, dCnt, len=0, idata;

  tiSetOutputPort(1,0,0,0);

  BANKOPEN(4,BT_UI4,0);

  vmeDmaConfig(2,5,1); 

  dCnt = tiReadBlock(dma_dabufp,8+(3*BLOCKLEVEL),1);

  if(dCnt<=0)
    {
      printf("TI: No data or error.  dCnt = %d\n",dCnt);
    }
  else
    {
      dma_dabufp += dCnt;
    }

  BANKCLOSE;

  BANKOPEN(5,BT_UI4,0);
  for (ii=TDC_ID1;ii<TDC_ID1+V767_NMOD;ii++) 
    {
      stat = v767Dready(ii);
      while( (ii<100) && (stat==0) )
	{
	  stat = v767Dready(ii);
	  ii++;
	}

      if(stat)
	{
	  dCnt = v767ReadEvent(ii, dma_dabufp, V767_MAX_WORDS_PER_EVENT);

	  if(dCnt <= 0) 
	    { /*Error*/
	      *dma_dabufp++ = 0xed000bad;
	      printf("v767(%d): No data or error.  dCnt = %d\n", ii, dCnt);
	    } 
	  else 
	    {
	      dma_dabufp += dCnt;
	    }
	}
      else
	{
	  printf("v767(%d): Timeout. Data not ready.\n", ii);
	}
    }
  BANKCLOSE;

  tiSetOutputPort(0,0,0,0);

}

void
rocCleanup()
{
  int islot=0;

  printf("%s: Reset all modules\n",__FUNCTION__);
  
}
