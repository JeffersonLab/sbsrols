/*************************************************************************
 *
 * fadc_lhrs.c -- readout of VXS crate on L-HRS for the SBS era
 * 
 * Version 1, Sept 2021, R. Michaels
 *
 * it was copied from ...
 *
 *  ti_slave5_list.c - Library of routines for readout and buffering of
 *                    events using a JLAB Trigger Interface V3 (TI) with
 *                    a Linux VME controller in CODA 3.0.
 *
 *                    This is for a TI in Slave Mode using fiber port
 *                    5 controlled by a Master TI or Trigger
 *                    Supervisor
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

/* Define port 5 as the Slave port */
#define TI_FLAG (TI_INIT_SLAVE_FIBER_5|TI_INIT_SKIP_FIRMWARE_CHECK)

/* Measured longest fiber length in system */
#define FIBER_LATENCY_OFFSET 0x50

#include "dmaBankTools.h"   /* Macros for handling CODA banks */
#include "tiprimary_list.c" /* Source required for CODA readout lists using the TI */
#include "sdLib.h"
#include "fadcLib.h"
#include "fadc250Config.h"

/* Library to pipe stdout to daLogMsg */
#include "dalmaRolLib.h"
#include "usrstrutils.c" // utils to pick up CODA ROC config parameter line

/* Define block level */
unsigned int BLOCKLEVEL = 1;
/* Define buffering level */
int BUFFERLEVEL=1;


/* FADC Defaults/Globals */
#define FADC_DAC_LEVEL    3100 //was 3100
#define FADC_WINDOW_WIDTH 100 //
/* using hall B firmware, MODE = 1=waveform.  7=pulse integral */
#define FADC_MODE          1  

#define FADC_LATENCY       195 // 
#define FADC_NSB            7 // # of samples *before* Threshold crossing (TC) to include in sum
#define FADC_NSA           100 // # of samples *after* Threshold crossing (TC) to include in sum

#define chan_mask  0x0000 // chan mask for threshold setting

extern int fadcA32Base;
extern int nfadc;
extern unsigned int fadcAddrList[FA_MAX_BOARDS];   

// start with 1 FADC slot 14.  bpm/raster/helicity FADC
#define NFADC 1
/* Address of first fADC250 */
/* slot .... do not use slot 2, see halog 3881921
        3 = 0x180000
        4 = 0x200000
        5 = 0x280000
        6 = 0x300000
        7 = 0x380000
        8 = 0x400000
        9 = 0x480000
       10 = 0x500000 
       14 = 0x700000 skip to slot 14 */
#define FADC_ADDR 0x180000
/* Increment address to find next fADC250 */
#define FADC_INCR 0x080000

FILE *fd;
char afile[80],strin[80],dum[80];
int islot,slotmin,ichan,bmask;

int sync_or_unbuff;
static int buffered;
static int event_cnt = 0;
static int icnt = 0;

int bthresh[NFADC*16];

unsigned int fadcSlotMask=0;

/* for the calculation of maximum data words in the block transfer */
/* this is calculated below, depending on the mode */
unsigned int MAXFADCWORDS=0;

unsigned int fadc_window_lat=FADC_LATENCY, fadc_window_width=FADC_WINDOW_WIDTH;



/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int stat;
  int iFlag, ifa,res, ichan1;


  UINT32 addr,laddr;

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */

  init_strings();
  buffered = getflag(BUFFERED);
  if (!buffered) BUFFERLEVEL=1 ;
  printf ("Buffer flag : %d\n",buffered);

  /* Define BLock Level */
  blockLevel = BLOCKLEVEL;


  vmeDmaConfig(2,5,1);

  /* Define BLock Level variable to a default */
  blockLevel = BLOCKLEVEL;


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


  /*****************
   *   FADC SETUP
   *****************/

  //Added by Maria to test delays March 17 2021
  // tiSetTriggerPulse(1,25,3,0);

/* load thresholds (= pedestals + an offset) from a file */

/* initialize to 1 first; indices 0 - NFADC 
     correspond to min-max slot addresses */
  for (islot=0; islot<NFADC; islot++ ) {
    for (ichan=0; ichan<16; ichan++) {
      bthresh[16*islot+ichan] = 1;  /* default threshold */
    }
  }
  slotmin=(FADC_ADDR>>19); /* minimum slot # */
  printf("slot minimum %d \n",slotmin);
  if (slotmin<0 && slotmin>NFADC) {
    printf("ERROR: slot min is impossible !\n");
  }  
  sprintf(afile,"/home/sbs-onl/bob/ped/thresh1.dat");
  fd = fopen(afile,"a+");
  if(fd==NULL) {  
      printf("Error opening threshold file %s    \n",afile);
  } else {
      printf("Reading threshold file \n");
      while(fgets(strin,100,fd)!=NULL) {
	/*      printf("input %s \n",strin); */
      if (strstr(strin,"slot")!=NULL || strstr(strin,"fadcThr")!=NULL) {

/* find slot number first */
        sscanf(strin,"%s %d",dum,&islot);
        islot = islot - slotmin;
        if( islot>=0 && islot<NFADC) {
/* Scan string again; WARNING: it may segfault if not exactly this format */
            sscanf(strin,"%s %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",dum,&islot,&bthresh[16*islot+0],&bthresh[16*islot+1],&bthresh[16*islot+2],&bthresh[16*islot+3],&bthresh[16*islot+4],&bthresh[16*islot+5],&bthresh[16*islot+6],&bthresh[16*islot+7],&bthresh[16*islot+8],&bthresh[16*islot+9],&bthresh[16*islot+10],&bthresh[16*islot+11],&bthresh[16*islot+12],&bthresh[16*islot+13],&bthresh[16*islot+14],&bthresh[16*islot+15]);
	}
      }
      }
  }
  for(ichan=0; ichan<16; ichan++) bthresh[16*(NFADC-1)+ichan]=1; /* BPM, raster */

  /*******************
   *   FADC250 SETUP
   *******************/
  fadcA32Base=0x09000000;
  iFlag = 0;
  iFlag |= FA_INIT_EXT_SYNCRESET; /* External (VXS) SyncReset*/
  iFlag |= FA_INIT_VXS_TRIG;      /* VXS Input Trigger */
  iFlag |= FA_INIT_VXS_CLKSRC;    /* Internal Clock Source (Will switch later) */
  iFlag |= FA_INIT_USE_ADDRLIST;  /* use the fadcAddrList */

  vmeSetQuietFlag(1);

/* Build the list of slots.  This is needed because we skip to slot 14 at the end */

// start with 1 FADC slot 14

/*  for (ifa = 0; ifa<8; ifa++) {
     fadcAddrList[ifa] = FADC_ADDR +ifa*FADC_INCR;
     } */
  fadcAddrList[0] = 0x700000;  // BPM/raster/helicity FADC in slot 14

  for (ifa=0; ifa<NFADC; ifa++) {
    printf("fadc addr[%d]=0x%x \n",ifa,fadcAddrList[ifa]);

  }

  faInit(FADC_ADDR, FADC_INCR, NFADC, iFlag);

  vmeSetQuietFlag(0);

  /* FADC firmware used */
  /* ctrl   proc   */
  /* 0x02b3 0x1b05 */

  if(nfadc>1)
    faEnableMultiBlock(1);

  fadcSlotMask=faScanMask();

  printf("nfadc %d  slot mask = 0x%x\n",nfadc, fadcSlotMask);

  for(ifa = 0; ifa< nfadc; ifa++) {

      printf("\nfadc slot  faSlot[%d] = %d  thresholds:\n",ifa,faSlot(ifa));
      faEnableBusError(faSlot(ifa));

      /* Set the internal DAC level */
      faSetDAC(faSlot(ifa), FADC_DAC_LEVEL, 0xffff);

      /* Set the threshold for data readout */
      for (ichan=0; ichan<16; ichan++) {
	bmask=(1<<ichan);
	/*        faSetThreshold(faSlot(ifa), bthresh[16*ifa+ichan], bmask);
		  printf(" %d ",bthresh[16*islot+ichan]); */
	/*        faSetThreshold(faSlot(ifa), 1, bmask); */
	/*       printf(" %d ",bthresh[16*islot+ichan]);*/
      }

/**********************************************************************************
* faSetProcMode(int id, int pmode, unsigned int PL, unsigned int PTW,
*    int NSB, unsigned int NSA, unsigned int NP,
*    unsigned int NPED, unsigned int MAXPED, unsigned int NSAT);
*
*  id    : fADC250 Slot number
*  pmode : Processing Mode
*          9 - Pulse Parameter (ped, sum, time)
*         10 - Debug Mode (9 + Raw Samples)
*    PL : Window Latency
*   PTW : Window Width
*   NSB : Number of samples before pulse over threshold
*   NSA : Number of samples after pulse over threshold
*    NP : Number of pulses processed per window
* the last 3 are deprecated -- hall B software
*  NPED : Number of samples to sum for pedestal
*MAXPED : Maximum value of sample to be included in pedestal sum
*  NSAT : Number of consecutive samples over threshold for valid pulse
*/

     faSetProcMode(faSlot(ifa),
		    FADC_MODE,
		    FADC_LATENCY,
		    FADC_WINDOW_WIDTH,
		    FADC_NSB,   /* NSB */
		    FADC_NSA,   /* NSA */
		    4,   /* NP */
		   0);
 

  } /* loop over slots of FADC */

  faGStatus(0);




}


/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{

   int ifa;

  /* Program/Init VME Modules Here */
  for(ifa=0;ifa<nfadc;ifa++)
    {
      /* undefined in this version :
                faEnableSyncSrc(faSlot(ifa));*/
      faSoftReset(faSlot(ifa),0);
      faResetToken(faSlot(ifa));
      faResetTriggerCount(faSlot(ifa));
    }



  /* EXAMPLE: User bank of banks added to prestart event */
  UEOPEN(500,BT_BANK,0);

  /* EXAMPLE: Bank of data in User Bank 500 */
  CBOPEN(1,BT_UI4,0);
  *rol->dabufp++ = 0x11112222;
  *rol->dabufp++ = 0x55556666;
  *rol->dabufp++ = 0xaabbccdd;
  CBCLOSE;

  UECLOSE;

  DALMAGO;
  tiStatus(0);
  DALMASTOP;

  printf("PreStart: FADC setup parameters mode %d  latency %d  window %d \n",FADC_MODE, FADC_LATENCY,FADC_WINDOW_WIDTH);


  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{

  int fadc_mode = 0;
  unsigned int pl=0, ptw=0, nsb=0, nsa=0, np=0;

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


 faGSetBlockLevel(blockLevel);

  /* Get the FADC mode and window size to determine max data size */
  /* I think this is in the hall B lib which I don't have yet */
  /*  faGetProcMode(faSlot(0), &fadc_mode, &pl, &ptw,
      &nsb, &nsa, &np); */

  /* Set Max words from fadc (proc mode == 1 produces the most)
     nfadc * ( Block Header + Trailer + 2  # 2 possible filler words
               blockLevel * ( Event Header + Header2 + Timestamp1 + Timestamp2 +
	                      nchan * (Channel Header + (WindowSize / 2) )
             ) +
     scaler readout # 16 channels + header/trailer
   */

  MAXFADCWORDS = nfadc * (4 + blockLevel * (4 + 20 * (1 + (fadc_window_width / 2))) + 18);

  printf("MAXFADCWORDS = %d %d %d %d \n",nfadc,blockLevel,fadc_window_width,MAXFADCWORDS);

  faGStatus(0);

  faGEnable(0, 0); 

}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

  faGDisable(0);
  faGStatus(0);
  sdStatus(1);

  DALMAGO;
  tiStatus(0);
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
  int ev_type = 0;
  int ifa, nwords;
  int roCount = 0, blockError = 0;
  int rval = OK;
  int sync_flag = 0, late_fail = 0;
  int roType = 2;
  unsigned int datascan = 0, scanmask = 0;
  unsigned int event_ty = 1, event_no = 0;
  unsigned int tmp_evheader = 0;
  int islot;
  int errFlag = 0;


  /* Set TI output 1 high for diagnostics */
  tiSetOutputPort(1,0,0,0);

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

  for(ii=0; ii< 800; ii++)
    tiGetBlockLimit();

 /* fADC250 Readout */
  BANKOPEN(250,BT_UI4,blockLevel);

  /* Mask of initialized modules */
  scanmask = faScanMask();
  /* Check scanmask for block ready up to 400 times */
  
  datascan = faGBlockReady(scanmask, 400);
  stat = (datascan == scanmask);

  if(stat)
    {
      if(nfadc == 1)
	roType = 1;   /* otherwise roType = 2   multiboard reaodut with token passing */
      nwords = faReadBlock(0, dma_dabufp, MAXFADCWORDS, roType);

      /* Check for ERROR in block read */
      blockError = faGetBlockError(1);

      if(blockError)
	{
	  printf("ERROR: in transfer (event = %d), nwords = 0x%x\n",
		 roCount, nwords);
	  for(ifa = 0; ifa < nfadc; ifa++)
	    faResetToken(faSlot(ifa));

	  if(nwords > 0)
	    dma_dabufp += nwords;
	}
      else
	{
	  dma_dabufp += nwords;
	  faResetToken(faSlot(0));
	}
    }
  else
    {
      printf("ERROR: Event %d: Datascan != Scanmask  (0x%08x != 0x%08x)\n",
	     roCount, datascan, scanmask);
    }

  BANKCLOSE;

  if(tiGetSyncEventFlag() == 1|| !buffered)
    {
      /* Flush out TI data, if it's there (out of sync) */
      int davail = tiBReady();
      if(davail > 0)
	{
	  printf("%s: ERROR: TI Data available (%d) after readout in SYNC event \n", __func__, davail);

	  while(tiBReady())
	    {
	      vmeDmaFlush(tiGetAdr32());
	    }
	}

      /* Flush out other modules too, if necessary */
      // flush FADCs
      scanmask = faScanMask();
      /* Check scanmask for block ready up to 100 times */
      datascan = faGBlockReady(scanmask, 100);
      stat = (datascan == scanmask);
      stat =0;
      if (stat > 0)
	{
	  printf("data left in FADC FIFO at sync event\n");
	  //FADC sync event bank
	  // nwords = faReadBlock(0, dma_dabufp, 5000, 2);	//changed rflag = 2 for Multi Block transfer 5/25/17
	  BANKOPEN(0xbad,BT_UI4,0);
	  *dma_dabufp++ = LSWAP(0xfadc250);
	  nwords = faReadBlock(faSlot(0), dma_dabufp, 7200, 2);
	  // nwords = 0;
	  // nwords = 0;
	  if (nwords < 0)
	    {
	      printf("ERROR: in transfer (event = %d), nwords = 0x%x\n",
		    event_cnt, nwords);
	        *dma_dabufp++ = LSWAP(0xda000bad);

	    }
	  else
	    {
	      dma_dabufp += nwords;
	    }
	    BANKCLOSE;
	  for (islot = 0; islot < nfadc; islot++)	// 5/25/17
	    faResetToken(faSlot(islot));
	  for(islot = 0; islot < nfadc; islot++)
	    {
	      int davail = faBready(faSlot(islot));
	      if(davail > 0)
		{
		  printf("%s: ERROR: fADC250 Data available (%d) after readout in SYNC event \n", __func__, davail);

		  while(faBready(faSlot(islot)))
		    {
		      vmeDmaFlush(faGetA32(faSlot(islot)));
		    }
		}

	    }
	}

    } /* if(tiGetSyncEventFlag() == 1|| !buffered) */




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
  dalmaClose();

  printf("%s: Reset all FADCs\n",__FUNCTION__);
  faGReset(1);

}
