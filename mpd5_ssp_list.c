/*************************************************************************
 *
 *  mpd5_ssp_list.c - Library of routines for readout and buffering of
 *                events using a JLAB Trigger Interface V3 (TI) with
 *                a Linux VME controller.
 *
 *       Readout of APV->MPD via SSP
 *
 */

/* Event Buffer definitions */
/* Default block level */
unsigned int BLOCKLEVEL=1;
#define BUFFERLEVEL 1
#define MAX_EVENT_LENGTH   32768*12*BLOCKLEVEL      /* Size in Bytes */
#define SSP_MAX_EVENT_LENGTH 32000*12*BLOCKLEVEL     //SSP block length
#define MAX_EVENT_POOL     50

// Uncomment below and re-compile to test SSP readout without MPD fiber enabled
//#define NO_MPD_TEST

/* Define Interrupt source and address */
#define TI_SLAVE
#ifdef TI_MASTER
#define TI_READOUT TI_READOUT_EXT_POLL  /* Poll for available data, external triggers */
#else
#define TI_READOUT TI_READOUT_TS_POLL  /* Poll for available data, TS triggers */
#endif
#define TI_ADDR    (21<<19)          /* GEO slot 20 */

#define FIBER_LATENCY_OFFSET 0x10  /* measured longest fiber length */

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "tiprimary_list.c" /* source required for CODA */
#include "dmaBankTools.h"
#include "mpdLib.h"
#include "mpdConfig.h"
#include "sspMpdConfig.h"
#include "sspLib.h"
#include "sdLib.h"

extern unsigned int sspA32Base;

int last_soft_err_cnt[32];

/* ssp defs */
int SSP_READOUT=1;
extern int sspSoftReset(int id);
extern int sspMpdGetSoftErrorCount(int id, int fiber);

/*MPD Definitions*/
extern uint32_t mpdRead32(volatile uint32_t * reg);
extern int I2C_SendStop(int id);

int fnMPD=0;
int mpd_evt[22];

extern volatile struct mpd_struct *MPDp[(MPD_MAX_BOARDS+1)]; /* pointers to MPD memory map */

// End of MPD definition


void sspPrintMPD_OB_STATUS(){
  uint32_t data;
  int k;
  for (k=0;k<fnMPD;k++) { // only active mpd set
    data = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.evb_fifo_word_count);
    printf ("ob_status.evb_fifo_word_count: %08x \n", data);

    data = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.event_count);
    printf ("ob_status.event_count: %08x \n", data);

    data = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.block_count);
    printf ("ob_status.block_count: %08x \n", data);


    data = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.trigger_count);
    printf ("ob_status.trigger_count: %08x \n", data);

    data = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.missed_trigger);
    printf ("ob_status.missed_trigger: %08x \n", data);

    data = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.incoming_trigger);
    printf ("ob_status.incoming_trigger: %08x \n", data);

    data = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.sdram_flag_wc);
    printf ("ob_status.sdram_flag_wc: %08x \n", data);

    data = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.output_buffer_flag_wc);
    printf ("ob_status.output_buffer_flag_wc: %08x \n", data);
    if( data & 0x20000000 )	// Evt_Fifo_Full
      printf ("MPD %d FIFO full\n", mpdSlot(k));

    printf("\n");

    //	data = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.output_buffer_rd_addr);
    //	printf ("ob_status.output_buffer_rd_addr: %08x \n", data);

    //	data = mpdRead32(&MPDp[mpdSlot(k)]->ob_status.output_buffer_wr_addr);
    //printf ("ob_status.output_buffer_wr_addr: %08x \n", data);

  }
}


void sspPrintBlock(unsigned int *pBuf, int dCnt){
  int val, pos = 0;
#ifdef SKIP_TAG5
  int tag;
#endif

  int d[6], apv, ch;
  int total = dCnt;
  while(dCnt--)
    {
      val = *pBuf++;
      val = LSWAP(val);printf("0x%08x", val);
      if((total - dCnt) % 8 != 7)
	printf(" ");

      if(val & 0x80000000)
	{
#ifdef SKIP_TAG5
	  tag = (val>>27) & 0xf;
#endif
	  pos = 0;
	}

#ifdef SKIP_TAG5
      if(tag != 5)
      	continue;
#endif

      if(pos==0)
	printf("MPD Rotary = %d, SSP Fiber = %d\n", (val>>0)&0x1f, (val>>16)&0x1f);
      else
	{
	  int idx = (pos-1) % 3;
	  d[idx*2+0] = (val>>0) & 0x1fff;
	  if(d[idx*2+0] & 0x1000) d[idx*2+0] |= 0xfffff000;
	  d[idx*2+1] = (val>>13) & 0x1fff;
	  if(d[idx*2+1] & 0x1000) d[idx*2+1] |= 0xfffff000;

	  if(idx == 0)
	    ch = (val>>26) & 0x1f;
	  else if(idx == 1)
	    ch|= ((val>>26) & 0x3) << 5;
	  else if(idx == 2)
	    {
	      apv = (val>>26) & 0x1f;

	      printf("APV%2d, CH%3d: %4d %4d %4d %4d %4d %4d\n", apv,ch,d[0],d[1],d[2],d[3],d[4],d[5]);
	    }
	}

      pos++;
    }
}

/* function prototype */
void rocTrigger(int arg);

void ssp_mpd_setup()
{
  static int just_once = 0;
  if( (just_once++) > 0)
    return;

  /*****************
   *   SSP SETUP
   *****************/
  extern int nSSP;
  sspA32Base = 0x09000000;
  int iFlag = 0, issp=0;
  int sspFiberBit = 0;
  uint32_t sspFiberMaskToInit;


  if(sspMpdConfigInit("/home/sbs-onl/bbgem-cfg/ssp_config.cfg") == ERROR)
    return;
  sspMpdConfigLoad();

  iFlag = SSP_INIT_SKIP_FIRMWARE_CHECK | SSP_INIT_MODE_VXS | 0xFFFF0000;

  sspInit(20<<19,1<<19,1,iFlag);

  sspMpdFiberReset(0);
  sspMpdFiberLinkReset(0,0xffffffff);


  for(issp=0; issp<nSSP; issp++)
    {
      sspCheckAddresses(sspSlot(issp));
      sspMpdDisable(sspSlot(issp), 0xffffffff);
#ifdef NO_MPD_TEST
      continue;
#endif
      sspFiberMaskToInit = mpdGetSSPFiberMask(sspSlot(issp));
      printf("sspSlot: %d, mask: 0x%08x", sspSlot(issp), sspFiberMaskToInit);

      while(sspFiberMaskToInit != 0){
	if((sspFiberMaskToInit & 0x1) == 1)
	  sspMpdEnable(sspSlot(issp), 0x1 << sspFiberBit);
	sspFiberMaskToInit = sspFiberMaskToInit >> 1;
	++sspFiberBit;
      }


      int build_all_samples = 1;
      //1 => For pedestal run, will write ADC samples from the APV (i.e. disable zero suppression)
      //0 => will apply the threshold and peak position logic to decide if data is written to the event (i.e. zero suppression enabled)

      int build_debug_headers = 0;
      //1 => will write extra debugging info headers about the common-mode processing (which APV chip reported data, avg A/B values and counts for each APV)
      //0 => disables extra debug info headers
      int enable_cm = 0;
      //1 => enables the common-mode subtraction logic
      //0 => disables the common-mode subtraction logic (so raw ADC samples only have the channel offset applied)

      sspMpdEbSetFlags(sspSlot(issp), build_all_samples, build_debug_headers, enable_cm);

      sspEnableBusError(sspSlot(issp));
      sspSetBlockLevel(sspSlot(issp),BLOCKLEVEL);

      //char* mpdSlot[10],apvId[10],cModeMin[10],cModeMax[10];
      int mpdSlot, apvId, cModeMin, cModeMax;
      int fiberID = -1, last_mpdSlot = -1;
      FILE *fcommon   = fopen("/home/sbs-onl/cfg/CommonModeRange.txt","r");
      FILE *fpedestal = fopen("/home/sbs-onl/cfg/pedestal.txt","r");
      if(fcommon==NULL){
	printf("no commonMode file\n");
      }else{
	printf("trying to read commonMode\n");
	while(fscanf(fcommon, "%d %d %d %d", &mpdSlot, &apvId, &cModeMin, &cModeMax)==4){
	  if(mpdSlot!=last_mpdSlot){
	    last_mpdSlot = mpdSlot;
	    fiberID++;
	  }
	  //printf("fiberID %d %d %d %d \n", fiberID, apvId, cModeMin, cModeMax);
	  cModeMin = 200;
	  cModeMax = 800;
	  sspMpdSetAvg(0, fiberID, apvId, cModeMin, cModeMax);
	}
	fclose(fcommon);
      }
      int stripNo;
      float ped_offset, ped_rms;
      fiberID = -1, last_mpdSlot = -1;
      if(fpedestal==NULL){
	printf("no pedestal file\n");
      }else{
	printf("trying to read pedestal \n");

	while(fscanf(fpedestal, "%d %d %d %f %f", &mpdSlot, &apvId, &stripNo,
		     &ped_offset, &ped_rms)==5 ){
	  if(mpdSlot!=last_mpdSlot){
	    last_mpdSlot = mpdSlot;
	    fiberID++;
	  }
	  //printf("fiberID %d %d %d %f %f \n", fiberID, apvId, stripNo, ped_offset, ped_rms);
	  ped_offset = 0;
	  ped_rms = 10;
	  sspMpdSetApvOffset(0, fiberID, apvId, stripNo, (int)ped_offset);
	  sspMpdSetApvThreshold(0, fiberID, apvId, stripNo, 5*(int)ped_rms);
	}
	fclose(fpedestal);
      }



    }
  sspSoftReset(0);
  sspPrintMigStatus(0);

  sspGStatus(0);
  sspMpdPrintStatus(0);
#ifdef NO_MPD_TEST
  return;
#endif

  /*****************
   *   MPD SETUP
   *****************/
  mpdSetPrintDebug(0x1);

  // discover MPDs and initialize memory mapping

  // In SSP mode, par1(fiber mask) and par3(number of mpds) are not used in mpdInit(par1, par2, par3, par4)
  // Instead, they come from the configuration file
  mpdInit(0x0, 0, 0, MPD_INIT_SSP_MODE | MPD_INIT_NO_CONFIG_FILE_CHECK);
  fnMPD = mpdGetNumberMPD();


  //fnMPD = 1;
  if (fnMPD<=0) { // test all possible vme slot ?
    printf("ERR: no MPD discovered, cannot continue\n");
    return;
  }

  printf(" MPD discovered = %d\n",fnMPD);

  // APV configuration on all active MPDs
  int error_status = OK;
  int k, i;
  for (k=0;k<fnMPD;k++) { // only active mpd set
    i = mpdSlot(k);

    int try_cnt = 0;

    mpdHISTO_MemTest(i);

  retry:

    printf(" Try initialize I2C mpd in slot %d\n",i);
    if (mpdI2C_Init(i) != OK) {
      printf("WRN: I2C fails on MPD %d\n",i);
    }

    printf("Try APV discovery and init on MPD slot %d\n",i);
    if (mpdAPV_Scan(i)<=0 && try_cnt < 3 ) { // no apd found, skip next
      try_cnt++;
      printf("failing retrying\n");
      goto retry;
    }



    if( try_cnt == 3 )
      {
	printf("APV blind scan failed for %d TIMES !!!!\n\n", try_cnt);
      }

    printf(" - APV Reset\n");
    fflush(stdout);
    if (mpdI2C_ApvReset(i) != OK)
      {
	printf(" * * FAILED\n");
	error_status = ERROR;
      }

    usleep(10);
    I2C_SendStop(i);


    // board configuration (APV-ADC clocks phase)
    printf("Do DELAY setting on MPD slot %d\n",i);
    mpdDELAY25_Set(i, mpdGetAdcClockPhase(i,0), mpdGetAdcClockPhase(i,1));



    // apv configuration
    printf("Configure %d APVs on MPD slot %d\n",mpdGetNumberAPV(i),i);


    // apv configuration
    mpdSetPrintDebug(0);
    printf(" - Configure Individual APVs\n");
    printf(" - - ");
    fflush(stdout);
    int itry, badTry = 0, iapv, saveError = error_status;
    error_status = OK;
    for (itry = 0; itry < 3; itry++)
      {
	if(badTry)
	  {
	    printf(" ******** RETRY ********\n");
	    printf(" - - ");
	    fflush(stdout);
	    error_status = OK;
	  }
	badTry = 0;
	for (iapv = 0; iapv < mpdGetNumberAPV(i); iapv++)
	  {
	    printf("%2d ", iapv);
	    fflush(stdout);

	    if (mpdAPV_Config(i, iapv) != OK)
	      {
		printf(" * * FAILED for APV %2d\n", iapv);
		if(iapv < (mpdGetNumberAPV(i) - 1))
		  printf(" - - ");
		fflush(stdout);
		error_status = ERROR;
		badTry = 1;
	      }
	  }
	printf("\n");
	fflush(stdout);
	if(badTry)
	  {
	    printf(" ***** APV RESET *****\n");
	    fflush(stdout);
	    mpdI2C_ApvReset(i);
	  }
	else
	  {
	    if(itry > 0)
	      {
		printf(" ****** SUCCESS!!!! ******\n");
		fflush(stdout);
	      }
	    break;
	  }

      }

    error_status |= saveError;


    // configure adc on MPD
    printf("Configure ADC on MPD slot %d\n",i);
    mpdADS5281_Config(i);

    // configure fir
    // not implemented yet

    // 101 reset on the APV
    printf("Do 101 Reset on MPD slot %d\n",i);
    mpdAPV_Reset101(i);

    // <- MPD+APV initialization ends here

  } // end loop on mpds
  //END of MPD configure

  int ibit;
  int impd, id, iapv;
  for (impd = 0; impd < fnMPD; impd++)
    {
      id = mpdSlot(impd);

      if (mpdGetApvEnableMask(id) != 0)
	{
	  printf("  MPD %2d : ", id);
	  iapv = 0;
	  for (ibit = 15; ibit >= 0; ibit--)
	    {
	      if (((ibit + 1) % 4) == 0)
		printf(" ");
	      if (mpdGetApvEnableMask(id) & (1 << ibit))
		{
		  printf("1");
		  iapv++;
		}
	      else
		{
		  printf(".");
		}
	    }
	  printf(" (#APV %d)\n", iapv);
	}
    }
  if (error_status != OK)
    daLogMsg("ERROR", "MPD initialization has errors");
}

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  printf("%s: Build date/time %s/%s\n", __func__, __DATE__, __TIME__);

  /*****************
   *   TI SETUP
   *****************/
#ifdef TI_MASTER
  tiSetTriggerSourceMask(TI_TRIGSRC_TSINPUTS | TI_TRIGSRC_PULSER |
			 TI_TRIGSRC_LOOPBACK | TI_TRIGSRC_VME);
  //  tiSetTriggerSource(TI_TRIGGER_TSINPUTS);
  //tiSetTriggerSource(TI_TRIGGER_RANDOM);

  /* Set needed TS input bits */
  tiEnableTSInput( TI_TSINPUT_1 );

  /* Load the trigger table that associates
     pins 21/22 | 23/24 | 25/26 : trigger1
     pins 29/30 | 31/32 | 33/34 : trigger2
  */
  tiLoadTriggerTable(0);
  // MPD 1 sample readout = 3.525 us = 8 * 480
  tiSetTriggerHoldoff(1,30,1); /* 1 trigger in 20*480ns window */
  //tiSetTriggerHoldoff(1,60,1); /* 1 trigger in 20*480ns window */
  tiSetTriggerHoldoff(2,0,0);  /* 2 trigger in don't care window */

  tiSetTriggerHoldoff(3,0,0);  /* 3 trigger in don't care window */
  tiSetTriggerHoldoff(4,20,1);  /* 4 trigger in 20*3840ns window */
  // tiSetTriggerHoldoff(4,1,1);  /* 4 trigger in 20*3840ns window */

  /*   /\* Set the sync delay width to 0x40*32 = 2.048us *\/ */
  tiSetSyncDelayWidth(0x54, 0x40, 1);

  /* Set number of events per block */
  tiSetBlockLevel(BLOCKLEVEL);

  tiSetEventFormat(1);

  tiSetBlockBufferLevel(BUFFERLEVEL);
#endif /* TI_MASTER */

  tiSetTriggerPulse(1,0,25,0);

  /* Set the busy source to non-default value (no Switch Slot B busy) */
  tiSetBusySource(TI_BUSY_LOOPBACK,1);

  /*   tiSetFiberDelay(10,0xcf); */

  tiSetOutputPort(1,1,0,0);

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

  // Setup in Prestart since TI 125MHz clock is used by SSP (it glitches at end of Download())
  ssp_mpd_setup();
  for(i=0;i<sizeof(last_soft_err_cnt)/sizeof(last_soft_err_cnt[0]);i++)
    last_soft_err_cnt[i] = -1;

  tiStatus(0);
  tiSetPrescale(0);
  tiSetOutputPort(1,1,0,0);

  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{
  int UseSdram, FastReadout;
  tiSetOutputPort(0,1,0,0);



  /* Enable modules, if needed, here */
  //  sspMpdMonEnable(0,7);
  /*Enable MPD*/
  UseSdram = mpdGetUseSdram(mpdSlot(0)); // assume sdram and fastreadout are the same for all MPDs
  FastReadout = mpdGetFastReadout(mpdSlot(0));
  printf(" UseSDRAM= %d , FastReadout= %d\n",UseSdram, FastReadout);
  int k, i;
  for (k=0;k<fnMPD;k++) { // only active mpd set
    i = mpdSlot(k);

    // mpd latest configuration before trigger is enabled
    mpdSetAcqMode(i, "process");

    // load pedestal and thr default values
    mpdPEDTHR_Write(i);

    // enable acq
    mpdDAQ_Enable(i);

    mpdTRIG_Enable(i);
    mpd_evt[i]=0;
  }


  /* Get the current block level */
  BLOCKLEVEL = tiGetCurrentBlockLevel();
  printf("%s: Current Block Level = %d\n",
	 __FUNCTION__,BLOCKLEVEL);
  //sspSoftReset(0);
  sspMpdPrintStatus(0);
  /* Use this info to change block level is all modules */

  tiSetBlockLimit(0); // 0: disables block limit
  tiStatus(0);
}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{
  tiSetOutputPort(1,1,0,0);
  tiStatus(0);

  //mpd close
  int k;
  for (k=0;k<fnMPD;k++) { // only active mpd set
    mpdTRIG_Disable(mpdSlot(k));
  }
  //mpd close

  printf("rocEnd: Ended after %d blocks\n",tiGetIntCount());
  tiSetBlockLimit(0);

}


/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  static int evt = 1;
  static int16_t words_expected = -1;
  int sync_flag = tiGetSyncEventFlag();
#ifdef LOUD_MPD_READOUT
  printf("*** This is start of event %d\n", evt);
#endif

  //usleep(1000);//for testing ssp wordcnt, needs to be removed
  int dCnt;
  int ssp_timeout;
  uint32_t bc, wc, ec;
  static int tcnt = 0;
  int count;
  int do_soft_err;
  //tiStatus(1);

  tiSetOutputPort(1,0,0,0);

  vmeDmaConfig(2,5,1);
  dCnt = tiReadTriggerBlock(dma_dabufp);
  if(dCnt<=0)
    {
      printf("No data or error.  dCnt = %d\n",dCnt);
      daLogMsg("ERROR","TI returned dCnt = %d\n",dCnt);
      tiSetBlockLimit(1);
    }
  else
    {
      dma_dabufp += dCnt;
    }

  /* Readout SSP */
  BANKOPEN(10,BT_UI4,0);

  ssp_timeout=0;
  int ssp_timeout_max=10000;

  while ((sspBReady(0)==0) && (ssp_timeout<ssp_timeout_max))
    {
      ssp_timeout++;
#ifdef DEBUG_TIMEOUT
      sspPrintEbStatus(0);
#endif
    }

#ifdef DEBUG_BREADY
  sspPrintEbStatus(0);
#endif

  tiSetOutputPort(1,1,0,0);

  /*
    printf("*** Below are Status of any event(good/bad both, no MPD access, only ssp)\n");
    sspMpdPrintStatus(0);
    getchar();
    sspPrintMPD_OB_STATUS();
    mpdGStatus(1);
    sspPrintEbStatus(0);
  */

  /*
    int result;
    for(i=11;i<12;i++)
    {
    printf("Port %d:\n", i);
    result = sspGetMpdRxLen(0, i);
    printf("MPD MAX RxLen %d: %d \n",i,     (result >>  0) & 0xffff);
    printf("MPD current RxLen %d: %d \n",i, (result >> 16) & 0xffff);

    result = sspGetEB_wordCount(0, i);
    printf("MPD EB word count: %d: %d \n",i,     (result >>  0) & 0xffff);
    printf("MPD elapsed time: %d: %d \n",i, (result >> 16) & 0xffff);
    printf("\n\n");
    }
  */

  do_soft_err = 0;
  int i;
  for(i=0;i<12;i++)
    {
      if( (1 << i) & mpdGetSSPFiberMask(0) )
	{
	  count = sspMpdGetSoftErrorCount(0,i);
	  if(last_soft_err_cnt[i] < 0)
	    last_soft_err_cnt[i] = count;
	  else if(last_soft_err_cnt[i] != count)
	    {
	      last_soft_err_cnt[i] = count;
	      do_soft_err = 1;
	    }
	}
    }

  if(do_soft_err)
    {
      printf("*** SSP SOFT ERROR ***\n");

      //  printf("*** Dumping ssp mpd monitor sspMpdMonDump()\n");
      //  sspMpdMonDump(0,7);
      //  printf("*** sspMpdMonDump() ends\n");

      printf("*** Status of MPD and SSP before reset ***\n");
      sspPrintMPD_OB_STATUS();
      sspMpdPrintStatus(0);
      mpdGStatus(1);
      sspPrintEbStatus(0);

      printf("*** Press Enter to continue***\n");
      getchar();
    }

  if (ssp_timeout == ssp_timeout_max )
    {
      printf("*** SSP TIMEOUT ***\n ");


      // sspMpdFiberReset(0);
      //sspSoftReset(0);

      printf("*** Status of MPD and SSP before reset ***\n");
      sspPrintMPD_OB_STATUS();
      sspMpdPrintStatus(0);
      mpdGStatus(1);
      sspPrintEbStatus(0);
      //      printf("*** Dumping ssp mpd monitor sspMpdMonDump()\n");
      //      sspMpdMonDump(0,7);
      //      printf("*** sspMpdMonDump() ends\n");
      //vmeDmaConfig(2,5,1);
      printf("Trying to read w/e there are in ssp...\n");
      sspGetEbStatus(0, &bc, &wc, &ec);
      dCnt = sspReadBlock(0, dma_dabufp, wc,1);
      unsigned int *pBuf = (unsigned int *)dma_dabufp;
      printf("dCnt read: %d\n", dCnt);
      if(dCnt > 0)
	dma_dabufp += dCnt;

      tcnt++;
      if(!(tcnt & 0x3ff))
	printf("tcnt = %u, EV Header: %u, MPD HDR = %u\n", tcnt&0xFFF, LSWAP(pBuf[1])&0xFFF, LSWAP(pBuf[5])&0xFFF);

      //      sspPrintBlock(pBuf, dCnt);
      printf("*** Press Enter to start reset procedure***\n");
      getchar();
      sspMpdFiberReset(0);
      /*
      //tiSetBlockLimit(1);

      printf("*** Invoking sspMpdFiberReset()***\n");
      sspMpdFiberReset(0);
      printf("*** Invoking sspSoftReset() (tempararily skipped for testing)***\n");
      // sspSoftReset(0);

      int i_re, k_re;
      printf("*** Invoking MPD resets***\n");
      for(k_re=0;k_re<fnMPD;k_re++)
      {
      i_re = mpdSlot(k_re);
      mpdDAQ_Disable(i_re);
      }
      usleep(100);
      for(k_re=0;k_re<fnMPD;k_re++)
      {
      i_re = mpdSlot(k_re);
      // mpd latest configuration before trigger is enabled
      mpdSetAcqMode(i_re, "process");

      // load pedestal and thr default values
      mpdPEDTHR_Write(i_re);

      // enable acq
      mpdDAQ_Enable(i_re);

      printf("Do 101 Reset on MPD slot %d\n",i);
      mpdAPV_Reset101(i_re);
      }

      //sspSoftReset(0);


      printf("\n\n*** Status of MPD and SSP after reset ***\n");
      sspPrintMPD_OB_STATUS();
      sspMpdPrintStatus(0);
      mpdGStatus(1);
      sspPrintEbStatus(0);
      printf("*** Reset done!!! press enter to continue...*** \n ");
      getchar();
      */
    }
  else
    {
#ifdef LOUD_MPD_READOUT
      printf("***This event doesn't have timeout, but printing data for checking\n");
      sspPrintEbStatus(0);
#endif
      dCnt = sspReadBlock(0, dma_dabufp, SSP_MAX_EVENT_LENGTH>>2,1);
#ifdef LOUD_MPD_READOUT
      unsigned int *pBuf = (unsigned int *)dma_dabufp;
      tcnt++;
      /* if(!(tcnt & 0x3ff)) */
      printf("tcnt = 0x%x\n",
	     tcnt);

      int iword;
      for(iword = 0; iword < 10; iword++)
	printf("  %2d: 0x%08x\n", iword,
	       LSWAP(pBuf[iword]));
      //         sspPrintBlock(pBuf, dCnt);
      printf("words read: Cnt = %d\n",dCnt);
      //      getchar();
#endif
      if(SSP_READOUT)
	{
	  dma_dabufp += dCnt;
	}
      else
	{
	  *dma_dabufp++ = LSWAP(ssp_timeout);
	}

      if(evt < 5) {
      }else if(words_expected == -1){
	words_expected = dCnt;
      }else{
	if(dCnt != words_expected){
	  //          char c = 'n';
	  //          time_t t;
	  //         FILE *f = fopen("/home/daq/ssp_failure.txt", "wt");
	  //          time(&t);
	  //          fprintf(f, "failed @ %s\n", ctime(&t));
	  //          fclose(f);
	  //	  printf("***unexpected count of words\n");
	  //	  printf("***printing data:\n");
	  //	  sspPrintBlock(pBuf, dCnt);
	  //	  printf("*** Dumping ssp mpd monitor sspMpdMonDump()\n");
	  //          sspMpdMonDump(0,7);
	  //	  printf("*** sspMpdMonDump() ends\n");
	  //	  printf("***printing status...\n");
	  //	  sspPrintMPD_OB_STATUS();
	  //	  sspMpdPrintStatus(0);
	  //	  mpdGStatus(1);
	  //	  sspPrintEbStatus(0);
	  //	  printf("Continue? ");
	  //          while(c != 'y')
	  //            scanf("%c", &c);
	}
      }

      if(dCnt<=0)
	{
	  printf("No data or error.  dCnt = %d\n",dCnt);
	  // tiSetBlockLimit(1); ---danning comment for the following try on resetting mpd
	  //---------trying to reset mpd ---danning


	  //	  FILE *fout;
	  //	  fout = fopen("errorCount_out.txt","a");
	  //	  fprintf(fout,"reset, wordcount: %d  \n",wordcnt);
	  //	  fclose(fout);


	}
      else
	{
	  //comment next line to disable GEM data
	  //dma_dabufp += dCnt;
	}


      /*
	printf("  dCnt = %d\n",dCnt);
	for(idata=0;idata<30;idata++)
	{
	if((idata%5)==0) printf("\n\t");
	datao = (unsigned int)LSWAP(the_event->data[idata]);
	printf("  0x%08x ",datao);


	// 	if( (datao & 0x00E00000) == 0x00A00000 ) {
	// 	    mpd_evt[i]++;
	// 	    evt=mpd_evt[i];
	// 	}
	// 	evt = (evt > mpd_evt[i]) ? mpd_evt[i] : evt; // evt is the smallest number of events of an MPD
	}

	printf("\n\n");
      */
    }
  // printf("Ssp soft resetting \n");sspSoftReset(0);
  //if(evt % 1000 == 0)
  ////    printf("*** This is end event number: %d , dCnt: %d \n", evt, dCnt);
  evt++;
  //getchar();

  //  sspMpdMonEnable(0,7);

  BANKCLOSE;

  /* Sync Event checks.   Modules should not have any more data here */
  if(sync_flag)
    {
#ifdef DEBUG_SYNC_CHECK
      printf("%s: (%d) Sync Event check\n",
	   __func__, tiGetIntCount());
#endif
      int bready = tiBReady(0);
      if (bready > 0)
	{
	  printf("%s: Error at sync event\n",
		 __func__);
	  printf("   TI blocks ready = %d\n",
		 bready);
	}


      sspGetEbStatus(0, &bc, &wc, &ec);
      if( (bc > 0) )//|| (wc > 0) || (ec > 0))
	{
	  printf("%s: Error at sync event\n",
		 __func__);
	  sspPrintEbStatus(0);
	}
      bready = sspBReady(0);
      if (bready > 0)
	{
	  printf("%s: Error at sync event\n",
		 __func__);
	  printf("   SSP blocks ready = %d\n",
		 bready);
	}

    }

  tiSetOutputPort(0,0,0,0);

}

void
rocCleanup()
{

}

void
setSSPData(int enable)
{
  vmeBusLock();

  if(enable)
    SSP_READOUT = 1;
  else
    SSP_READOUT = 0;

  vmeBusUnlock();
}

/*
  Local Variables:
  compile-command: "make -k -B mpd5_ssp_list.so"
  End:
*/
