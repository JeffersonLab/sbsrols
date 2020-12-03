/*************************************************************************
 *
 *  mpd_list.c - Library of routines for readout and buffering of
 *                events using a JLAB Trigger Interface V3 (TI) with
 *                a Linux VME controller.
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     100
#define MAX_EVENT_LENGTH   1024*800	/* Size in Bytes */

/* Define TI_MASTER or TI_SLAVE in Makefile !! */
/* Poll for available data, external triggers */

#ifdef TI_MASTER
#define TI_READOUT TI_READOUT_EXT_POLL
#endif

#ifdef TI_SLAVE
#define TI_READOUT TI_READOUT_TS_POLL
#endif


/* GEO slot 21 */
#define TI_ADDR    (21 << 19)

#define FIBER_LATENCY_OFFSET 0x10	/* measured longest fiber length */
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "tiprimary_list.c"	/* source required for CODA */
#include "mpdLib.h"
#include "mpdConfig.h"
#include "dmaBankTools.h"

/*MPD Definitions*/

int fnMPD = 0;
int errorStreak = 0;
extern int mpdOutputBufferBaseAddr;	/* output buffer base address */
extern int I2C_SendStop(int id);
#define DMA_BUFSIZE 80000

// End of MPD definition

/* Buffer to store initialized MPD/APV table */
char apvbuffer[1028];
char errorbuffer[1028];
char *bufp;

/* Default block level */
unsigned int BLOCKLEVEL = 1;
#define BUFFERLEVEL 1

/* function prototype */
void rocTrigger(int arg);
int resetMPDs(unsigned int *broken_list, int nbroken);

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{
  int error_status = OK;

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2, 3, 1);

  /*****************
   *   TI SETUP
   *****************/
  /* Set prompt output width (100 + 2) * 4 = 408 ns */
  tiSetPromptTriggerWidth(127);

  /* Set crate ID */
  tiSetCrateID(0x01);		/* ROC 1 */

/* #define INTRANDOMPULSER */
#ifdef INTRANDOMPULSER
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

#ifdef TI_MASTER
  /* Set the busy source to non-default value (no Switch Slot B busy) */
  tiSetBusySource(TI_BUSY_LOOPBACK, 1);

  /* Set number of events per block */
  tiSetBlockLevel(BLOCKLEVEL);

  if(strcmp("crate0_1",rol->usrString) == 0)
    {
      /* Add Crate1 as a slave */
      tiAddSlave(1);
    }

  if(strcmp("crate0_12",rol->usrString) == 0)
    {
      /* Add Crate1 as a slave */
      tiAddSlave(1); // port #1
      /* Add Crate2 as a slave */
      tiAddSlave(5); // port #5
    }

  if(strcmp("crate0_2",rol->usrString) == 0)
    {
      /* Add Crate2 as a slave */
      tiAddSlave(5); // port #5
    }
#endif

  tiSetBlockBufferLevel(BUFFERLEVEL);

  tiStatus(0);

  /*****************
   *   MPD SETUP
   *****************/
  int rval = OK;
  unsigned int first_mpd_address = (2 << 19);
  int numberOfMPDsToScan;
  unsigned int errSlotMask = 0;


  /*Read config file and fill internal variables */
  rval =
    mpdConfigInit("/home/sbs-onl/infn-cfg/newConfigFile/mpd_config.cfg");
  first_mpd_address = (2 << 19);
  numberOfMPDsToScan = 19;

  if(rval != OK)
    {
      daLogMsg("ERROR","Error in configuration file");
      error_status = ERROR;
    }
  mpdConfigLoad();

  /* Init and config MPD+APV */

  // discover MPDs and initialize memory mapping
  //mpdInit(first_mpd_address, 0x80000, 19, 0x0);
  mpdInit(first_mpd_address, 0x80000, numberOfMPDsToScan, 0x0);
  fnMPD = mpdGetNumberMPD();

  if (fnMPD <= 0)
    {				// test all possible vme slot ?
      printf("ERR: no MPD discovered, cannot continue\n");
      return;
    }

  printf("MPD discovered = %d\n", fnMPD);

  printf("\n");

  // APV configuration on all active MPDs
  int impd, iapv, id;
  for (impd = 0; impd < fnMPD; impd++)
    {				// only active mpd set
      id = mpdSlot(impd);
      printf("MPD slot %2d config:\n", id);
      error_status = OK;

      rval = mpdHISTO_MemTest(id);

      printf(" - Initialize I2C\n");
      fflush(stdout);
      if (mpdI2C_Init(id) != OK)
	{
	  printf(" * * FAILED\n");
	  error_status = ERROR;
	  errSlotMask |= (1 << id);
	}

#if 0
      printf(" - APV Reset\n");
      if (mpdI2C_ApvReset(id) != OK)
	{
	  printf(" * * FAILED\n");
	  error_status = ERROR;
	}
#endif
      printf(" - APV discovery and init\n");

      fflush(stdout);
      mpdSetPrintDebug(0x0);
      if (mpdAPV_Scan(id) <= 0)
	{			// no apd found, skip next
	  printf(" * * None Found\n");
	  error_status = ERROR;
	  errSlotMask |= (1 << id);
	  continue;
	}
      mpdSetPrintDebug(0);

      // apv reset
      printf(" - APV Reset\n");
      fflush(stdout);
      if (mpdI2C_ApvReset(id) != OK)
	{
	  printf(" * * FAILED\n");
	  errSlotMask |= (1 << id);
	  error_status = ERROR;
	}

      usleep(10);
      I2C_SendStop(id);
      // board configuration (APV-ADC clocks phase)
      // (do this while APVs are resetting)
      printf(" - DELAY setting\n");
      fflush(stdout);
      if (mpdDELAY25_Set
	  (id, mpdGetAdcClockPhase(id, 0), mpdGetAdcClockPhase(id, 1)) != OK)
	{
	  printf(" * * FAILED\n");
	  errSlotMask |= (1 << id);
	  error_status = ERROR;
	}

      // apv configuration
      mpdSetPrintDebug(0);
      printf(" - Configure Individual APVs\n");
      printf(" - - ");
      fflush(stdout);
      int itry, badTry = 0, saveError = error_status;
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
	  for (iapv = 0; iapv < mpdGetNumberAPV(id); iapv++)
	    {
	      printf("%2d ", iapv);
	      fflush(stdout);

	      if (mpdAPV_Config(id, iapv) != OK)
		{
		  printf(" * * FAILED for APV %2d\n", iapv);
		  if(iapv < (mpdGetNumberAPV(id) - 1))
		    printf(" - - ");
		  fflush(stdout);
		  error_status = ERROR;
		  badTry = 1;
		  break;
		}
	    }
	  printf("\n");
	  fflush(stdout);
	  if(badTry)
	    {
	      printf(" ***** APV RESET *****\n");
	      fflush(stdout);
	      mpdI2C_ApvReset(id);
	    }
	  else
	    {
	      if(itry > 0)
		{
		  printf(" ****** SUCCESS!!!! ******\n");
		  error_status = OK;
		  fflush(stdout);
		}
	      break;
	    }

	}

      error_status |= saveError;

      if(error_status == ERROR)
	errSlotMask |= (1 << id);

      mpdSetPrintDebug(0);

      // configure adc on MPD
      printf(" - Configure ADC\n");
      fflush(stdout);
      if (mpdADS5281_Config(id) != OK)
	{
	  printf(" * * FAILED\n");
	  errSlotMask |= (1 << id);
	  error_status = ERROR;
	}

      // configure fir
      // not implemented yet

      // RESET101 on the APV
      printf(" - Do APV RESET101\n");
      fflush(stdout);
      if (mpdAPV_Reset101(id) != OK)
	{
	  printf(" * * FAILED\n");
	  errSlotMask |= (1 << id);
	  error_status = ERROR;
	}

      // <- MPD+APV initialization ends here
      printf("\n");
      fflush(stdout);
    }				// end loop on mpds
  //END of MPD configure

  mpdGStatus(1);

  // summary report
  bufp = (char *) &(apvbuffer[0]);

  rval = sprintf(bufp, "\n");
  if(rval > 0)
    bufp += rval;
  rval = sprintf(bufp, "Configured APVs (ADC 15 ... 0)\n");
  if(rval > 0)
    bufp += rval;

  int ibit;
  for (impd = 0; impd < fnMPD; impd++)
    {
      id = mpdSlot(impd);

      if (mpdGetApvEnableMask(id) != 0)
	{
	  rval = sprintf(bufp, "  MPD %2d : ", id);
	  if(rval > 0)
	    bufp += rval;
	  iapv = 0;
	  for (ibit = 15; ibit >= 0; ibit--)
	    {
	      if (((ibit + 1) % 4) == 0)
		{
		  rval = sprintf(bufp, " ");
		  if(rval > 0)
		    bufp += rval;
		}
	      if (mpdGetApvEnableMask(id) & (1 << ibit))
		{
		  rval = sprintf(bufp, "1");
		  if(rval > 0)
		    bufp += rval;
		  iapv++;
		}
	      else
		{
		  rval = sprintf(bufp, ".");
		  if(rval > 0)
		    bufp += rval;
		}
	    }
	  rval = sprintf(bufp, " (#APV %d)", iapv);
	  if(rval > 0)
	    bufp += rval;
	  if(errSlotMask & (1 << id))
	    {
	      rval = sprintf(bufp, " INIT ERRORS\n");
	      if(rval > 0)
		bufp += rval;
	    }
	  else
	    {
	      rval = sprintf(bufp, "\n");
	      if(rval > 0)
		bufp += rval;
	    }
	}
      else
	{
	  rval = sprintf(bufp,
			 "  MPD %2d :                                INIT ERRORS\n", id);
	  if(rval > 0)
	    bufp += rval;
	}
    }
  rval = sprintf(bufp, "\n");
  if(rval > 0)
    bufp += rval;

  printf("%s",apvbuffer);

  if (errSlotMask != 0)
    daLogMsg("ERROR", "MPD initialization has errors");


  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{

  tiStatus(0);
  vmeCheckMutexHealth(1);
  tiSetBlockLimit(0);

  printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{
  int impd, id;
  /* Enable modules, if needed, here */
  daLogMsg("INFO", apvbuffer);


  /*Enable MPD */
  mpdOutputBufferBaseAddr = 0x08000000;
  for (impd = 0; impd < fnMPD; impd++)
    {				// only active mpd set
      id = mpdSlot(impd);

      // mpd latest configuration before trigger is enabled
      mpdSetAcqMode(id, "process");

      // load pedestal and thr default values
      mpdPEDTHR_Write(id);

      // enable acq
      mpdDAQ_Enable(id);

      if (mpdAPV_Reset101(id) != OK)
        {
          printf("MPD Slot %2d: Reset101 FAILED\n", id);
        }
    }

  /*
     Get the current block level
     Use this info to change block level is all modules
   */

  BLOCKLEVEL = tiGetCurrentBlockLevel();
  printf("%s: Current Block Level = %d\n", __FUNCTION__, BLOCKLEVEL);

  /* Check MPDs for data */
  int sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords;
  int obuf_nblock = 0, empty = 0, full = 0, nwords = 0;
  for (impd = 0; impd < fnMPD; impd++)
    {				// only active mpd set
      id = mpdSlot(impd);
      mpdSDRAM_GetParam(id, &sd_init, &sd_overrun, &sd_rdaddr, &sd_wraddr,
			&sd_nwords);

      if ((sd_nwords != 0) || (sd_overrun == 1) || (sd_init == 0))
	{
	  printf("ERROR: Slot %2d SDRAM status: \n"
		 "init=%d, overrun=%d, rdaddr=0x%x, wraddr=0x%x, nwords=%d\n",
		 id, sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords);
	}

      obuf_nblock = mpdOBUF_GetBlockCount(id);
      mpdOBUF_GetFlags(id, &empty, &full, &nwords);

      if ((obuf_nblock != 0) || (empty == 0) || (full == 1) || (nwords != 0))
	{
	  printf("ERROR: Slot %2d OBUF status: \n"
		 "nblock = %d  empty=%d  full=%d  nwords=%d\n",
		 id, obuf_nblock, empty, full, nwords);
	}
    }

  mpdGStatus(1);

#ifdef INTRANDOMPULSER
  /* Enable Random at rate 500kHz/(2^7) = ~3.9kHz */
  tiSetRandomTrigger(1,0x7);
#endif
}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

  int impd;

#ifdef INTRANDOMPULSER
  /* Disable random pulser */
  tiDisableRandomTrigger();
#endif

  tiStatus(0);

  //mpd disable
  for (impd = 0; impd < fnMPD; impd++)
    {
      mpdDAQ_Disable(mpdSlot(impd));
    }

  printf("rocEnd: Ended after %d blocks\n", tiGetIntCount());

}

/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int arg)
{
  int dCnt, iw;
  int ti_data_offset = 0;
  int UseSdram, FastReadout;
  int empty, full, nwords, obuf_nblock;
  int nwread;
  int blen;
  int verbose_level = 0;
  int errFlagMask = 0;
  int errSlotMask = 0;
  int mpd_data_offset = 0;

#ifdef CHECK_EXPECTED_SIZE
  int expected_data_size[17] = {0, 0, 9440, 9440, 9440, 9440, 11796, 11796, 7868,
				   0, 9440, 9440, 9440, 9440, 11796, 11796, 7868};
#endif

  ti_data_offset = ((unsigned long) (dma_dabufp) - (unsigned long) (&the_event->data[0])) >> 2;

  /* Readout TI data */
  vmeDmaConfig(2, 3, 1);
  dCnt = tiReadTriggerBlock(dma_dabufp);

  if (dCnt <= 0)
    {
      printf("No data or error.  dCnt = %d\n", dCnt);
      ti_data_offset = 0;
    }
  else
    {
      dma_dabufp += dCnt;
    }

  if (verbose_level > 1)
    {
      printf(" (TI dump data on screen)\n");
      if (dCnt > 0)
	{

	  for (iw = 0; iw < ((dCnt > 16) ? 16 : dCnt); iw++)
	    {
	      uint32_t datao =
		LSWAP(the_event->data[iw + ti_data_offset]);

	      if (verbose_level > 1)
		{
		  if ((iw % 8) == 0)
		    {
		      printf("0x%06x:", iw);
		    }
		  printf(" 0x%08x", datao);

		  if (((iw % 8) == 7) || (iw == (dCnt - 1)))
		    {
		      printf("\n");
		    }
		}
	    }


	  printf(" - Summary: TI data word count = %d\n\n", dCnt);
	}
    }

  /* Data bank (10) for MPD data */
  BANKOPEN(10, BT_UI4, 0);

  // assume sdram and fastreadout are the same for all MPDs
  UseSdram = mpdGetUseSdram(mpdSlot(0));
  FastReadout = mpdGetFastReadout(mpdSlot(0));

  if(verbose_level > 0)
    {
      printf("\n\n ========= "
	     "UseSDRAM= %d , FastReadout= %d   Readout Event = %d\n",
	     UseSdram, FastReadout, tiGetIntCount());
    }

  // -> now trigger can be enabled

  int tout, impd, id;
  for (impd = 0; impd < fnMPD; impd++)
    {				// only active mpd set
      id = mpdSlot(impd);

      if (verbose_level > 0)
	printf("MPD %d: \n", id);

      // prepare internal variables for readout @@ use old buffer scheme, need improvement
      mpdArmReadout(id);

      if (UseSdram)
	{
	  blen = DMA_BUFSIZE;
	}
      else
	{
	  blen = mpdApvGetBufferAvailable(id, 0);
	}

      nwread = 0;

      if (UseSdram)
	{

	  int sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords;

	  mpdSDRAM_GetParam(id, &sd_init, &sd_overrun, &sd_rdaddr, &sd_wraddr,
			    &sd_nwords);
	  if (verbose_level > 0)
	    printf(" - SDRAM status: init=%d, overrun=%d, "
		   "rdaddr=0x%x, wraddr=0x%x, nwords=%d\n",
		   sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords);

	  tout = 0;

	  while (mpdOBUF_GetBlockCount(id) == 0 && tout < 1000)
	    {
	      usleep(10);
	      tout++;
	    }

	  if (tout == 1000)
	    {
	      errFlagMask |= (1 << 0);
	      errSlotMask |= (1 << id);

	      printf("WARNING: *** Timeout while waiting for data in mpd %d"
		     " - check MPD/APV configuration\n", id);
	    }

	  obuf_nblock = mpdOBUF_GetBlockCount(id);
	  // evb_nblock = mpdGetBlockCount(i);

	  if(obuf_nblock != 1)
	    printf(" Slot %2d - OBUF block count: %d \n", id, obuf_nblock);

	  if (obuf_nblock > 0)
	    {			// read data

	      mpdOBUF_GetFlags(id, &empty, &full, &nwords);

	      if (verbose_level > 0)
		printf(" - OBUF status: empty=%d, full=%d, nwords=%d\n",
		       empty, full, nwords);

	      if (FastReadout > 0)
		{		//64bit transfer
		  if (nwords < 128)
		    {
		      empty = 1;
		    }
		  else
		    {
		      nwords *= 2;
		    }
		}

	      if (full)
		{
		  printf("\n\n **** OUTPUT BUFFER FIFO is FULL in MPD %d "
			 "!!! RESET EVERYTHING !!!\n\n", id);

		  errSlotMask |= (1 << id);
		  errFlagMask |= (1 << 1);
		}
	      if (verbose_level > 0)
		printf(" - OBUF Data Ready: %d (32b-words)\n", nwords);

#ifdef CHECK_EXPECTED_SIZE
	      if (nwords != expected_data_size[id]) {
		printf(" - MPD %d has unexpected data size: %d, size expected: %d, event: %d \n", id, nwords, expected_data_size[id], tiGetIntCount());
		errSlotMask |= (1 << id);
		errFlagMask |= (1 << 3);
		//	tiSetBlockLimit(1);
	      }
#endif /* CHECK_EXPECTED_SIZE */

	      if (nwords > 0)	// was >=
		{
		  if (nwords > blen / 4)
		    {
		      nwords = blen / 4;
		    }

		  mpd_data_offset =
		    ((unsigned long) (dma_dabufp) - (unsigned long) (&the_event->data[0])) >> 2;
		  mpdOBUF_Read(id, dma_dabufp, nwords, &nwread);

		  if (verbose_level > 0)
		    printf(" - Readout: %d (32b-words)", nwread);

		  if (nwords != nwread)
		    {
		      printf
			(" * ERROR: MPD %2d OBUF Data Ready (%d) != Data Readout (%d)\n",
			 id, nwords, nwread);
		      errSlotMask |= (1 << id);
		      errFlagMask |= (1 << 2);
		    }

		  dma_dabufp += nwread;
		}
	    }
	  else
	    usleep(10);

	}
      else
	{			// if not Sdram
	  // FIXME: THIS PROCEDURE IS CURRENTLY BROKEN
	  mpdFIFO_IsEmpty(id, 0, &empty);	//  read fifo channel=0 status

	  if (!empty)
	    {			// read fifo
	      nwread = blen / 4;
	      mpdFIFO_ReadSingle(id, 0, mpdApvGetBufferPointer(id, 0, 0),
				 &nwread, 20);
	      if (nwread == 0)
		{
		  printf(" * ERROR: word read count is 0, "
			 "while some words are expected back\n");
		  errFlagMask = (1 << 0);
		}
	    }

	}

      if (verbose_level > 1)
	{
	  printf(" (dump data on screen)\n");
	  if (nwread > 0)
	    {

	      for (iw = 0; iw < ((nwread > 40) ? 40 : nwread); iw++)
		{
		  uint32_t datao =
		    LSWAP(the_event->data[iw + mpd_data_offset]);

		  if (verbose_level > 1)
		    {
		      if ((iw % 8) == 0)
			{
			  printf("0x%06x:", iw);
			}
		      printf(" 0x%08x", datao);

		      if (((iw % 8) == 7) || (iw == (nwread - 1)))
			{
			  printf("\n");
			}
		    }
		}


	      printf(" - Summary: nwords=%d  nwread=%d\n\n", nwords, nwread);
	    }
	}
    }				// active mpd loop

  if(errFlagMask){
    errorStreak++;
  }else{
    errorStreak = 0;
  }

#ifdef STOP_STREAK
  if (errorStreak >= 10){
    printf("Readout error for 10 continuous events, setting TI block limit to 1\n");
    extern int tiDoAck;
    tiDoAck=0;
    printf("   DAQ STOPPED BECAUSE OF THIS ERROR\n");
    printf("   PLEASE NOTIFY BRYAN (moffit@jlab.org)\n");
    getchar();
    getchar();
    getchar();
  }
#endif

  if (errFlagMask)
    {
      bufp = (char *) &(errorbuffer[0]);
      int rval = 0;

      int ibit = 0, nbroken = 0;
      unsigned int broken_list[21] =
	{
	  0,0,0,0,0,0,0,
	  0,0,0,0,0,0,0,
	  0,0,0,0,0,0,0
	};
      int broken_status = OK;

      daLogMsg("WARN","Errors in readout for event %d",tiGetIntCount());

      rval = sprintf(bufp, "\n * * ERRORS in Readout. Types: \n");
      if(rval > 0)
	bufp += rval;

      if(errFlagMask & (1<<0))
	{
	  rval = sprintf(bufp, "   * Empty\n");
	  if(rval > 0)
	    bufp += rval;
	}
      if(errFlagMask & (1<<1))
	{
	  rval = sprintf(bufp, "   * Full\n");
	  if(rval > 0)
	    bufp += rval;
	}
      if(errFlagMask & (1<<2))
	{
	  rval = sprintf(bufp, "   * nwords ready != nwords readout\n");
	  if(rval > 0)
	    bufp += rval;
	}
      if(errFlagMask & (1<<3))
	{
	  rval = sprintf(bufp, "   * nwords not as expected\n");
	  if(rval > 0)
	    bufp += rval;
	}

      rval = sprintf(bufp, " * * MPDS with errors: \n     ");
      if(rval > 0)
	bufp += rval;

      for (ibit = 0; ibit < 21; ibit++)
	{
	  if(errSlotMask & (1 << ibit))
	    {
	      rval = sprintf(bufp, " %2d", ibit);
	      if(rval > 0)
		bufp += rval;
	      broken_list[nbroken++] = ibit;
	    }
	}
      rval = sprintf(bufp, "\n");
      if(rval > 0)
	bufp += rval;

      daLogMsg("WARN",errorbuffer);

      if( errFlagMask & (1<<0) || errFlagMask & (1<<1) ||  errFlagMask & (1<<2)){
	tiStatus(1);
	mpdGStatus(1);
	printf(" * * Trying a reset \n");
	daLogMsg("INFO","Resetting MPDs with ERRORs");
	broken_status = resetMPDs((unsigned int *)&broken_list, nbroken);

	if(broken_status != OK)
	  {
	    printf("ERROR: Unable to reset MPDs with ERRORS\n");
	    daLogMsg("ERROR","Unable to reset MPDs with ERRORS");
	    tiSetBlockLimit(5);
	}
	else
	  {
	    printf(" * Success?  Be sure to check the data!\n");
	    daLogMsg("INFO","MPDs reset.  CHECK THE DATA");
	  }
      }

    }

  BANKCLOSE;

}

void
rocCleanup()
{
  int impd = 0, ia = 0;

  printf("%s: Free single read buffers\n", __FUNCTION__);

  for (impd = 0; impd < fnMPD; impd++)
    {
      for (ia = 0; ia < 16; ia++)
	mpdApvBufferFree(mpdSlot(impd), ia);
    }

#ifdef TI_MASTER
  tiResetSlaveConfig();
#endif
  printf("%s: Enable Fiber-mode\n", __func__);
  for(impd = 0; impd < fnMPD; impd++)
    {
      mpdFiberEnable(mpdSlot(impd));
    }

}

int
resetMPDs(unsigned int *broken_list, int nbroken)
{
  int impd = 0,  id = 0, rval = OK;
  static int ncalls = 0;

  printf("%s: Number of calls = %d\n",
	 __func__, ncalls);

  for (impd = 0; impd < nbroken; impd++)
    {
      id = broken_list[impd];
      mpdDAQ_Disable(id);
    }

  for (impd = 0; impd < nbroken; impd++)
    {				// only active mpd set
      id = broken_list[impd];

      // mpd latest configuration before trigger is enabled
      mpdSetAcqMode(id, "process");

      // load pedestal and thr default values
      mpdPEDTHR_Write(id);

      // enable acq
      mpdDAQ_Enable(id);

      if (mpdAPV_Reset101(id) != OK)
	{
	  printf("MPD Slot %2d: Reset101 FAILED\n", id);
	  rval = ERROR;
	}
    }

  /* Check MPDs for data */
  int sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords;
  int obuf_nblock = 0, empty = 0, full = 0, nwords = 0;
  for (impd = 0; impd < nbroken; impd++)
    {				// only active mpd set
      id = broken_list[impd];
      mpdSDRAM_GetParam(id, &sd_init, &sd_overrun, &sd_rdaddr, &sd_wraddr,
			&sd_nwords);

      if ((sd_nwords != 0) || (sd_overrun == 1) || (sd_init == 0))
	{
	  printf("ERROR: Slot %2d SDRAM status: \n"
		 "init=%d, overrun=%d, rdaddr=0x%x, wraddr=0x%x, nwords=%d\n",
		 id, sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords);
	  rval = ERROR;
	}

      obuf_nblock = mpdOBUF_GetBlockCount(id);
      mpdOBUF_GetFlags(id, &empty, &full, &nwords);

      if ((obuf_nblock != 0) || (empty == 0) || (full == 1) || (nwords != 0))
	{
	  printf("ERROR: Slot %2d OBUF status: \n"
		 "nblock = %d  empty=%d  full=%d  nwords=%d\n",
		 id, obuf_nblock, empty, full, nwords);
	  rval = ERROR;
	}
    }

  return rval;
}


/*
  Local Variables:
  compile-command: "make -k -B mpd_list.so mpd_slave_list.so"
  End:
 */
