/*************************************************************************
 *
 *  vtp_roc_mpdro.c - Library of routines for readout of events using a
 *                    JLAB Trigger Interface V3 (TI) with a VTP in
 *                    CODA 3.0.
 *
 *                    This is for a VTP with serial connection to a TI
 *
 */

#define VTPMPD_BANK 3561

/* Event Buffer definitions */
#define MAX_EVENT_LENGTH 40960
#define MAX_EVENT_POOL   100

#include <VTP_source.h>

/* Need this for the payloadport -> vme slot map */
#include "jvme_loan.h"

/* Note that ROCID is a static readout list variable that gets set
   automatically in the ROL initialization at Download. */

#define NUM_VTP_CONNECTIONS 1

/* define an array of Payload port Config Structures */
PP_CONF ppInfo[16];

int blklevel = 1;

/* trigBankType:
   Type 0xff10 is RAW trigger No timestamps
   Type 0xff11 is RAW trigger with timestamps (64 bits)
*/
int trigBankType = 0xff10;
int firstEvent;

/* Data necessary to connect using EMUSocket
#define CMSG_MAGIC_INT1 0x634d7367
#define CMSG_MAGIC_INT2 0x20697320
#define CMSG_MAGIC_INT3 0x636f6f6c
*/
unsigned int emuData[] = {0x634d7367,0x20697320,0x636f6f6c,6,0,4196352,1,1};

/* Start of MPD specific configuration */
#include "vtp_mpdro.c"

void rocStatus();
/**
                        DOWNLOAD
**/
void
rocDownload()
{

  int stat;
  char buf[1000];
  /* Firmware files for VTP */
  const char *fwpath="/home/sbs-onl/vtp/vtp/firmware";
  const char *z7file="fe_vtp_vxs_readout_z7.bin";
  const char *v7file="fe_vtp_v7_mpd.bin";

  firstEvent = 1;

  if(vtpInit(VTP_INIT_CLK_VXS_250))
    {
      printf("vtpInit() **FAILED**. User should not continue.\n");
      return;
    }
#define RELOAD_FIRMWARE
#ifdef RELOAD_FIRMWARE
  /* Load firmware here */
  sprintf(buf, "%s/%s", fwpath, z7file);
  if(vtpZ7CfgLoad(buf) != OK)
    {
      printf("Z7 programming failed... (%s)\n", buf);
    }

  printf("loading V7 firmware...\n");
  sprintf(buf, "%s/%s", fwpath, v7file);
  if(vtpV7CfgLoad(buf) != OK)
    {
      printf("V7 programming failed... (%s)\n", buf);
    }
#endif /* RELOAD_FIRMWARE */


  /* ltm4676_print_status(); */

  if(vtpInit(VTP_INIT_CLK_VXS_250))
    {
      printf("vtpInit() **FAILED**. User should not continue.\n");
      return;
    }

  firstEvent = 1;


 /* print some connection info from the ROC */
  printf(" **Info from ROC Connection Structure**\n");
  printf("   ROC Type = %s\n", rol->rlinkP->type);
  printf("   EMU name = %s\n", rol->rlinkP->name);
  printf("   EMU IP   = %s\n", rol->rlinkP->net);
  printf("   EMU port = %d\n", rol->rlinkP->port);

  /* Configure the ROC*/
  *(rol->async_roc) = 1;  // don't send Control events to the EB
  vtpRocReset(0);
  printf(" Set ROC ID = %d \n",ROCID);
  vtpRocConfig(ROCID, 0, 8, 0);  /* Use defaults for other parameters MaxRecSize, Max#Blocks, timeout*/
  emuData[4] = ROCID;  /* define ROCID in the EB Connection data as well*/

  daLogMsg("INFO","Call vtpMpdDownload");
  vtpMpdDownload();

  rocStatus();
}

/**
                        PRESTART
**/
void
rocPrestart()
{


  unsigned int emuip, emuport;

  VTPflag = 0;

  printf("%s: rol->usrConfig = %s\n",
	 __func__, rol->usrConfig);

  daLogMsg("INFO","Call vtpMpdPrestart");
  vtpMpdPrestart();

  /* Reset the ROC */
  vtpRocReset(0);

  /* Initialize the TI Interface */
  vtpTiLinkInit();


   /* Readback the VTP 10Gig network registers and connect */
  unsigned char ipaddr[4];
  unsigned char subnet[4];
  unsigned char gateway[4];
  unsigned char mac[6];
  unsigned char destip[4];
  unsigned short destipport;

  /*Read it back to to make sure */
  vtpRocGetTcpCfg(ipaddr, subnet, gateway, mac, destip, &destipport);

  printf(" Readback of TCP CLient Registers:\n");
  printf("   ipaddr=%d.%d.%d.%d\n",ipaddr[0],ipaddr[1],ipaddr[2],ipaddr[3]);
  printf("   subnet=%d.%d.%d.%d\n",subnet[0],subnet[1],subnet[2],subnet[3]);
  printf("   gateway=%d.%d.%d.%d\n",gateway[0],gateway[1],gateway[2],gateway[3]);
  printf("   mac=%02x:%02x:%02x:%02x:%02x:%02x\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  printf("   destip=%d.%d.%d.%d\n",destip[0],destip[1],destip[2],destip[3]);
  printf("   destipport=%d\n",destipport);


  /* Make the Connection . Pass Data needed to complete connection with the EMU */
  vtpRocTcpConnect(1,emuData,8);

  /* Reset and Configure the MIG and ROC Event Builder */
  vtpRocMigReset();


  memset(ppInfo, 0, sizeof(ppInfo));

  /* The fibermask from vtp_mpdro.c */
  uint32_t fibermask = mpdGetVTPFiberMask();
  uint32_t lanemask = 0, payloadport = 0;
  uint32_t bankmask = 0;
  int ppmask=0;

  int iquad, ifiber, bankid = 1;

  for(iquad = 0; iquad < 8; iquad++)
    {
      if(fibermask & (0xF << (iquad*4)))
	{
	  /* e.g. 0 quad -> fiber 0-3 -> VME slot 3 */
	  /* ... convert to payload port */
	  payloadport = vmeSlot2vxsPayloadPort(iquad + 3);

	  bankmask = 0;
	  for(ifiber = 0; ifiber < 4; ifiber++)
	    {
	      if( (fibermask >> (iquad * 4) ) & (1 << ifiber))
		bankmask |= (bankid << (ifiber * 8));
	    }

	  /* Configure payloadport with MPD in lanes */
	  printf("Configure VME slot %2d (payload port %2d) with MPD bankmask 0x%08x\n",
		 iquad+3, payloadport, bankmask);
	  ppmask |= vtpPayloadConfig(payloadport, ppInfo, 2,
			   0, bankmask);
	}
    }

  printf("vtpPayloadConfig ppmask = 0x%04x\n",ppmask);

  /* Initialize and program the ROC Event Builder*/
  vtpRocEbStop();
  vtpRocEbInit(VTPMPD_BANK,6,7);   // define bank1 tag = 3562, bank2 tag = 6, bank3 tag = 7
  vtpRocEbConfig(ppInfo,0);  // blocklevel=0 will skip setting the block level


  /* Reset the data Link between V7 ROC EB and the Zync FPGA ROC */
  vtpRocEbioReset();


  /* Set TI readout to Hardware mode */
  vtpTiLinkSetMode(1);

  /* Enable Async&EB Events for ROC   bit2 - Async, bit1 - Sync, bit0 V7-EB */
  vtpRocEnable(0x5);


  /* Print Run Number and Run Type */
  printf(" Run Number = %d, Run Type = %d \n",rol->runNumber,rol->runType);

  /*Send Prestart Event*/
  vtpRocEvioWriteControl(0xffd1,rol->runNumber,rol->runType);

  rocStatus();
  vtpMpdApvConfigStatus();

  printf(" Done with User Prestart\n");

}

/**
                        PAUSE
**/
void
rocPause()
{
  VTPflag = 0;
  CDODISABLE(VTP, 1, 0);
  vtpMpdPause();
}

/**
                        GO
**/
void
rocGo()
{
  int chmask = 0;

  daLogMsg("INFO","Call vtpMpdGo");
  vtpMpdGo();

  /* Clear TI Link recieve FIFO */
  vtpTiLinkResetFifo(1);

#ifdef CHECKSERDES
  chmask = vtpSerdesCheckLinks();
  printf("VTP Serdes link up mask = 0x%05x\n",chmask);

  printf("Calling vtpSerdesStatusAll()\n");
  vtpSerdesStatusAll();
#endif

  /* Get the current Block Level from the TI */
  blklevel = vtpTiLinkGetBlockLevel(0);
  printf("\nBlock level read from TI Link = %d\n", blklevel);

  /* Update the ROC EB blocklevel in the EVIO banks */
  vtpRocEbSetBlockLevel(blklevel);

  /* Start the ROC Event Builder */
  vtpRocEbStart();

  /*Send Go Event*/
  vtpRocEvioWriteControl(0xffd2,0,*(rol->nevents));

  rocStatus();

  /* Enable to recieve Triggers */
  CDOENABLE(VTP, 1, 0);
  VTPflag = 0;

}

void
rocStatus()
{
  extern void vtpMpdApvConfigStatus();
  extern int mpdOutputBufferCheck();
  /* Put out some Status' for debug */
  DALMAGO;
  mpdGStatus(0);
  vtpMpdPrintStatus(mpdGetVTPFiberMask(),0);

  vtpRocStatus(0);
  DALMASTOP;
}


/**
                        END
**/
void
rocEnd()
{
  unsigned int ntrig;
  unsigned long long nlongs;

  VTPflag = 0;
  CDODISABLE(VTP, 1, 0);

  daLogMsg("INFO","Call vtpMpdEnd");
  vtpMpdEnd();

  /* Get total event information and set the Software ROC counters */
  ntrig = vtpRocGetTriggerCnt();
  *(rol->nevents) = ntrig;
  *(rol->last_event) = ntrig;


  /*Send End Event*/
  vtpRocEvioWriteControl(0xffd4,rol->runNumber,*(rol->nevents));


  /* Disable the ROC EB */
  vtpRocEbStop();


  rocStatus();
  mpdOutputBufferCheck();

  /* Disconnect the socket */
  vtpRocTcpConnect(0,0,0);


  /* Print final Stats */
  nlongs = vtpRocGetNlongs();
  *(rol->totalwds) = nlongs;
  printf(" TOTAL Triggers = %d   Nlongs = %llu\n",ntrig, nlongs);

}

/**
                        READOUT
**/
void
rocTrigger(int EVTYPE)
{

/* Right now this is a dummy routine as the trigger and readout is
   running in the FPGAs. In principle however the ROC can poll on
   some parameter which will allow it to enter this routine and the
   User can insert an asynchonous event into the data stream.

   Also the ROC can reqire that every trigger is managed by this routine.
   Esentially, one can force the FPGA to get an acknowledge of the trigger
   by the software ROC.

*/



}

/**
                        READOUT ACKNOWLEDGE
**/
void
rocTrigger_done()
{
  CDOACK(VTP, 0, 0);
}

/**
                        RESET
**/
void
rocReset()
{
  daLogMsg("INFO","Call vtpMpdReset");
  vtpMpdReset();

  daLogMsg("INFO","Call vtpMpdCleanup");
  vtpMpdCleanup();
  /* Disconnect the socket */
  vtpRocTcpConnect(0,0,0);

}

void
rocLoad()
{
  int stat;

  /* Open VTP library */
  stat = vtpOpen(VTP_FPGA_OPEN | VTP_I2C_OPEN | VTP_SPI_OPEN);
  if(stat < 0)
    {
      printf(" Unable to Open VTP driver library.\n");
    }


}

void
rocCleanup()
{
  /* Close the VTP Library */
  vtpClose(VTP_FPGA_OPEN|VTP_I2C_OPEN|VTP_SPI_OPEN);
}

/*
  Local Variables:
  compile-command: "make -k vtp_roc_mpdro.so"
  End:
 */
