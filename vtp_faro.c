/*************************************************************************
 *
 *  vtp_faro.c -      Library of routines for readout of events using a
 *                    JLAB Trigger Interface V3 (TI) with a VTP in
 *                    CODA 3.0.
 *
 *                    This is for a VTP with serial connection to a TI
 *                    with readout of fa250 triggered data via VXS
 *
 */

#define VTPFA250_BANK 0x02FA

/* Event Buffer definitions */
#define MAX_EVENT_LENGTH 40960
#define MAX_EVENT_POOL   100

#include <VTP_source.h>

/* Need this for the payloadport -> vme slot map */
#include "jvme_loan.h"

/* Routines to add string buffers to banks */
#define BYTESWAPIT
#include "/adaqfs/home/sbs-onl/rol_common/rocUtils.c"

/* Note that ROCID is a static readout list variable that gets set
   automatically in the ROL initialization at Download. */

#define NUM_VTP_CONNECTIONS 1

/* define an array of Payload port Config Structures */
PP_CONF ppInfo[16];

int blklevel = 1;

/* Data necessary to connect using EMUSocket
#define CMSG_MAGIC_INT1 0x634d7367
#define CMSG_MAGIC_INT2 0x20697320
#define CMSG_MAGIC_INT3 0x636f6f6c
*/
unsigned int emuData[] = {0x634d7367,0x20697320,0x636f6f6c,6,0,4196352,1,1};

/* Filenames obtained from the platform or other config files */
char VTP_CONFIG_FILENAME[250];

/* default config filenames, if they are not defined in COOL */
#define DEFAULT_VTP_CONFIG "/home/sbs-onl/vtp/cfg/hcalvtp1.config"

/**
                        DOWNLOAD
**/
void
rocDownload()
{

  int stat;
  char buf[1000];
  const char *fwpath="/home/sbs-onl/vtp/vtp/firmware";
  const char *z7file="fe_vtp_vxs_readout_z7_nov16.bin";
  const char *v7file="fe_vtp_vxs_readout_v7_aug8.bin";


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


  /* ltm4676_print_status(); */

  if(vtpInit(VTP_INIT_CLK_VXS_250))
  {
    printf("vtpInit() **FAILED**. User should not continue.\n");
    return;
  }


  /* FIXME: For MPD, we got this from the config file
     Get ROC Output network info from VTP */
  stat = vtpRocReadNetFile(0);
  if(stat != OK) {
    daLogMsg("ERROR","Can't load VTP ROC network interface information from file");
    ROL_SET_ERROR;
  }else{
    printf(" **Info from ROC Network Output File (in /mnt/boot/)**\n");
    printf("   IP1 = 0x%08x\n", VTP_NET_OUT.ip[0]);
    printf("   GW1 = 0x%08x\n", VTP_NET_OUT.gw[0]);
    printf("   SM1 = 0x%08x\n", VTP_NET_OUT.sm[0]);
    printf("   MAC1 =   0x%08x 0x%08x\n", VTP_NET_OUT.mac[0][0], VTP_NET_OUT.mac[1][0]);
    printf("\n");
  }

 /* print some connection info from the ROC */
  printf(" **Info from ROC Connection Structure**\n");
  printf("   ROC Type = %s\n", rol->rlinkP->type);
  printf("   EMU name = %s\n", rol->rlinkP->name);
  printf("   EMU IP   = %s\n", rol->rlinkP->net);
  printf("   EMU port = %d\n", rol->rlinkP->port);

  /* Configure the ROC*/
  /*  *(rol->async_roc) = 1; */  // don't send Control events to the EB (Set in VTP_source.h)
  vtpRocReset(0);
  printf(" Set ROC ID = %d \n",ROCID);
  vtpRocConfig(ROCID, 0, 64, 0);  /* Use defaults for other parameters MaxRecSize, Max#Blocks, Timeout*/
  emuData[4] = ROCID;  /* define ROCID in the EB Connection data as well*/

  DALMAGO;
  vtpRocStatus(0);
  DALMASTOP;
}

/**
                        PRESTART
**/
void
rocPrestart()
{

  int status;
  unsigned int emuip, emuport;
  int ppmask=0;

  VTPflag = 0;

  printf("%s: rol->usrConfig = %s\n",
	 __func__, rol->usrConfig);

  /* Read Config file and Intialize VTP */
  extern int vtpReadConfigFile(char *filename);
  extern int vtpDownloadAll();

  vtpInitGlobals();
  if(vtpReadConfigFile(rol->usrConfig) == ERROR)
    {
      printf("%s: Error using rol->usrConfig %s.\n",
	     __func__, rol->usrConfig);

      printf("  trying %s\n",
	     DEFAULT_VTP_CONFIG);

      if(vtpConfig(DEFAULT_VTP_CONFIG) == ERROR)
	{
	  daLogMsg("ERROR","Error loading VTP configuration file");
	  return;
	}
      else
	{
	  strncpy(VTP_CONFIG_FILENAME, DEFAULT_VTP_CONFIG, 250);
	}
    }
  else
    {
      strncpy(VTP_CONFIG_FILENAME, rol->usrConfig, 250);
    }
  vtpDownloadAll();

  /* Get EB connection info to program the VTP TCP stack */
  emuip = vtpRoc_inet_addr(rol->rlinkP->net);
  //emuip = vtpRoc_inet_addr("10.10.10.1");
  //emuip = 0x81396de8;  // test for DEBUG
  emuport = rol->rlinkP->port;
  //emuport = 46101;  //tmp for ncat testing
  printf(" EMU IP = 0x%08x  Port= %d\n",emuip, emuport);


  /* Soft Reset the VTP to clear any buffers and counters */
  vtpV7SoftReset();

  /* Reset the ROC */
  vtpRocReset(0);

  /* Initialize the TI Interface */
  vtpTiLinkInit();

   /* Readback the VTP 10Gig network registers and connect */
  unsigned char ipaddr[4];
  unsigned char subnet[4];
  unsigned char gateway[4];
  unsigned char mac[6];
  unsigned char destipaddr[4];
  unsigned short destipport;

  /*Read it back to to make sure */
  vtpRocGetTcpCfg(ipaddr, subnet, gateway, mac, destipaddr, &destipport);

  /* Set the emu ip and port */
  vtpRocSetTcpCfg(ipaddr, subnet, gateway, mac, emuip, emuport);

  printf(" Readback of TCP CLient Registers:\n");
  printf("   ipaddr=%d.%d.%d.%d\n",ipaddr[0],ipaddr[1],ipaddr[2],ipaddr[3]);
  printf("   subnet=%d.%d.%d.%d\n",subnet[0],subnet[1],subnet[2],subnet[3]);
  printf("   gateway=%d.%d.%d.%d\n",gateway[0],gateway[1],gateway[2],gateway[3]);
  printf("   mac=%02x:%02x:%02x:%02x:%02x:%02x\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  printf("   emuip=0x%08x\n",emuip);
  printf("   emuport=%d\n",emuport);


  /* Make the Connection . Pass Data needed to complete connection with the EMU */
  vtpRocTcpConnect(1,emuData,8);

  /* Reset and Configure the MIG and ROC Event Builder */
  vtpRocMigReset();



  /* FIXME: Need to get the active vme slots, convert to payload port, build to bank 1 */
  uint32_t vmemask = (0xff << 3); /* slots 3 - 10 */
  vmemask |= (0xff << 12); /* slots 12 - 20 */

  uint32_t ivme;
  for(ivme = 3; ivme < 22; ivme++)
    {
      if(vmemask & (1 << ivme))
	{
	  uint32_t payloadport = vmeSlot2vxsPayloadPort(ivme);
	     /* Configure payloadport with FADC250 */
	  printf("Configure VME slot %2d (payload port %2d) with FADC250\n",
		 ivme, payloadport);
	  ppmask |= vtpPayloadConfig(payloadport, ppInfo, 1, 1, 0x1);
	}
    }

  printf("vtpPayloadConfig ppmask = 0x%04x\n",ppmask);

  /* Initialize and program the ROC Event Builder*/
  vtpRocEbStop();
  vtpRocEbInit(VTPFA250_BANK,6,7); /* define bank1 tag = VTPFA250_BANK, bank2 tag = 6, bank3 tag = 7 */
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
  status = vtpRocEvioWriteControl(EV_PRESTART,rol->runNumber,rol->runType);
  if(status == ERROR) {
    /* We were unable to send the Prestart Event so we cannot complete the transition
       set the error condition to tell the ROC */
    ROL_SET_ERROR;
  }


  /* Send a User Event - read in a File */
  //  vtpRocFile2Event("/daqfs/home/abbottd/test.txt",(unsigned char *)&ubuf[0],130, ROCID, (MAXBUFSIZE<<2));
  // printf(" User Event header = 0x%08x 0x%08x 0x%08x 0x%08x\n", ubuf[0], ubuf[1], ubuf[2], ubuf[3]);
  // status = vtpRocEvioWriteUserEvent(ubuf);
  //if(status != OK) {
  // printf(" rocPrestart: ERROR: vtpRocEvioWriteUserEvent() failed\n");
  //}


  printf(" Done with User Prestart\n");

}

/**
                        PAUSE
**/
void
rocPause()
{
  CDODISABLE(VTP, 1, 0);
}

/**
                        GO
**/
void
rocGo()
{
  int chmask = 0;

  /* Clear TI Link recieve FIFO */
  vtpTiLinkResetFifo(1);

  printf("Calling vtpSerdesStatusAll()\n");
  vtpSerdesStatusAll();

  chmask = vtpSerdesCheckLinks();
  printf("VTP Serdes link up mask = 0x%05x\n",chmask);


  /* Get the current Block Level from the TI */
  blklevel = vtpTiLinkGetBlockLevel(0);
  printf("\nBlock level read from TI Link = %d\n", blklevel);


  /* Update the ROC EB blocklevel in the EVIO banks */
  vtpRocEbSetBlockLevel(blklevel);


  /* Start the ROC Event Builder */
  vtpRocEbStart();

  /*Send Go Event*/
  vtpRocEvioWriteControl(EV_GO,0,*(rol->nevents));

  DALMAGO;
  vtpRocStatus(0);
  DALMASTOP;

  /* Enable with val = 7 to recieve Block Triggers or 5 to run without software intervention */
  CDOENABLE(VTP, 1, 5);

}


/**
                        END
**/
void
rocEnd()
{
  int status;
  unsigned int ntrig;
  unsigned long long nlongs, nbytes;

  CDODISABLE(VTP, 1, 0);

  /* Get total event information and set the Software ROC counters */
  ntrig = vtpRocGetTriggerCnt();
  *(rol->nevents) = ntrig;
  *(rol->last_event) = ntrig;


  /*Send End Event*/
  vtpRocEvioWriteControl(EV_END,rol->runNumber,*(rol->nevents));


  /* Disable the ROC EB */
  vtpRocEbStop();


  DALMAGO;
  vtpRocStatus(0);
  DALMASTOP;

  /* Disconnect the socket */
  status = vtpRocTcpConnect(0,0,0);
  if(status == ERROR) {
    printf("rocEnd: Error closing socket\n");
  }


  /* Print final Stats */
  nlongs = vtpRocGetNlongs();
  *(rol->totalwds) = nlongs;
  vtpRocGetNBytes(&nbytes);
  printf(" TOTAL Triggers = %d   Nlongs = %lld (0x%llx Bytes)\n",ntrig, nlongs, nbytes);

}

/**
                        READOUT
**/
void
rocTrigger(int EVTYPE)
{

  int ii, ntrig, ntack=0;

/* Right now this is a mostly dummy routine as the trigger and readout is
   running in the FPGAs. In principle however the ROC can poll on
   a register holding the number of unacknowledged readout triggers which will allow it
   to enter this routine and the User can insert an asynchonous event into the data stream.

   Also the ROC can require that every trigger block is managed by this routine.
   Essentially, one is forcing the FPGA to get an acknowledge of the trigger
   by the software ROC.

*/

  ntack = vtpRocPoll();   /* check how many triggers need to be acknowledged */

  /* Get total event information and update the Software ROC counters */
  ntrig = vtpRocGetTriggerCnt();
  *(rol->nevents) = ntrig;
  *(rol->last_event) = ntrig;

  //  printf("%s: In trigger routine: %d triggers being acknowledged\n",__func__,ntack);

  for(ii=0;ii<ntack;ii++) {
    vtpRocWriteBank(NULL);     // Write a NULL bank to just acknowledge the trigger
  }


}

/**
                        READOUT ACKNOWLEDGE
**/
void
rocTrigger_done()
{
  CDOACK(VTP, 1, 0);   /* This does nothing right now - DJA */
}

/**
                        RESET
**/
void
rocReset()
{

  /* Force Disconnect of the socket */
  vtpRocTcpConnect(-1,0,0);

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
  compile-command: "make -k vtp_faro.so"
  End:
 */
