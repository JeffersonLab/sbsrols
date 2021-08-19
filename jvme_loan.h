/* VXS Payload Port to VME Slot map */
#define MAX_VME_SLOTS 21    /* This is either 20 or 21 */
static int maxVmeSlots=MAX_VME_SLOTS;
unsigned short PayloadPort21[MAX_VME_SLOTS+1] =
  {
    0,     /* Filler for mythical VME slot 0 */
    0,     /* VME Controller */
    17, 15, 13, 11, 9, 7, 5, 3, 1,
    0,     /* Switch Slot A - SD */
    0,     /* Switch Slot B - CTP/GTP */
    2, 4, 6, 8, 10, 12, 14, 16,
    18     /* VME Slot Furthest to the Right - TI */
  };

unsigned short PayloadPort20[MAX_VME_SLOTS+1] =
  {
    0,     /* Filler for mythical VME slot 0 */
    17, 15, 13, 11, 9, 7, 5, 3, 1,
    0,     /* Switch Slot A - SD */
    0,     /* Switch Slot B - CTP/GTP */
    2, 4, 6, 8, 10, 12, 14, 16,
    18,     /* VME Slot Furthest to the Right - TI */
    0
  };


int
vmeSetMaximumVMESlots(int slots)
{
  if((slots<1)||(slots>MAX_VME_SLOTS))
    {
      printf("%s: ERROR: Invalid slots (%d)\n",
	     __func__,slots);
      return ERROR;
    }
  maxVmeSlots = slots;

  return OK;
}

/*!
 Routine to return the VME slot, provided the VXS payload port.

 @return VME Slot number.
*/
int
vxsPayloadPort2vmeSlot(int payloadport)
{
  int rval=0;
  int islot;
  unsigned short *PayloadPort;

  if(payloadport<1 || payloadport>18)
    {
      printf("%s: ERROR: Invalid payloadport %d\n",
	     __func__,payloadport);
      return ERROR;
    }

  if(maxVmeSlots==20)
    PayloadPort = PayloadPort20;
  else if(maxVmeSlots==21)
    PayloadPort = PayloadPort21;
  else
    {
      printf("%s: ERROR: No lookup table for maxVmeSlots = %d\n",
	     __func__,maxVmeSlots);
      return ERROR;
    }

  for(islot=1;islot<MAX_VME_SLOTS+1;islot++)
    {
      if(payloadport == PayloadPort[islot])
	{
	  rval = islot;
	  break;
	}
    }

  if(rval==0)
    {
      printf("%s: ERROR: Unable to find VME Slot from Payload Port %d\n",
	     __func__,payloadport);
      rval=ERROR;
    }

  return rval;
}

/*!
 Routine to return the VME slot mask, provided the VXS payload port mask.

 @return VME Slot mask.
*/
unsigned int
vxsPayloadPortMask2vmeSlotMask(unsigned int ppmask)
{
  int ipp=0;
  unsigned int vmemask=0;

  for(ipp=0; ipp<18; ipp++)
    {
      if(ppmask & (1<<ipp))
	vmemask |= (1<<vxsPayloadPort2vmeSlot(ipp+1));
    }

  return vmemask;
}

/*!
  Routine to return the VXS Payload Port provided the VME slot

  @return VXS Payload Port number.
*/
int
vmeSlot2vxsPayloadPort(int vmeslot)
{
  int rval=0;
  unsigned short *PayloadPort;

  if(vmeslot<1 || vmeslot>maxVmeSlots)
    {
      printf("%s: ERROR: Invalid VME slot %d\n",
	     __func__,vmeslot);
      return ERROR;
    }

  if(maxVmeSlots==20)
    PayloadPort = PayloadPort20;
  else if(maxVmeSlots==21)
    PayloadPort = PayloadPort21;
  else
    {
      printf("%s: ERROR: No lookup table for maxVmeSlots = %d\n",
	     __func__,maxVmeSlots);
      return ERROR;
    }

  rval = (int)PayloadPort[vmeslot];

  if(rval==0)
    {
      printf("%s: ERROR: Unable to find Payload Port from VME Slot %d\n",
	     __func__,vmeslot);
      rval=ERROR;
    }

  return rval;
}

/*!
  Routine to return the VXS Payload Port mask provided the VME slot mask

  @return VXS Payload Port mask.
*/
unsigned int
vmeSlotMask2vxsPayloadPortMask(unsigned int vmemask)
{
  int islot=0;
  unsigned int ppmask=0;

  for(islot=0; islot<22; islot++)
    {
      if(vmemask & (1<<islot))
	ppmask |= (1<<(vmeSlot2vxsPayloadPort(islot)-1));
    }

  return ppmask;
}
