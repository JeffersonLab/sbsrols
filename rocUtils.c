/*
 *
 *  A few helpful routines to help do stuff in the ROC readout list
 *
 */

#include <byteswap.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Routine to read in a file (as text) and create a User Event in a buffer

   Maybe you can pass the rol->dabufp pointer.  that'd be great.
*/

int
rocFile2Event(const char *fname, uint8_t *buf,
	      uint16_t banktag, uint8_t banknum,
	      int32_t maxbytes)
{

  int ii, c, ilen, rem;
  FILE *fid;
  int maxb = 1024 * 1024 * 4;	/* max output buffer size */
  uint32_t ev_header[2];
  uint32_t *u32p;

  if(fname == NULL)
    {
      printf("%s: ERROR: No filename was specified\n", __func__);
      return -1;
    }


  if((banktag == 0) || (banktag >= 0xff00))
    {
      printf("%s: ERROR: Invalid User Bank tag specified (0x%04x)\n",
	     __func__, banktag);
      return -1;
    }

  if((maxbytes == 0) || (maxbytes > maxb))
    maxbytes = maxb;		/* Default to 4 MB */


  fid = fopen(fname, "r");
  if(fid == NULL)
    {
      printf("%s: ERROR: The file %s does not exist \n", __func__, fname);
      return -1;
    }
  else
    {
      printf("%s: INFO: Opened file %s for reading into User Event \n",
	     __func__, fname);
    }

  /* Read in the file to the buffer */
  ilen = 8;			/* leave the header words */
  u32p = (uint32_t *)&buf[0]; /* pointer to beginng of buffer */
  while((ilen < maxbytes) && ((c = getc(fid)) != EOF))
    {
      buf[ilen++] = c;
    }
  fclose(fid);
  printf("%s: INFO: Read %d bytes into buffer\n", __func__, ilen);

  /* Make sure we null terminate the buffer and pad it out to an integal number of words */
  rem = 4 - (ilen & 0x3);
  if(rem < 4)
    {
      for(ii = 0; ii < rem; ii++)
	buf[(ilen + ii)] = 0;
      ilen += ii;
      //      rem = 0;
    }
  else
    {
      rem = 0;			/* no padding necessary just NULL terminate */
      buf[ilen] = 0;
    }

  /* Write the header info in the buffer */
  /* Bank Tag */
  ev_header[0] = (ilen>>2) - 1; /* divide by 4  and subtract 1 */
  ev_header[1] = (banktag<<16)|(rem<<14)|(3<<8)| banknum;
  memcpy(&buf[0],&ev_header[0],8);
  /* u32p[0] =  (ilen >> 2) - 1;	/\* divide by 4  and subtract 1 *\/ */
  /* u32p[1] = (banktag << 16) | (rem << 14) | (3 << 8) | 0; */
  /* printf("%s: INFO: Copied Event Header = 0x%08x 0x%08x \n", */
  /* 	 __func__, ev_header[0], ev_header[1]); */

  /*Swap the bytes going to the Async Event 32 bit Fifo */
#ifdef BYTESWAPIT
  if(0)
    {
      uint8_t aa, bb;
      for(ii = 8; ii <= (ev_header[0] << 2); ii += 4)
	{
	  aa = buf[ii];
	  bb = buf[ii + 1];
	  buf[ii] = buf[ii + 3];
	  buf[ii + 1] = buf[ii + 2];
	  buf[ii + 2] = bb;
	  buf[ii + 3] = aa;
	}
    }
  else
    {
      for(ii = 0; ii <= (ev_header[0]); ii++)
	u32p[ii] = bswap_32(u32p[ii]);
    }
#endif /* BYTESWAPIT */


  return (ev_header[0] + 1);

}
