
// rtpdump.cpp : Defines the entry point for the console application.
//

#include "StdAfx.h"

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#ifdef WIN32
#include <Winsock2.h>
#else
#include <netinet/in.h>
#endif


int main(int argc, char* argv[])

{
  unsigned int bufsize, pacno=0, temp32;
  unsigned short temp16;
  unsigned char buf[65000];
  int i, intime;
  FILE *f, *stat;


  if (argc != 2)
  {
    printf ("Usage: %s <H.264 RTP packet file>\n", argv[0]);
    return -1;
  }

  if (NULL == (f = fopen (argv[1], "rb")))
  {
    printf ("%s: cannot open H.264 packet file %s for reading\n", argv[0], argv[1]);
    return -2;
  }

  if (NULL == (stat = fopen ("stat.txt", "w")))
  {
    return -2;
  }

  while (1)
  {
    if (4 != fread (&bufsize, 1, 4, f))
      return 0;
    if (4 != fread (&intime, 1, 4, f))
    {
      printf ("Panic, cannot read timestamp, old software version file?\n");
      return -1;
    }
    printf ("\n\npacket #%4d containing %5d bytes\n", pacno++, bufsize);
    if (bufsize != fread (buf, 1, bufsize, f))
    {
      printf ("Problems while reading buffer, exit\n");
      return -3;
    }

    for (i=0; i< 25; i++)
      printf ("%02x ", buf[i]);
    printf ("\n");
    printf ("Version (V): %d\n", (buf[0] >> 6) & 0x03);
    printf ("Padding (P): %d\n", (buf[0] >> 5) & 0x01);
    printf ("Extension (X): %d\n", (buf[0] >> 4) & 0x01);
    printf ("CSRC count (CC): %d\n", (buf[0] >> 0) & 0x0F);
    printf ("Marker bit (M): %d\n", (buf[1] >> 7) & 0x01);
    printf ("Payload Type (PT): %d\n", (buf[1] >> 0) & 0x7F);
    memcpy (&temp16, &buf[2], 2);
    printf ("Sequence Number: %d\n", ntohs(temp16));
	
	fprintf(stat,"Packet #%4d processed with %4d bytes\n", ntohs(temp16),bufsize);

    memcpy (&temp32, &buf[4], 4);
    printf ("Timestamp: %d\n", ntohl(temp32));
    memcpy (&temp32, &buf[4], 4);
    printf ("SSRC: %d\n", ntohl(temp32));

    printf ("First Byte: 0x%x\n", buf[12]);

  }
}

