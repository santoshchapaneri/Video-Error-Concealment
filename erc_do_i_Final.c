
/*!
 *************************************************************************************
 * \file 
 *      erc_do_i.c
 *
 * \brief
 *      Intra (I) frame error concealment algorithms for decoder
 *
 *  \author
 *      - Ari Hourunranta                <ari.hourunranta@nokia.com>
 *      - Viktor Varsa                     <viktor.varsa@nokia.com>
 *      - Ye-Kui Wang                   <wyk@ieee.org>
 *		- Santosh Chapaneri				<santoshchapaneri@gmail.com>
 *************************************************************************************
 */

#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "erc_do.h"

static void concealBlocks( int lastColumn, int lastRow, int comp, frame *recfr, int32 picSizeX, int *condition );
static void pixMeanInterpolateBlock( imgpel *src[], imgpel *block, int blockSize, int frameWidth );

//Santosh

#define DIR_INT		   0
#define DIR_MODE	   1

#define LOSSAREA	   0 
#define ENTROPYAREA    0

#define SWDI           1
#define PMODES_INT	   0
#define PMODES_UPDATED 0

#define DIRENTROPYTHRESH 2.2

#define THRESH		0
#define THRESH_LOW 100.0 //means 40% of max gradient magnitude

#define INF -270
#define LUMA 0
#define CHROMA 1

typedef struct direction
{
	int p1; int p2; double d1; double d2;
} Direction;

double **angles;

static void   pixDirectionalInterpolateBlock( imgpel *src[], imgpel *block, int blockSize, int frameWidth, int comp, int row_idx, int column_idx);
static double findbestdir(imgpel *src[], int blockSize, int frameWidth, int *nbrs);
static double findBestAngle(double *gradientBoundary, double *angleBoundary, double maxgrad, imgpel *src[], int blockSize);static double bestangle(double counter[]);
static double quantize_angle(double angle);
static int    mean_interpolate(imgpel *src[], int blockSize, int frameWidth, int row, int column);
static void   incCounter(double counter[], double angle, double gradient);
static double counterAngle(double *gradientBoundary, double *angleBoundary, double maxgrad, imgpel *src[], int blockSize);

static void  pixDirInterpolateBlockwithPModes(imgpel *src[], imgpel *block, int blockSize, int frameWidth, int comp, int row_idx, int column_idx);
static int   findDominantPmodeChroma(imgpel *src[], int blockSize, int row_idx, int column_idx, int frameWidth);
static int   findDominantPmodeLuma(imgpel *src[], int blockSize, int block_row, int block_column, int frameWidth);
static float findDistance(int pixval[4], int ipmode, int maxval, int minval);

static void boundary_pixels_pmodes(imgpel *src[], int blockSize, int frameWidth, double theta, int row, int column, struct direction *dir_pixel, int comp);
static void boundary_pmode0(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel);
static void boundary_pmode1(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel);
static void boundary_pmode3(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel);
static void boundary_pmode4(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel);
static void boundary_pmode5(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel);
static void boundary_pmode6(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel);
static void boundary_pmode7(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel);
static void boundary_pmode8(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel);

static int  findDominantPmodeLumaUpdated(imgpel *src[], int blockSize, int block_row, int block_column, int frameWidth, int *fMDI, int *fBI, int edgeDir[9], int edgeStrength[9], double *dirEntropy, int *numDED);
static void showEntropyArea(double dirEntropy, int blockSize, imgpel *block, int frameWidth);


#define PI  3.1415926
#if !defined(WIN32) || defined(__GNUC__)
#define max(a, b)      ((a) > (b) ? (a) : (b))  //!< Macro returning max value
#define min(a, b)      ((a) < (b) ? (a) : (b))  //!< Macro returning min value
#endif

/*!
 ************************************************************************
 * \brief
 *      The main function for Intra frame concealment.
 *      Calls "concealBlocks" for each color component (Y,U,V) seperately
 * \return
 *      0, if the concealment was not successful and simple concealment should be used
 *      1, otherwise (even if none of the blocks were concealed)
 * \param recfr
 *      Reconstructed frame buffer
 * \param picSizeX
 *      Width of the frame in pixels
 * \param picSizeY
 *      Height of the frame in pixels
 * \param errorVar   
 *      Variables for error concealment
 ************************************************************************
 */
int ercConcealIntraFrame( frame *recfr, int32 picSizeX, int32 picSizeY, ercVariables_t *errorVar ) 
{
  int lastColumn = 0, lastRow = 0, i, j;

  /* if concealment is on */
  if ( errorVar && errorVar->concealment ) 
  {
    /* if there are segments to be concealed */
    if ( errorVar->nOfCorruptedSegments ) 
    { 
      /* Y */
      lastRow = (int) (picSizeY>>3);
      lastColumn = (int) (picSizeX>>3);

	  //Santosh
	  if (DIR_INT || SWDI)
	  {
		  angles = (double**)malloc(lastColumn*sizeof(double*));
		  for(i=0;i<lastColumn;i++)
			  angles[i] = (double*)malloc(lastRow*sizeof(double));
		  for(i=0;i<lastColumn;i++)
			  for(j=0;j<lastRow;j++)
				  angles[i][j] = INF;
	  }
	  //End

      concealBlocks( lastColumn, lastRow, 0, recfr, picSizeX, errorVar->yCondition );
      
      /* U (dimensions halved compared to Y) */
      lastRow = (int) (picSizeY>>4);
      lastColumn = (int) (picSizeX>>4);
      concealBlocks( lastColumn, lastRow, 1, recfr, picSizeX, errorVar->uCondition );
      
      /* V ( dimensions equal to U ) */
      concealBlocks( lastColumn, lastRow, 2, recfr, picSizeX, errorVar->vCondition );
    }
    return 1;
  }
  else
    return 0;
}

/*!
 ************************************************************************
 * \brief
 *      Conceals the MB at position (row, column) using pixels from predBlocks[]
 *      using pixMeanInterpolateBlock()
 * \param currFrame
 *      current frame
 * \param row
 *      y coordinate in blocks
 * \param column
 *      x coordinate in blocks
 * \param predBlocks[]   
 *      list of neighboring source blocks (numbering 0 to 7, 1 means: use the neighbor)
 * \param frameWidth
 *      width of frame in pixels
 * \param mbWidthInBlocks
 *      2 for Y, 1 for U/V components
 ************************************************************************
 */
void ercPixConcealIMB(imgpel *currFrame, int row, int column, int predBlocks[], int frameWidth, int mbWidthInBlocks)
{
   imgpel *src[8]={NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
   imgpel *currBlock = NULL;

   int comp;

   /* collect the reliable neighboring blocks */
   if (predBlocks[0])
      src[0] = currFrame + (row-mbWidthInBlocks)*frameWidth*8 + (column+mbWidthInBlocks)*8;
   if (predBlocks[1])
      src[1] = currFrame + (row-mbWidthInBlocks)*frameWidth*8 + (column-mbWidthInBlocks)*8;
   if (predBlocks[2])
      src[2] = currFrame + (row+mbWidthInBlocks)*frameWidth*8 + (column-mbWidthInBlocks)*8;
   if (predBlocks[3])
      src[3] = currFrame + (row+mbWidthInBlocks)*frameWidth*8 + (column+mbWidthInBlocks)*8;
   if (predBlocks[4])
      src[4] = currFrame + (row-mbWidthInBlocks)*frameWidth*8 + column*8;
   if (predBlocks[5])
      src[5] = currFrame + row*frameWidth*8 + (column-mbWidthInBlocks)*8;
   if (predBlocks[6])
      src[6] = currFrame + (row+mbWidthInBlocks)*frameWidth*8 + column*8;
   if (predBlocks[7])
      src[7] = currFrame + row*frameWidth*8 + (column+mbWidthInBlocks)*8;
   
   currBlock = currFrame + row*frameWidth*8 + column*8;
   
   if(!DIR_INT && !PMODES_INT && !PMODES_UPDATED && !LOSSAREA && !SWDI)
   {
	   pixMeanInterpolateBlock( src, currBlock, mbWidthInBlocks*8, frameWidth );
	   return;
   }

   //Santosh
   if(mbWidthInBlocks==2)
	   comp = LUMA;
   else
	   comp = CHROMA;

   //Concealment with Gradient Based Directional Interpolation
   if(DIR_INT && !LOSSAREA && !SWDI)
   {
	   pixDirectionalInterpolateBlock( src, currBlock, mbWidthInBlocks*8, frameWidth, comp, row, column);
	   return;
   }

   if((PMODES_INT || PMODES_UPDATED || SWDI) && !LOSSAREA)
   {
	   //OR Use Pmodes for Error Concealment
	   //Determine here whether to do 16x16 MB concealment or splitting to 8x8 blocks wrt pmodes
	   //If CIF frame, then 16x16
	   //IF QCIF frame and high neighbor spatial activity, then split to 8x8 blocks, else 16x16 MB concealment
	   // Above comments are for older code of pmodes - June 2012
       pixDirInterpolateBlockwithPModes( src, currBlock, mbWidthInBlocks*8, frameWidth, comp, row, column);
   }
}

/*!
 ************************************************************************
 * \brief
 *      This function checks the neighbours of a Macroblock for usability in
 *      concealment. First the OK Macroblocks are marked, and if there is not 
 *      enough of them, then the CONCEALED ones as well.
 *      A "1" in the the output array means reliable, a "0" non reliable MB.
 *      The block order in "predBlocks":
 *              1 4 0
 *              5 x 7
 *              2 6 3   
 *      i.e., corners first.
 * \return
 *      Number of useable neighbour Macroblocks for concealment.
 * \param predBlocks[] 
 *      Array for indicating the valid neighbor blocks
 * \param currRow 
 *      Current block row in the frame
 * \param currColumn    
 *      Current block column in the frame
 * \param condition    
 *      The block condition (ok, lost) table
 * \param maxRow      
 *      Number of block rows in the frame
 * \param maxColumn   
 *      Number of block columns in the frame
 * \param step          
 *      Number of blocks belonging to a MB, when counting
 *      in vertical/horizontal direction. (Y:2 U,V:1)
 * \param fNoCornerNeigh 
 *      No corner neighbours are considered
 ************************************************************************
 */
int ercCollect8PredBlocks( int predBlocks[], int currRow, int currColumn, int *condition, 
                          int maxRow, int maxColumn, int step, byte fNoCornerNeigh )
{
  int srcCounter = 0, srcCountMin = (fNoCornerNeigh ? 2 : 4),
    threshold = ERC_BLOCK_CONCEALED;
  
  memset( predBlocks, 0, 8*sizeof(int) );
  
  /* collect the reliable neighboring blocks */
  do 
  {
    srcCounter = 0;
    /* Top */
    if (currRow > 0 && condition[ (currRow-1)*maxColumn + currColumn ] >= threshold ) 
    {                           //ERC_BLOCK_OK (3) or ERC_BLOCK_CONCEALED (2)
      predBlocks[4] = condition[ (currRow-1)*maxColumn + currColumn ];
      srcCounter++;
    }
    /* Bottom */
    if ( currRow < (maxRow-step) && condition[ (currRow+step)*maxColumn + currColumn ] >= threshold ) 
    {
      predBlocks[6] = condition[ (currRow+step)*maxColumn + currColumn ];
      srcCounter++;
    }
    
    if ( currColumn > 0 ) 
    {
      /* Left */
      if ( condition[ currRow*maxColumn + currColumn - 1 ] >= threshold ) 
      {
        predBlocks[5] = condition[ currRow*maxColumn + currColumn - 1 ];
        srcCounter++;
      }
      
      if ( !fNoCornerNeigh ) 
      {
        /* Top-Left */
        if ( currRow > 0 && condition[ (currRow-1)*maxColumn + currColumn - 1 ] >= threshold ) 
        {
          predBlocks[1] = condition[ (currRow-1)*maxColumn + currColumn - 1 ];
          srcCounter++;
        }
        /* Bottom-Left */
        if ( currRow < (maxRow-step) && condition[ (currRow+step)*maxColumn + currColumn - 1 ] >= threshold ) 
        {
          predBlocks[2] = condition[ (currRow+step)*maxColumn + currColumn - 1 ];
          srcCounter++;
        }
      }
    }
    
    if ( currColumn < (maxColumn-step) ) 
    {
      /* Right */
      if ( condition[ currRow*maxColumn+currColumn + step ] >= threshold ) 
      {
        predBlocks[7] = condition[ currRow*maxColumn+currColumn + step ];
        srcCounter++;
      }
      
      if ( !fNoCornerNeigh ) 
      {
        /* Top-Right */
        if ( currRow > 0 && condition[ (currRow-1)*maxColumn + currColumn + step ] >= threshold ) 
        {
          predBlocks[0] = condition[ (currRow-1)*maxColumn + currColumn + step ];
          srcCounter++;
        }
        /* Bottom-Right */
        if ( currRow < (maxRow-step) && condition[ (currRow+step)*maxColumn + currColumn + step ] >= threshold ) 
        {
          predBlocks[3] = condition[ (currRow+step)*maxColumn + currColumn + step ];
          srcCounter++;
        }
      }
    }
    /* prepare for the next round */
    threshold--;
    if (threshold < ERC_BLOCK_CONCEALED)
      break;
  } while ( srcCounter < srcCountMin);
  
  return srcCounter;
}

/*!
 ************************************************************************
 * \brief
 *      collects prediction blocks only from the current column
 * \return
 *      Number of useable neighbour Macroblocks for concealment.
 * \param predBlocks[] 
 *      Array for indicating the valid neighbor blocks
 * \param currRow 
 *      Current block row in the frame
 * \param currColumn    
 *      Current block column in the frame
 * \param condition    
 *      The block condition (ok, lost) table
 * \param maxRow      
 *      Number of block rows in the frame
 * \param maxColumn   
 *      Number of block columns in the frame
 * \param step          
 *      Number of blocks belonging to a MB, when counting
 *      in vertical/horizontal direction. (Y:2 U,V:1)
 ************************************************************************
 */
int ercCollectColumnBlocks( int predBlocks[], int currRow, int currColumn, int *condition, int maxRow, int maxColumn, int step )
{
  int srcCounter = 0, threshold = ERC_BLOCK_CORRUPTED;
  
  memset( predBlocks, 0, 8*sizeof(int) );
  
  /* in this case, row > 0 and row < 17 */
  if ( condition[ (currRow-1)*maxColumn + currColumn ] > threshold ) 
  {
    predBlocks[4] = 1;
    srcCounter++;
  }
  if ( condition[ (currRow+step)*maxColumn + currColumn ] > threshold ) 
  {
    predBlocks[6] = 1;
    srcCounter++;
  }
  
  return srcCounter;
}

/*!
 ************************************************************************
 * \brief
 *      Core for the Intra blocks concealment.
 *      It is called for each color component (Y,U,V) seperately
 *      Finds the corrupted blocks and calls pixel interpolation functions 
 *      to correct them, one block at a time.
 *      Scanning is done vertically and each corrupted column is corrected
 *      bi-directionally, i.e., first block, last block, first block+1, last block -1 ...
 * \param lastColumn  
 *      Number of block columns in the frame
 * \param lastRow     
 *      Number of block rows in the frame
 * \param comp
 *      color component
 * \param recfr
 *      Reconstructed frame buffer
 * \param picSizeX
 *      Width of the frame in pixels
 * \param condition
 *      The block condition (ok, lost) table
 ************************************************************************
 */
static void concealBlocks( int lastColumn, int lastRow, int comp, frame *recfr, int32 picSizeX, int *condition )
{
  int row, column, srcCounter = 0,  thr = ERC_BLOCK_CORRUPTED,
      lastCorruptedRow = -1, firstCorruptedRow = -1, currRow = 0, 
      areaHeight = 0, i = 0, smoothColumn = 0;
  int predBlocks[8], step = 1;
  
  /* in the Y component do the concealment MB-wise (not block-wise):
  this is useful if only whole MBs can be damaged or lost */
  if ( comp == 0 )
    step = 2;
  else
    step = 1;
  
  for ( column = 0; column < lastColumn; column += step ) 
  {
    for ( row = 0; row < lastRow; row += step ) 
    {
      if ( condition[row*lastColumn+column] <= thr ) 
      {
        firstCorruptedRow = row;
        /* find the last row which has corrupted blocks (in same continuous area) */
        for ( lastCorruptedRow = row+step; lastCorruptedRow < lastRow; lastCorruptedRow += step ) 
        {
          /* check blocks in the current column */
          if ( condition[ lastCorruptedRow*lastColumn + column ] > thr ) 
          {
            /* current one is already OK, so the last was the previous one */
            lastCorruptedRow -= step;
            break;
          }
        }
        if ( lastCorruptedRow >= lastRow ) 
        {
          /* correct only from above */
          lastCorruptedRow = lastRow-step;
          for ( currRow = firstCorruptedRow; currRow < lastRow; currRow += step ) 
          {
            srcCounter = ercCollect8PredBlocks( predBlocks, currRow, column, condition, lastRow, lastColumn, step, 1 );
          
            switch( comp ) 
            {
            case 0 :
              ercPixConcealIMB( recfr->yptr, currRow, column, predBlocks, picSizeX, 2);
              break;
            case 1 :
              ercPixConcealIMB( recfr->uptr, currRow, column, predBlocks, (picSizeX>>1), 1);
              break;
            case 2 :
              ercPixConcealIMB( recfr->vptr, currRow, column, predBlocks, (picSizeX>>1), 1);
              break;
            }
            
            if ( comp == 0 ) 
            {
              condition[ currRow*lastColumn+column] = ERC_BLOCK_CONCEALED;
              condition[ currRow*lastColumn+column + 1] = ERC_BLOCK_CONCEALED;
              condition[ currRow*lastColumn+column + lastColumn] = ERC_BLOCK_CONCEALED;
              condition[ currRow*lastColumn+column + lastColumn + 1] = ERC_BLOCK_CONCEALED;
            }
            else 
            {
              condition[ currRow*lastColumn+column] = ERC_BLOCK_CONCEALED;
            }
            
          }
          row = lastRow;
        } 
        else if ( firstCorruptedRow == 0 ) 
        {
          /* correct only from below */
          for ( currRow = lastCorruptedRow; currRow >= 0; currRow -= step ) 
          {
            srcCounter = ercCollect8PredBlocks( predBlocks, currRow, column, condition, lastRow, lastColumn, step, 1 );
            
            switch( comp ) 
            {
            case 0 :
              ercPixConcealIMB( recfr->yptr, currRow, column, predBlocks, picSizeX, 2);
              break;
            case 1 :
              ercPixConcealIMB( recfr->uptr, currRow, column, predBlocks, (picSizeX>>1), 1);
              break;
            case 2 :
              ercPixConcealIMB( recfr->vptr, currRow, column, predBlocks, (picSizeX>>1), 1);
              break;
            }
            
            if ( comp == 0 ) 
            {
              condition[ currRow*lastColumn+column] = ERC_BLOCK_CONCEALED;
              condition[ currRow*lastColumn+column + 1] = ERC_BLOCK_CONCEALED;
              condition[ currRow*lastColumn+column + lastColumn] = ERC_BLOCK_CONCEALED;
              condition[ currRow*lastColumn+column + lastColumn + 1] = ERC_BLOCK_CONCEALED;
            }
            else 
            {
              condition[ currRow*lastColumn+column] = ERC_BLOCK_CONCEALED;
            }
            
          }
          
          row = lastCorruptedRow+step;
        }
        else 
        {
          /* correct bi-directionally */
          
          row = lastCorruptedRow+step;
          areaHeight = lastCorruptedRow-firstCorruptedRow+step;
          
          /* 
          *  Conceal the corrupted area switching between the up and the bottom rows 
          */
          for ( i = 0; i < areaHeight; i += step ) 
          {
            if ( i % 2 ) 
            {
              currRow = lastCorruptedRow;
              lastCorruptedRow -= step;
            }
            else 
            {
              currRow = firstCorruptedRow;
              firstCorruptedRow += step;
            }
            
            if (smoothColumn > 0) 
            {
              srcCounter = ercCollectColumnBlocks( predBlocks, currRow, column, condition, lastRow, lastColumn, step );
            }
            else 
            {
              srcCounter = ercCollect8PredBlocks( predBlocks, currRow, column, condition, lastRow, lastColumn, step, 1 );
            }
            
            switch( comp ) 
            {
            case 0 :
              ercPixConcealIMB( recfr->yptr, currRow, column, predBlocks, picSizeX, 2);
              break;
              
            case 1 :
              ercPixConcealIMB( recfr->uptr, currRow, column, predBlocks, (picSizeX>>1), 1);
              break;
              
            case 2 :
              ercPixConcealIMB( recfr->vptr, currRow, column, predBlocks, (picSizeX>>1), 1);
              break;
            }
            
            if ( comp == 0 ) 
            {
              condition[ currRow*lastColumn+column] = ERC_BLOCK_CONCEALED;
              condition[ currRow*lastColumn+column + 1] = ERC_BLOCK_CONCEALED;
              condition[ currRow*lastColumn+column + lastColumn] = ERC_BLOCK_CONCEALED;
              condition[ currRow*lastColumn+column + lastColumn + 1] = ERC_BLOCK_CONCEALED;
            }
            else 
            {
              condition[ currRow*lastColumn+column ] = ERC_BLOCK_CONCEALED;
            }
          }
        }

        lastCorruptedRow = -1;
        firstCorruptedRow = -1;

      }
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *      Does the actual pixel based interpolation for block[]
 *      using weighted average
 * \param src[] 
 *      pointers to neighboring source blocks
 * \param block     
 *      destination block
 * \param blockSize
 *      16 for Y, 8 for U/V components
 * \param frameWidth
 *      Width of the frame in pixels
 ************************************************************************
 */
static void pixMeanInterpolateBlock( imgpel *src[], imgpel *block, int blockSize, int frameWidth )
{
  int row, column, k, tmp, srcCounter = 0, weight = 0, bmax = blockSize - 1;
  
  k = 0;
  for ( row = 0; row < blockSize; row++ ) 
  {
    for ( column = 0; column < blockSize; column++ ) 
    {
      tmp = 0;
      srcCounter = 0;
      /* above */
      if ( src[4] != NULL )   
      {
        weight = blockSize-row;
        tmp += weight * (*(src[4]+bmax*frameWidth+column));
        srcCounter += weight;
      }
      /* left */
      if ( src[5] != NULL )   
      {
        weight = blockSize-column;
        tmp += weight * (*(src[5]+row*frameWidth+bmax));
        srcCounter += weight;
      }
      /* below */
      if ( src[6] != NULL )   
      {
        weight = row+1;
        tmp += weight * (*(src[6]+column));
        srcCounter += weight;
      }
      /* right */
      if ( src[7] != NULL )   
      {
        weight = column+1;
        tmp += weight * (*(src[7]+row*frameWidth));
        srcCounter += weight;
      }

      if ( srcCounter > 0 )
        block[ k + column ] = (byte)(tmp/srcCounter);
      else 
        block[ k + column ] = blockSize == 8 ? img->dc_pred_value_chroma : img->dc_pred_value_luma;

	  if(LOSSAREA)
		  block[ k + column ] = 0;
    }
    k += frameWidth;
  }
  
}


//Santosh
/*!
 ************************************************************************
 * \brief
 *      Does the actual pixel based Directional interpolation for block[]
 *      using Sobel edge detecting masks. Mask is applied to available neighboring
 *      boundary pixels, then the dominant edge direction is determined
 *      and the pixel is concealed in the direction of dominant edge using weighted interpolation. 
 * \param src[] 
 *      pointers to neighboring source blocks
 * \param block     
 *      destination block
 * \param blockSize
 *      16 for Y, 8 for U/V components
 * \param frameWidth
 *      Width of the frame in pixels
 ************************************************************************
 */
static void pixDirectionalInterpolateBlock(imgpel *src[], imgpel *block, int blockSize, int frameWidth, int comp, int row_idx, int column_idx)
{
	int row, column, k, nbrs=0, srcCounter = 0, weight = 0, bmax = blockSize - 1;
	double theta=0.0;
	struct direction dir_pixel;

	if(comp==LUMA)
	{
		theta = findbestdir(src, blockSize, frameWidth, &nbrs);
		angles[column_idx][row_idx] = theta;
	}
	else
		theta = angles[column_idx<<1][row_idx<<1];

	if((theta == INF) || (nbrs<2))
	{
		pixMeanInterpolateBlock( src, block, blockSize, frameWidth );
		return;
	}

	//If theta is finite, do the weighted interpolation in the direction of this dominant edge.
	//Need atleast two neighbors for doing directional interpolation i.e. nbrs>=2
	//Check distance 

	k = 0;
	for ( row = 0; row < blockSize; row++ ) 
	{
		for ( column = 0; column < blockSize; column++ )
		{			
			//Compute dir_pixel.p1,p2,d1,d2
			boundary_pixels_pmodes(src, blockSize, frameWidth, theta, row, column, &dir_pixel, comp);

			//Interpolation
			if( (dir_pixel.p1==-1 && dir_pixel.p2==-1) || (dir_pixel.d1==0.0 && dir_pixel.d2==0.0) )
				block[k + column] = mean_interpolate(src, blockSize, frameWidth, row, column);
			else if(dir_pixel.p1==-1 && dir_pixel.p2!=-1)
				block[k+column] = dir_pixel.p2;
			else if(dir_pixel.p2==-1 && dir_pixel.p1!=-1)
				block[k+column] = dir_pixel.p1;
			else
				block[k + column] = (int)((dir_pixel.d2*dir_pixel.p1 + dir_pixel.d1*dir_pixel.p2)/(dir_pixel.d1+dir_pixel.d2));
		
			if(LOSSAREA)
				block[k+column] = 0; 
		}
		k += frameWidth;
	}
}

static double findbestdir(imgpel *src[], int blockSize, int frameWidth, int *nbrs)
{
	int row, column, idx, bmax = blockSize - 1;
	int gradientx, gradienty;
	double counter[16]; //counter for 16 quantized angles
	double dominant_angle=0.0, sumNum=0.0, sumDen=0.0;
	double *gradientBoundary, *angleBoundary, maxgrad=0.0;
	
	gradientBoundary = (double *)malloc(4*blockSize*sizeof(double));
	angleBoundary = (double *)malloc(4*blockSize*sizeof(double));

/*Note:
	idx = 0 to blockSize-1 is for Above pixels
	idx = blockSize to 2*blockSize-1 is for Left pixels
	idx = 2*blockSize to 3*blockSize-1 is for Below pixels
	idx = 3*blockSize to 4*blockSize-1 is for Right pixels
*/
    //Zero Initialization
	for(idx=0;idx<4*blockSize;idx++)
	{
		gradientBoundary[idx] = 0.0;
		angleBoundary[idx] = 0.0;
	}

	for(idx=0;idx<16;idx++)
		counter[idx] = 0.0;

	*nbrs = 0;
	//Find the dominant edge direction amongst the available boundary pixels (2-pixel wide boundary pixels are taken here)
	//Calculate gradient and angle at each such boundary pixel

	sumNum = 0.0; sumDen = 0.0;
	/* Above */
	if ( src[4] != NULL )   
	{
		for ( column = 1; column < blockSize-1; column++ ) 
		{
			gradientx = 0; gradienty = 0;
			
			//each pixel = *(src[4]+(bmax-1)*frameWidth+column)
			
			gradientx = (-1) * (*(src[4]+(bmax-2)*frameWidth+(column-1))) + (-2) * (*(src[4]+(bmax-1)*frameWidth+(column-1))) + (-1) * (*(src[4]+(bmax)*frameWidth+(column-1)));
			gradientx += (1) * (*(src[4]+(bmax-2)*frameWidth+(column+1))) + (2) * (*(src[4]+(bmax-1)*frameWidth+(column+1))) + (1) * (*(src[4]+(bmax)*frameWidth+(column+1)));

			gradienty = (-1) * (*(src[4]+(bmax-2)*frameWidth+(column-1))) + (-2) * (*(src[4]+(bmax-2)*frameWidth+column)) + (-1) * (*(src[4]+(bmax-2)*frameWidth+(column+1)));
			gradienty += (1) * (*(src[4]+bmax*frameWidth+(column-1))) + (2) * (*(src[4]+bmax*frameWidth+column)) + (1) * (*(src[4]+bmax*frameWidth+(column+1)));  	

			gradientBoundary[column] = sqrt((double)(gradientx * gradientx + gradienty * gradienty));
			if(gradientx==0)
				angleBoundary[column] = 90;
			else
				angleBoundary[column] = (atan((double)(gradienty)/(gradientx))) * (180/PI);

			if(angleBoundary[column]>=-90 && angleBoundary[column]<-84)
				angleBoundary[column] = 90;

			angleBoundary[column] = quantize_angle(angleBoundary[column]);

			if(gradientBoundary[column] > THRESH)
				incCounter(counter, angleBoundary[column], gradientBoundary[column]);
			
			if(gradientBoundary[column] > THRESH)
			{
				sumNum += angleBoundary[column] * gradientBoundary[column];
				sumDen += gradientBoundary[column];
			}

			if(maxgrad<gradientBoundary[column])
				maxgrad = gradientBoundary[column];
		}
		(*nbrs)++;
	}

	/* Left */
	if ( src[5] != NULL )   
	{
		for ( row = 1; row < blockSize-1; row++ ) 
		{
			gradientx = 0; gradienty = 0;

			//each pixel = *(src[5]+row*frameWidth+(bmax-1))
	
			gradientx = (-2) * (*(src[5]+row*frameWidth+(bmax-2))) + (2) * (*(src[5]+row*frameWidth+(bmax)));
			gradientx += (-1) * (*(src[5]+(row-1)*frameWidth+(bmax-2))) + (1) * (*(src[5]+(row-1)*frameWidth+(bmax)));
			gradientx += (-1) * (*(src[5]+(row+1)*frameWidth+(bmax-2))) + (1) * (*(src[5]+(row+1)*frameWidth+(bmax)));

			gradienty = (-1) * (*(src[5]+(row-1)*frameWidth+(bmax-2))) + (-2) * (*(src[5]+(row-1)*frameWidth+(bmax-1))) + (-1) * (*(src[5]+(row-1)*frameWidth+(bmax)));
			gradienty += (1) * (*(src[5]+(row+1)*frameWidth+(bmax-2))) + (2) * (*(src[5]+(row+1)*frameWidth+(bmax-1))) + (1) * (*(src[5]+(row+1)*frameWidth+(bmax)));

			gradientBoundary[blockSize+row] = sqrt((double)(gradientx * gradientx + gradienty * gradienty));
			if(gradientx==0)
				angleBoundary[blockSize+row] = 90;
			else
				angleBoundary[blockSize+row] = (atan((double)(gradienty)/(gradientx))) * (180/PI);

			if(angleBoundary[blockSize+row]>=-90 && angleBoundary[blockSize+row]<-84)
				angleBoundary[blockSize+row] = 90;

			angleBoundary[blockSize+row] = quantize_angle(angleBoundary[blockSize+row]);

			if(gradientBoundary[blockSize+row] > THRESH)
				incCounter(counter, angleBoundary[blockSize+row], gradientBoundary[blockSize+row]);

			if(gradientBoundary[blockSize+row] > THRESH)
			{
				sumNum += angleBoundary[blockSize+row] * gradientBoundary[blockSize+row];
				sumDen += gradientBoundary[blockSize+row];
			}

			if(maxgrad<gradientBoundary[blockSize+row])
				maxgrad = gradientBoundary[blockSize+row];
		}
		(*nbrs)++;
	}

	/* Below */
	if ( src[6] != NULL )   
	{
		for ( column = 1; column < blockSize-1; column++ ) 
		{
			gradientx = 0; gradienty = 0;

			//each pixel = *(src[6]+1*frameWidth+column) //since we want 2nd row of below block
	
			gradientx = (-1) * (*(src[6]+0*frameWidth+(column-1))) + (-2) * (*(src[6]+1*frameWidth+(column-1))) + (-1) * (*(src[6]+2*frameWidth+(column-1)));
			gradientx += (1) * (*(src[6]+0*frameWidth+(column+1))) + (2) * (*(src[6]+1*frameWidth+(column+1))) + (1) * (*(src[6]+2*frameWidth+(column+1)));

			gradienty = (-2) * (*(src[6]+0*frameWidth+column)) + (2) * (*(src[6]+2*frameWidth+column));
			gradienty += (-1) * (*(src[6]+0*frameWidth+(column-1))) + (1) * (*(src[6]+2*frameWidth+(column-1)));
			gradienty += (-1) * (*(src[6]+0*frameWidth+(column+1))) + (1) * (*(src[6]+2*frameWidth+(column+1)));
			
			gradientBoundary[2*blockSize+column] = sqrt((double)(gradientx * gradientx + gradienty * gradienty));
			if(gradientx==0)
				angleBoundary[2*blockSize+column] = 90;
			else
				angleBoundary[2*blockSize+column] = (atan((double)(gradienty)/(gradientx))) * (180/PI);

			if(angleBoundary[2*blockSize+column]>=-90 && angleBoundary[2*blockSize+column]<-84)
				angleBoundary[2*blockSize+column] = 90;

			angleBoundary[2*blockSize+column] = quantize_angle(angleBoundary[2*blockSize+column]);

			if(gradientBoundary[2*blockSize+column] > THRESH)
				incCounter(counter, angleBoundary[2*blockSize+column], gradientBoundary[2*blockSize+column]);

			if(gradientBoundary[2*blockSize+column] > THRESH)
			{
				sumNum += angleBoundary[2*blockSize+column] * gradientBoundary[2*blockSize+column];
				sumDen += gradientBoundary[2*blockSize+column];
			}

			if(maxgrad<gradientBoundary[2*blockSize+column])
				maxgrad = gradientBoundary[2*blockSize+column];
		}
		(*nbrs)++;
	}

	/* Right */
	if ( src[7] != NULL )   
	{
		for ( row = 1; row < blockSize-1; row++ ) 
		{
			gradientx = 0; gradienty = 0;

			//each pixel = *(src[7]+row*frameWidth+1) //since we want 2nd column of right block
			
			gradientx = (-2) * (*(src[7]+row*frameWidth+0)) + (2) * (*(src[7]+row*frameWidth+2));
			gradientx += (-1) * (*(src[7]+(row-1)*frameWidth+0)) + (1) * (*(src[7]+(row-1)*frameWidth+2));
			gradientx += (-1) * (*(src[7]+(row+1)*frameWidth+0)) + (1) * (*(src[7]+(row+1)*frameWidth+2));

			gradienty = (-1) * (*(src[7]+(row-1)*frameWidth+0)) + (-2) * (*(src[7]+(row-1)*frameWidth+1)) + (-1) * (*(src[7]+(row-1)*frameWidth+2));
			gradienty += (1) * (*(src[7]+(row+1)*frameWidth+0)) + (2) * (*(src[7]+(row+1)*frameWidth+1)) + (1) * (*(src[7]+(row+1)*frameWidth+2));
			
			gradientBoundary[3*blockSize+row] = sqrt((double)(gradientx * gradientx + gradienty * gradienty));
			if(gradientx==0)
				angleBoundary[3*blockSize+row] = 90;
			else
				angleBoundary[3*blockSize+row] = (atan((double)(gradienty)/(gradientx))) * (180/PI);

			if(angleBoundary[3*blockSize+row]>=-90 && angleBoundary[3*blockSize+row]<-84)
				angleBoundary[3*blockSize+row] = 90;

			angleBoundary[3*blockSize+row] = quantize_angle(angleBoundary[3*blockSize+row]);

			if(gradientBoundary[3*blockSize+row] > THRESH)
				incCounter(counter, angleBoundary[3*blockSize+row], gradientBoundary[3*blockSize+row]);

			if(gradientBoundary[3*blockSize+row] > THRESH)
			{
				sumNum += angleBoundary[3*blockSize+row] * gradientBoundary[3*blockSize+row];
				sumDen += gradientBoundary[3*blockSize+row];
			}

			if(maxgrad<gradientBoundary[3*blockSize+row])
				maxgrad = gradientBoundary[3*blockSize+row];
		}
		(*nbrs)++;
	}

	if(sumDen==0.0)
		dominant_angle = INF; //do normal bilinear interpolation
	else
		dominant_angle = sumNum/sumDen; //directional interpolation in this direction

	//Finding dominant angle based on adaptive thresholding of gradient magnitude
	//dominant_angle = findBestAngle(gradientBoundary, angleBoundary, maxgrad, src, blockSize);

	//Quantize this dominant angle
	dominant_angle = quantize_angle(dominant_angle);

	//Finding dominant angle with the counter
	if(DIR_MODE)
		dominant_angle = bestangle(counter); //Best method

	//Counter method with adaptive threshold
	//dominant_angle = counterAngle(gradientBoundary, angleBoundary, maxgrad, src, blockSize);

	//Finding the edge direction
	if(dominant_angle>=0 && dominant_angle<=90)
		dominant_angle = dominant_angle - 90;
	else if(dominant_angle<0  && dominant_angle>=-90)
		dominant_angle = dominant_angle + 90;

	dominant_angle = quantize_angle(dominant_angle);

	return dominant_angle;
}

static double findBestAngle(double *gradientBoundary, double *angleBoundary, double maxgrad, imgpel *src[], int blockSize)
{
	double bestangle, low_gradient;
	double sumNum, sumDen;
	int i;

	low_gradient = maxgrad * (1 - THRESH_LOW/100);

	sumNum = 0.0;
	sumDen = 0.0;

	if(src[4]!=NULL)
	{
		for(i=1;i<blockSize-1;i++)
		{
			if(gradientBoundary[i]>=low_gradient)
			{
				sumNum += gradientBoundary[i]*angleBoundary[i];
				sumDen += gradientBoundary[i];
			}
		}
	}

	if(src[5]!=NULL)
	{
		for(i=blockSize+1;i<2*blockSize-1;i++)
		{
			if(gradientBoundary[i]>=low_gradient)
			{
				sumNum += gradientBoundary[i]*angleBoundary[i];
				sumDen += gradientBoundary[i];
			}
		}
	}

	if(src[6]!=NULL)
	{
		for(i=2*blockSize+1;i<3*blockSize-1;i++)
		{
			if(gradientBoundary[i]>=low_gradient)
			{
				sumNum += gradientBoundary[i]*angleBoundary[i];
				sumDen += gradientBoundary[i];
			}
		}
	}
	
	if(src[7]!=NULL)
	{
		for(i=3*blockSize+1;i<4*blockSize-1;i++)
		{
			if(gradientBoundary[i]>=low_gradient)
			{
				sumNum += gradientBoundary[i]*angleBoundary[i];
				sumDen += gradientBoundary[i];
			}
		}
	}

	bestangle = sumNum/sumDen;
	return bestangle;
}

// Using DIR_MODE
static double bestangle(double counter[])
{
	int i, maxidx, flag=0;
	double max;

	for(i=0;i<16;i++)
	{
		if(counter[i]!=0)
		{
			flag=1;
			break;
		}
	}
	if(!flag)
		return 90;

	max = counter[0];
	maxidx = 0;
	for(i=1;i<16;i++)
	{
		if(max<counter[i])
		{
			max = counter[i];
			maxidx = i;
		}
	}

	switch(maxidx)
	{
	case 0:
		return -78.0;
		break;
	case 1:
		return -67.5;
		break;
	case 2:
		return -56.0;
		break;
	case 3:
		return -45.0;
		break;
	case 4:
		return -34.0;
		break;
	case 5:
		return -22.5;
		break;
	case 6:
		return -11.0;
		break;
	case 7:
		return 0.0;
		break;
	case 8:
		return 11.0;
		break;
	case 9:
		return 22.5;
		break;
	case 10:
		return 34.0;
		break;
	case 11:
		return 45.0;
		break;
	case 12:
		return 56.0;
		break;
	case 13:
		return 67.5;
		break;
	case 14:
		return 78.0;
		break;
	case 15:
		return 90.0;
		break;
	}
	return 90;
}

static double quantize_angle(double angle)
{
	double angle_q, angle_in;
	angle_in = fabs(angle); //absolute value;

	if(angle == INF)
		return INF;

	//8 directions
	if(angle_in>=0 && angle_in<=11)
		angle_q = 0;
	else if(angle_in>11 && angle_in<=34)
		angle_q = 22.5;
	else if(angle_in>34 && angle_in<=56)
		angle_q = 45;
	else if(angle_in>56 && angle_in<=79)
		angle_q = 67.5;
	else if(angle_in>79 && angle_in<=90)
		angle_q = 90;

	if(angle<0 && angle_q!=0)
		angle_q = -angle_q;
	if(angle_q==-90)
		angle_q=90;

	return angle_q;
}

static int mean_interpolate(imgpel *src[], int blockSize, int frameWidth, int row, int column)
{
	int value, weight, tmp=0, srcCounter=0, bmax=blockSize-1;
	int d1, d2, p1, p2;

	//Concealing as per the nearest available neighnor only
	//4 cases:

	d1=-1; d2=-1; p1=-1; p2=-1;

	if( (row+1)<=blockSize/2 && (column+1)<=blockSize/2 )
	{
		//Above and Left
		if(src[4]!=NULL)
		{
			d1 = row+1;
			p1 = *(src[4]+bmax*frameWidth+column);
		}
		if(src[5]!=NULL)
		{
			d2 = column+1;
			p2 = *(src[5]+row*frameWidth+bmax);
		}
	}
	if( (row+1)<=blockSize/2 && (column+1)>blockSize/2 )
	{
		//Above and Right
		if(src[4]!=NULL)
		{
			d1 = row+1;
			p1 = *(src[4]+bmax*frameWidth+column);
		}
		if(src[7]!=NULL)
		{
			d2 = blockSize-column;
			p2 = *(src[7]+row*frameWidth);
		}
	}
	if( (row+1)>blockSize/2 && (column+1)<=blockSize/2 )
	{
		//Below and Left
		if(src[6]!=NULL)
		{
			d1 = blockSize-row;
			p1 = *(src[6]+column);
		}
		if(src[5]!=NULL)
		{
			d2 = column+1;
			p2 = *(src[5]+row*frameWidth+bmax);
		}
	}
	if( (row+1)>blockSize/2 && (column+1)>blockSize/2 )
	{
		//Below and Right
		if(src[6]!=NULL)
		{
			d1 = blockSize-row;
			p1 = *(src[6]+column);
		}
		if(src[7]!=NULL)
		{
			d2 = blockSize-column;
			p2 = *(src[7]+row*frameWidth);
		}
	}

	if(p1==-1 && p2!=-1)
		value = p2;
	if(p1!=-1 && p2==-1)
		value = p1;
	if(p1!=-1 && p2!=-1)
		value = (p1*d2 + p2*d1)/(d1+d2);

	if(p1==-1 && p2==-1)
	{
		// above 
		if ( src[4] != NULL )
		{
			weight = blockSize-row;
			tmp += weight * (*(src[4]+bmax*frameWidth+column));
			srcCounter += weight;
		}
		// left 
		if ( src[5] != NULL )
		{
			weight = blockSize-column;
			tmp += weight * (*(src[5]+row*frameWidth+bmax));
			srcCounter += weight;
		}
		// below 
		if ( src[6] != NULL )
		{
			weight = row+1;
			tmp += weight * (*(src[6]+column));
			srcCounter += weight;
		}
		// right 
		if ( src[7] != NULL )
		{
			weight = column+1;
			tmp += weight * (*(src[7]+row*frameWidth));
			srcCounter += weight;
		}
		if ( srcCounter > 0 )
			value = (byte)(tmp/srcCounter);
		else
			value = blockSize == 8 ? img->dc_pred_value_chroma : img->dc_pred_value_luma;
	}

	return value;
}

static void incCounter(double counter[], double angle, double gradient)
{
	angle = quantize_angle(angle);

	if(angle==-78)
		counter[0] += gradient;
	else if(angle==-67.5)
		counter[1] += gradient;
	else if(angle==-56)
		counter[2] += gradient;
	else if(angle==-45)
		counter[3] += gradient;
	else if(angle==-34)
		counter[4] += gradient;
	else if(angle==-22.5)
		counter[5] += gradient;
	else if(angle==-11)
		counter[6] += gradient;
	else if(angle==0)
		counter[7] += gradient;
	else if(angle==11)
		counter[8] += gradient;
	else if(angle==22.5)
		counter[9] += gradient;
	else if(angle==34)
		counter[10] += gradient;
	else if(angle==45)
		counter[11] += gradient;
	else if(angle==56)
		counter[12] += gradient;
	else if(angle==67.5)
		counter[13] += gradient;
	else if(angle==78)
		counter[14] += gradient;
	else if(angle==90)
		counter[15] += gradient;
}

static double counterAngle(double *gradientBoundary, double *angleBoundary, double maxgrad, imgpel *src[], int blockSize)
{
	double best_angle, counter[16], low_gradient;
	int i;

	for(i=0;i<15;i++)
		counter[i] = 0.0;

	low_gradient = maxgrad * (1 - THRESH_LOW/100);

	if(src[4]!=NULL)
	{
		for(i=1;i<blockSize-1;i++)
		{
			if(gradientBoundary[i]>=low_gradient)
			{
				incCounter(counter, angleBoundary[i], gradientBoundary[i]);
			}
		}
	}

	if(src[5]!=NULL)
	{
		for(i=blockSize+1;i<2*blockSize-1;i++)
		{
			if(gradientBoundary[i]>=low_gradient)
			{
				incCounter(counter, angleBoundary[i], gradientBoundary[i]);
			}
		}
	}

	if(src[6]!=NULL)
	{
		for(i=2*blockSize+1;i<3*blockSize-1;i++)
		{
			if(gradientBoundary[i]>=low_gradient)
			{
				incCounter(counter, angleBoundary[i], gradientBoundary[i]);
			}
		}
	}
	
	if(src[7]!=NULL)
	{
		for(i=3*blockSize+1;i<4*blockSize-1;i++)
		{
			if(gradientBoundary[i]>=low_gradient)
			{
				incCounter(counter, angleBoundary[i], gradientBoundary[i]);
			}
		}
	}

	best_angle = bestangle(counter);
	return best_angle;
}



/*!
 ************************************************************************
 * \brief
 *      Does the actual pixel based Directional interpolation for block[]
 *      using the information of PMODES of neighboring macroblocks. 
 *		Sobel edge detecting masks are used to find edge magnitudes. Mask is applied to available neighboring
 *      boundary pixels, then the dominant edge direction is determined
 *      and the pixel is concealed in the direction of dominant edge using weighted interpolation. 
 * \param src[] 
 *      pointers to neighboring source blocks
 * \param block     
 *      destination block
 * \param blockSize
 *      16 for Y, 8 for U/V components
 * \param frameWidth
 *      Width of the frame in pixels
 ************************************************************************
 */
static void pixDirInterpolateBlockwithPModes(imgpel *src[], imgpel *block, int blockSize, int frameWidth, int comp, int row_idx, int column_idx)
{
	int row, column, k, dominant_pmode;
	int block_row, block_column;
	double theta, theta1, theta2;
	struct direction dir_pixel;
	imgpel *block2;
	int block2k, dpm1, dpm2, numDED = 0;
	int fMDI=0, fBI=0, edgeDir[9], edgeStrength[9];
	double dirEntropy = 0.0;
	
	//Angles as per 9 prediction modes. Note that angle for pmode=2 is not applicable.
	double angle[] = {90.0, 0.0, -1.0, -45.0, 45.0, 67.5, 22.5, -67.5, -22.5};

	block2 = (imgpel*)malloc(256*sizeof(imgpel));

	// for multiple DEDs
	for (k=0;k<9;k++)
	{
		edgeDir[k] = 0;
		edgeStrength[k] = 0;
	}

	// for debugging purposes only
	if(column_idx==34 && row_idx==32)
		column_idx=34;

	//Multiples of 4x4 blocks index
	block_row = row_idx<<1;
	block_column = column_idx<<1;

	//Finding the Dominant pmode from the neighboring MBs
	if(comp==LUMA)
	{
		//Perpendicular to Direction, mag = max - min
		if (PMODES_UPDATED || SWDI)
			dominant_pmode = findDominantPmodeLumaUpdated(src, blockSize, block_row, block_column, frameWidth, &fMDI, &fBI, edgeDir, edgeStrength, &dirEntropy, &numDED);
		else
			dominant_pmode = findDominantPmodeLuma(src, blockSize, block_row, block_column, frameWidth);
		
		theta = angle[dominant_pmode];
	}
	else
	{
		dominant_pmode  = findDominantPmodeChroma(src, blockSize, row_idx, column_idx, frameWidth);
		switch(dominant_pmode)
		{
		case 0:
			theta = INF;
			break;
		case 1:
			theta = 0;
			break;
		case 2:
			theta = 90;
			break;
		case 3:
			theta = -45;
			break;
		}
	}	

	if (SWDI)
	{
		if ( (numDED == 0) || (numDED > 2) || (dirEntropy >= DIRENTROPYTHRESH) )
		{
			// Apply BI
			pixMeanInterpolateBlock( src, block, blockSize, frameWidth );
		}
		else
		{
			// Apply DI
			pixDirectionalInterpolateBlock( src, block, blockSize, frameWidth, comp, row_idx, column_idx);
		}
		return;
	}

	if (comp==LUMA && fBI && PMODES_UPDATED)
	{
		// Too many edge directions OR Directional Entropy >= Threshold
		// In this case, better to do BI than DI
		pixMeanInterpolateBlock( src, block, blockSize, frameWidth );

		if (ENTROPYAREA)
		{
			showEntropyArea(dirEntropy, blockSize, block, frameWidth);
		}
		return;
	}

	if (comp==LUMA && fMDI && PMODES_UPDATED)
	{
		// Multiple Direction Interpolation
		for (k=0;k<9;k++)
		{
			if (edgeDir[k] == 1)
			{
				theta1 = angle[k];
				edgeDir[k] = 0;
				dpm1 = k;
				break;
			}
		}
		for (k=0;k<9;k++)
		{
			if (edgeDir[k] == 1)
			{
				theta2 = angle[k];
				edgeDir[k] = 0;
				dpm2 = k;
				break;
			}
		}

		// Got two angles, now MDI

		// Interpolate on theta1
		k = 0; 
		for ( row = 0; row < blockSize; row++ ) 
		{
			for ( column = 0; column < blockSize; column++ )
			{			
				//Directional Interpolation
				//Compute dir_pixel.p1,p2,d1,d2
				boundary_pixels_pmodes(src, blockSize, frameWidth, theta1, row, column, &dir_pixel, comp);		
				
				if( (dir_pixel.p1==-1 && dir_pixel.p2==-1) )// || (dir_pixel.d1==0.0 && dir_pixel.d2==0.0) )
					block[k + column] = mean_interpolate(src, blockSize, frameWidth, row, column);
				else if(dir_pixel.p1==-1 && dir_pixel.p2!=-1)
					block[k+column] = dir_pixel.p2;
				else if(dir_pixel.p2==-1 && dir_pixel.p1!=-1)
					block[k+column] = dir_pixel.p1;
				else
					block[k + column] = (int)((dir_pixel.d2*dir_pixel.p1 + dir_pixel.d1*dir_pixel.p2)/(dir_pixel.d1+dir_pixel.d2));

				if(LOSSAREA)
					block[k+column] = 0; 
			}
			k += frameWidth;
		}

		// Interpolate on theta2
		k = 0; 
		for ( row = 0; row < blockSize; row++ ) 
		{
			for ( column = 0; column < blockSize; column++ )
			{			
				//Directional Interpolation
				//Compute dir_pixel.p1,p2,d1,d2
				boundary_pixels_pmodes(src, blockSize, frameWidth, theta2, row, column, &dir_pixel, comp);		
			
				if( (dir_pixel.p1==-1 && dir_pixel.p2==-1) )// || (dir_pixel.d1==0.0 && dir_pixel.d2==0.0) )
					block2[k + column] = mean_interpolate(src, blockSize, frameWidth, row, column);
				else if(dir_pixel.p1==-1 && dir_pixel.p2!=-1)
					block2[k+column] = dir_pixel.p2;
				else if(dir_pixel.p2==-1 && dir_pixel.p1!=-1)
					block2[k+column] = dir_pixel.p1;
				else
					block2[k + column] = (int)((dir_pixel.d2*dir_pixel.p1 + dir_pixel.d1*dir_pixel.p2)/(dir_pixel.d1+dir_pixel.d2));

				if(LOSSAREA)
					block2[k+column] = 0; 
			}
			k += 16;
		}

		// Weighted Average of two DIs => MDI 
		k=0;block2k=0;
		for ( row = 0; row < blockSize; row++ ) 
		{
			for ( column = 0; column < blockSize; column++ )
			{	
				//block[k + column] = (block[k + column] + block2[block2k + column])/2;
				block[k + column] = (block[k + column]*edgeStrength[dpm1] + block2[block2k + column]*edgeStrength[dpm2])/(edgeStrength[dpm1]+edgeStrength[dpm2]);
			}
			k+=frameWidth; block2k+=16;
		}

		if(LOSSAREA)
			block[k+column] = 0; 
	}
	else
	{
		// Single Direction Interpolation
		k = 0; 
		for ( row = 0; row < blockSize; row++ ) 
		{
			for ( column = 0; column < blockSize; column++ )
			{			
				//Directional Interpolation
				//Compute dir_pixel.p1,p2,d1,d2
				boundary_pixels_pmodes(src, blockSize, frameWidth, theta, row, column, &dir_pixel, comp);		
				
				if( (dir_pixel.p1==-1 && dir_pixel.p2==-1) )// || (dir_pixel.d1==0.0 && dir_pixel.d2==0.0) )
					block[k + column] = mean_interpolate(src, blockSize, frameWidth, row, column);
				else if(dir_pixel.p1==-1 && dir_pixel.p2!=-1)
					block[k+column] = dir_pixel.p2;
				else if(dir_pixel.p2==-1 && dir_pixel.p1!=-1)
					block[k+column] = dir_pixel.p1;
				else
					block[k + column] = (int)((dir_pixel.d2*dir_pixel.p1 + dir_pixel.d1*dir_pixel.p2)/(dir_pixel.d1+dir_pixel.d2));

				if(LOSSAREA)
					block[k+column] = 0; 
			}
			k += frameWidth;
		}
	}

	if (ENTROPYAREA && comp==LUMA && PMODES_UPDATED)
	{
		showEntropyArea(dirEntropy, blockSize, block, frameWidth);
	}
}

static int findDominantPmodeLuma(imgpel *src[], int blockSize, int block_row, int block_column, int frameWidth)
{
	int maxval, minval, maxmag, counter[9]; //counter for 9 pmodes of Intra4x4
	int k, row, column, bmax = blockSize-1, ipmode, dominant_pmode = 2, pixval[4];
	float distance;
	
	/* Prediction Modes:
	0: Vertical
	1: Horizontal
	2: Mean
	3: Diagonal Down Left
	4: Diagonal Down Right
	5: Vertical Right
	6: Horizontal Down
	7: Vertical Left
	8: Horizontal Up
	*/
	for(k=0;k<9;k++)
		counter[k] = 0;

	//Find the mode type of nbr MB, if 16x16, then all 4 4x4 blocks pmode same as that

	/* Above */
	if ( src[4] != NULL )   
	{
		for ( column = 0; column < blockSize; column+=(blockSize>>2) ) 
		{
			//Each 4x4 block
			ipmode = img->ipredmode[block_column + column/(blockSize>>2)][block_row-1]; //this is the edge direction
			maxval = 0; minval = 255;

			switch(ipmode)
			{
			case 0:
				pixval[0] = *(src[4]+(bmax-2)*frameWidth+(column+0)); pixval[1] = *(src[4]+(bmax-2)*frameWidth+(column+1));
				pixval[2] = *(src[4]+(bmax-2)*frameWidth+(column+2)); pixval[3] = *(src[4]+(bmax-2)*frameWidth+(column+3));
				break;
			case 1:
				pixval[0] = *(src[4]+(bmax-3)*frameWidth+(column+2)); pixval[1] = *(src[4]+(bmax-2)*frameWidth+(column+2));
				pixval[2] = *(src[4]+(bmax-1)*frameWidth+(column+2)); pixval[3] = *(src[4]+(bmax-0)*frameWidth+(column+2));
				break;
			case 3:
				pixval[0] = *(src[4]+(bmax-3)*frameWidth+(column+0)); pixval[1] = *(src[4]+(bmax-2)*frameWidth+(column+1));
				pixval[2] = *(src[4]+(bmax-1)*frameWidth+(column+2)); pixval[3] = *(src[4]+(bmax-0)*frameWidth+(column+3));
				break;
			case 4:
				pixval[0] = *(src[4]+(bmax-3)*frameWidth+(column+3)); pixval[1] = *(src[4]+(bmax-2)*frameWidth+(column+2));
				pixval[2] = *(src[4]+(bmax-1)*frameWidth+(column+1)); pixval[3] = *(src[4]+(bmax-0)*frameWidth+(column+0));
				break;
			case 5:
				pixval[0] = *(src[4]+(bmax-1)*frameWidth+(column+0)); pixval[1] = *(src[4]+(bmax-1)*frameWidth+(column+1));
				pixval[2] = *(src[4]+(bmax-2)*frameWidth+(column+2)); pixval[3] = *(src[4]+(bmax-2)*frameWidth+(column+3));
				break;
			case 6:
				pixval[0] = *(src[4]+(bmax-3)*frameWidth+(column+2)); pixval[1] = *(src[4]+(bmax-2)*frameWidth+(column+2));
				pixval[2] = *(src[4]+(bmax-1)*frameWidth+(column+1)); pixval[3] = *(src[4]+(bmax-0)*frameWidth+(column+1));
				break;
			case 7:
				pixval[0] = *(src[4]+(bmax-2)*frameWidth+(column+0)); pixval[1] = *(src[4]+(bmax-2)*frameWidth+(column+1));
				pixval[2] = *(src[4]+(bmax-1)*frameWidth+(column+2)); pixval[3] = *(src[4]+(bmax-1)*frameWidth+(column+3));
				break;
			case 8:
				pixval[0] = *(src[4]+(bmax-3)*frameWidth+(column+1)); pixval[1] = *(src[4]+(bmax-2)*frameWidth+(column+1));
				pixval[2] = *(src[4]+(bmax-1)*frameWidth+(column+2)); pixval[3] = *(src[4]+(bmax-0)*frameWidth+(column+2));
				break;
			}

			if(ipmode!=2)
			{
				maxval = max(maxval, max(pixval[0], max(pixval[1], max(pixval[2], pixval[3]) ) ) );
				minval = min(minval, min(pixval[0], min(pixval[1], min(pixval[2], pixval[3]) ) ) );
				distance = findDistance(pixval, ipmode, maxval, minval);		
				//counter[ipmode] += (int)((maxval - minval)*distance);
				//counter[ipmode] += (int)((maxval - minval)/distance);
				counter[ipmode] += (int)(maxval - minval);
			}

		}
	}

	/* Left */
	if ( src[5] != NULL )   
	{
		for ( row = 0; row < blockSize; row+=(blockSize>>2) ) 
		{
			//Each 4x4 block
			ipmode = img->ipredmode[block_column-1][block_row + row/(blockSize>>2)]; //this is the edge direction
			maxval = 0; minval = 255;

			switch(ipmode)
			{
			case 0:
				pixval[0] = *(src[5]+(row+1)*frameWidth+(bmax-3)); pixval[1] = *(src[5]+(row+1)*frameWidth+(bmax-2));
				pixval[2] = *(src[5]+(row+1)*frameWidth+(bmax-1)); pixval[3] = *(src[5]+(row+1)*frameWidth+(bmax-0));
				break;
			case 1:
				pixval[0] = *(src[5]+(row+0)*frameWidth+(bmax-1)); pixval[1] = *(src[5]+(row+1)*frameWidth+(bmax-1));
				pixval[2] = *(src[5]+(row+2)*frameWidth+(bmax-1)); pixval[3] = *(src[5]+(row+3)*frameWidth+(bmax-1));
				break;
			case 3:
				pixval[0] = *(src[5]+(row+0)*frameWidth+(bmax-3)); pixval[1] = *(src[5]+(row+1)*frameWidth+(bmax-2));
				pixval[2] = *(src[5]+(row+2)*frameWidth+(bmax-1)); pixval[3] = *(src[5]+(row+3)*frameWidth+(bmax-0));
				break;
			case 4:
				pixval[0] = *(src[5]+(row+0)*frameWidth+(bmax-0)); pixval[1] = *(src[5]+(row+1)*frameWidth+(bmax-1));
				pixval[2] = *(src[5]+(row+2)*frameWidth+(bmax-2)); pixval[3] = *(src[5]+(row+3)*frameWidth+(bmax-3));
				break;
			case 5:
				pixval[0] = *(src[5]+(row+1)*frameWidth+(bmax-0)); pixval[1] = *(src[5]+(row+1)*frameWidth+(bmax-1));
				pixval[2] = *(src[5]+(row+2)*frameWidth+(bmax-2)); pixval[3] = *(src[5]+(row+2)*frameWidth+(bmax-3));
				break;
			case 6:
				pixval[0] = *(src[5]+(row+0)*frameWidth+(bmax-1)); pixval[1] = *(src[5]+(row+1)*frameWidth+(bmax-1));
				pixval[2] = *(src[5]+(row+2)*frameWidth+(bmax-2)); pixval[3] = *(src[5]+(row+3)*frameWidth+(bmax-2));
				break;
			case 7:
				pixval[0] = *(src[5]+(row+1)*frameWidth+(bmax-3)); pixval[1] = *(src[5]+(row+1)*frameWidth+(bmax-2));
				pixval[2] = *(src[5]+(row+2)*frameWidth+(bmax-1)); pixval[3] = *(src[5]+(row+2)*frameWidth+(bmax-0));
				break;
			case 8:
				pixval[0] = *(src[5]+(row+0)*frameWidth+(bmax-2)); pixval[1] = *(src[5]+(row+1)*frameWidth+(bmax-2));
				pixval[2] = *(src[5]+(row+2)*frameWidth+(bmax-1)); pixval[3] = *(src[5]+(row+3)*frameWidth+(bmax-1));
				break;
			}

			if(ipmode!=2)
			{
				maxval = max(maxval, max(pixval[0], max(pixval[1], max(pixval[2], pixval[3]) ) ) );
				minval = min(minval, min(pixval[0], min(pixval[1], min(pixval[2], pixval[3]) ) ) );
				distance = findDistance(pixval, ipmode, maxval, minval);
				//counter[ipmode] += (int)((maxval - minval)*distance);
				//counter[ipmode] += (int)((maxval - minval)/distance);
				counter[ipmode] += (int)(maxval - minval);
			}

		}
	}

	/* Below */
	if ( src[6] != NULL )   
	{
		for ( column = 0; column < blockSize; column+=(blockSize>>2) ) 
		{
			//Each 4x4 block
			ipmode = img->ipredmode[block_column + column/(blockSize>>2)][block_row + (blockSize>>2)]; //this is the edge direction			
			maxval = 0; minval = 255;

			switch(ipmode)
			{
			case 0:
				pixval[0] = *(src[6]+1*frameWidth+(column+0)); pixval[1] = *(src[6]+1*frameWidth+(column+1));
				pixval[2] = *(src[6]+1*frameWidth+(column+2)); pixval[3] = *(src[6]+1*frameWidth+(column+3));
				break;
			case 1:
				pixval[0] = *(src[6]+0*frameWidth+(column+2)); pixval[1] = *(src[6]+1*frameWidth+(column+2));
				pixval[2] = *(src[6]+2*frameWidth+(column+2)); pixval[3] = *(src[6]+3*frameWidth+(column+2));
				break;
			case 3:
				pixval[0] = *(src[6]+0*frameWidth+(column+0)); pixval[1] = *(src[6]+1*frameWidth+(column+1));
				pixval[2] = *(src[6]+2*frameWidth+(column+2)); pixval[3] = *(src[6]+3*frameWidth+(column+3));
				break;
			case 4:
				pixval[0] = *(src[6]+0*frameWidth+(column+3)); pixval[1] = *(src[6]+1*frameWidth+(column+2));
				pixval[2] = *(src[6]+2*frameWidth+(column+1)); pixval[3] = *(src[6]+3*frameWidth+(column+0));
				break;
			case 5:
				pixval[0] = *(src[6]+1*frameWidth+(column+3)); pixval[1] = *(src[6]+1*frameWidth+(column+2));
				pixval[2] = *(src[6]+2*frameWidth+(column+1)); pixval[3] = *(src[6]+2*frameWidth+(column+0));
				break;
			case 6:
				pixval[0] = *(src[6]+0*frameWidth+(column+2)); pixval[1] = *(src[6]+1*frameWidth+(column+2));
				pixval[2] = *(src[6]+2*frameWidth+(column+1)); pixval[3] = *(src[6]+3*frameWidth+(column+1));
				break;
			case 7:
				pixval[0] = *(src[6]+1*frameWidth+(column+0)); pixval[1] = *(src[6]+1*frameWidth+(column+1));
				pixval[2] = *(src[6]+2*frameWidth+(column+2)); pixval[3] = *(src[6]+2*frameWidth+(column+3));
				break;
			case 8:
				pixval[0] = *(src[6]+0*frameWidth+(column+1)); pixval[1] = *(src[6]+1*frameWidth+(column+1));
				pixval[2] = *(src[6]+2*frameWidth+(column+2)); pixval[3] = *(src[6]+3*frameWidth+(column+2));
				break;
			}

			if(ipmode!=2)
			{
				maxval = max(maxval, max(pixval[0], max(pixval[1], max(pixval[2], pixval[3]) ) ) );
				minval = min(minval, min(pixval[0], min(pixval[1], min(pixval[2], pixval[3]) ) ) );
				distance = findDistance(pixval, ipmode, maxval, minval);
				//counter[ipmode] += (int)((maxval - minval)*distance);
				//counter[ipmode] += (int)((maxval - minval)/distance);
				counter[ipmode] += (int)(maxval - minval);
			}

		}
	}

	/* Right */
	if ( src[7] != NULL )   
	{
		for ( row = 0; row < blockSize; row+=(blockSize>>2) ) 
		{
			//Each 4x4 block
			ipmode = img->ipredmode[block_column + (blockSize>>2)][block_row + row/(blockSize>>2)]; //this is the edge direction			
			maxval = 0; minval = 255;

			switch(ipmode)
			{
			case 0:
				pixval[0] = *(src[7]+(row+1)*frameWidth+0); pixval[1] = *(src[7]+(row+1)*frameWidth+1);
				pixval[2] = *(src[7]+(row+1)*frameWidth+2); pixval[3] = *(src[7]+(row+1)*frameWidth+3);
				break;
			case 1:
				pixval[0] = *(src[7]+(row+0)*frameWidth+2); pixval[1] = *(src[7]+(row+1)*frameWidth+2);
				pixval[2] = *(src[7]+(row+2)*frameWidth+2); pixval[3] = *(src[7]+(row+3)*frameWidth+2);
				break;
			case 3:
				pixval[0] = *(src[7]+(row+0)*frameWidth+0); pixval[1] = *(src[7]+(row+1)*frameWidth+1);
				pixval[2] = *(src[7]+(row+2)*frameWidth+2); pixval[3] = *(src[7]+(row+3)*frameWidth+3);
				break;
			case 4:
				pixval[0] = *(src[7]+(row+0)*frameWidth+3); pixval[1] = *(src[7]+(row+1)*frameWidth+2);
				pixval[2] = *(src[7]+(row+2)*frameWidth+1); pixval[3] = *(src[7]+(row+3)*frameWidth+0);
				break;
			case 5:
				pixval[0] = *(src[7]+(row+1)*frameWidth+3); pixval[1] = *(src[7]+(row+1)*frameWidth+2);
				pixval[2] = *(src[7]+(row+2)*frameWidth+1); pixval[3] = *(src[7]+(row+2)*frameWidth+0);
				break;
			case 6:
				pixval[0] = *(src[7]+(row+0)*frameWidth+2); pixval[1] = *(src[7]+(row+1)*frameWidth+2);
				pixval[2] = *(src[7]+(row+2)*frameWidth+1); pixval[3] = *(src[7]+(row+3)*frameWidth+1);
				break;
			case 7:
				pixval[0] = *(src[7]+(row+1)*frameWidth+0); pixval[1] = *(src[7]+(row+1)*frameWidth+1);
				pixval[2] = *(src[7]+(row+2)*frameWidth+2); pixval[3] = *(src[7]+(row+2)*frameWidth+3);
				break;
			case 8:
				pixval[0] = *(src[7]+(row+0)*frameWidth+1); pixval[1] = *(src[7]+(row+1)*frameWidth+1);
				pixval[2] = *(src[7]+(row+2)*frameWidth+2); pixval[3] = *(src[7]+(row+3)*frameWidth+2);
				break;
			}

			if(ipmode!=2)
			{
				maxval = max(maxval, max(pixval[0], max(pixval[1], max(pixval[2], pixval[3]) ) ) );
				minval = min(minval, min(pixval[0], min(pixval[1], min(pixval[2], pixval[3]) ) ) );
				distance = findDistance(pixval, ipmode, maxval, minval);
				//counter[ipmode] += (int)((maxval - minval)*distance);
				//counter[ipmode] += (int)((maxval - minval)/distance);
				counter[ipmode] += (int)(maxval - minval);
			}

		}
	}

	//Now, find the dominant_pmode with maximum edge magnitude
	maxmag = counter[0];
	dominant_pmode = 0;

	for(k=1;k<9;k++)
	{
		if(maxmag < counter[k])
		{
			maxmag = counter[k];
			dominant_pmode = k;
		}
	}

	return dominant_pmode;
}

static float findDistance(int pixval[4], int ipmode, int maxval, int minval)
{
	int i, maxidx, minidx;
	float distance;

	for(i=0;i<4;i++)
	{
		if(maxval==pixval[i])
			maxidx=i;
		if(minval==pixval[i])
			minidx=i;
	}

	if(ipmode==0 || ipmode==1)
		distance = (float)(maxidx-minidx);

	if(ipmode==3 || ipmode==4)
		distance = (float)((maxidx-minidx)*1.4);

	if(ipmode==5 || ipmode==6 || ipmode==7 || ipmode==8)
	{
		if( (maxidx==0 && minidx==1) || (maxidx==1 && minidx==0) || (maxidx==2 && minidx==3) || (maxidx==3 && minidx==2) )
			distance = (float)1.0;

		if( (maxidx==1 && minidx==2) || (maxidx==2 && minidx==1) )
			distance = (float)1.4;

		if( (maxidx==0 && minidx==2) || (maxidx==2 && minidx==0) || (maxidx==1 && minidx==3) || (maxidx==3 && minidx==1) )
			distance = (float)2.2;

		if( (maxidx==0 && minidx==3) || (maxidx==3 && minidx==0) )
			distance = (float)3.0;
	}

	if(distance<0)
		distance=-distance;

	return distance;
}

static int findDominantPmodeChroma(imgpel *src[], int blockSize, int row_idx, int column_idx, int frameWidth)
{
	int count, ipmode, dominant_pmode = 0, mb_nbr, counter[4]; //counter for 4 pmodes of Intra8x8 Chroma
	
	/* Prediction Modes for Chroma:
	0: DC_Pred (No Angle, do bilinear interpolation)
	1: Horizontal (0)
	2: Vertical (90)
	3: Plane (-45)
	*/
	for(count=0;count<4;count++)
		counter[count] = 0;
	
	//Current MB number
	mb_nbr = row_idx*img->PicWidthInMbs + column_idx;

	//Above mb_nbr - img->PicWidthInMbs
	//Left mb_nbr - 1 
	//Below mb_nbr + img->PicWidthInMbs
	//Right mb_nbr + 1

	/* Above */
	if ( src[4] != NULL )   
	{
		ipmode = img->mb_data[mb_nbr - img->PicWidthInMbs].c_ipred_mode;
		counter[ipmode]++;
	}

	/* Left */
	if ( src[5] != NULL )   
	{
		ipmode = img->mb_data[mb_nbr - 1].c_ipred_mode;
		counter[ipmode]++;
	}

	/* Below */
	if ( src[6] != NULL )   
	{
		ipmode = img->mb_data[mb_nbr + img->PicWidthInMbs].c_ipred_mode;
		counter[ipmode]++;
	}

	/* Right */
	if ( src[7] != NULL )   
	{
		ipmode = img->mb_data[mb_nbr + 1].c_ipred_mode;
		counter[ipmode]++;
	}

	//Now, find the dominant_pmode with maximum ipmode count
	count = 0;
	dominant_pmode = 0;

	//do not check for mode=0, as it is only mean, no edge indications

	if(counter[1]>count)
	{
		count = counter[1];
		dominant_pmode = 1;
	}
	if(counter[2]>count)
	{
		count = counter[2];
		dominant_pmode = 2;
	}
	if(counter[3]>count)
	{
		count = counter[3];
		dominant_pmode = 3;
	}

	return dominant_pmode;
}

static void boundary_pixels_pmodes(imgpel *src[], int blockSize, int frameWidth, double theta, int row, int column, struct direction *dir_pixel, int comp)
{
	int i1=-1, j1=-1, i2=-1, j2=-1, bmax = blockSize-1, ipmode;

	//Compute boundary pixels
	dir_pixel->p1 = -1; dir_pixel->p2 = -1; dir_pixel->d1 = -1.0; dir_pixel->d2 = -1.0;

	if(theta==-1.0 || theta==INF)
		return;

	if(comp==CHROMA)
	{
		if(theta==INF)
			return;
		if(theta==0) //Horizontal
			boundary_pmode1(src, blockSize, frameWidth, row, column, dir_pixel);
		if(theta==90) //Vertical
			boundary_pmode0(src, blockSize, frameWidth, row, column, dir_pixel);
		if(theta==-45) //Plane
			boundary_pmode3(src, blockSize, frameWidth, row, column, dir_pixel);
		return;
	}

	//Else LUMA:

	if(theta==90.0)
		ipmode=0;
	else if(theta==0.0)
		ipmode=1;
	else if(theta==-45.0)
		ipmode=3;
	else if(theta==45.0)
		ipmode=4;
	else if(theta==67.5)
		ipmode=5;
	else if(theta==22.5)
		ipmode=6;
	else if(theta==-67.5)
		ipmode=7;
	else if(theta==-22.5)
		ipmode=8;
	else if(theta==-1.0)
		ipmode=2;

	switch(ipmode)
	{
	case 0://90
		boundary_pmode0(src, blockSize, frameWidth, row, column, dir_pixel);
		break;
	case 1://0
		boundary_pmode1(src, blockSize, frameWidth, row, column, dir_pixel);
		break;
	case 3://-45
		boundary_pmode3(src, blockSize, frameWidth, row, column, dir_pixel);
		break;
	case 4://45
		boundary_pmode4(src, blockSize, frameWidth, row, column, dir_pixel);
		break;
	case 5://67.5
		boundary_pmode5(src, blockSize, frameWidth, row, column, dir_pixel);
		break;
	case 6://22.5
		boundary_pmode6(src, blockSize, frameWidth, row, column, dir_pixel);
		break;
	case 7://-67.5
		boundary_pmode7(src, blockSize, frameWidth, row, column, dir_pixel);
		break;
	case 8://-22.5
		boundary_pmode8(src, blockSize, frameWidth, row, column, dir_pixel);
		break;
	default://ipmode = 2
		break;
	}
}


static void boundary_pmode0(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel)
{
	// theta = 90 => pmode = 0

	//BELOW
	if(src[6]!=NULL)
	{
		dir_pixel->p2 = *(src[6]+column);
		dir_pixel->d2 = blockSize-row;
	}

	//ABOVE
	if(src[4]!=NULL)
	{
		dir_pixel->p1 = *(src[4]+(blockSize-1)*frameWidth+column);
		dir_pixel->d1 = row+1;
	}
}

static void boundary_pmode1(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel)
{
	// theta = 0 => pmode = 1

	//LEFT
	if(src[5]!=NULL)
	{
		dir_pixel->p2 = *(src[5]+row*frameWidth+blockSize-1);
		dir_pixel->d2 = column+1;
	}

	//RIGHT
	if(src[7]!=NULL)
	{
		dir_pixel->p1 = *(src[7]+row*frameWidth);
		dir_pixel->d1 = blockSize-column;
	}
}

static void boundary_pmode3(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel)
{
	// theta = -45 => pmode = 3
	int i1, j1, i2, j2, bmax=blockSize-1;

	if ((row + column) <= (blockSize/2 - 1))
	{
		// Above and Left
		
		// LEFT
		if (src[5] != NULL)
		{
			i1 = row + (column+1);
			if(i1<0 || i1>bmax)
			{
				dir_pixel->p1 = *(src[5]+row*frameWidth+bmax);
				dir_pixel->d1 = column+1;
			}
			else
			{
				dir_pixel->p1 = *(src[5]+i1*frameWidth+bmax);
				dir_pixel->d1 = sqrt( pow(column+1,2) + pow(i1-row,2) );
			}
		}

		// ABOVE
		if (src[4] != NULL)
		{
			j2 = column + (row+1);
			if(j2<0 || j2>bmax)
			{
				dir_pixel->p2 = *(src[4]+bmax*frameWidth+column);
				dir_pixel->d2 = row+1;
			}
			else
			{
				dir_pixel->p2 = *(src[4]+bmax*frameWidth+j2);
				dir_pixel->d2 = sqrt( pow(row+1,2) + pow(j2-column,2) );
			}
		}
	}
	else
	{
		// Below and Right

		// BELOW
		if (src[6] != NULL)
		{
			j1 = column - (blockSize-row);
			if(j1<0 || j1>bmax)
			{
				dir_pixel->p1 = *(src[6]+column);
				dir_pixel->d1 = blockSize - row;
			}
			else
			{
				dir_pixel->p1 = *(src[6]+j1);
				dir_pixel->d1 = sqrt( pow((blockSize-row),2) + pow(j1-column,2) );
			}
		}

		//RIGHT
		if (src[7] != NULL)
		{
			i2 = row - (blockSize-column);
			if(i2<0 || i2>bmax)
			{
				dir_pixel->p2 = *(src[7]+row*frameWidth);
				dir_pixel->d2 = blockSize - column;
			}
			else
			{
				dir_pixel->p2 = *(src[7]+i2*frameWidth);
				dir_pixel->d2 = sqrt( pow(blockSize-column,2) + pow(i2-row,2) );
			}
		}
	}
}

static void boundary_pmode4(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel)
{
	// theta = 45 => pmode = 4
	int i1, j1, i2, j2, bmax=blockSize-1;

	if (row <= column)
	{
		// Above and Right

		if (src[7]!=NULL)
		{
			//RIGHT
			i2 = row + (blockSize-column);
			if(i2<0 || i2>bmax)
			{
				dir_pixel->p2 = *(src[7]+row*frameWidth);
				dir_pixel->d2 = blockSize - column;
			}
			else
			{
				dir_pixel->p2 = *(src[7]+i2*frameWidth);
				dir_pixel->d2 = sqrt( pow(blockSize-column,2) + pow(i2-row,2) );
			}
		}

		// ABOVE
		if (src[4] != NULL)
		{
			j1 = column - (row+1);
			if(j1<0 || j1>bmax)
			{
				dir_pixel->p1 = *(src[4]+bmax*frameWidth+column);
				dir_pixel->d1 = row + 1;
			}
			else
			{
				dir_pixel->p1 = *(src[4]+bmax*frameWidth+j1);
				dir_pixel->d1 = sqrt( pow(row+1,2) + pow(j1-column,2) );
			}
		}
	}
	else
	{
		// Below and Left

		//LEFT
		if (src[5]!=NULL)
		{
			i1 = row - (column+1);
			if(i1<0 || i1>bmax)
			{
				dir_pixel->p1 = *(src[5]+row*frameWidth+bmax);
				dir_pixel->d1 = column + 1;
			}
			else
			{
				dir_pixel->p1 = *(src[5]+i1*frameWidth+bmax);
				dir_pixel->d1 = sqrt( pow(column+1,2) + pow(i1-row,2) );
			}
		}

		// BELOW
		if (src[6]!=NULL)
		{
			j2 = column + (blockSize-row);
			if(j2<0 || j2>bmax)
			{
				dir_pixel->p2 = *(src[6]+column);
				dir_pixel->d2 = blockSize - row;
			}
			else
			{
				dir_pixel->p2 = *(src[6]+j2);
				dir_pixel->d2 = sqrt( pow((blockSize-row),2) + pow(j2-column,2) );
			}
		}
	}
}

static void boundary_pmode5(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel)
{
	// theta = 67.5 => pmode = 5
	
	double f_i1, f_i2, f_j1, f_j2;
	int i11, i22, j11, j22, i1, j1, i2, j2, bmax = blockSize-1;

	// row = 2*column
	// row = 2*column - blockSize
	// 3 regions bounded by these two equations

	if (row > 2*column)
	{
		// Left and Below

		//LEFT
		if (src[5] != NULL)
		{
			f_i1 = row - (column+1)*2.5;
			i1 = (int)floor(f_i1);
			if(i1<0 || i1>bmax)
			{
				dir_pixel->p1 = *(src[5]+row*frameWidth+bmax);
				dir_pixel->d1 = column+1;
			}
			else
			{
				i11  = (i1-1)<0 ? 0 : i1-1;
				dir_pixel->p1 = ( *(src[5]+i1*frameWidth+bmax) + *(src[5]+i11*frameWidth+bmax) )/2;
				dir_pixel->d1 = sqrt( pow(column+1,2) + pow(f_i1-row,2) );
			}
		}
		
		// BELOW
		if (src[6] != NULL)
		{
			f_j2 = column + (16.5-row)/2.5;
			j2 = (int)ceil(f_j2);
			if(j2<0 || j2>bmax)
			{
				dir_pixel->p2 = *(src[6]+column);
				dir_pixel->d2 = blockSize - row;
			}
			else
			{
				j22 = (j2-1)<0 ? 0 : j2-1;
				dir_pixel->p2 = ( *(src[6]+j2) + *(src[6]+j22) )/2;
				dir_pixel->d2 = sqrt( pow((blockSize-row),2) + pow(f_j2-column,2) );
			}
		}
	}
	else if ( (row <= 2*column) && (row > (2*column - blockSize)) )
	{
		// Above and Below

		// ABOVE
		if (src[4] != NULL)
		{
			f_j1 = column - (row+1)/2.5;
			j1 = (int)floor(f_j1);
			if(j1<0 || j1>bmax)
			{
				dir_pixel->p1 = *(src[4]+bmax*frameWidth+column);
				dir_pixel->d1 = row+1;
			}
			else
			{
				j11 = (j1+1)>bmax ? bmax : j1+1;
				dir_pixel->p1 = ( *(src[4]+bmax*frameWidth+j1) + *(src[4]+bmax*frameWidth+j11) )/2;
				dir_pixel->d1 = sqrt( pow(row+1,2) + pow(f_j1-column,2) );
			}
		}

		// BELOW
		if (src[6] != NULL)
		{
			f_j2 = column + (16.5-row)/2.5;
			j2 = (int)ceil(f_j2);
			if(j2<0 || j2>bmax)
			{
				dir_pixel->p2 = *(src[6]+column);
				dir_pixel->d2 = blockSize - row;
			}
			else
			{
				j22 = (j2-1)<0 ? 0 : j2-1;
				dir_pixel->p2 = ( *(src[6]+j2) + *(src[6]+j22) )/2;
				dir_pixel->d2 = sqrt( pow((blockSize-row),2) + pow(f_j2-column,2) );
			}
		}
	}
	else
	{
		// Above and Right
		
		// ABOVE
		if (src[4] != NULL)
		{
			f_j1 = column - (row+1)/2.5;
			j1 = (int)floor(f_j1);
			if(j1<0 || j1>bmax)
			{
				dir_pixel->p1 = *(src[4]+bmax*frameWidth+column);
				dir_pixel->d1 = row+1;
			}
			else
			{
				j11 = (j1+1)>bmax ? bmax : j1+1;
				dir_pixel->p1 = ( *(src[4]+bmax*frameWidth+j1) + *(src[4]+bmax*frameWidth+j11) )/2;
				dir_pixel->d1 = sqrt( pow(row+1,2) + pow(f_j1-column,2) );
			}
		}

		//RIGHT
		if (src[7]!=NULL)
		{
			f_i2 = row + (16.5-column)*(2.5);
			i2 = (int)floor(f_i2);
			if(i2<0 || i2>bmax)
			{
				dir_pixel->p2 = *(src[7]+row*frameWidth);
				dir_pixel->d2 = blockSize - column;
			}
			else
			{
				i22 = (i2-1)<0 ? 0 : i2-1;
				dir_pixel->p2 = (*(src[7]+i2*frameWidth) + *(src[7]+i22*frameWidth) )/2;
				dir_pixel->d2 = sqrt( pow(blockSize-column,2) + pow(f_i2-row,2) );
			}
		}
	}
}

static void boundary_pmode6(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel)
{
	// theta = 22.5 => pmode = 6
	
	double f_i1, f_i2, f_j1, f_j2;
	int i11, i22, j11, j22, i1, j1, i2, j2, bmax = blockSize-1;

	if (row < column/2)
	{
		// Above and Right

		// ABOVE
		if (src[4] != NULL)
		{
			f_j1 = column - (row+1)/0.5;
			j1 = (int)ceil(f_j1);
			if(j1<0 || j1>bmax)
			{
				dir_pixel->p1 = *(src[4]+bmax*frameWidth+column);
				dir_pixel->d1 = row+1;
			}
			else
			{
				j11 = (j1+1)>bmax ? bmax : j1+1;
				dir_pixel->p1 = ( *(src[4]+bmax*frameWidth+j1) + *(src[4]+bmax*frameWidth+j11) )/2;
				dir_pixel->d1 = sqrt( pow(row+1,2) + pow(f_j1-column,2) );
			}
		}
		
		//RIGHT
		if (src[7] != NULL)
		{
			f_i2 = row + (16.5-column)*(0.5);
			i2 = (int)floor(f_i2);
			if(i2<0 || i2>bmax)
			{
				dir_pixel->p2 = *(src[7]+row*frameWidth);
				dir_pixel->d2 = blockSize - column;
			}
			else
			{
				i22 = (i2+1)>bmax ? bmax : i2+1;
				dir_pixel->p2 = (*(src[7]+i2*frameWidth) + *(src[7]+i22*frameWidth) )/2;
				dir_pixel->d2 = sqrt( pow(blockSize-column,2) + pow(f_i2-row,2) );
			}
		}
	}
	else if ((row >= column/2) && (row < (column/2 + blockSize/2)))
	{
		// Left and Right

		//LEFT
		if (src[5] != NULL)
		{
			f_i1 = row - (column+1)*0.5;
			i1 = (int)floor(f_i1);
			if(i1<0 || i1>bmax)
			{
				dir_pixel->p1 = *(src[5]+row*frameWidth+bmax);
				dir_pixel->d1 = column+1;
			}
			else
			{
				i11 = (i1+1)>bmax ? bmax : i1+1;
				dir_pixel->p1 = ( *(src[5]+i1*frameWidth+bmax) + *(src[5]+i11*frameWidth+bmax) )/2;
				dir_pixel->d1 = sqrt( pow(column+1,2) + pow(f_i1-row,2) );
			}
		}

		//RIGHT
		if (src[7] != NULL)
		{
			f_i2 = row + (16.5-column)*(0.5);
			i2 = (int)floor(f_i2);
			if(i2<0 || i2>bmax)
			{
				dir_pixel->p2 = *(src[7]+row*frameWidth);
				dir_pixel->d2 = blockSize - column;
			}
			else
			{
				i22 = (i2+1)>bmax ? bmax : i2+1;
				dir_pixel->p2 = (*(src[7]+i2*frameWidth) + *(src[7]+i22*frameWidth) )/2;
				dir_pixel->d2 = sqrt( pow(blockSize-column,2) + pow(f_i2-row,2) );
			}
		}
	}
	else // row >= (column/2 + blockSize/2)
	{
		// Left and Below

		//LEFT
		if (src[5] != NULL)
		{
			f_i1 = row - (column+1)*0.5;
			i1 = (int)floor(f_i1);
			if(i1<0 || i1>bmax)
			{
				dir_pixel->p1 = *(src[5]+row*frameWidth+bmax);
				dir_pixel->d1 = column+1;
			}
			else
			{
				i11 = (i1+1)>bmax ? bmax : i1+1;
				dir_pixel->p1 = ( *(src[5]+i1*frameWidth+bmax) + *(src[5]+i11*frameWidth+bmax) )/2;
				dir_pixel->d1 = sqrt( pow(column+1,2) + pow(f_i1-row,2) );
			}
		}

		// BELOW
		if (src[6] != NULL)
		{
			f_j2 = column + (16.5-row)/0.5;
			j2 = (int)ceil(f_j2);
			if(j2<0 || j2>bmax)
			{
				dir_pixel->p2 = *(src[6]+column);
				dir_pixel->d2 = blockSize - row;
			}
			else
			{
				j22 = (j2-1)<0 ? 0 : j2-1;
				dir_pixel->p2 = ( *(src[6]+j2) + *(src[6]+j22) )/2;
				dir_pixel->d2 = sqrt( pow((blockSize-row),2) + pow(f_j2-column,2) );
			}
		}
	}
}

static void boundary_pmode7(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel)
{
	// theta = -67.5 => pmode = 7
	
	double f_i1, f_i2, f_j1, f_j2;
	int i11, i22, j11, j22, i1, j1, i2, j2, bmax = blockSize-1;

	if (row < (blockSize-2*column))
	{
		// Above and Left

		// ABOVE
		if (src[4] != NULL)
		{
			f_j2 = column + (row+1)/2.5;
			j2 = (int)floor(f_j2);
			if(j2<0 || j2>bmax)
			{
				dir_pixel->p2 = *(src[4]+bmax*frameWidth+column);
				dir_pixel->d2 = row+1;
			}
			else
			{
				j22 = (j2+1)>bmax ? bmax : j2+1;
				dir_pixel->p2 = ( *(src[4]+bmax*frameWidth+j2) + *(src[4]+bmax*frameWidth+j22) )/2;
				dir_pixel->d2 = sqrt( pow(row+1,2) + pow(f_j2-column,2) );
			}
		}

		//LEFT
		if (src[5] != NULL)
		{
			f_i1 = row + (column+1)*2.5;
			i1 = (int)ceil(f_i1);
			if(i1<0 || i1>bmax)
			{
				dir_pixel->p1 = *(src[5]+row*frameWidth+bmax);
				dir_pixel->d1 = column+1;
			}
			else
			{
				i11  = (i1-1)<0 ? 0 : i1-1;
				dir_pixel->p1 = ( *(src[5]+i1*frameWidth+bmax) + *(src[5]+i11*frameWidth+bmax) )/2;
				dir_pixel->d1 = sqrt( pow(column+1,2) + pow(f_i1-row,2) );
			}
		}
	}
	else if ( (row >= (blockSize-2*column)) && (row < (2*blockSize-2*column)) )
	{
		// Above and Below

		// ABOVE
		if (src[4] != NULL)
		{
			f_j2 = column + (row+1)/2.5;
			j2 = (int)floor(f_j2);
			if(j2<0 || j2>bmax)
			{
				dir_pixel->p2 = *(src[4]+bmax*frameWidth+column);
				dir_pixel->d2 = row+1;
			}
			else
			{
				j22 = (j2+1)>bmax ? bmax : j2+1;
				dir_pixel->p2 = ( *(src[4]+bmax*frameWidth+j2) + *(src[4]+bmax*frameWidth+j22) )/2;
				dir_pixel->d2 = sqrt( pow(row+1,2) + pow(f_j2-column,2) );
			}
		}

		// BELOW
		if (src[6] != NULL)
		{
			f_j1 = column - (16.5-row)/2.5;
			j1 = (int)ceil(f_j1);
			if(j1<0 || j1>bmax)
			{
				dir_pixel->p1 = *(src[6]+column);
				dir_pixel->d1 = blockSize - row;
			}
			else
			{
				j11 = (j1-1)<0 ? 0 : j1-1;
				dir_pixel->p1 = ( *(src[6]+j1) + *(src[6]+j11) )/2;
				dir_pixel->d1 = sqrt( pow((blockSize-row),2) + pow(f_j1-column,2) );
			}
		}
	}
	else //row >= (2*blockSize-2*column)
	{
		// Right and Below

		//RIGHT
		if (src[7] != NULL)
		{
			f_i2 = row - (16.5-column)*(2.5);
			i2 = (int)floor(f_i2);
			if(i2<0 || i2>bmax)
			{
				dir_pixel->p2 = *(src[7]+row*frameWidth);
				dir_pixel->d2 = blockSize - column;
			}
			else
			{
				i22 = (i2+1)>bmax ? bmax : i2+1;
				dir_pixel->p2 = (*(src[7]+i2*frameWidth) + *(src[7]+i22*frameWidth) )/2;
				dir_pixel->d2 = sqrt( pow(blockSize-column,2) + pow(f_i2-row,2) );
			}
		}

		// BELOW
		if (src[6] != NULL)
		{
			f_j1 = column - (16.5-row)/2.5;
			j1 = (int)ceil(f_j1);
			if(j1<0 || j1>bmax)
			{
				dir_pixel->p1 = *(src[6]+column);
				dir_pixel->d1 = blockSize - row;
			}
			else
			{
				j11 = (j1-1)<0 ? 0 : j1-1;
				dir_pixel->p1 = ( *(src[6]+j1) + *(src[6]+j11) )/2;
				dir_pixel->d1 = sqrt( pow((blockSize-row),2) + pow(f_j1-column,2) );
			}
		}
	}
}

static void boundary_pmode8(imgpel *src[], int blockSize, int frameWidth, int row, int column, struct direction *dir_pixel)
{
	double theta = -(-22.5);//pmode = 8
	// theta = -22.5 => pmode = 8
	
	double f_i1, f_i2, f_j1, f_j2;
	int i11, i22, j11, j22, i1, j1, i2, j2, bmax = blockSize-1;

	if (row < (blockSize/2 - column/2))
	{
		// Above and Left

		// ABOVE
		if (src[4] != NULL)
		{
			f_j2 = column + (row+1)/0.5;
			j2 = (int)ceil(f_j2);
			if(j2<0 || j2>bmax)
			{
				dir_pixel->p2 = *(src[4]+bmax*frameWidth+column);
				dir_pixel->d2 = row+1;
			}
			else
			{
				j22 = (j2-1)<0 ? 0 : j2-1;
				dir_pixel->p2 = ( *(src[4]+bmax*frameWidth+j2) + *(src[4]+bmax*frameWidth+j22) )/2;
				dir_pixel->d2 = sqrt( pow(row+1,2) + pow(f_j2-column,2) );
			}
		}

		//LEFT
		if (src[5] != NULL)
		{
			f_i1 = row + (column+1)*0.5;
			i1 = (int)floor(f_i1);
			if(i1<0 || i1>bmax)
			{
				dir_pixel->p1 = *(src[5]+row*frameWidth+bmax);
				dir_pixel->d1 = column+1;
			}
			else
			{
				i11 = (i1+1)>bmax ? bmax : i1+1;
				dir_pixel->p1 = ( *(src[5]+i1*frameWidth+bmax) + *(src[5]+i11*frameWidth+bmax) )/2;
				dir_pixel->d1 = sqrt( pow(column+1,2) + pow(f_i1-row,2) );
			}
		}
	}
	else if ( (row >= (blockSize/2 - column/2)) && (row < (blockSize - column/2)) )
	{
		// Left and Right

		//LEFT
		if (src[5] != NULL)
		{
			f_i1 = row + (column+1)*0.5;
			i1 = (int)floor(f_i1);
			if(i1<0 || i1>bmax)
			{
				dir_pixel->p1 = *(src[5]+row*frameWidth+bmax);
				dir_pixel->d1 = column+1;
			}
			else
			{
				i11 = (i1+1)>bmax ? bmax : i1+1;
				dir_pixel->p1 = ( *(src[5]+i1*frameWidth+bmax) + *(src[5]+i11*frameWidth+bmax) )/2;
				dir_pixel->d1 = sqrt( pow(column+1,2) + pow(f_i1-row,2) );
			}
		}

		//RIGHT
		if (src[7] != NULL)
		{
			f_i2 = row - (16.5-column)*(0.5);
			i2 = (int)floor(f_i2);
			if(i2<0 || i2>bmax)
			{
				dir_pixel->p2 = *(src[7]+row*frameWidth);
				dir_pixel->d2 = blockSize - column;
			}
			else
			{
				i22 = (i2+1)>bmax ? bmax : i2+1;
				dir_pixel->p2 = (*(src[7]+i2*frameWidth) + *(src[7]+i22*frameWidth) )/2;
				dir_pixel->d2 = sqrt( pow(blockSize-column,2) + pow(f_i2-row,2) );
			}
		}
	}
	else //row >= (blockSize - column/2)
	{
		// Right and Below

		//RIGHT
		if (src[7] != NULL)
		{
			f_i2 = row - (16.5-column)*(0.5);
			i2 = (int)floor(f_i2);
			if(i2<0 || i2>bmax)
			{
				dir_pixel->p2 = *(src[7]+row*frameWidth);
				dir_pixel->d2 = blockSize - column;
			}
			else
			{
				i22 = (i2+1)>bmax ? bmax : i2+1;
				dir_pixel->p2 = (*(src[7]+i2*frameWidth) + *(src[7]+i22*frameWidth) )/2;
				dir_pixel->d2 = sqrt( pow(blockSize-column,2) + pow(f_i2-row,2) );
			}
		}

		// BELOW
		if (src[6] != NULL)
		{
			f_j1 = column - (16.5-row)/0.5;
			j1 = (int)floor(f_j1);
			if(j1<0 || j1>bmax)
			{
				dir_pixel->p1 = *(src[6]+column);
				dir_pixel->d1 = blockSize - row;
			}
			else
			{
				j11 = (j1+1)>bmax ? bmax : j1+1;
				dir_pixel->p1 = ( *(src[6]+j1) + *(src[6]+j11) )/2;
				dir_pixel->d1 = sqrt( pow((blockSize-row),2) + pow(f_j1-column,2) );
			}
		}
	}
}

static int findDominantPmodeLumaUpdated(imgpel *src[], int blockSize, int block_row, int block_column, int frameWidth, int *fMDI, int *fBI, int edgeDir[9], int edgeStrength[9], double *dirEntropy, int *numDED)
{
	int edgeMagnitude[9], numOccurence[9];//counter for 9 pmodes of Intra4x4
	int k, row, column, bmax = blockSize-1, ipmode, dominant_pmode = 2;
	int pixvalP[4], pixvalQ[4], index, edgemagsum=0, maxedgemag, totalEdgeStrength = 0;
	double probEdge[9];
	*fMDI = 0; *fBI = 0; *dirEntropy = 0.0, *numDED = 0;
	
	/* Prediction Modes:
	0: Vertical
	1: Horizontal
	2: Mean
	3: Diagonal Down Left
	4: Diagonal Down Right
	5: Vertical Right
	6: Horizontal Down
	7: Vertical Left
	8: Horizontal Up
	*/
	for(k=0;k<9;k++)
	{
		edgeMagnitude[k] = 0;
		numOccurence[k]  = 0;
		edgeStrength[k]  = 0;
		edgeDir[k]       = 0;
		probEdge[k]      = 0.0;
	}

	//Find the mode type of nbr MB, if 16x16, then all 4 4x4 blocks pmode same as that

	/* Above */
	if ( src[4] != NULL )   
	{
		for ( column = 0; column < blockSize; column+=(blockSize>>2) ) 
		{
			//Each 4x4 block
			ipmode = img->ipredmode[block_column + column/(blockSize>>2)][block_row-1]; //this is the edge direction
			numOccurence[ipmode]++;

			switch(ipmode)
			{
			case 0:
				pixvalP[0] = *(src[4]+(bmax-3)*frameWidth+(column+0)); pixvalQ[0] = *(src[4]+(bmax-3)*frameWidth+(column+3));
				pixvalP[1] = *(src[4]+(bmax-2)*frameWidth+(column+0)); pixvalQ[1] = *(src[4]+(bmax-2)*frameWidth+(column+3));
				pixvalP[2] = *(src[4]+(bmax-1)*frameWidth+(column+0)); pixvalQ[2] = *(src[4]+(bmax-1)*frameWidth+(column+3));
				pixvalP[3] = *(src[4]+(bmax-0)*frameWidth+(column+0)); pixvalQ[3] = *(src[4]+(bmax-0)*frameWidth+(column+3));
				break;
			case 1:
				pixvalP[0] = *(src[4]+(bmax-3)*frameWidth+(column+0)); pixvalQ[0] = *(src[4]+(bmax-0)*frameWidth+(column+0));
				pixvalP[1] = *(src[4]+(bmax-3)*frameWidth+(column+1)); pixvalQ[1] = *(src[4]+(bmax-0)*frameWidth+(column+1));
				pixvalP[2] = *(src[4]+(bmax-3)*frameWidth+(column+2)); pixvalQ[2] = *(src[4]+(bmax-0)*frameWidth+(column+2));
				pixvalP[3] = *(src[4]+(bmax-3)*frameWidth+(column+3)); pixvalQ[3] = *(src[4]+(bmax-0)*frameWidth+(column+3));
				break;
			case 3:
			case 7:
			case 8:
				pixvalP[0] = *(src[4]+(bmax-3)*frameWidth+(column+0)); pixvalQ[0] = *(src[4]+(bmax-1)*frameWidth+(column+2));
				pixvalP[1] = *(src[4]+(bmax-3)*frameWidth+(column+1)); pixvalQ[1] = *(src[4]+(bmax-1)*frameWidth+(column+3));
				pixvalP[2] = *(src[4]+(bmax-2)*frameWidth+(column+0)); pixvalQ[2] = *(src[4]+(bmax-0)*frameWidth+(column+2));
				pixvalP[3] = *(src[4]+(bmax-2)*frameWidth+(column+1)); pixvalQ[3] = *(src[4]+(bmax-0)*frameWidth+(column+3));
				break;
			case 4:
			case 5:
			case 6:
				pixvalP[0] = *(src[4]+(bmax-3)*frameWidth+(column+2)); pixvalQ[0] = *(src[4]+(bmax-1)*frameWidth+(column+0));
				pixvalP[1] = *(src[4]+(bmax-3)*frameWidth+(column+3)); pixvalQ[1] = *(src[4]+(bmax-1)*frameWidth+(column+1));
				pixvalP[2] = *(src[4]+(bmax-2)*frameWidth+(column+2)); pixvalQ[2] = *(src[4]+(bmax-0)*frameWidth+(column+0));
				pixvalP[3] = *(src[4]+(bmax-2)*frameWidth+(column+3)); pixvalQ[3] = *(src[4]+(bmax-0)*frameWidth+(column+1));
				break;
			}
			
			if(ipmode!=2)
			{
				edgemagsum = 0;
				for (index=0;index<4;index++)
				{
					edgemagsum += abs(pixvalP[index] - pixvalQ[index]);
				}
				edgeMagnitude[ipmode] += edgemagsum;
			}
		}
	}

	/* Left */
	if ( src[5] != NULL )   
	{
		for ( row = 0; row < blockSize; row+=(blockSize>>2) ) 
		{
			//Each 4x4 block
			ipmode = img->ipredmode[block_column-1][block_row + row/(blockSize>>2)]; //this is the edge direction
			numOccurence[ipmode]++;

			switch(ipmode)
			{
			case 0:
				pixvalP[0] = *(src[5]+(row+0)*frameWidth+(bmax-3)); pixvalQ[0] = *(src[5]+(row+0)*frameWidth+(bmax-0));
				pixvalP[1] = *(src[5]+(row+1)*frameWidth+(bmax-3)); pixvalQ[1] = *(src[5]+(row+1)*frameWidth+(bmax-0));
				pixvalP[2] = *(src[5]+(row+2)*frameWidth+(bmax-3)); pixvalQ[2] = *(src[5]+(row+2)*frameWidth+(bmax-0));
				pixvalP[3] = *(src[5]+(row+3)*frameWidth+(bmax-3)); pixvalQ[3] = *(src[5]+(row+3)*frameWidth+(bmax-0));
				break;
			case 1:
				pixvalP[0] = *(src[5]+(row+0)*frameWidth+(bmax-3)); pixvalQ[0] = *(src[5]+(row+3)*frameWidth+(bmax-3));
				pixvalP[1] = *(src[5]+(row+0)*frameWidth+(bmax-2)); pixvalQ[1] = *(src[5]+(row+3)*frameWidth+(bmax-2));
				pixvalP[2] = *(src[5]+(row+0)*frameWidth+(bmax-1)); pixvalQ[2] = *(src[5]+(row+3)*frameWidth+(bmax-1));
				pixvalP[3] = *(src[5]+(row+0)*frameWidth+(bmax-0)); pixvalQ[3] = *(src[5]+(row+3)*frameWidth+(bmax-0));
				break;
			case 3:
			case 7:
			case 8:
				pixvalP[0] = *(src[5]+(row+0)*frameWidth+(bmax-3)); pixvalQ[0] = *(src[5]+(row+2)*frameWidth+(bmax-1));
				pixvalP[1] = *(src[5]+(row+0)*frameWidth+(bmax-2)); pixvalQ[1] = *(src[5]+(row+2)*frameWidth+(bmax-0));
				pixvalP[2] = *(src[5]+(row+1)*frameWidth+(bmax-3)); pixvalQ[2] = *(src[5]+(row+3)*frameWidth+(bmax-1));
				pixvalP[3] = *(src[5]+(row+1)*frameWidth+(bmax-2)); pixvalQ[3] = *(src[5]+(row+3)*frameWidth+(bmax-0));
				break;
			case 4:
			case 5:
			case 6:
				pixvalP[0] = *(src[5]+(row+0)*frameWidth+(bmax-1)); pixvalQ[0] = *(src[5]+(row+2)*frameWidth+(bmax-3));
				pixvalP[1] = *(src[5]+(row+0)*frameWidth+(bmax-0)); pixvalQ[1] = *(src[5]+(row+2)*frameWidth+(bmax-2));
				pixvalP[2] = *(src[5]+(row+1)*frameWidth+(bmax-1)); pixvalQ[2] = *(src[5]+(row+3)*frameWidth+(bmax-3));
				pixvalP[3] = *(src[5]+(row+1)*frameWidth+(bmax-0)); pixvalQ[3] = *(src[5]+(row+3)*frameWidth+(bmax-2));
				break;
			}

			if(ipmode!=2)
			{
				edgemagsum = 0;
				for (index=0;index<4;index++)
				{
					edgemagsum += abs(pixvalP[index] - pixvalQ[index]);
				}
				edgeMagnitude[ipmode] += edgemagsum;
			}

		}
	}

	/* Below */
	if ( src[6] != NULL )   
	{
		for ( column = 0; column < blockSize; column+=(blockSize>>2) ) 
		{
			//Each 4x4 block
			ipmode = img->ipredmode[block_column + column/(blockSize>>2)][block_row + (blockSize>>2)]; //this is the edge direction			
			numOccurence[ipmode]++;

			switch(ipmode)
			{
			case 0:
				pixvalP[0] = *(src[6]+(0)*frameWidth+(column+0)); pixvalQ[0] = *(src[6]+(0)*frameWidth+(column+3));
				pixvalP[1] = *(src[6]+(1)*frameWidth+(column+0)); pixvalQ[1] = *(src[6]+(1)*frameWidth+(column+3));
				pixvalP[2] = *(src[6]+(2)*frameWidth+(column+0)); pixvalQ[2] = *(src[6]+(2)*frameWidth+(column+3));
				pixvalP[3] = *(src[6]+(3)*frameWidth+(column+0)); pixvalQ[3] = *(src[6]+(3)*frameWidth+(column+3));
				break;
			case 1:
				pixvalP[0] = *(src[6]+(0)*frameWidth+(column+0)); pixvalQ[0] = *(src[6]+(3)*frameWidth+(column+0));
				pixvalP[1] = *(src[6]+(0)*frameWidth+(column+1)); pixvalQ[1] = *(src[6]+(3)*frameWidth+(column+1));
				pixvalP[2] = *(src[6]+(0)*frameWidth+(column+2)); pixvalQ[2] = *(src[6]+(3)*frameWidth+(column+2));
				pixvalP[3] = *(src[6]+(0)*frameWidth+(column+3)); pixvalQ[3] = *(src[6]+(3)*frameWidth+(column+3));
				break;
			case 3:
			case 7:
			case 8:
				pixvalP[0] = *(src[6]+(0)*frameWidth+(column+0)); pixvalQ[0] = *(src[6]+(2)*frameWidth+(column+2));
				pixvalP[1] = *(src[6]+(0)*frameWidth+(column+1)); pixvalQ[1] = *(src[6]+(2)*frameWidth+(column+3));
				pixvalP[2] = *(src[6]+(1)*frameWidth+(column+0)); pixvalQ[2] = *(src[6]+(3)*frameWidth+(column+2));
				pixvalP[3] = *(src[6]+(1)*frameWidth+(column+1)); pixvalQ[3] = *(src[6]+(3)*frameWidth+(column+3));
				break;
			case 4:
			case 5:
			case 6:
				pixvalP[0] = *(src[6]+(0)*frameWidth+(column+2)); pixvalQ[0] = *(src[6]+(2)*frameWidth+(column+0));
				pixvalP[1] = *(src[6]+(0)*frameWidth+(column+3)); pixvalQ[1] = *(src[6]+(2)*frameWidth+(column+1));
				pixvalP[2] = *(src[6]+(1)*frameWidth+(column+2)); pixvalQ[2] = *(src[6]+(3)*frameWidth+(column+0));
				pixvalP[3] = *(src[6]+(1)*frameWidth+(column+3)); pixvalQ[3] = *(src[6]+(3)*frameWidth+(column+1));
				break;
			}

			if(ipmode!=2)
			{
				edgemagsum = 0;
				for (index=0;index<4;index++)
				{
					edgemagsum += abs(pixvalP[index] - pixvalQ[index]);
				}
				edgeMagnitude[ipmode] += edgemagsum;
			}

		}
	}

	/* Right */
	if ( src[7] != NULL )   
	{
		for ( row = 0; row < blockSize; row+=(blockSize>>2) ) 
		{
			//Each 4x4 block
			ipmode = img->ipredmode[block_column + (blockSize>>2)][block_row + row/(blockSize>>2)]; //this is the edge direction			
			numOccurence[ipmode]++;

			switch(ipmode)
			{
			case 0:
				pixvalP[0] = *(src[7]+(row+0)*frameWidth+(0)); pixvalQ[0] = *(src[7]+(row+0)*frameWidth+(3));
				pixvalP[1] = *(src[7]+(row+1)*frameWidth+(0)); pixvalQ[1] = *(src[7]+(row+1)*frameWidth+(3));
				pixvalP[2] = *(src[7]+(row+2)*frameWidth+(0)); pixvalQ[2] = *(src[7]+(row+2)*frameWidth+(3));
				pixvalP[3] = *(src[7]+(row+3)*frameWidth+(0)); pixvalQ[3] = *(src[7]+(row+3)*frameWidth+(3));
				break;
			case 1:
				pixvalP[0] = *(src[7]+(row+0)*frameWidth+(0)); pixvalQ[0] = *(src[7]+(row+3)*frameWidth+(0));
				pixvalP[1] = *(src[7]+(row+0)*frameWidth+(1)); pixvalQ[1] = *(src[7]+(row+3)*frameWidth+(1));
				pixvalP[2] = *(src[7]+(row+0)*frameWidth+(2)); pixvalQ[2] = *(src[7]+(row+3)*frameWidth+(2));
				pixvalP[3] = *(src[7]+(row+0)*frameWidth+(3)); pixvalQ[3] = *(src[7]+(row+3)*frameWidth+(3));
				break;
			case 3:
			case 7:
			case 8:
				pixvalP[0] = *(src[7]+(row+0)*frameWidth+(0)); pixvalQ[0] = *(src[7]+(row+2)*frameWidth+(2));
				pixvalP[1] = *(src[7]+(row+0)*frameWidth+(1)); pixvalQ[1] = *(src[7]+(row+2)*frameWidth+(3));
				pixvalP[2] = *(src[7]+(row+1)*frameWidth+(0)); pixvalQ[2] = *(src[7]+(row+3)*frameWidth+(2));
				pixvalP[3] = *(src[7]+(row+1)*frameWidth+(1)); pixvalQ[3] = *(src[7]+(row+3)*frameWidth+(3));
				break;
			case 4:
			case 5:
			case 6:
				pixvalP[0] = *(src[7]+(row+0)*frameWidth+(2)); pixvalQ[0] = *(src[7]+(row+2)*frameWidth+(0));
				pixvalP[1] = *(src[7]+(row+0)*frameWidth+(3)); pixvalQ[1] = *(src[7]+(row+2)*frameWidth+(1));
				pixvalP[2] = *(src[7]+(row+1)*frameWidth+(2)); pixvalQ[2] = *(src[7]+(row+3)*frameWidth+(0));
				pixvalP[3] = *(src[7]+(row+1)*frameWidth+(3)); pixvalQ[3] = *(src[7]+(row+3)*frameWidth+(1));
				break;
			}

			if(ipmode!=2)
			{
				edgemagsum = 0;
				for (index=0;index<4;index++)
				{
					edgemagsum += abs(pixvalP[index] - pixvalQ[index]);
				}
				edgeMagnitude[ipmode] += edgemagsum;
			}

		}
	}

	for(k=0;k<9;k++)
	{
		//edgeStrength[k] = numOccurence[k]*edgeMagnitude[k];
		edgeStrength[k] = edgeMagnitude[k];
		totalEdgeStrength += edgeStrength[k];
	}

	//Now, find the dominant_pmode with maximum edge magnitude
	maxedgemag = edgeStrength[0];
	dominant_pmode = 0;

	for(k=1;k<9;k++)
	{
		if(maxedgemag < edgeStrength[k])
		{
			maxedgemag = edgeStrength[k];
			dominant_pmode = k;
		}
	}
	// Mark this dominant edge direction
	edgeDir[dominant_pmode] = 1;
	(*numDED)++;

	// Find if there are any more strong edges (70% of max edgeStrength)
	for(k=0;k<9;k++)
	{
		if (k == dominant_pmode)
			continue;

		if (edgeStrength[k] > 0.45*maxedgemag)
		{
			edgeDir[k] = 1;
			(*numDED)++;
		}
	}

	// Compute directional entropy
	// First find probabilities
	for (k=0;k<9;k++)
	{
		probEdge[k] = (float)edgeStrength[k]/totalEdgeStrength;
	}

	for (k=0;k<9;k++)
	{
		if (probEdge[k] != 0)
		{
			*dirEntropy += (-probEdge[k])*(log(probEdge[k])/log(2));
		}
	}

	if ((*numDED > 2) || (*dirEntropy >= DIRENTROPYTHRESH))
		*fBI = 1; // Apply BI
	else if (*numDED == 2)
		*fMDI = 1; // Apply MDI in 2 directions
	else
		*fMDI = 0; // Apply Single DI

	return dominant_pmode;
}

static void showEntropyArea(double dirEntropy, int blockSize, imgpel *block, int frameWidth)
{
	int pixVal, row, column, k;

	if (dirEntropy < 0.5)
	{
		pixVal = 0;
	}
	else if (dirEntropy >= 0.5 && dirEntropy < 1.0)
	{
		pixVal = 50;
	}
	else if (dirEntropy >= 1.0 && dirEntropy < 1.5)
	{
		pixVal = 100;
	}
	else if (dirEntropy >= 15 && dirEntropy < 2.0)
	{
		pixVal = 150;
	}
	else if (dirEntropy >= 2.0 && dirEntropy < 2.5)
	{
		pixVal = 200;
	}
	else
		pixVal = 255;

	k = 0; 
	for ( row = 0; row < blockSize; row++ ) 
	{
		for ( column = 0; column < blockSize; column++ )
		{			
			block[k+column] = pixVal; 
		}
		k += frameWidth;
	}
}
