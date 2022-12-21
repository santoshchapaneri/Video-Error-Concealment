
/*!
 *************************************************************************************
 * \file 
 *      erc_do_p.c
 *
 * \brief
 *      Inter (P) frame error concealment algorithms for decoder
 *
 *  \author
 *      - Viktor Varsa                     <viktor.varsa@nokia.com>
 *      - Ye-Kui Wang                      <wyk@ieee.org>
 *      - Jill Boyce                       <jill.boyce@thomson.net>
 *      - Saurav K Bandyopadhyay           <saurav@ieee.org>
 *      - Zhenyu Wu                        <Zhenyu.Wu@thomson.net
 *      - Purvin Pandit                    <Purvin.Pandit@thomson.net>
 *
 *************************************************************************************
 */

#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include "mbuffer.h"
#include "global.h"
#include "memalloc.h"
#include "erc_do.h"
#include "image.h"

extern int erc_mvperMB;
struct img_par *erc_img;

// picture error concealment
// concealment_head points to first node in list, concealment_end points to 
// last node in list. Initialise both to NULL, meaning no nodes in list yet
struct concealment_node *concealment_head = NULL;
struct concealment_node *concealment_end = NULL;

// static function declarations
static int concealByCopy(frame *recfr, int currMBNum,
  objectBuffer_t *object_list, int32 picSizeX);
static int concealByTrial(frame *recfr, imgpel *predMB, 
                          int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                          int32 picSizeX, int32 picSizeY, int *yCondition);
static int edgeDistortion (int predBlocks[], int currYBlockNum, imgpel *predMB, 
                           imgpel *recY, int32 picSizeX, int32 regionSize);
static void copyBetweenFrames (frame *recfr, 
   int currYBlockNum, int32 picSizeX, int32 regionSize);
static void buildPredRegionYUV(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB);

// picture error concealment
static void buildPredblockRegionYUV(struct img_par *img, int32 *mv, 
                                    int x, int y, imgpel *predMB, int list);
static void CopyImgData(imgpel **inputY, imgpel ***inputUV, imgpel **outputY, 
                        imgpel ***outputUV, int img_width, int img_height);


static void copyPredMB (int currYBlockNum, imgpel *predMB, frame *recfr, 
                        int32 picSizeX, int32 regionSize);

extern const unsigned char subblk_offset_y[3][8][4];
extern const unsigned char subblk_offset_x[3][8][4];
static int uv_div[2][4] = {{0, 1, 1, 0}, {0, 1, 0, 0}}; //[x/y][yuv_format]


//Santosh
#define OBMA		1
#define FOURMODES	0
#define ABS			0
#define OBMC		0
#define OBMC_THRESHOLD 0
#define OBMC_TR 10
#define NIL  352

#define ECMODE1 1
#define ECMODE2 2
#define ECMODE3 3
#define ECMODE4 4
#define ECMODE5 5
#define ECMODE6 6
#define ECMODE7 7
#define ECMODE8 8
#define SEC     9

imgpel *boundary;
extern StorablePicture *no_reference_picture;

//Matrices for OBMC computation - 16x16 case

static int H_E[16][16] = 
{
	{4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4},
	{4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4},
	{5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5},
	{5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5},
	{5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5},
	{5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5},
	{5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5},
	{5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5},
	{5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5},
	{5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5},
	{5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5},
	{5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5},
	{5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5},
	{5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5},
	{4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4},
	{4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4},
};

static int H_LR[16][16] = 
{
	{2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
	{2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
	{1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1},
	{1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1},
	{1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1},
	{2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
	{2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
};

static int H_TD[16][16] = 
{
	{2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2},
	{2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2},
	{2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2},
	{2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2},
	{2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2},
	{2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2},
	{2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2},
	{2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2},
	{2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2},
	{2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2},
	{2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2},
	{2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2},
	{2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2},
	{2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2},
	{2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2},
	{2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2},
};

//Matrices for OBMC computation - 8x8 case

static int H_E_8x8[8][8] = 
{
	{4, 5, 5, 5, 5, 5, 5, 4},
	{5, 5, 5, 5, 5, 5, 5, 5},
	{5, 5, 6, 6, 6, 6, 5, 5},
	{5, 5, 6, 6, 6, 6, 5, 5},
	{5, 5, 6, 6, 6, 6, 5, 5},
	{5, 5, 6, 6, 6, 6, 5, 5},
	{5, 5, 5, 5, 5, 5, 5, 5},
	{4, 5, 5, 5, 5, 5, 5, 4},
};

static int H_LR_8x8[8][8] = 
{
	{2, 2, 2, 2, 2, 2, 2, 2},
	{1, 1, 2, 2, 2, 2, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1},
	{1, 1, 2, 2, 2, 2, 1, 1},
	{2, 2, 2, 2, 2, 2, 2, 2},
};

static int H_TD_8x8[8][8] = 
{
	{2, 1, 1, 1, 1, 1, 1, 2},
	{2, 2, 1, 1, 1, 1, 2, 2},
	{2, 2, 1, 1, 1, 1, 2, 2},
	{2, 2, 1, 1, 1, 1, 2, 2},
	{2, 2, 1, 1, 1, 1, 2, 2},
	{2, 2, 1, 1, 1, 1, 2, 2},
	{2, 2, 1, 1, 1, 1, 2, 2},
	{2, 1, 1, 1, 1, 1, 1, 2},
};


static void buildOuterPredRegionYUV(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary);
void get_boundary(int ref_frame, StorablePicture **list, int x_pos, int y_pos, struct img_par *img, int index, int above[4], int left[4], int below[4], int right[4]);
static int edgeDistortionOBMA (int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary);

static int concealABS(frame *recfr, imgpel *predMB, int currMBNum, objectBuffer_t *object_list, 
					  int predBlocks[], int32 picSizeX, int32 picSizeY, int *yCondition);
int find_mb_ecmode(int predBlocks[],int numMBPerLine,int currMBNum);

int fourmodes_find_mb_ecmode(int predBlocks[],int numMBPerLine,int currMBNum);

static int concealABS_ECMODE1(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition);
static int concealABS_ECMODE2(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition);
static int concealABS_ECMODE3(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition);
static int concealABS_ECMODE4(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition);
static int concealABS_ECMODE5(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition);
static int concealABS_ECMODE6(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition);
static int concealABS_ECMODE7(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition);
static int concealABS_ECMODE8(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition);

static void buildOuterPredRegionYUV_ECMODE2(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos);
static int edgeDistortion_ECMODE2 (int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos);

static void buildOuterPredRegionYUV_ECMODE3(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos);
static int edgeDistortion_ECMODE3(int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos);

static void buildOuterPredRegionYUV_ECMODE4(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos);
static int edgeDistortion_ECMODE4(int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos);

static void buildOuterPredRegionYUV_ECMODE5(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos);
static int edgeDistortion_ECMODE5(int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos);

static void buildOuterPredRegionYUV_ECMODE6(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos);
static int edgeDistortion_ECMODE6(int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos);

static void buildOuterPredRegionYUV_ECMODE7(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos);
static int edgeDistortion_ECMODE7(int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos);

static void buildOuterPredRegionYUV_ECMODE8(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos);
static int edgeDistortion_ECMODE8(int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos);

static void OBMC_MB(imgpel *predMB, imgpel *predMB_OBMC, int predBlocks[], objectBuffer_t *object_list, int currMBNum, int numMBPerLine, int picSizeX);

int findaveragemv(int allmv[8][2],int comp);
int findmedianmv(int allmv[8][2],int comp);

/*!
 ************************************************************************
 * \brief
 *      The main function for Inter (P) frame concealment.
 * \return
 *      0, if the concealment was not successful and simple concealment should be used
 *      1, otherwise (even if none of the blocks were concealed)
 * \param recfr
 *      Reconstructed frame buffer
 * \param object_list
 *      Motion info for all MBs in the frame
 * \param picSizeX
 *      Width of the frame in pixels
 * \param picSizeY
 *      Height of the frame in pixels
 * \param errorVar   
 *      Variables for error concealment
 * \param chroma_format_idc   
 *      Chroma format IDC
 ************************************************************************
 */
int ercConcealInterFrame(frame *recfr, objectBuffer_t *object_list, 
                         int32 picSizeX, int32 picSizeY, ercVariables_t *errorVar, int chroma_format_idc ) 
{
  int lastColumn = 0, lastRow = 0, predBlocks[8];
  int lastCorruptedRow = -1, firstCorruptedRow = -1, currRow = 0, 
    row, column, columnInd, areaHeight = 0, i = 0;
  imgpel *predMB;

  
  /* if concealment is on */
  if ( errorVar && errorVar->concealment ) 
  {
    /* if there are segments to be concealed */
    if ( errorVar->nOfCorruptedSegments ) 
    {
      if (chroma_format_idc != YUV400)
        predMB = (imgpel *) malloc ( (256 + (img->mb_cr_size_x*img->mb_cr_size_y)*2) * sizeof (imgpel));
      else
        predMB = (imgpel *) malloc(256 * sizeof (imgpel));

	  //Above, Left, Below, Right: each has 16 pixels on outer boundary
	  boundary = (imgpel*)malloc((16*4)*sizeof(imgpel));

	  //erc_mvperMB=1;//Remove this
      
      if ( predMB == NULL ) no_mem_exit("ercConcealInterFrame: predMB");
      
      lastRow = (int) (picSizeY>>4);
      lastColumn = (int) (picSizeX>>4);
      
      for ( columnInd = 0; columnInd < lastColumn; columnInd ++) 
      {        
        column = ((columnInd%2) ? (lastColumn - columnInd/2 -1) : (columnInd/2));
        
        for ( row = 0; row < lastRow; row++) 
        {
			//Santosh
			//erc_mvperMB = 20;

          if ( errorVar->yCondition[MBxy2YBlock(column, row, 0, picSizeX)] <= ERC_BLOCK_CORRUPTED ) 
          {                           // ERC_BLOCK_CORRUPTED (1) or ERC_BLOCK_EMPTY (0)
            firstCorruptedRow = row;
            /* find the last row which has corrupted blocks (in same continuous area) */
            for ( lastCorruptedRow = row+1; lastCorruptedRow < lastRow; lastCorruptedRow++) 
            {
              /* check blocks in the current column */
              if (errorVar->yCondition[MBxy2YBlock(column, lastCorruptedRow, 0, picSizeX)] > ERC_BLOCK_CORRUPTED) 
              {
                /* current one is already OK, so the last was the previous one */
                lastCorruptedRow --;
                break;
              }
            }
            if ( lastCorruptedRow >= lastRow ) 
            {
              /* correct only from above */
              lastCorruptedRow = lastRow-1;
              for ( currRow = firstCorruptedRow; currRow < lastRow; currRow++ ) 
              {
                
                ercCollect8PredBlocks (predBlocks, (currRow<<1), (column<<1), 
                  errorVar->yCondition, (lastRow<<1), (lastColumn<<1), 2, 0);      
                
                if(erc_mvperMB >= MVPERMB_THR)
				{
					if(ABS)
						concealABS(recfr, predMB, 
						currRow*lastColumn+column, object_list, predBlocks, 
						picSizeX, picSizeY,
						errorVar->yCondition);		
					else
						concealByTrial(recfr, predMB, 
						currRow*lastColumn+column, object_list, predBlocks, 
						picSizeX, picSizeY,
						errorVar->yCondition);
				}
                  
                else 
                  concealByCopy(recfr, currRow*lastColumn+column, 
                    object_list, picSizeX);
                
                ercMarkCurrMBConcealed (currRow*lastColumn+column, -1, picSizeX, errorVar);
              }
              row = lastRow;
            } 
            else if ( firstCorruptedRow == 0 ) 
            {
              /* correct only from below */
              for ( currRow = lastCorruptedRow; currRow >= 0; currRow-- ) 
              {
                
                ercCollect8PredBlocks (predBlocks, (currRow<<1), (column<<1), 
                  errorVar->yCondition, (lastRow<<1), (lastColumn<<1), 2, 0);      
                
                if(erc_mvperMB >= MVPERMB_THR)
                {
					if(ABS)
						concealABS(recfr, predMB, 
						currRow*lastColumn+column, object_list, predBlocks, 
						picSizeX, picSizeY,
						errorVar->yCondition);		
					else
						concealByTrial(recfr, predMB, 
						currRow*lastColumn+column, object_list, predBlocks, 
						picSizeX, picSizeY,
						errorVar->yCondition);
				}
                else 
                  concealByCopy(recfr, currRow*lastColumn+column, 
                    object_list, picSizeX);
                
                ercMarkCurrMBConcealed (currRow*lastColumn+column, -1, picSizeX, errorVar);
              }
              
              row = lastCorruptedRow+1;
            }
            else 
            {
              /* correct bi-directionally */
              
              row = lastCorruptedRow+1;
              
              areaHeight = lastCorruptedRow-firstCorruptedRow+1;
              
              /* 
              *  Conceal the corrupted area switching between the up and the bottom rows 
              */
              for ( i = 0; i < areaHeight; i++) 
              {
                if ( i % 2 ) 
                {
                  currRow = lastCorruptedRow;
                  lastCorruptedRow --;
                }
                else 
                {
                  currRow = firstCorruptedRow;
                  firstCorruptedRow ++; 
                }
                
                ercCollect8PredBlocks (predBlocks, (currRow<<1), (column<<1), 
                  errorVar->yCondition, (lastRow<<1), (lastColumn<<1), 2, 0);      
                
                if(erc_mvperMB >= MVPERMB_THR)
                {
					if(ABS)
						concealABS(recfr, predMB, 
						currRow*lastColumn+column, object_list, predBlocks, 
						picSizeX, picSizeY,
						errorVar->yCondition);		
					else
						concealByTrial(recfr, predMB, 
						currRow*lastColumn+column, object_list, predBlocks, 
						picSizeX, picSizeY,
						errorVar->yCondition);
				}
                else
                  concealByCopy(recfr, currRow*lastColumn+column, 
                    object_list, picSizeX);
                
                ercMarkCurrMBConcealed (currRow*lastColumn+column, -1, picSizeX, errorVar);                
              }
            }
            lastCorruptedRow = -1;
            firstCorruptedRow = -1;
          }
        }
      }
    
      free(predMB);
    }
    return 1;
  }
  else
    return 0;
}

/*!
 ************************************************************************
 * \brief
 *      It conceals a given MB by simply copying the pixel area from the reference image 
 *      that is at the same location as the macroblock in the current image. This correcponds 
 *      to COPY MBs. 
 * \return
 *      Always zero (0).
 * \param recfr
 *      Reconstructed frame buffer
 * \param currMBNum
 *      current MB index
 * \param object_list
 *      Motion info for all MBs in the frame
 * \param picSizeX
 *      Width of the frame in pixels
 ************************************************************************
 */
static int concealByCopy(frame *recfr, int currMBNum,
  objectBuffer_t *object_list, int32 picSizeX)
{
  objectBuffer_t *currRegion;
   
  currRegion = object_list+(currMBNum<<2);
  currRegion->regionMode = REGMODE_INTER_COPY;
   
  currRegion->xMin = (xPosMB(currMBNum,picSizeX)<<4);
  currRegion->yMin = (yPosMB(currMBNum,picSizeX)<<4);
   
  copyBetweenFrames (recfr, MBNum2YBlock(currMBNum,0,picSizeX), picSizeX, 16);
   
  return 0;
}

/*!
 ************************************************************************
 * \brief
 *      Copies the co-located pixel values from the reference to the current frame. 
 *      Used by concealByCopy
 * \param recfr
 *      Reconstructed frame buffer
 * \param currYBlockNum
 *      index of the block (8x8) in the Y plane
 * \param picSizeX
 *      Width of the frame in pixels
 * \param regionSize      
 *      can be 16 or 8 to tell the dimension of the region to copy
 ************************************************************************
 */
static void copyBetweenFrames (frame *recfr, 
   int currYBlockNum, int32 picSizeX, int32 regionSize)
{
  int j, k, location, xmin, ymin;
  StorablePicture* refPic = listX[0][0];

  /* set the position of the region to be copied */
  xmin = (xPosYBlock(currYBlockNum,picSizeX)<<3);
  ymin = (yPosYBlock(currYBlockNum,picSizeX)<<3);
   
  for (j = ymin; j < ymin + regionSize; j++)
    for (k = xmin; k < xmin + regionSize; k++)
    {
      location = j * picSizeX + k; 
//th      recfr->yptr[location] = dec_picture->imgY[j][k];
      recfr->yptr[location] = refPic->imgY[j][k];
    }
     
    for (j = ymin >> uv_div[1][dec_picture->chroma_format_idc]; j < (ymin + regionSize) >> uv_div[1][dec_picture->chroma_format_idc]; j++)
      for (k = xmin >> uv_div[0][dec_picture->chroma_format_idc]; k < (xmin + regionSize) >> uv_div[0][dec_picture->chroma_format_idc]; k++)
      {
//        location = j * picSizeX / 2 + k;
        location = ((j * picSizeX) >> uv_div[0][dec_picture->chroma_format_idc]) + k;
        
//th        recfr->uptr[location] = dec_picture->imgUV[0][j][k];
//th        recfr->vptr[location] = dec_picture->imgUV[1][j][k];
        recfr->uptr[location] = refPic->imgUV[0][j][k];
        recfr->vptr[location] = refPic->imgUV[1][j][k];
      }                                
}

/*!
 ************************************************************************
 * \brief
 *      It conceals a given MB by using the motion vectors of one reliable neighbor. That MV of a 
 *      neighbor is selected wich gives the lowest pixel difference at the edges of the MB 
 *      (see function edgeDistortion). This corresponds to a spatial smoothness criteria.
 * \return
 *      Always zero (0).
 * \param recfr
 *      Reconstructed frame buffer
 * \param predMB
 *      memory area for storing temporary pixel values for a macroblock
 *      the Y,U,V planes are concatenated y = predMB, u = predMB+256, v = predMB+320
 * \param currMBNum
 *      current MB index
 * \param object_list
 *      array of region structures storing region mode and mv for each region
 * \param predBlocks
 *      status array of the neighboring blocks (if they are OK, concealed or lost)
 * \param picSizeX
 *      Width of the frame in pixels
 * \param picSizeY
 *      Height of the frame in pixels
 * \param yCondition
 *      array for conditions of Y blocks from ercVariables_t
 ************************************************************************
 */
static int concealByTrial(frame *recfr, imgpel *predMB, 
                          int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                          int32 picSizeX, int32 picSizeY, int *yCondition)
{
  int predMBNum = 0, numMBPerLine,
      compSplit1 = 0, compSplit2 = 0, compLeft = 1, comp = 0, compPred, order = 1,
      fInterNeighborExists, numIntraNeighbours,
      fZeroMotionChecked, predSplitted = 0,
      threshold = ERC_BLOCK_OK,
      minDist, currDist, i, k, bestDir;
  int32 regionSize;
  objectBuffer_t *currRegion;
  int32 mvBest[3] , mvPred[3], *mvptr;

  //Santosh
  int32 allmv[8][2]; //array for storing all the nbr MVs
//  int amv[3], mmv[3], pmv[3];

  //Santosh
  imgpel *predMB_OBMC;
	
  if(OBMC)
	predMB_OBMC = (imgpel *) malloc ( (256 + (img->mb_cr_size_x*img->mb_cr_size_y)*2) * sizeof (imgpel));

  //initialization
  for(i=0;i<8;i++)
	  for(k=0;k<2;k++)
		  allmv[i][k] = NIL; 
  
  numMBPerLine = (int) (picSizeX>>4);
  
  comp = 0;
  regionSize = 16;

  do 
  { /* 4 blocks loop */
    
    currRegion = object_list+(currMBNum<<2)+comp;
    
    /* set the position of the region to be concealed */
    
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    
    do 
    { /* reliability loop */
      
      minDist = 0; 
      fInterNeighborExists = 0; 
      numIntraNeighbours = 0; 
      fZeroMotionChecked = 0;
      
      /* loop the 4 neighbours */
      for (i = 4; i < 8; i++) 
      {        
        /* if reliable, try it */
        if (predBlocks[i] >= threshold) 
        {
          switch (i) 
          {
          case 4:
            predMBNum = currMBNum-numMBPerLine;
            compSplit1 = 2;
            compSplit2 = 3;
            break;
              
          case 5:
            predMBNum = currMBNum-1;
            compSplit1 = 1;
            compSplit2 = 3;
            break;
              
          case 6:
            predMBNum = currMBNum+numMBPerLine;
            compSplit1 = 0;
            compSplit2 = 1;
            break;
              
          case 7:
            predMBNum = currMBNum+1;
            compSplit1 = 0;
            compSplit2 = 2;
            break;
          }
          
          /* try the concealment with the Motion Info of the current neighbour
          only try if the neighbour is not Intra */
          if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
            isBlock(object_list,predMBNum,compSplit2,INTRA))
          {            
            numIntraNeighbours++;
          } 
          else 
          {
            /* if neighbour MB is splitted, try both neighbour blocks */
            for (predSplitted = isSplitted(object_list, predMBNum), 
              compPred = compSplit1;
              predSplitted >= 0;
              compPred = compSplit2,
              predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
            {              
              /* if Zero Motion Block, do the copying. This option is tried only once */
              if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
              {                
                if (fZeroMotionChecked) 
                {
                  continue;
                }
                else 
                {
                  fZeroMotionChecked = 1;

                  mvPred[0] = mvPred[1] = 0;
                  mvPred[2] = 0;

				  if(OBMA)
					buildOuterPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB, boundary);
				  else
					buildPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB);
                }
              }
              /* build motion using the neighbour's Motion Parameters */
              else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
              {
                continue;
              }
              else 
              {
                mvptr = getParam(object_list, predMBNum, compPred, mv);
                
                mvPred[0] = mvptr[0];
                mvPred[1] = mvptr[1];
                mvPred[2] = mvptr[2];		

				if(OBMA)
					buildOuterPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB, boundary);
				else
					buildPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB);
              }

			  //Store this MV
			  if(isSplitted(object_list, predMBNum))
			  {
				  if(predSplitted)
				  {
					  allmv[(i-4)*2][0] = mvPred[0];
					  allmv[(i-4)*2][1] = mvPred[1];
				  }
				  else
				  {
					  allmv[(i-4)*2+1][0] = mvPred[0];
					  allmv[(i-4)*2+1][1] = mvPred[1];
				  }					
			  }
			  else
			  {
				  //copy the MV
				  allmv[(i-4)*2][0] = mvPred[0];
				  allmv[(i-4)*2][1] = mvPred[1];
				  allmv[(i-4)*2+1][0] = mvPred[0];
				  allmv[(i-4)*2+1][1] = mvPred[1];
			  }
              
			  /* measure absolute boundary pixel difference */
			  if(OBMA)
				currDist = edgeDistortionOBMA(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize, boundary);
			  else
				currDist = edgeDistortion(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize);
			  
			  /* if so far best -> store the pixels as the best concealment */
              if (currDist < minDist || !fInterNeighborExists) 
              {                
                minDist = currDist;
                bestDir = i;
                
                for (k=0;k<3;k++) 
                  mvBest[k] = mvPred[k];
                
                currRegion->regionMode = 
                  (isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
                  ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
                  ((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);
                
                copyPredMB(MBNum2YBlock(currMBNum,comp,picSizeX), predMB, recfr, 
                  picSizeX, regionSize);
              }
              
              fInterNeighborExists = 1;
            }
          }
        }
    }
    
    threshold--;
    
    } while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
    /* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0;
      mvPred[2] = 0;

      if(OBMA)
		buildOuterPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB, boundary);
	  else
		buildPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB);

	  if(OBMA)
		currDist = edgeDistortionOBMA(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize, boundary);
	  else
		currDist = edgeDistortion(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = 
          ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);
        
        copyPredMB(MBNum2YBlock(currMBNum,comp,picSizeX), predMB, recfr, 
          picSizeX, regionSize);
      }
    }


	//Santosh
	/* Try other MV candidates here.....PMV, AMV, MMV, MVD
	PMV = MV of co-located MB in previous (Inter) frame 
	AMV = Average MV of nbr MBs
	MMV = Median MV of nbr MBs
	MVD = Diagonal MVs
	*/

	/*Average MV*/
/*	amv[0] = findaveragemv(allmv,0);
	amv[1] = findaveragemv(allmv,1);
	amv[2] = 0;
*/	
/*	if(OBMA)
		buildOuterPredRegionYUV(erc_img, amv, currRegion->xMin, currRegion->yMin, predMB, boundary);
	else
		buildPredRegionYUV(erc_img, amv, currRegion->xMin, currRegion->yMin, predMB);

	if(OBMA)
		currDist = edgeDistortionOBMA(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize, boundary);
	else
		currDist = edgeDistortion(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize);
      
    if (currDist < minDist || !fInterNeighborExists) 
    {
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = 
          ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);
        
        copyPredMB(MBNum2YBlock(currMBNum,comp,picSizeX), predMB, recfr, 
          picSizeX, regionSize);
    }

	/*Median MV*/
/*	mmv[0] = findmedianmv(allmv,0);
	mmv[0] = findmedianmv(allmv,1);
	mmv[2] = 0;
*/
/*	if(OBMA)
		buildOuterPredRegionYUV(erc_img, mmv, currRegion->xMin, currRegion->yMin, predMB, boundary);
	else
		buildPredRegionYUV(erc_img, mmv, currRegion->xMin, currRegion->yMin, predMB);

	if(OBMA)
		currDist = edgeDistortionOBMA(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize, boundary);
	else
		currDist = edgeDistortion(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize);
      
    if (currDist < minDist || !fInterNeighborExists) 
    {
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = 
          ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);
        
        copyPredMB(MBNum2YBlock(currMBNum,comp,picSizeX), predMB, recfr, 
          picSizeX, regionSize);
    }

	/*PMV = MV of co-located MB in previous (Inter) frame*/
/*	if(listX[0][0]->slice_type==0)
	{
		//Reference frame is INTER 

		//CHECK for mb_type nneded - TO DO!
		pmv[0] = listX[0][0]->mv[0][currRegion->yMin>>2][currRegion->xMin>>2][0];
		pmv[1] = listX[0][0]->mv[0][currRegion->yMin>>2][currRegion->xMin>>2][1];
		pmv[2] = 0;

		if(OBMA)
			buildOuterPredRegionYUV(erc_img, mmv, currRegion->xMin, currRegion->yMin, predMB, boundary);
		else
			buildPredRegionYUV(erc_img, mmv, currRegion->xMin, currRegion->yMin, predMB);

		if(OBMA)
			currDist = edgeDistortionOBMA(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize, boundary);
		else
			currDist = edgeDistortion(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize);
      
		if (currDist < minDist || !fInterNeighborExists) 
		{
			minDist = currDist;            
			for (k=0;k<3;k++) 
			  mvBest[k] = mvPred[k];
        
			currRegion->regionMode = 
			  ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);
        
			copyPredMB(MBNum2YBlock(currMBNum,comp,picSizeX), predMB, recfr, 
			  picSizeX, regionSize);
		}

	}	
*/
	/*Diagonal 4 MVs*/
/*
	//Top-Left
	if(predBlocks[1]>=ERC_BLOCK_CONCEALED)
	{
		predMBNum = currMBNum-numMBPerLine-1;
		mvptr = getParam(object_list, predMBNum, 3, mv);        
        mvPred[0] = mvptr[0];
        mvPred[1] = mvptr[1];
        mvPred[2] = mvptr[2];
		
		if(OBMA)
			buildOuterPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB, boundary);
		else
			buildPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB);

		if(OBMA)
			currDist = edgeDistortionOBMA(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize, boundary);
		else
			currDist = edgeDistortion(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize);
      
		if (currDist < minDist || !fInterNeighborExists) 
		{
			minDist = currDist;            
			for (k=0;k<3;k++) 
			  mvBest[k] = mvPred[k];
        
			currRegion->regionMode = 
			  ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);
        
			copyPredMB(MBNum2YBlock(currMBNum,comp,picSizeX), predMB, recfr, 
			  picSizeX, regionSize);
		}
	}

	//Top-Right
	if(predBlocks[0]>=ERC_BLOCK_CONCEALED)
	{
		predMBNum = currMBNum-numMBPerLine+1;
		mvptr = getParam(object_list, predMBNum, 2, mv);        
        mvPred[0] = mvptr[0];
        mvPred[1] = mvptr[1];
        mvPred[2] = mvptr[2];
		
		if(OBMA)
			buildOuterPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB, boundary);
		else
			buildPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB);

		if(OBMA)
			currDist = edgeDistortionOBMA(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize, boundary);
		else
			currDist = edgeDistortion(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize);
      
		if (currDist < minDist || !fInterNeighborExists) 
		{
			minDist = currDist;            
			for (k=0;k<3;k++) 
			  mvBest[k] = mvPred[k];
        
			currRegion->regionMode = 
			  ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);
        
			copyPredMB(MBNum2YBlock(currMBNum,comp,picSizeX), predMB, recfr, 
			  picSizeX, regionSize);
		}
	}

	//Bottom-Left
	if(predBlocks[2]>=ERC_BLOCK_CONCEALED)
	{
		predMBNum = currMBNum+numMBPerLine-1;
		mvptr = getParam(object_list, predMBNum, 1, mv);        
        mvPred[0] = mvptr[0];
        mvPred[1] = mvptr[1];
        mvPred[2] = mvptr[2];
		
		if(OBMA)
			buildOuterPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB, boundary);
		else
			buildPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB);

		if(OBMA)
			currDist = edgeDistortionOBMA(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize, boundary);
		else
			currDist = edgeDistortion(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize);
      
		if (currDist < minDist || !fInterNeighborExists) 
		{
			minDist = currDist;            
			for (k=0;k<3;k++) 
			  mvBest[k] = mvPred[k];
        
			currRegion->regionMode = 
			  ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);
        
			copyPredMB(MBNum2YBlock(currMBNum,comp,picSizeX), predMB, recfr, 
			  picSizeX, regionSize);
		}
	}

	//Bottom-Right
	if(predBlocks[3]>=ERC_BLOCK_CONCEALED)
	{
		predMBNum = currMBNum+numMBPerLine+1;
		mvptr = getParam(object_list, predMBNum, 0, mv);        
        mvPred[0] = mvptr[0];
        mvPred[1] = mvptr[1];
        mvPred[2] = mvptr[2];
		
		if(OBMA)
			buildOuterPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB, boundary);
		else
			buildPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB);

		if(OBMA)
			currDist = edgeDistortionOBMA(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize, boundary);
		else
			currDist = edgeDistortion(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize);
      
		if (currDist < minDist || !fInterNeighborExists) 
		{
			minDist = currDist;            
			for (k=0;k<3;k++) 
			  mvBest[k] = mvPred[k];
        
			currRegion->regionMode = 
			  ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);
        
			copyPredMB(MBNum2YBlock(currMBNum,comp,picSizeX), predMB, recfr, 
			  picSizeX, regionSize);
		}
	}
*/

    for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];

	//We found the best MV....now do OBMC
	if(OBMC)
	{
		//construct MB with best mv found above
		if(mvBest[0]!=NIL)
		{
			buildPredRegionYUV(erc_img, mvBest, currRegion->xMin, currRegion->yMin, predMB);
		}

		OBMC_MB(predMB,predMB_OBMC,predBlocks,object_list,currMBNum,numMBPerLine,picSizeX);
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB_OBMC, recfr, picSizeX, regionSize);
	}
    
    yCondition[MBNum2YBlock(currMBNum,comp,picSizeX)] = ERC_BLOCK_CONCEALED;
    comp = (comp+order+4)%4;
    compLeft--;
    
    } while (compLeft);
    
    return 0;
}

/*!
************************************************************************
* \brief
*      Builds the motion prediction pixels from the given location (in 1/4 pixel units) 
*      of the reference frame. It not only copies the pixel values but builds the interpolation 
*      when the pixel positions to be copied from is not full pixel (any 1/4 pixel position).
*      It copies the resulting pixel vlaues into predMB.
* \param img
*      The pointer of img_par struture of current frame
* \param mv
*      The pointer of the predicted MV of the current (being concealed) MB
* \param x
*      The x-coordinate of the above-left corner pixel of the current MB
* \param y
*      The y-coordinate of the above-left corner pixel of the current MB
* \param predMB
*      memory area for storing temporary pixel values for a macroblock
*      the Y,U,V planes are concatenated y = predMB, u = predMB+256, v = predMB+320
************************************************************************
*/
static void buildPredRegionYUV(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB)
{
  int tmp_block[BLOCK_SIZE][BLOCK_SIZE];
  int i=0,j=0,ii=0,jj=0,i1=0,j1=0,j4=0,i4=0;
  int jf=0;
  int uv;
  int vec1_x=0,vec1_y=0;
  int ioff,joff;
  imgpel *pMB = predMB;
  
  int ii0,jj0,ii1,jj1,if1,jf1,if0,jf0;
  int mv_mul;
  
  //FRExt
  int f1_x, f1_y, f2_x, f2_y, f3, f4, ifx;
  int b8, b4;
  int yuv = dec_picture->chroma_format_idc - 1;
  
  int ref_frame = max (mv[2], 0); // !!KS: quick fix, we sometimes seem to get negative ref_pic here, so restrict to zero an above

  /* Update coordinates of the current concealed macroblock */
  img->mb_x = x/MB_BLOCK_SIZE;
  img->mb_y = y/MB_BLOCK_SIZE;
  img->block_y = img->mb_y * BLOCK_SIZE;
  img->pix_c_y = img->mb_y * img->mb_cr_size_y;
  img->block_x = img->mb_x * BLOCK_SIZE;
  img->pix_c_x = img->mb_x * img->mb_cr_size_x;

  mv_mul=4;

  // luma *******************************************************

  for(j=0;j<MB_BLOCK_SIZE/BLOCK_SIZE;j++)
  {
    joff=j*4;
    j4=img->block_y+j;
    for(i=0;i<MB_BLOCK_SIZE/BLOCK_SIZE;i++)
    {
      ioff=i*4;
      i4=img->block_x+i;

      vec1_x = i4*4*mv_mul + mv[0];
      vec1_y = j4*4*mv_mul + mv[1];

      get_block(ref_frame, listX[0], vec1_x,vec1_y,img,tmp_block);

      for(ii=0;ii<BLOCK_SIZE;ii++)
        for(jj=0;jj<MB_BLOCK_SIZE/BLOCK_SIZE;jj++)
          img->mpr[ii+ioff][jj+joff]=tmp_block[ii][jj];
    }
  }

  for (i = 0; i < 16; i++)
  {
    for (j = 0; j < 16; j++)
    {
      pMB[i*16+j] = img->mpr[j][i];
    }
  }
  pMB += 256;

  if (dec_picture->chroma_format_idc != YUV400)
  {
    // chroma *******************************************************
    f1_x = 64/img->mb_cr_size_x;
    f2_x=f1_x-1;

    f1_y = 64/img->mb_cr_size_y;
    f2_y=f1_y-1;

    f3=f1_x*f1_y;
    f4=f3>>1;

    for(uv=0;uv<2;uv++)
    {
      for (b8=0;b8<(img->num_blk8x8_uv/2);b8++)
      {
        for(b4=0;b4<4;b4++)
        {
          joff = subblk_offset_y[yuv][b8][b4];
          j4=img->pix_c_y+joff;
          ioff = subblk_offset_x[yuv][b8][b4];
          i4=img->pix_c_x+ioff;

          for(jj=0;jj<4;jj++)
          {
            jf=(j4+jj)/(img->mb_cr_size_y/4);     // jf  = Subblock_y-coordinate
            for(ii=0;ii<4;ii++)
            {
              ifx=(i4+ii)/(img->mb_cr_size_x/4);  // ifx = Subblock_x-coordinate

              i1=(i4+ii)*f1_x + mv[0];
              j1=(j4+jj)*f1_y + mv[1];
            
              ii0=max (0, min (i1/f1_x,   dec_picture->size_x_cr-1));
              jj0=max (0, min (j1/f1_y,   dec_picture->size_y_cr-1));
              ii1=max (0, min ((i1+f2_x)/f1_x, dec_picture->size_x_cr-1));
              jj1=max (0, min ((j1+f2_y)/f1_y, dec_picture->size_y_cr-1));
            
              if1=(i1 & f2_x);
              jf1=(j1 & f2_y);
              if0=f1_x-if1;
              jf0=f1_y-jf1;
            
              img->mpr[ii+ioff][jj+joff]=(if0*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii0]+
                                          if1*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii1]+
                                          if0*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii0]+
                                          if1*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii1]+f4)/f3;
            }
          }
        }
      }

      for (i = 0; i < 8; i++)
      {
        for (j = 0; j < 8; j++)
        {
          pMB[i*8+j] = img->mpr[j][i];
        }
      }
      pMB += 64;

    }
  }
}
/*!
 ************************************************************************
 * \brief
 *      Copies pixel values between a YUV frame and the temporary pixel value storage place. This is
 *      used to save some pixel values temporarily before overwriting it, or to copy back to a given 
 *      location in a frame the saved pixel values.
 * \param currYBlockNum   
 *      index of the block (8x8) in the Y plane
 * \param predMB          
 *      memory area where the temporary pixel values are stored
 *      the Y,U,V planes are concatenated y = predMB, u = predMB+256, v = predMB+320
 * \param recfr           
 *      pointer to a YUV frame
 * \param picSizeX        
 *      picture width in pixels
 * \param regionSize      
 *      can be 16 or 8 to tell the dimension of the region to copy
 ************************************************************************
 */
static void copyPredMB (int currYBlockNum, imgpel *predMB, frame *recfr, 
                        int32 picSizeX, int32 regionSize) 
{
  
  int j, k, xmin, ymin, xmax, ymax;
  int32 locationTmp, locationPred;
  int uv_x = uv_div[0][dec_picture->chroma_format_idc];
  int uv_y = uv_div[1][dec_picture->chroma_format_idc];
  
  xmin = (xPosYBlock(currYBlockNum,picSizeX)<<3);
  ymin = (yPosYBlock(currYBlockNum,picSizeX)<<3);
  xmax = xmin + regionSize -1;
  ymax = ymin + regionSize -1;
  
  for (j = ymin; j <= ymax; j++) 
  {
    for (k = xmin; k <= xmax; k++)
    {
      locationPred = j * picSizeX + k;
      locationTmp = (j-ymin) * 16 + (k-xmin);
      dec_picture->imgY[j][k] = predMB[locationTmp];
    }
  }
  
  if (dec_picture->chroma_format_idc != YUV400)
  {
    for (j = (ymin>>uv_y); j <= (ymax>>uv_y); j++) 
    {
      for (k = (xmin>>uv_x); k <= (xmax>>uv_x); k++)
      {
        locationPred = ((j * picSizeX) >> uv_x) + k;
        locationTmp = (j-(ymin>>uv_y)) * img->mb_cr_size_x + (k-(xmin>>1)) + 256;
        dec_picture->imgUV[0][j][k] = predMB[locationTmp];
		
        locationTmp += 64;
        
        dec_picture->imgUV[1][j][k] = predMB[locationTmp];
      }
    }
  }

/*
  //Use this to see the damaged/lost area
  for (j = ymin; j <= ymax; j++) 
  {
    for (k = xmin; k <= xmax; k++)
    {
      locationPred = j * picSizeX + k;
      locationTmp = (j-ymin) * 16 + (k-xmin);
      //dec_picture->imgY[j][k] = predMB[locationTmp];
	  dec_picture->imgY[j][k] = 0;
    }
  }
  
  if (dec_picture->chroma_format_idc != YUV400)
  {
    for (j = (ymin>>uv_y); j <= (ymax>>uv_y); j++) 
    {
      for (k = (xmin>>uv_x); k <= (xmax>>uv_x); k++)
      {
        locationPred = ((j * picSizeX) >> uv_x) + k;
        locationTmp = (j-(ymin>>uv_y)) * img->mb_cr_size_x + (k-(xmin>>1)) + 256;
        //dec_picture->imgUV[0][j][k] = predMB[locationTmp];
		dec_picture->imgUV[0][j][k] = 0;
		
        locationTmp += 64;
        
        //dec_picture->imgUV[1][j][k] = predMB[locationTmp];
		dec_picture->imgUV[1][j][k] = 0;
      }
    }
  }
*/

}

/*!
 ************************************************************************
 * \brief
 *      Calculates a weighted pixel difference between edge Y pixels of the macroblock stored in predMB
 *      and the pixels in the given Y plane of a frame (recY) that would become neighbor pixels if 
 *      predMB was placed at currYBlockNum block position into the frame. This "edge distortion" value
 *      is used to determine how well the given macroblock in predMB would fit into the frame when
 *      considering spatial smoothness. If there are correctly received neighbor blocks (status stored 
 *      in predBlocks) only they are used in calculating the edge distorion; otherwise also the already
 *      concealed neighbor blocks can also be used.
 * \return 
 *      The calculated weighted pixel difference at the edges of the MB.
 * \param predBlocks      
 *      status array of the neighboring blocks (if they are OK, concealed or lost)
 * \param currYBlockNum   
 *      index of the block (8x8) in the Y plane
 * \param predMB          
 *      memory area where the temporary pixel values are stored
 *      the Y,U,V planes are concatenated y = predMB, u = predMB+256, v = predMB+320
 * \param recY            
 *      pointer to a Y plane of a YUV frame
 * \param picSizeX        
 *      picture width in pixels
 * \param regionSize      
 *      can be 16 or 8 to tell the dimension of the region to copy
 ************************************************************************
 */
static int edgeDistortion (int predBlocks[], int currYBlockNum, imgpel *predMB, 
                           imgpel *recY, int32 picSizeX, int32 regionSize)
{
  int i, j, distortion, numOfPredBlocks, threshold = ERC_BLOCK_OK;
  imgpel *currBlock = NULL, *neighbor = NULL;
  int32 currBlockOffset = 0;
  
  currBlock = recY + (yPosYBlock(currYBlockNum,picSizeX)<<3)*picSizeX + (xPosYBlock(currYBlockNum,picSizeX)<<3);
  
  do 
  {
    
    distortion = 0; numOfPredBlocks = 0;
    
    /* loop the 4 neighbours */
    for (j = 4; j < 8; j++) 
    {
      /* if reliable, count boundary pixel difference */
      if (predBlocks[j] >= threshold) 
      {
        
        switch (j) 
        {
        case 4:
          neighbor = currBlock - picSizeX;
          for ( i = 0; i < regionSize; i++ ) 
          {
            distortion += mabs((int)(predMB[i] - neighbor[i]));
          }
          break;          
        case 5:
          neighbor = currBlock - 1;
          for ( i = 0; i < regionSize; i++ ) 
          {
            distortion += mabs((int)(predMB[i*16] - neighbor[i*picSizeX]));
          }
          break;                
        case 6:
          neighbor = currBlock + regionSize*picSizeX;
          currBlockOffset = (regionSize-1)*16;
          for ( i = 0; i < regionSize; i++ ) 
          {
            distortion += mabs((int)(predMB[i+currBlockOffset] - neighbor[i]));
          }
          break;                
        case 7:
          neighbor = currBlock + regionSize;
          currBlockOffset = regionSize-1;
          for ( i = 0; i < regionSize; i++ ) 
          {
            distortion += mabs((int)(predMB[i*16+currBlockOffset] - neighbor[i*picSizeX]));
          }
          break;
        }
        
        numOfPredBlocks++;
      }
    }
    
    threshold--;
    if (threshold < ERC_BLOCK_CONCEALED)
      break;
  } while (numOfPredBlocks == 0);
  
  if(numOfPredBlocks == 0)
  {
    return 0;
    // assert (numOfPredBlocks != 0); !!!KS hmm, trying to continue...
  }
  return (distortion/numOfPredBlocks);
}

// picture error concealment below

/*!
************************************************************************
* \brief
* The motion prediction pixels are calculated from the given location (in 
* 1/4 pixel units) of the referenced frame. It copies the sub block from the 
* corresponding reference to the frame to be concealed.
*
*************************************************************************
*/
static void buildPredblockRegionYUV(struct img_par *img, int32 *mv, 
                                    int x, int y, imgpel *predMB, int list)
{
    int tmp_block[BLOCK_SIZE][BLOCK_SIZE];
    int i=0,j=0,ii=0,jj=0,i1=0,j1=0,j4=0,i4=0;
    int jf=0;
    int uv;
    int vec1_x=0,vec1_y=0;
    int ioff,joff;
    imgpel *pMB = predMB;

    int ii0,jj0,ii1,jj1,if1,jf1,if0,jf0;
    int mv_mul;

    //FRExt
    int f1_x, f1_y, f2_x, f2_y, f3, f4, ifx;
    int yuv = dec_picture->chroma_format_idc - 1;

    int ref_frame = mv[2];

    /* Update coordinates of the current concealed macroblock */

    img->mb_x = x/BLOCK_SIZE;
    img->mb_y = y/BLOCK_SIZE;
    img->block_y = img->mb_y * BLOCK_SIZE;
    img->pix_c_y = img->mb_y * img->mb_cr_size_y/4;
    img->block_x = img->mb_x * BLOCK_SIZE;
    img->pix_c_x = img->mb_x * img->mb_cr_size_x/4;

    mv_mul=4;

    // luma *******************************************************

    vec1_x = x*mv_mul + mv[0];
    vec1_y = y*mv_mul + mv[1];

    get_block(ref_frame, listX[list], vec1_x,vec1_y,img,tmp_block);

    for(ii=0;ii<BLOCK_SIZE;ii++)
        for(jj=0;jj<MB_BLOCK_SIZE/BLOCK_SIZE;jj++)
            img->mpr[ii][jj]=tmp_block[ii][jj];


    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            pMB[i*4+j] = img->mpr[j][i];
        }
    }
    pMB += 16;

    if (dec_picture->chroma_format_idc != YUV400)
    {
        // chroma *******************************************************
        f1_x = 64/(img->mb_cr_size_x);
        f2_x=f1_x-1;

        f1_y = 64/(img->mb_cr_size_y);
        f2_y=f1_y-1;

        f3=f1_x*f1_y;
        f4=f3>>1;

        for(uv=0;uv<2;uv++)
        {
            joff = subblk_offset_y[yuv][0][0];
            j4=img->pix_c_y+joff;
            ioff = subblk_offset_x[yuv][0][0];
            i4=img->pix_c_x+ioff;

            for(jj=0;jj<2;jj++)
            {
                jf=(j4+jj)/(img->mb_cr_size_y/4);     // jf  = Subblock_y-coordinate
                for(ii=0;ii<2;ii++)
                {
                    ifx=(i4+ii)/(img->mb_cr_size_x/4);  // ifx = Subblock_x-coordinate

                    i1=(i4+ii)*f1_x + mv[0];
                    j1=(j4+jj)*f1_y + mv[1];

                    ii0=max (0, min (i1/f1_x,   dec_picture->size_x_cr-1));
                    jj0=max (0, min (j1/f1_y,   dec_picture->size_y_cr-1));
                    ii1=max (0, min ((i1+f2_x)/f1_x, dec_picture->size_x_cr-1));
                    jj1=max (0, min ((j1+f2_y)/f1_y, dec_picture->size_y_cr-1));

                    if1=(i1 & f2_x);
                    jf1=(j1 & f2_y);
                    if0=f1_x-if1;
                    jf0=f1_y-jf1;

                    img->mpr[ii][jj]=(if0*jf0*listX[list][ref_frame]->imgUV[uv][jj0][ii0]+
                        if1*jf0*listX[list][ref_frame]->imgUV[uv][jj0][ii1]+
                        if0*jf1*listX[list][ref_frame]->imgUV[uv][jj1][ii0]+
                        if1*jf1*listX[list][ref_frame]->imgUV[uv][jj1][ii1]+f4)/f3;
                }
            }

            for (i = 0; i < 2; i++)
            {
                for (j = 0; j < 2; j++)
                {
                    pMB[i*2+j] = img->mpr[j][i];
                }
            }
            pMB += 4;

        }
    }
}

/*!
************************************************************************
* \brief
*    compares two stored pictures by picture number for qsort in descending order
*
************************************************************************
*/
static int compare_pic_by_pic_num_desc( const void *arg1, const void *arg2 )
{
    if ( (*(StorablePicture**)arg1)->pic_num < (*(StorablePicture**)arg2)->pic_num)
        return 1;
    if ( (*(StorablePicture**)arg1)->pic_num > (*(StorablePicture**)arg2)->pic_num)
        return -1;
    else
        return 0;
}

/*!
************************************************************************
* \brief
*    compares two stored pictures by picture number for qsort in descending order
*
************************************************************************
*/
static int compare_pic_by_lt_pic_num_asc( const void *arg1, const void *arg2 )
{
    if ( (*(StorablePicture**)arg1)->long_term_pic_num < (*(StorablePicture**)arg2)->long_term_pic_num)
        return -1;
    if ( (*(StorablePicture**)arg1)->long_term_pic_num > (*(StorablePicture**)arg2)->long_term_pic_num)
        return 1;
    else
        return 0;
}

/*!
************************************************************************
* \brief
*    compares two stored pictures by poc for qsort in ascending order
*
************************************************************************
*/
static int compare_pic_by_poc_asc( const void *arg1, const void *arg2 )
{
    if ( (*(StorablePicture**)arg1)->poc < (*(StorablePicture**)arg2)->poc)
        return -1;
    if ( (*(StorablePicture**)arg1)->poc > (*(StorablePicture**)arg2)->poc)
        return 1;
    else
        return 0;
}


/*!
************************************************************************
* \brief
*    compares two stored pictures by poc for qsort in descending order
*
************************************************************************
*/
static int compare_pic_by_poc_desc( const void *arg1, const void *arg2 )
{
    if ( (*(StorablePicture**)arg1)->poc < (*(StorablePicture**)arg2)->poc)
        return 1;
    if ( (*(StorablePicture**)arg1)->poc > (*(StorablePicture**)arg2)->poc)
        return -1;
    else
        return 0;
}

/*!
************************************************************************
* \brief
*    Copy image data from one array to another array
************************************************************************
*/

static
void CopyImgData(imgpel **inputY, imgpel ***inputUV, imgpel **outputY, 
                 imgpel ***outputUV, int img_width, int img_height)
{
    int x, y;

    for (y=0; y<img_height; y++)
        for (x=0; x<img_width; x++)
            outputY[y][x] = inputY[y][x];

    for (y=0; y<img_height/2; y++)
        for (x=0; x<img_width/2; x++)
        {
            outputUV[0][y][x] = inputUV[0][y][x];
            outputUV[1][y][x] = inputUV[1][y][x];
        }
}

/*!
************************************************************************
* \brief
*    Copies the last reference frame for concealing reference frame loss. 
************************************************************************
*/

static StorablePicture *
get_last_ref_pic_from_dpb()
{
    int used_size = dpb.used_size - 1;
    int i;

    for(i = used_size; i >= 0; i--)
    {
        if (dpb.fs[i]->is_used==3)
        {
            if (((dpb.fs[i]->frame->used_for_reference) && 
                (!dpb.fs[i]->frame->is_long_term)) /*||  ((dpb.fs[i]->frame->used_for_reference==0)
                                                   && (dpb.fs[i]->frame->slice_type == P_SLICE))*/ )
            {
                return dpb.fs[i]->frame;
            }
        }
    }

    return NULL;
}

/*!
************************************************************************
* \brief
* Conceals the lost reference or non reference frame by either frame copy 
* or motion vector copy concealment. 
*
************************************************************************
*/

static void copy_to_conceal(StorablePicture *src, StorablePicture *dst, ImageParameters *img)
{
    int i=0;
    int mv[3];
    int multiplier;
    imgpel *predMB, *storeYUV;
    int j, y, x, mb_height, mb_width, ii=0, jj=0;
    int uv;
    int mm, nn;
    int scale = 1;
    // struct inp_par *test;

    img->current_mb_nr = 0;

    dst->PicSizeInMbs  = src->PicSizeInMbs;

    dst->slice_type = src->slice_type = img->conceal_slice_type;

    dst->idr_flag = 0; //since we do not want to clears the ref list

    dst->no_output_of_prior_pics_flag = src->no_output_of_prior_pics_flag;
    dst->long_term_reference_flag = src->long_term_reference_flag;
    dst->adaptive_ref_pic_buffering_flag = src->adaptive_ref_pic_buffering_flag = 0;
    dst->chroma_format_idc = src->chroma_format_idc;
    dst->frame_mbs_only_flag = src->frame_mbs_only_flag;
    dst->frame_cropping_flag = src->frame_cropping_flag;
    dst->frame_cropping_rect_left_offset = src->frame_cropping_rect_left_offset;
    dst->frame_cropping_rect_right_offset = src->frame_cropping_rect_right_offset;
    dst->frame_cropping_rect_bottom_offset = src->frame_cropping_rect_bottom_offset;
    dst->frame_cropping_rect_top_offset = src->frame_cropping_rect_top_offset;
    dst->qp = src->qp;
    dst->slice_qp_delta = src->slice_qp_delta;

    dec_picture = src;

    // Conceals the missing frame by frame copy concealment
    if (img->conceal_mode==1)
    {     
        // We need these initializations for using deblocking filter for frame copy
        // concealment as well.
        dst->PicWidthInMbs = src->PicWidthInMbs;
        dst->PicSizeInMbs = src->PicSizeInMbs;

        CopyImgData(src->imgY, src->imgUV, 
            dst->imgY, dst->imgUV,
            img->width, img->height);

    }

    // Conceals the missing frame by motion vector copy concealment
    if (img->conceal_mode==2)
    {
        if (dec_picture->chroma_format_idc != YUV400)
        {
            storeYUV = (imgpel *) malloc ( (16 + (img->mb_cr_size_x*img->mb_cr_size_y)*2/16) * sizeof (imgpel));
        }
        else
        {
            storeYUV = (imgpel *) malloc (16  * sizeof (imgpel));
        }

        erc_img = img;

        dst->PicWidthInMbs = src->PicWidthInMbs;
        dst->PicSizeInMbs = src->PicSizeInMbs;
        mb_width = dst->PicWidthInMbs;
        mb_height = (dst->PicSizeInMbs)/(dst->PicWidthInMbs);
        scale = (img->conceal_slice_type == B_SLICE) ? 2 : 1;

        if(img->conceal_slice_type == B_SLICE)
            init_lists_for_non_reference_loss(dst->slice_type, img->currentSlice->structure);
        else
            init_lists(dst->slice_type, img->currentSlice->structure);

        multiplier = BLOCK_SIZE;

        for(i=0;i<mb_height*4;i++)
        {
            mm = i*BLOCK_SIZE;
            for(j=0;j<mb_width*4;j++)
            {                       
                nn = j*BLOCK_SIZE;

                mv[0] = src->mv[LIST_0][i][j][0] / scale;
                mv[1] = src->mv[LIST_0][i][j][1] / scale;
                mv[2] = src->ref_idx[LIST_0][i][j];


                if(mv[2]<0)
                    mv[2]=0;

                dst->mv[LIST_0][i][j][0] = mv[0];
                dst->mv[LIST_0][i][j][1] = mv[1];
                dst->ref_idx[LIST_0][i][j] = mv[2];

                x = (j)*multiplier;
                y = (i)*multiplier;

                if ((mm%16==0) && (nn%16==0))
                    img->current_mb_nr++;

                buildPredblockRegionYUV(erc_img, mv, x, y, storeYUV, LIST_0);

                predMB = storeYUV;

                for(ii=0;ii<multiplier;ii++)
                {
                    for(jj=0;jj<multiplier;jj++)
                    {
                        dst->imgY[i*multiplier+ii][j*multiplier+jj] = predMB[ii*(multiplier)+jj];
                    }
                }

                predMB = predMB + (multiplier*multiplier);

                if (dec_picture->chroma_format_idc != YUV400)
                {

                    for(uv=0;uv<2;uv++)
                    {
                        for(ii=0;ii< (multiplier/2);ii++)
                        {
                            for(jj=0;jj< (multiplier/2);jj++)
                            {
                                dst->imgUV[uv][i*multiplier/2 +ii][j*multiplier/2 +jj] = predMB[ii*(multiplier/2)+jj];
                            }
                        }
                        predMB = predMB + (multiplier*multiplier/4);
                    }
                }
            }
        }
        free(storeYUV);
    }
}

/*!
************************************************************************
* \brief
* Uses the previous reference pic for concealment of reference frames
*
************************************************************************
*/

static void
copy_prev_pic_to_concealed_pic(StorablePicture *picture, ImageParameters *img)
{

    StorablePicture *ref_pic;
    /* get the last ref pic in dpb */
    ref_pic = get_last_ref_pic_from_dpb();

    assert(ref_pic != NULL);

    /* copy all the struc from this to current concealment pic */
    img->conceal_slice_type = P_SLICE;
    copy_to_conceal(ref_pic, picture, img);
}


/*!
************************************************************************
* \brief
* This function conceals a missing reference frame. The routine is called
* based on the difference in frame number. It conceals an IDR frame loss
* based on the sudden decrease in frame number.
*
************************************************************************
*/

void conceal_lost_frames(ImageParameters *img)
{
    int CurrFrameNum;
    int UnusedShortTermFrameNum;
    StorablePicture *picture = NULL;
    int tmp1 = img->delta_pic_order_cnt[0];
    int tmp2 = img->delta_pic_order_cnt[1];
    int i;

    img->delta_pic_order_cnt[0] = img->delta_pic_order_cnt[1] = 0;

    // printf("A gap in frame number is found, try to fill it.\n");

    if(img->IDR_concealment_flag == 1)
    {
        // Conceals an IDR frame loss. Uses the reference frame in the previous 
        // GOP for concealment.
        UnusedShortTermFrameNum = 0;
        img->last_ref_pic_poc = -img->poc_gap;
        img->earlier_missing_poc = 0;
    }
    else
        UnusedShortTermFrameNum = (img->pre_frame_num + 1) % img->MaxFrameNum;

    CurrFrameNum = img->frame_num;

    while (CurrFrameNum != UnusedShortTermFrameNum)
    {
        picture = alloc_storable_picture (FRAME, img->width, img->height, img->width_cr, img->height_cr);

        picture->coded_frame = 1;
        picture->pic_num = UnusedShortTermFrameNum;
        picture->frame_num = UnusedShortTermFrameNum;
        picture->non_existing = 0;
        picture->is_output = 0;
        picture->used_for_reference = 1;
        picture->concealed_pic = 1;

        picture->adaptive_ref_pic_buffering_flag = 0;

        img->frame_num = UnusedShortTermFrameNum;

        picture->top_poc=img->last_ref_pic_poc + img->ref_poc_gap;
        picture->bottom_poc=picture->top_poc;
        picture->frame_poc=picture->top_poc;
        picture->poc=picture->top_poc;
        img->last_ref_pic_poc = picture->poc;

        copy_prev_pic_to_concealed_pic(picture, img);

        //if (UnusedShortTermFrameNum == 0)
        if(img->IDR_concealment_flag == 1)
        {
            picture->slice_type = I_SLICE;
            picture->idr_flag = 1;
            flush_dpb();
            picture->top_poc= 0;
            picture->bottom_poc=picture->top_poc;
            picture->frame_poc=picture->top_poc;
            picture->poc=picture->top_poc;
            img->last_ref_pic_poc = picture->poc;
        }

        store_picture_in_dpb(picture);

        picture=NULL;

        img->pre_frame_num = UnusedShortTermFrameNum;
        UnusedShortTermFrameNum = (UnusedShortTermFrameNum + 1) % img->MaxFrameNum;

        // update reference flags and set current flag.
        for(i=16;i>0;i--)
        {
          ref_flag[i] = ref_flag[i-1];
        }
        ref_flag[0] = 0;
    }
    img->delta_pic_order_cnt[0] = tmp1;
    img->delta_pic_order_cnt[1] = tmp2;
    img->frame_num = CurrFrameNum;
}

/*!
************************************************************************
* \brief
* Updates the reference list for motion vector copy concealment for non-
* reference frame loss.
*
************************************************************************
*/

void update_ref_list_for_concealment()
{
    unsigned i, j;
    for (i=0, j=0; i<dpb.used_size; i++)
    {
        if (dpb.fs[i]->concealment_reference)
        {
            dpb.fs_ref[j++]=dpb.fs[i];
        }
    }

    dpb.ref_frames_in_buffer = active_pps->num_ref_idx_l0_active_minus1;
}

/*!
************************************************************************
* \brief
*    Initialize the list based on the B frame or non reference 'p' frame 
*    to be concealed. The function initialize listX[0] and list 1 depending 
*    on current picture type
*
************************************************************************
*/
void init_lists_for_non_reference_loss(int currSliceType, PictureStructure currPicStructure)
{
    unsigned i;
    int j;
    int MaxFrameNum = 1 << (active_sps->log2_max_frame_num_minus4 + 4);
    int diff;

    int list0idx = 0;
    int list0idx_1 = 0;

    StorablePicture *tmp_s;

    if (currPicStructure == FRAME)
    {
        for(i=0;i<dpb.ref_frames_in_buffer; i++)
        {
            if(dpb.fs[i]->concealment_reference == 1)
            {
                if(dpb.fs[i]->frame_num > img->frame_to_conceal)
                    dpb.fs_ref[i]->frame_num_wrap = dpb.fs[i]->frame_num - MaxFrameNum;
                else
                    dpb.fs_ref[i]->frame_num_wrap = dpb.fs[i]->frame_num;
                dpb.fs_ref[i]->frame->pic_num = dpb.fs_ref[i]->frame_num_wrap;
            }
        }
    }

    if (currSliceType == P_SLICE)
    {
        // Calculate FrameNumWrap and PicNum
        if (currPicStructure == FRAME)  
        {
            for(i=0;i<dpb.used_size; i++)
            {
                if(dpb.fs[i]->concealment_reference == 1)
                {
                    listX[0][list0idx++] = dpb.fs[i]->frame;
                }
            }
            // order list 0 by PicNum
            qsort((void *)listX[0], list0idx, sizeof(StorablePicture*), compare_pic_by_pic_num_desc);
            listXsize[0] = list0idx;
        }
    }

    if (currSliceType == B_SLICE)
    {
        if (currPicStructure == FRAME)  
        {
            //      for(i=0;i<dpb.ref_frames_in_buffer; i++)
            for(i=0;i<dpb.used_size; i++)
            {
                if(dpb.fs[i]->concealment_reference == 1)
                {
                    if(img->earlier_missing_poc > dpb.fs[i]->frame->poc)
                        listX[0][list0idx++] = dpb.fs[i]->frame;
                }
            }

            qsort((void *)listX[0], list0idx, sizeof(StorablePicture*), compare_pic_by_poc_desc);
            list0idx_1 = list0idx;

            //      for(i=0;i<dpb.ref_frames_in_buffer; i++)
            for(i=0;i<dpb.used_size; i++)
            {
                if(dpb.fs[i]->concealment_reference == 1)
                {
                    if(img->earlier_missing_poc < dpb.fs[i]->frame->poc)
                        listX[0][list0idx++] = dpb.fs[i]->frame;
                }
            }

            qsort((void *)&listX[0][list0idx_1], list0idx-list0idx_1, sizeof(StorablePicture*), compare_pic_by_poc_asc);

            for (j=0; j<list0idx_1; j++)
            {
                listX[1][list0idx-list0idx_1+j]=listX[0][j];
            }
            for (j=list0idx_1; j<list0idx; j++)
            {
                listX[1][j-list0idx_1]=listX[0][j];
            }

            listXsize[0] = listXsize[1] = list0idx;

            qsort((void *)&listX[0][listXsize[0]], list0idx-listXsize[0], sizeof(StorablePicture*), compare_pic_by_lt_pic_num_asc);
            qsort((void *)&listX[1][listXsize[0]], list0idx-listXsize[0], sizeof(StorablePicture*), compare_pic_by_lt_pic_num_asc);
            listXsize[0] = listXsize[1] = list0idx;
        }
    }

    if ((listXsize[0] == listXsize[1]) && (listXsize[0] > 1))
    {
        // check if lists are identical, if yes swap first two elements of listX[1]
        diff=0;
        for (j = 0; j< listXsize[0]; j++)
        {
            if (listX[0][j]!=listX[1][j])
                diff=1;
        }
        if (!diff)
        {
            tmp_s = listX[1][0];
            listX[1][0]=listX[1][1];
            listX[1][1]=tmp_s;
        }
    }


    // set max size
    listXsize[0] = min (listXsize[0], (int)active_sps->num_ref_frames);
    listXsize[1] = min (listXsize[1], (int)active_sps->num_ref_frames);

    listXsize[1] = 0;
    // set the unused list entries to NULL
    for (i=listXsize[0]; i< (MAX_LIST_SIZE) ; i++)
    {
        listX[0][i] = NULL;
    }
    for (i=listXsize[1]; i< (MAX_LIST_SIZE) ; i++)
    {
        listX[1][i] = NULL;
    }
}


/*!
************************************************************************
* \brief
* Get from the dpb the picture corresponding to a POC.  The POC varies 
* depending on whether it is a frame copy or motion vector copy concealment. 
* The frame corresponding to the POC is returned.
*
************************************************************************
*/

StorablePicture *get_pic_from_dpb(int missingpoc, unsigned int *pos)
{
    int used_size = dpb.used_size - 1;
    int i, concealfrom = 0;

    if(img->conceal_mode == 1)
        concealfrom = missingpoc - img->poc_gap;
    else if (img->conceal_mode == 2)
        concealfrom = missingpoc + img->poc_gap;

    for(i = used_size; i >= 0; i--)
    {
        if(dpb.fs[i]->poc == concealfrom)
        {
            *pos = i;
            return dpb.fs[i]->frame;
        }
    }

    return NULL;
}

/*!
************************************************************************
* \brief
* Function to sort the POC and find the lowest number in the POC list
* Compare the integers
*
************************************************************************
*/

int comp(const void *i, const void *j)
{
    return *(int *)i - *(int *)j;
}

/*!
************************************************************************
* \brief
* Initialises a node, allocates memory for the node, and returns
* a pointer to the new node.
*
************************************************************************
*/

struct concealment_node * init_node( StorablePicture* picture, int missingpoc )
{
    struct concealment_node *ptr;

    ptr = (struct concealment_node *) calloc( 1, sizeof(struct concealment_node ) );

    if( ptr == NULL )
        return (struct concealment_node *) NULL;
    else {
        ptr->picture = picture;
        ptr->missingpocs = missingpoc;
        ptr->next = NULL;
        return ptr;
    }
}

/*!
************************************************************************
* \brief
* Prints the details of a node
*
************************************************************************
*/

void print_node( struct concealment_node *ptr )
{
    printf("Missing POC=%d\n", ptr->missingpocs );
}


/*!
************************************************************************
* \brief
* Prints all nodes from the current address passed to it.
*
************************************************************************
*/

void print_list( struct concealment_node *ptr )
{
    while( ptr != NULL )
    {
        print_node( ptr );
        ptr = ptr->next;
    }
}

/*!
************************************************************************
* \brief
* Adds a node to the end of the list.
*
************************************************************************
*/


void add_node( struct concealment_node *concealment_new )
{
    if( concealment_head == NULL )
    {
        concealment_end = concealment_head = concealment_new;
        return;
    }
    concealment_end->next = concealment_new;
    concealment_end = concealment_new;
}


/*!
************************************************************************
* \brief
* Deletes the specified node pointed to by 'ptr' from the list
*
************************************************************************
*/


void delete_node( struct concealment_node *ptr )
{
    // We only need to delete the first node in the linked list
    if( ptr == concealment_head ) {
        concealment_head = concealment_head->next;
        if( concealment_end == ptr )
            concealment_end = concealment_end->next;
        free(ptr);
    }
}

/*!
************************************************************************
* \brief
* Deletes all nodes from the place specified by ptr
*
************************************************************************
*/

void delete_list( struct concealment_node *ptr )
{
    struct concealment_node *temp;

    if( concealment_head == NULL ) return;

    if( ptr == concealment_head ) {
        concealment_head = NULL;
        concealment_end = NULL;
    }
    else 
    {
        temp = concealment_head;

        while( temp->next != ptr )
            temp = temp->next;
        concealment_end = temp;
    }

    while( ptr != NULL ) {
        temp = ptr->next;
        free( ptr );
        ptr = temp;
    }
}

/*!
************************************************************************
* \brief
* Stores the missing non reference frames in the concealment buffer. The
* detection is based on the POC difference in the sorted POC array. A missing 
* non reference frame is detected when the dpb is full. A singly linked list 
* is maintained for storing the missing non reference frames. 
*
************************************************************************
*/

void conceal_non_ref_pics(int diff)
{
    int missingpoc = 0;
    unsigned int i, pos;
    StorablePicture *conceal_from_picture = NULL;
    StorablePicture *conceal_to_picture = NULL;
    struct concealment_node *concealment_ptr = NULL;
    int temp_used_size = dpb.used_size;

    if(dpb.used_size == 0 )
        return;

    qsort(pocs_in_dpb, dpb.size, sizeof(int), comp);

    for(i=0;i<dpb.size-diff;i++)
    {
        dpb.used_size = dpb.size;
        if((pocs_in_dpb[i+1]-pocs_in_dpb[i])>img->poc_gap)
        {
            conceal_to_picture = alloc_storable_picture (FRAME, img->width, img->height, img->width_cr, img->height_cr);

            missingpoc = pocs_in_dpb[i] + img->poc_gap;
            // Diagnostics
            // printf("\n missingpoc = %d\n",missingpoc);

            if(missingpoc > img->earlier_missing_poc)
            {
                img->earlier_missing_poc = missingpoc;
                conceal_to_picture->top_poc= missingpoc;
                conceal_to_picture->bottom_poc=missingpoc;
                conceal_to_picture->frame_poc=missingpoc;
                conceal_to_picture->poc=missingpoc;
                conceal_from_picture = get_pic_from_dpb(missingpoc, &pos);

                assert(conceal_from_picture != NULL);

                dpb.used_size = pos+1;

                img->frame_to_conceal = conceal_from_picture->frame_num + 1;

                update_ref_list_for_concealment();
                img->conceal_slice_type = B_SLICE;
                copy_to_conceal(conceal_from_picture, conceal_to_picture, img);
                concealment_ptr = init_node( conceal_to_picture, missingpoc );
                add_node(concealment_ptr);
                // Diagnostics
                // print_node(concealment_ptr);
            }
        }
    }

    //restore the original value
    //dpb.used_size = dpb.size;
    dpb.used_size = temp_used_size;
}

/*!
************************************************************************
* \brief
* Perform Sliding window decoded reference picture marking process. It
* maintains the POC s stored in the dpb at a specific instance.   
*
************************************************************************
*/

void sliding_window_poc_management(StorablePicture *p)
{
    unsigned int i;

    if (dpb.used_size == dpb.size)
    {
        for(i=0;i<dpb.size-1; i++)
            pocs_in_dpb[i] = pocs_in_dpb[i+1];
    }

//    pocs_in_dpb[dpb.used_size-1] = p->poc;
}


/*!
************************************************************************
* \brief
* Outputs the non reference frames. The POCs in the concealment buffer are
* sorted in ascending order and outputted when the lowest POC in the 
* concealment buffer is lower than the lowest in the dpb. The linked list 
* entry corresponding to the outputted POC is immediately deleted.
*
************************************************************************
*/

void write_lost_non_ref_pic(int poc, int p_out)
{
    FrameStore concealment_fs;
    if(poc > 0)
    {
        if((poc - dpb.last_output_poc) > img->poc_gap)
        {

            concealment_fs.frame = concealment_head->picture;
            concealment_fs.is_output = 0;
            concealment_fs.is_reference = 0;
            concealment_fs.is_used = 3;

            write_stored_frame(&concealment_fs, p_out);
            delete_node(concealment_head);
        }
    }
}

/*!
************************************************************************
* \brief
* Conceals frame loss immediately after the IDR. This special case produces
* the same result for either frame copy or motion vector copy concealment.
*
************************************************************************
*/

void write_lost_ref_after_idr(int pos)
{
    int temp = 1;

    if(last_out_fs->frame == NULL)
    {
        last_out_fs->frame = alloc_storable_picture (FRAME, img->width, img->height, 
            img->width_cr, img->height_cr);
        last_out_fs->is_used = 3;                        
    }

    if(img->conceal_mode == 2)
    {
        temp = 2;
        img->conceal_mode = 1;
    }
    copy_to_conceal(dpb.fs[pos]->frame, last_out_fs->frame, img);

    img->conceal_mode = temp;
}


//Santosh
//************************************************************************************************************************

static void buildOuterPredRegionYUV(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary)
{
  int tmp_block[BLOCK_SIZE][BLOCK_SIZE];
  int i=0,j=0,ii=0,jj=0,i1=0,j1=0,j4=0,i4=0;
  int jf=0;
  int uv;
  int vec1_x=0,vec1_y=0;
  int ioff,joff;
  imgpel *pMB = predMB;
  
  int ii0,jj0,ii1,jj1,if1,jf1,if0,jf0;
  int mv_mul;

  int above[4], left[4], below[4], right[4], index;
  
  //FRExt
  int f1_x, f1_y, f2_x, f2_y, f3, f4, ifx;
  int b8, b4;
  int yuv = dec_picture->chroma_format_idc - 1;
  
  int ref_frame = max (mv[2], 0); // !!KS: quick fix, we sometimes seem to get negative ref_pic here, so restrict to zero an above

  /* Update coordinates of the current concealed macroblock */
  img->mb_x = x/MB_BLOCK_SIZE;
  img->mb_y = y/MB_BLOCK_SIZE;
  img->block_y = img->mb_y * BLOCK_SIZE;
  img->pix_c_y = img->mb_y * img->mb_cr_size_y;
  img->block_x = img->mb_x * BLOCK_SIZE;
  img->pix_c_x = img->mb_x * img->mb_cr_size_x;

  mv_mul=4;

  // luma *******************************************************

  for(i=0;i<4;i++)
  {
	  above[i]=0;left[i]=0;below[i]=0;right[i]=0;
  }

  for(j=0;j<MB_BLOCK_SIZE/BLOCK_SIZE;j++)
  {
    joff=j*4;
    j4=img->block_y+j;
    for(i=0;i<MB_BLOCK_SIZE/BLOCK_SIZE;i++)
    {
      ioff=i*4;
      i4=img->block_x+i;

      vec1_x = i4*4*mv_mul + mv[0];
      vec1_y = j4*4*mv_mul + mv[1];

      get_block(ref_frame, listX[0], vec1_x,vec1_y,img,tmp_block);

	  index = i+4*j;
	  get_boundary(ref_frame, listX[0], vec1_x, vec1_y, img, index, above, left, below, right);
	  
	  if(j==0)
	  {
		  boundary[4*i+0]=above[0];boundary[4*i+1]=above[1];boundary[4*i+2]=above[2];boundary[4*i+3]=above[3];
	  }

	  if(i==0)
	  {
		  boundary[16+4*j+0]=left[0];boundary[16+4*j+1]=left[1];boundary[16+4*j+2]=left[2];boundary[16+4*j+3]=left[3];
	  }

	  if(j==3) //MB_BLOCK_SIZE/BLOCK_SIZE - 1 = 3
	  {
		  boundary[32+4*i+0]=below[0];boundary[32+4*i+1]=below[1];boundary[32+4*i+2]=below[2];boundary[32+4*i+3]=below[3];
	  }

	  if(i==3)
	  {
		  boundary[48+4*j+0]=right[0];boundary[48+4*j+1]=right[1];boundary[48+4*j+2]=right[2];boundary[48+4*j+3]=right[3];
	  }

      for(ii=0;ii<BLOCK_SIZE;ii++)
        for(jj=0;jj<MB_BLOCK_SIZE/BLOCK_SIZE;jj++)
          img->mpr[ii+ioff][jj+joff]=tmp_block[ii][jj];
    }
  }

  for (i = 0; i < 16; i++)
  {
    for (j = 0; j < 16; j++)
    {
      pMB[i*16+j] = img->mpr[j][i];
    }
  }

  pMB += 256;

  if (dec_picture->chroma_format_idc != YUV400)
  {
    // chroma *******************************************************
    f1_x = 64/img->mb_cr_size_x;
    f2_x=f1_x-1;

    f1_y = 64/img->mb_cr_size_y;
    f2_y=f1_y-1;

    f3=f1_x*f1_y;
    f4=f3>>1;

    for(uv=0;uv<2;uv++)
    {
      for (b8=0;b8<(img->num_blk8x8_uv/2);b8++)
      {
        for(b4=0;b4<4;b4++)
		{
          joff = subblk_offset_y[yuv][b8][b4];
          j4=img->pix_c_y+joff;
          ioff = subblk_offset_x[yuv][b8][b4];
          i4=img->pix_c_x+ioff;

          for(jj=0;jj<4;jj++)
          {
            jf=(j4+jj)/(img->mb_cr_size_y/4);     // jf  = Subblock_y-coordinate
            for(ii=0;ii<4;ii++)
            {
              ifx=(i4+ii)/(img->mb_cr_size_x/4);  // ifx = Subblock_x-coordinate

              i1=(i4+ii)*f1_x + mv[0];
              j1=(j4+jj)*f1_y + mv[1];
            
              ii0=max (0, min (i1/f1_x,   dec_picture->size_x_cr-1));
              jj0=max (0, min (j1/f1_y,   dec_picture->size_y_cr-1));
              ii1=max (0, min ((i1+f2_x)/f1_x, dec_picture->size_x_cr-1));
              jj1=max (0, min ((j1+f2_y)/f1_y, dec_picture->size_y_cr-1));
            
              if1=(i1 & f2_x);
              jf1=(j1 & f2_y);
              if0=f1_x-if1;
              jf0=f1_y-jf1;
            
              img->mpr[ii+ioff][jj+joff]=(if0*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii0]+
                                          if1*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii1]+
                                          if0*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii0]+
                                          if1*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii1]+f4)/f3;
            }
          }
        }
      }

      for (i = 0; i < 8; i++)
      {
        for (j = 0; j < 8; j++)
        {
          pMB[i*8+j] = img->mpr[j][i];
        }
      }
      pMB += 64;

    }
  }
}


void get_boundary(int ref_frame, StorablePicture **list, int x_pos, int y_pos, struct img_par *img, int index, int above[4], int left[4], int below[4], int right[4])
{
  int dx, dy;
  int x, y;
  int i, j;
  int maxold_x,maxold_y;
  int result;
  int pres_x;
  int pres_y; 
  int tmp_res[4][9];
  static const int COEF[6] = { 1, -5, 20, 20, -5, 1 };

  int block[4][4];

  if (list[ref_frame] == no_reference_picture && img->framepoc < img->recovery_poc)
  {
      printf("list[ref_frame] is equal to 'no reference picture' before RAP\n");

      /* fill the boundary with sample value 128 */
	  for(i=0;i<BLOCK_SIZE;i++)
	  {
		  above[i]=128;left[i]=128;below[i]=128;right[i]=128;
	  }
      
      return;
  }

  dx = x_pos&3;
  dy = y_pos&3;
  
  x_pos = (x_pos-dx)/4;
  y_pos = (y_pos-dy)/4;

  maxold_x = dec_picture->size_x-1;
  maxold_y = dec_picture->size_y-1;

  if (dec_picture->mb_field[img->current_mb_nr])
    maxold_y = dec_picture->size_y/2 - 1;

  if (dx == 0 && dy == 0) 
  {  /* fullpel position */
	 
	  if(index==0||index==1||index==2||index==3)
	  {
		  //Above
		  for(j=0;j<4;j++)
		  {
			  above[j] = list[ref_frame]->imgY[max(0,min(maxold_y,y_pos-1))][max(0,min(maxold_x,x_pos+j))];
		  }
	  }

	  if(index==0||index==4||index==8||index==12)
	  {
		  //Left
		  for(i=0;i<4;i++)
		  {
			  left[i] = list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+i))][max(0,min(maxold_x,x_pos-1))];
		  }
	  }

	  if(index==12||index==13||index==14||index==15)
	  {
		  //Below
		  for(j=0;j<4;j++)
		  {
			  below[j] = list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+BLOCK_SIZE))][max(0,min(maxold_x,x_pos+j))];
		  }
	  }

	  if(index==3||index==7||index==11||index==15)
	  {
		  //Right
		  for(i=0;i<4;i++)
		  {
			  right[i] = list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+i))][max(0,min(maxold_x,x_pos+BLOCK_SIZE))];
		  }
	  }
  }
  else 
  { /* other positions */

    if (dy == 0) 
    { /* No vertical interpolation */
		
		if(index==0||index==1||index==2||index==3)
		{
			//Above
			for(i=0;i<BLOCK_SIZE;i++)
			{
				for (result = 0, x = -2; x < 4; x++)
					result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos-1))][max(0,min(maxold_x,x_pos+i+x))]*COEF[x+2];				
				above[i] = max(0, min(img->max_imgpel_value, (result+16)/32));
			}

			if((dx&1)==1)
			{
				for(i=0;i<BLOCK_SIZE;i++)
				{
					above[i] = (above[i] + list[ref_frame]->imgY[max(0,min(maxold_y,y_pos-1))][max(0,min(maxold_x,x_pos+i+dx/2))] +1 )/2;
				}
			}
		}
		if(index==0||index==4||index==8||index==12)
		{
			//Left
			for(i=0;i<BLOCK_SIZE;i++)
			{
				for (result = 0, x = -2; x < 4; x++)
					result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+i))][max(0,min(maxold_x,x_pos+i+x-1))]*COEF[x+2];				
				left[i] = max(0, min(img->max_imgpel_value, (result+16)/32));
			}

			if((dx&1)==1)
			{
				for(i=0;i<BLOCK_SIZE;i++)
				{
					left[i] = (left[i] + list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+i))][max(0,min(maxold_x,x_pos-1+i+dx/2))] +1 )/2;
				}
			}
		}
	    if(index==12||index==13||index==14||index==15)
		{
		    //Below
			for(i=0;i<BLOCK_SIZE;i++)
			{
				for (result = 0, x = -2; x < 4; x++)
					result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+BLOCK_SIZE))][max(0,min(maxold_x,x_pos+i+x))]*COEF[x+2];				
				below[i] = max(0, min(img->max_imgpel_value, (result+16)/32));
			}

			if((dx&1)==1)
			{
				for(i=0;i<BLOCK_SIZE;i++)
				{
					below[i] = (below[i] + list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+BLOCK_SIZE))][max(0,min(maxold_x,x_pos+i+dx/2))] +1 )/2;
				}
			}
		}
	    if(index==3||index==7||index==11||index==15)
		{
		    //Right
			for(i=0;i<BLOCK_SIZE;i++)
			{
				for (result = 0, x = -2; x < 4; x++)
					result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+i))][max(0,min(maxold_x,x_pos+i+x+BLOCK_SIZE))]*COEF[x+2];			
				right[i] = max(0, min(img->max_imgpel_value, (result+16)/32));
			}

			if((dx&1)==1)
			{
				for(i=0;i<BLOCK_SIZE;i++)
				{
					right[i] = (right[i] + list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+i))][max(0,min(maxold_x,x_pos+BLOCK_SIZE+i+dx/2))] +1 )/2;
				}
			}
		}
    }
    else if (dx == 0) 
    {  /* No horizontal interpolation */

		if(index==0||index==1||index==2||index==3)
		{
			//Above
			for(i=0;i<BLOCK_SIZE;i++)
			{
				for (result = 0, y = -2; y < 4; y++)
					result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+y-1))][max(0,min(maxold_x,x_pos+i))]*COEF[y+2];				
				above[i] = max(0, min(img->max_imgpel_value, (result+16)/32));
			}

			if((dy&1)==1)
			{
				for(i=0;i<BLOCK_SIZE;i++)
				{
					above[i] = (above[i] + list[ref_frame]->imgY[max(0,min(maxold_y,y_pos-1+dy/2))][max(0,min(maxold_x,x_pos+i))] +1 )/2;
				}
			}
		}
		if(index==0||index==4||index==8||index==12)
		{
			//Left
			for(i=0;i<BLOCK_SIZE;i++)
			{
				for (result = 0, y = -2; y < 4; y++)
					result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+y+i))][max(0,min(maxold_x,x_pos+i-1))]*COEF[y+2];
				left[i] = max(0, min(img->max_imgpel_value, (result+16)/32));
			}

			if((dy&1)==1)
			{
				for(i=0;i<BLOCK_SIZE;i++)
				{
					left[i] = (left[i] + list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+i+dy/2))][max(0,min(maxold_x,x_pos-1+i))] +1 )/2;
				}
			}
		}
	    if(index==12||index==13||index==14||index==15)
		{
		    //Below
			for(i=0;i<BLOCK_SIZE;i++)
			{
				for (result = 0, y = -2; y < 4; y++)
					result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+y+BLOCK_SIZE))][max(0,min(maxold_x,x_pos+i))]*COEF[y+2];
				below[i] = max(0, min(img->max_imgpel_value, (result+16)/32));
			}

			if((dy&1)==1)
			{
				for(i=0;i<BLOCK_SIZE;i++)
				{
					below[i] = (below[i] + list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+BLOCK_SIZE+dy/2))][max(0,min(maxold_x,x_pos+i))] +1 )/2;
				}
			}
		}
	    if(index==3||index==7||index==11||index==15)
		{
		    //Right
			for(i=0;i<BLOCK_SIZE;i++)
			{
				for (result = 0, y = -2; y < 4; y++)
					result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+y+i))][max(0,min(maxold_x,x_pos+i+BLOCK_SIZE))]*COEF[y+2];
				right[i] = max(0, min(img->max_imgpel_value, (result+16)/32));
			}

			if((dy&1)==1)
			{
				for(i=0;i<BLOCK_SIZE;i++)
				{
					right[i] = (right[i] + list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+i+dy/2))][max(0,min(maxold_x,x_pos+BLOCK_SIZE+i))] +1 )/2;
				}
			}
		}

    }
    else if (dx == 2) 
    {  /* Vertical & horizontal interpolation */

		if(index==0||index==1||index==2||index==3)
		{
			//Above

			for (j = -2; j < BLOCK_SIZE+3; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					for (tmp_res[i][j+2] = 0, x = -2; x < 4; x++)
					{
						tmp_res[i][j+2] += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos-1+j))][max(0,min(maxold_x,x_pos+i+x))]*COEF[x+2];
					}
				}
			}

			for (j = 0; j < 1; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					for (result = 0, y = -2; y < 4; y++)
						result += tmp_res[i][j+y+2]*COEF[y+2];
					above[i] = max(0, min(img->max_imgpel_value, (result+512)/1024));
				}
			}

			if ((dy&1) == 1)
			{
				for (j = 0; j < 1; j++)
					for (i = 0; i < BLOCK_SIZE; i++)
						above[i] = (above[i] + max(0, min(img->max_imgpel_value, (tmp_res[i][j+2+dy/2]+16)/32)) +1 )/2;
			}

		}
		if(index==0||index==4||index==8||index==12)
		{
			//Left
			for (j = -2; j < BLOCK_SIZE+3; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					for (tmp_res[i][j+2] = 0, x = -2; x < 4; x++)
					{
						tmp_res[i][j+2] += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j))][max(0,min(maxold_x,x_pos-1+i+x))]*COEF[x+2];
					}
				}
			}

			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = 0; i < 1; i++)
				{
					for (result = 0, y = -2; y < 4; y++)
						result += tmp_res[i][j+y+2]*COEF[y+2];
					left[j] = max(0, min(img->max_imgpel_value, (result+512)/1024));
				}
			}

			if ((dy&1) == 1)
			{
				for (j = 0; j < BLOCK_SIZE; j++)
					for (i = 0; i < 1; i++)
						left[j] = (left[j] + max(0, min(img->max_imgpel_value, (tmp_res[i][j+2+dy/2]+16)/32)) +1 )/2;
			}
		}

		if(index==12||index==13||index==14||index==15)
		{
			//Below

			for (j = -2; j < BLOCK_SIZE+3; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					for (tmp_res[i][j+2] = 0, x = -2; x < 4; x++)
					{
						tmp_res[i][j+2] += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+BLOCK_SIZE+j))][max(0,min(maxold_x,x_pos+i+x))]*COEF[x+2];
					}
				}
			}

			for (j = 0; j < 1; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					for (result = 0, y = -2; y < 4; y++)
						result += tmp_res[i][j+y+2]*COEF[y+2];
					below[i] = max(0, min(img->max_imgpel_value, (result+512)/1024));
				}
			}

			if ((dy&1) == 1)
			{
				for (j = 0; j < 1; j++)
					for (i = 0; i < BLOCK_SIZE; i++)
						below[i] = (below[i] + max(0, min(img->max_imgpel_value, (tmp_res[i][j+2+dy/2]+16)/32)) +1 )/2;
			}
		}

		if(index==3||index==7||index==11||index==15)
		{
			//Right
			for (j = -2; j < BLOCK_SIZE+3; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					for (tmp_res[i][j+2] = 0, x = -2; x < 4; x++)
					{
						tmp_res[i][j+2] += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j))][max(0,min(maxold_x,x_pos+BLOCK_SIZE+i+x))]*COEF[x+2];
					}
				}
			}

			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = 0; i < 1; i++)
				{
					for (result = 0, y = -2; y < 4; y++)
						result += tmp_res[i][j+y+2]*COEF[y+2];
					right[j] = max(0, min(img->max_imgpel_value, (result+512)/1024));
				}
			}

			if ((dy&1) == 1)
			{
				for (j = 0; j < BLOCK_SIZE; j++)
					for (i = 0; i < 1; i++)
						right[j] = (right[j] + max(0, min(img->max_imgpel_value, (tmp_res[i][j+2+dy/2]+16)/32)) +1 )/2;
			}
		}
    }
    else if (dy == 2)
    {  /* Horizontal & vertical interpolation */

		if(index==0||index==1||index==2||index==3)
		{
			//Above
			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = -2; i < BLOCK_SIZE+3; i++)
				{
					for (tmp_res[j][i+2] = 0, y = -2; y < 4; y++)
					{
						tmp_res[j][i+2] += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos-1+j+y))][max(0,min(maxold_x,x_pos+i))]*COEF[y+2];
					}
				}
			}

			for (j = 0; j < 1; j++)
			{	
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					for (result = 0, x = -2; x < 4; x++)
						result += tmp_res[j][i+x+2]*COEF[x+2];
					
					above[i] = max(0, min(img->max_imgpel_value, (result+512)/1024));
				}
			}

			if ((dx&1) == 1)
			{
				for (j = 0; j < 1; j++)
					for (i = 0; i < BLOCK_SIZE; i++)
						above[i] = (above[i] + max(0, min(img->max_imgpel_value, (tmp_res[j][i+2+dx/2]+16)/32))+1)/2;
			}

		}
		if(index==0||index==4||index==8||index==12)
		{
			//Left
			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = -2; i < BLOCK_SIZE+3; i++)
				{
					for (tmp_res[j][i+2] = 0, y = -2; y < 4; y++)
					{
						tmp_res[j][i+2] += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j+y))][max(0,min(maxold_x,x_pos-1+i))]*COEF[y+2];
					}
				}
			}

			for (j = 0; j < BLOCK_SIZE; j++)
			{	
				for (i = 0; i < 1; i++)
				{
					for (result = 0, x = -2; x < 4; x++)
						result += tmp_res[j][i+x+2]*COEF[x+2];
					
					left[j] = max(0, min(img->max_imgpel_value, (result+512)/1024));
				}
			}

			if ((dx&1) == 1)
			{
				for (j = 0; j < BLOCK_SIZE; j++)
					for (i = 0; i < 1; i++)
						left[j] = (left[j] + max(0, min(img->max_imgpel_value, (tmp_res[j][i+2+dx/2]+16)/32))+1)/2;
			}
		}

		if(index==12||index==13||index==14||index==15)
		{
			//Below
			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = -2; i < BLOCK_SIZE+3; i++)
				{
					for (tmp_res[j][i+2] = 0, y = -2; y < 4; y++)
					{
						tmp_res[j][i+2] += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+BLOCK_SIZE+j+y))][max(0,min(maxold_x,x_pos+i))]*COEF[y+2];
					}
				}
			}

			for (j = 0; j < 1; j++)
			{	
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					for (result = 0, x = -2; x < 4; x++)
						result += tmp_res[j][i+x+2]*COEF[x+2];
					
					below[i] = max(0, min(img->max_imgpel_value, (result+512)/1024));
				}
			}

			if ((dx&1) == 1)
			{
				for (j = 0; j < 1; j++)
					for (i = 0; i < BLOCK_SIZE; i++)
						below[i] = (below[i] + max(0, min(img->max_imgpel_value, (tmp_res[j][i+2+dx/2]+16)/32))+1)/2;
			}
		}

		if(index==3||index==7||index==11||index==15)
		{
			//Right
			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = -2; i < BLOCK_SIZE+3; i++)
				{
					for (tmp_res[j][i+2] = 0, y = -2; y < 4; y++)
					{
						tmp_res[j][i+2] += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j+y))][max(0,min(maxold_x,x_pos+BLOCK_SIZE+i))]*COEF[y+2];
					}
				}
			}

			for (j = 0; j < BLOCK_SIZE; j++)
			{	
				for (i = 0; i < 1; i++)
				{
					for (result = 0, x = -2; x < 4; x++)
						result += tmp_res[j][i+x+2]*COEF[x+2];
					
					right[j] = max(0, min(img->max_imgpel_value, (result+512)/1024));
				}
			}

			if ((dx&1) == 1)
			{
				for (j = 0; j < BLOCK_SIZE; j++)
					for (i = 0; i < 1; i++)
						right[j] = (right[j] + max(0, min(img->max_imgpel_value, (tmp_res[j][i+2+dx/2]+16)/32))+1)/2;
			}
		}

    }
    else
    {  /* Diagonal interpolation */

		if(index==0||index==1||index==2||index==3)
		{
			//Above

			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					pres_y = dy == 1 ? y_pos-1+j : y_pos-1+j+1;
					pres_y = max(0,min(maxold_y,pres_y));
					for (result = 0, x = -2; x < 4; x++)
						result += list[ref_frame]->imgY[pres_y][max(0,min(maxold_x,x_pos+i+x))]*COEF[x+2];
					block[i][j] = max(0, min(img->max_imgpel_value, (result+16)/32));
				}
			}

			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					pres_x = dx == 1 ? x_pos+i : x_pos+i+1;
					pres_x = max(0,min(maxold_x,pres_x));
					for (result = 0, y = -2; y < 4; y++)
						result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j+y))][pres_x]*COEF[y+2];
					block[i][j] = (block[i][j] + max(0, min(img->max_imgpel_value, (result+16)/32)) +1 ) / 2;
				}
			}

			for(i=0;i<4;i++)
				above[i]=block[i][0];

		}
		if(index==0||index==4||index==8||index==12)
		{
			//Left
			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					pres_y = dy == 1 ? y_pos+j : y_pos+j+1;
					pres_y = max(0,min(maxold_y,pres_y));
					for (result = 0, x = -2; x < 4; x++)
						result += list[ref_frame]->imgY[pres_y][max(0,min(maxold_x,x_pos+i+x))]*COEF[x+2];
					block[i][j] = max(0, min(img->max_imgpel_value, (result+16)/32));
				}
			}

			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					pres_x = dx == 1 ? x_pos-1+i : x_pos-1+i+1;
					pres_x = max(0,min(maxold_x,pres_x));
					for (result = 0, y = -2; y < 4; y++)
						result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j+y))][pres_x]*COEF[y+2];
					block[i][j] = (block[i][j] + max(0, min(img->max_imgpel_value, (result+16)/32)) +1 ) / 2;
				}
			}

			for(i=0;i<4;i++)
				left[i]=block[0][i];
		}

		if(index==12||index==13||index==14||index==15)
		{
			//Below
			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					pres_y = dy == 1 ? y_pos+BLOCK_SIZE+j : y_pos+BLOCK_SIZE+j+1;
					pres_y = max(0,min(maxold_y,pres_y));
					for (result = 0, x = -2; x < 4; x++)
						result += list[ref_frame]->imgY[pres_y][max(0,min(maxold_x,x_pos+i+x))]*COEF[x+2];
					block[i][j] = max(0, min(img->max_imgpel_value, (result+16)/32));
				}
			}

			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					pres_x = dx == 1 ? x_pos+i : x_pos+i+1;
					pres_x = max(0,min(maxold_x,pres_x));
					for (result = 0, y = -2; y < 4; y++)
						result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j+y))][pres_x]*COEF[y+2];
					block[i][j] = (block[i][j] + max(0, min(img->max_imgpel_value, (result+16)/32)) +1 ) / 2;
				}
			}

			for(i=0;i<4;i++)
				below[i]=block[i][0];
		}

		if(index==3||index==7||index==11||index==15)
		{
			//Right
			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					pres_y = dy == 1 ? y_pos+j : y_pos+j+1;
					pres_y = max(0,min(maxold_y,pres_y));
					for (result = 0, x = -2; x < 4; x++)
						result += list[ref_frame]->imgY[pres_y][max(0,min(maxold_x,x_pos+i+x))]*COEF[x+2];
					block[i][j] = max(0, min(img->max_imgpel_value, (result+16)/32));
				}
			}

			for (j = 0; j < BLOCK_SIZE; j++)
			{
				for (i = 0; i < BLOCK_SIZE; i++)
				{
					pres_x = dx == 1 ? x_pos+BLOCK_SIZE+i : x_pos+BLOCK_SIZE+i+1;
					pres_x = max(0,min(maxold_x,pres_x));
					for (result = 0, y = -2; y < 4; y++)
						result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j+y))][pres_x]*COEF[y+2];
					block[i][j] = (block[i][j] + max(0, min(img->max_imgpel_value, (result+16)/32)) +1 ) / 2;
				}
			}

			for(i=0;i<4;i++)
				right[i]=block[0][i];
		}
	}
  }
}


static int edgeDistortionOBMA (int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary)
{
  int i, j, distortion, numOfPredBlocks, threshold = ERC_BLOCK_OK;
  imgpel *currBlock = NULL, *neighbor = NULL;
  int32 currBlockOffset = 0;
  
  currBlock = recY + (yPosYBlock(currYBlockNum,picSizeX)<<3)*picSizeX + (xPosYBlock(currYBlockNum,picSizeX)<<3);
  
  do 
  {    
    distortion = 0; numOfPredBlocks = 0;
    
    /* loop the 4 neighbours */
    for (j = 4; j < 8; j++) 
    {
      /* if reliable, count boundary pixel difference */
      if (predBlocks[j] >= threshold) 
      {
        
        switch (j) 
        {
        case 4:
          neighbor = currBlock - picSizeX;
          for ( i = 0; i < regionSize; i++ ) 
          {
            distortion += mabs((int)(boundary[i] - neighbor[i]));
          }
          break;          
        case 5:
          neighbor = currBlock - 1;
          for ( i = 0; i < regionSize; i++ ) 
          {
            distortion += mabs((int)(boundary[16+i] - neighbor[i*picSizeX]));
          }
          break;                
        case 6:
          neighbor = currBlock + regionSize*picSizeX;
          currBlockOffset = (regionSize-1)*16;
          for ( i = 0; i < regionSize; i++ ) 
          {
            distortion += mabs((int)(boundary[32+i] - neighbor[i]));
          }
          break;                
        case 7:
          neighbor = currBlock + regionSize;
          currBlockOffset = regionSize-1;
          for ( i = 0; i < regionSize; i++ ) 
          {
            distortion += mabs((int)(boundary[48+i] - neighbor[i*picSizeX]));
          }
          break;
        }
        
        numOfPredBlocks++;
      }
    }
    
    threshold--;
    if (threshold < ERC_BLOCK_CONCEALED)
      break;
  } while (numOfPredBlocks == 0);
  
  if(numOfPredBlocks == 0)
    return 0;

  return (distortion/numOfPredBlocks);
}

int findaveragemv(int allmv[8][2],int comp)
{
	int avg,i,count=0,sum=0;

	for(i=0;i<8;i++)
	{
		if(allmv[i][comp]!=NIL)
		{
			count++;
			sum+=allmv[i][comp];
		}
	}
	avg=sum/count;
	return avg;
}

int findmedianmv(int allmv[8][2],int comp)
{
	int i,j,k,mvs[8],tmp,median;
	
	j=0;
	for(i=0;i<8;i++)
	{
		if(allmv[i][comp]!=NIL)
		{
			mvs[j++]=allmv[i][comp];
		}
	}
	//length of mvs = j

	//Sort mvs in ascending order
	for (i=0;i<j;i++) 
	{ 
		//Pick out each element in turn.
		tmp=mvs[i];
		k=i-1;
		while (k >= 0 && mvs[k] > tmp) 
		{ 
			//Look for the place to insert it.
			mvs[k+1]=mvs[k];
			k--;
		}
		mvs[k+1]=tmp; //Insert it.
	}

	//Find the median
	if(j/2==1)
	{
		median = mvs[j/2];
	}
	else
	{
		median = (mvs[j/2]+mvs[j/2-1])/2;
	}

	return median;
}

/**************************************************************************************************************/
/**************************************************************************************************************/
/**************************************************************************************************************/
//Adaptive Block Size Temporal Concealment

static int concealABS(frame *recfr, imgpel *predMB, int currMBNum, objectBuffer_t *object_list, 
					  int predBlocks[], int32 picSizeX, int32 picSizeY, int *yCondition)
{
	int numMBPerLine, mb_ecmode=ECMODE1, threshold = ERC_BLOCK_CONCEALED; //default values
	
	numMBPerLine = (int) (picSizeX>>4);

	//Find the ECMode for this MB from the neighbors

	if(FOURMODES)
		mb_ecmode = fourmodes_find_mb_ecmode(predBlocks,numMBPerLine,currMBNum);
	else
		mb_ecmode = find_mb_ecmode(predBlocks,numMBPerLine,currMBNum);

	printf("%d\t%d\n",currMBNum, mb_ecmode);

	switch(mb_ecmode)
	{
	case ECMODE1:
		concealABS_ECMODE1(recfr,predMB,currMBNum,object_list,predBlocks,picSizeX,picSizeY,yCondition);
		break;
	case ECMODE2:
		concealABS_ECMODE2(recfr,predMB,currMBNum,object_list,predBlocks,picSizeX,picSizeY,yCondition);
		break;
	case ECMODE3:
		concealABS_ECMODE3(recfr,predMB,currMBNum,object_list,predBlocks,picSizeX,picSizeY,yCondition);
		break;
	case ECMODE4:
		concealABS_ECMODE4(recfr,predMB,currMBNum,object_list,predBlocks,picSizeX,picSizeY,yCondition);
		break;
	case ECMODE5:
		concealABS_ECMODE5(recfr,predMB,currMBNum,object_list,predBlocks,picSizeX,picSizeY,yCondition);
		break;
	case ECMODE6:
		concealABS_ECMODE6(recfr,predMB,currMBNum,object_list,predBlocks,picSizeX,picSizeY,yCondition);
		break;
	case ECMODE7:
		concealABS_ECMODE7(recfr,predMB,currMBNum,object_list,predBlocks,picSizeX,picSizeY,yCondition);
		break;
	case ECMODE8:
		concealABS_ECMODE8(recfr,predMB,currMBNum,object_list,predBlocks,picSizeX,picSizeY,yCondition);
		break;
	}

	return 0;
}

int find_mb_ecmode(int predBlocks[],int numMBPerLine,int currMBNum)
{
	int i, predMBNum=0, numIntraNeighbours=0, threshold=ERC_BLOCK_CONCEALED, mb_ecmode=ECMODE1; //default mode
	int mode_A=4, mode_L=4, mode_B=4, mode_R=4;
	int mode_A_available=0, mode_L_available=0, mode_B_available=0, mode_R_available=0;

	for (i = 4; i < 8; i++) 
    {
        if (predBlocks[i] >= threshold) 
        {
			switch (i) 
			{
			case 4:
				predMBNum = currMBNum-numMBPerLine;
				mode_A = img->mb_data[predMBNum].mb_type;
				mode_A_available = 1;
				break;              
			case 5:
				predMBNum = currMBNum-1;
				mode_L = img->mb_data[predMBNum].mb_type;
				mode_L_available = 1;
				break;              
			case 6:
				predMBNum = currMBNum+numMBPerLine;
				mode_B = img->mb_data[predMBNum].mb_type;
				mode_B_available = 1;
				break;              
			case 7:
				predMBNum = currMBNum+1;
				mode_R = img->mb_data[predMBNum].mb_type;
				mode_R_available = 1;
				break;
			}
		}
	}

	//Determine if this MB is to be concealed using SEC or TEC
	if(mode_A_available && mode_A>=9)
		numIntraNeighbours++;
	if(mode_L_available && mode_L>=9)
		numIntraNeighbours++;
	if(mode_B_available && mode_B>=9)
		numIntraNeighbours++;
	if(mode_R_available && mode_R>=9)
		numIntraNeighbours++;

	if(numIntraNeighbours==4) 
		return ECMODE1;

	//Determine adaptively the block size for this MB to conceal using the nbr mb_type

	if( (mode_A_available && (mode_A==1 || mode_A==2)) && (mode_B_available && (mode_B==1 || mode_B==2)) &&
		(mode_L_available && (mode_L==1 || mode_L==3)) && (mode_R_available && (mode_R==1 || mode_R==3)) )
	{
		mb_ecmode = ECMODE1;
	}
	else if( (mode_A_available && (mode_A==1 || mode_A==2)) && (mode_B_available && (mode_B==1 || mode_B==2)) &&
		     (mode_L_available && (mode_L==2 || mode_L==8)) && (mode_R_available && (mode_R==2 || mode_R==8)) )
	{
		mb_ecmode = ECMODE2;
	}
	else if( (mode_A_available && (mode_A==3 || mode_A==8)) && (mode_B_available && (mode_B==3 || mode_B==8)) &&
		     (mode_L_available && (mode_L==1 || mode_L==3)) && (mode_R_available && (mode_R==1 || mode_R==3)) )
	{
		mb_ecmode = ECMODE3;
	}
	else if( (mode_A_available && (mode_A==3 || mode_A==8)) && (mode_B_available && (mode_B==3 || mode_B==8)) &&
		     (mode_L_available && (mode_L==2 || mode_L==8)) && (mode_R_available && (mode_R==2 || mode_R==8)) )
	{
		mb_ecmode = ECMODE4;
	}
	else if( (mode_A_available && (mode_A==1 || mode_A==2)) && (mode_B_available && (mode_B==3 || mode_B==8)) &&
		     (mode_L_available && (mode_L==2 || mode_L==8)) && (mode_R_available && (mode_R==2 || mode_R==8)) )
	{
		mb_ecmode = ECMODE5;
	}
	else if( (mode_A_available && (mode_A==3 || mode_A==8)) && (mode_B_available && (mode_B==1 || mode_B==2)) &&
		     (mode_L_available && (mode_L==2 || mode_L==8)) && (mode_R_available && (mode_R==2 || mode_R==8)) )
	{
		mb_ecmode = ECMODE6;
	}
	else if( (mode_A_available && (mode_A==3 || mode_A==8)) && (mode_B_available && (mode_B==3 || mode_B==8)) &&
		     (mode_L_available && (mode_L==2 || mode_L==8)) && (mode_R_available && (mode_R==1 || mode_R==3)) )
	{
		mb_ecmode = ECMODE7;
	}
	else if( (mode_A_available && (mode_A==3 || mode_A==8)) && (mode_B_available && (mode_B==3 || mode_B==8)) &&
		     (mode_L_available && (mode_L==1 || mode_L==3)) && (mode_R_available && (mode_R==2 || mode_R==8)) )
	{
		mb_ecmode = ECMODE8;
	}
	else if( (mode_L_available&&(mode_L==2||mode_L==8)) && (mode_B_available&&(mode_B==3||mode_B==8)) &&
		     (!mode_A_available || mode_A==9 || (mode_A_available&&(mode_A==1||mode_A==2||mode_A==0))) && 
			 (!mode_R_available || mode_R==9 || (mode_R_available&&(mode_R==2||mode_R==8||mode_R==0))) )         
	{
		mb_ecmode = ECMODE5;
	}
	else if( (mode_R_available&&(mode_R==2||mode_R==8)) && (mode_B_available&&(mode_B==3||mode_B==8)) &&
		     (!mode_A_available || mode_A==9 || (mode_A_available&&(mode_A==1||mode_A==2||mode_A==0))) && 
			 (!mode_L_available || mode_L==9 || (mode_L_available&&(mode_L==2||mode_L==8||mode_L==0))) )         
	{
		mb_ecmode = ECMODE5;
	}
	else if( (mode_L_available&&(mode_L==2||mode_L==8)) && (mode_A_available&&(mode_A==3||mode_A==8)) &&
		     (!mode_B_available || mode_B==9 || (mode_B_available&&(mode_B==1||mode_B==2||mode_B==0))) && 
			 (!mode_R_available || mode_R==9 || (mode_R_available&&(mode_R==2||mode_R==8||mode_R==0))) )         
	{
		mb_ecmode = ECMODE6;
	}
	else if( (mode_R_available&&(mode_R==2||mode_R==8)) && (mode_A_available&&(mode_A==3||mode_A==8)) &&
		     (!mode_B_available || mode_B==9 || (mode_B_available&&(mode_B==1||mode_B==2||mode_B==0))) && 
			 (!mode_L_available || mode_L==9 || (mode_L_available&&(mode_L==2||mode_L==8||mode_L==0))) )         
	{
		mb_ecmode = ECMODE6;
	}
	else if( (mode_L_available&&(mode_L==2||mode_L==8)) && (mode_A_available&&(mode_A==3||mode_A==8)) &&
		     (!mode_B_available || mode_B==9 || (mode_B_available&&(mode_B==3||mode_B==8||mode_B==0))) && 
			 (!mode_R_available || mode_R==9 || (mode_R_available&&(mode_R==1||mode_R==3||mode_R==0))) )         
	{
		mb_ecmode = ECMODE7;
	}
	else if( (mode_L_available&&(mode_L==2||mode_L==8)) && (mode_B_available&&(mode_B==3||mode_B==8)) &&
		     (!mode_A_available || mode_A==9 || (mode_A_available&&(mode_A==3||mode_A==8||mode_A==0))) && 
			 (!mode_R_available || mode_R==9 || (mode_R_available&&(mode_R==1||mode_R==3||mode_R==0))) )         
	{
		mb_ecmode = ECMODE7;
	}
	else if( (mode_R_available&&(mode_R==2||mode_R==8)) && (mode_A_available&&(mode_A==3||mode_A==8)) &&
		     (!mode_B_available || mode_B==9 || (mode_B_available&&(mode_B==3||mode_B==8||mode_B==0))) && 
			 (!mode_L_available || mode_L==9 || (mode_L_available&&(mode_L==1||mode_L==3||mode_L==0))) )         
	{
		mb_ecmode = ECMODE8;
	}
	else if( (mode_R_available&&(mode_R==2||mode_R==8)) && (mode_B_available&&(mode_B==3||mode_B==8)) &&
		     (!mode_A_available || mode_A==9 || (mode_A_available&&(mode_A==3||mode_A==8||mode_A==0))) && 
			 (!mode_L_available || mode_L==9 || (mode_L_available&&(mode_L==1||mode_L==3||mode_L==0))) )         
	{
		mb_ecmode = ECMODE8;
	}
	else if( (mode_L_available && (mode_L==2 || mode_L==8)) && (mode_R_available && (mode_R==2 || mode_R==8)) && 
		     (!mode_A_available || mode_A==9 || (mode_A_available&&(mode_A==1||mode_A==2||mode_A==0))) && 
			 (!mode_R_available || mode_R==9 || (mode_R_available&&(mode_R==2||mode_R==8||mode_R==0))) )
	{
		mb_ecmode = ECMODE2;
	}
	else if( (mode_A_available && (mode_A==3 || mode_A==8)) && (mode_B_available && (mode_B==3 || mode_B==8)) &&
		     (!mode_L_available || mode_L==9 || (mode_L_available&&(mode_L==1||mode_L==3||mode_L==0))) && 
			 (!mode_R_available || mode_R==9 || (mode_R_available&&(mode_R==1||mode_R==3||mode_R==0))) )
	{
		mb_ecmode = ECMODE3;
	}

	return mb_ecmode;
	//return ECMODE1;
}

/**********************************************************************************************************************/
//EC Mode 1
static int concealABS_ECMODE1(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition)
{
	//STEPS :- 
	//1. Adaptively decide the block mode for concealment
	//2. MV candidates: MVh, MVv, ZMV, AMV, MMV, PMV, MVd
	//3. OBMA as dsm criterion, use even the concealed internal nbrs i.e. modified OBMA
	//4. OBMC for block compensation - this will be final concealed block
	
	int predMBNum = 0, numMBPerLine, compSplit1 = 0, compSplit2 = 0, compLeft = 1, comp = 0, compPred, order = 1,
        fInterNeighborExists, numIntraNeighbours, fZeroMotionChecked, predSplitted = 0, threshold = ERC_BLOCK_OK,
        minDist, currDist, i, k, bestDir;
	int32 regionSize, mvBest[3] , mvPred[3], *mvptr;
	objectBuffer_t *currRegion;
	
	//Santosh
 	imgpel *predMB_OBMC;
	
	if(OBMC)
		predMB_OBMC = (imgpel *) malloc ( (256 + (img->mb_cr_size_x*img->mb_cr_size_y)*2) * sizeof (imgpel));

	numMBPerLine = (int) (picSizeX>>4);  
	comp = 0;
	regionSize = 16;

	//Get the current region
	currRegion = object_list+(currMBNum<<2); //comp=0, since we are working on whole 16x16 MB

	/* set the position of the region to be concealed */
    
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/*Reliability Loop*/
		minDist = 0; //Minimum Distortion
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;

		/*Looping 4 neighbors*/
		for(i=4;i<8;i++)
		{
			/* If reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 4:
					predMBNum = currMBNum-numMBPerLine;
					compSplit1 = 2;
					compSplit2 = 3;
					break;              
				case 5:
					predMBNum = currMBNum-1;
					compSplit1 = 1;
					compSplit2 = 3;
					break;              
				case 6:
					predMBNum = currMBNum+numMBPerLine;
					compSplit1 = 0;
					compSplit2 = 1;
					break;              
				case 7:
					predMBNum = currMBNum+1;
					compSplit1 = 0;
					compSplit2 = 2;
					break;
				}

				/* Try the concealment with the Motion Info of the current neighbour
				   only if the neighbour is not Intra */
				if(isBlock(object_list,predMBNum,compSplit1,INTRA) || isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				} 
				else
				{
					/* If neighbour MB is splitted, try both neighbour blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0;compPred = compSplit2,predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
							 /* If Zero Motion Block, do the copying. This option is tried only once */
							if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
							{
								if (fZeroMotionChecked) 
								{
									continue;
								}
								else 
								{
									fZeroMotionChecked = 1;
									mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;

									if(OBMA)
										buildOuterPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB, boundary);
									else
										buildPredRegionYUV(erc_img,mvPred,currRegion->xMin,currRegion->yMin,predMB);
								}
							}
							else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
							{
								continue;
							}
							else
							{
								mvptr = getParam(object_list, predMBNum, compPred, mv);                
								mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

								if(OBMA)
									buildOuterPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB, boundary);
								else
									buildPredRegionYUV(erc_img,mvPred,currRegion->xMin,currRegion->yMin,predMB);
							}

							/* Measure absolute boundary pixel difference */
							if(OBMA)
								currDist = edgeDistortionOBMA(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize, boundary);
							else
								currDist = edgeDistortion(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize);
							
							/* If so far best -> store the pixels as the best concealment */
							if (currDist < minDist || !fInterNeighborExists) 
							{
								minDist = currDist;
								bestDir = i;
            
								for (k=0;k<3;k++) 
									mvBest[k] = mvPred[k];
            
								currRegion->regionMode = (isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
														 ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
														 ((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);
								
								if(!OBMC)
									copyPredMB(MBNum2YBlock(currMBNum,comp,picSizeX), predMB, recfr, picSizeX, regionSize);
							}

							fInterNeighborExists = 1;
					}
				}          
			}
		}

		threshold--;
	}while((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));

	/* Always try zero motion */
    if (!fZeroMotionChecked) 
    {
		mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;

		if(OBMA)
			buildOuterPredRegionYUV(erc_img, mvPred, currRegion->xMin, currRegion->yMin, predMB, boundary);
		else
			buildPredRegionYUV(erc_img,mvPred,currRegion->xMin,currRegion->yMin,predMB);

		if(OBMA)
			currDist = edgeDistortionOBMA(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize, boundary);
		else
			currDist = edgeDistortion(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),predMB, recfr->yptr, picSizeX, regionSize);
      
		if (currDist < minDist || !fInterNeighborExists) 
		{
			minDist = currDist;            
			for (k=0;k<3;k++) 
				mvBest[k] = mvPred[k];
        
			currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);
        
			if(!OBMC)
				copyPredMB(MBNum2YBlock(currMBNum,comp,picSizeX), predMB, recfr, picSizeX, regionSize);
		}
	}

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];
    
	//We found the best MV....now do OBMC
	if(OBMC)
	{
		//construct MB with best mv found above
		if(mvBest[0]!=NIL)
		{
			buildPredRegionYUV(erc_img, mvBest, currRegion->xMin, currRegion->yMin, predMB);
		}

		OBMC_MB(predMB,predMB_OBMC,predBlocks,object_list,currMBNum,numMBPerLine,picSizeX);
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB_OBMC, recfr, picSizeX, regionSize);
	}

    yCondition[MBNum2YBlock(currMBNum,0,picSizeX)] = ERC_BLOCK_CONCEALED;

	if(OBMC)
		free(predMB_OBMC);

	return 0;
}

/**********************************************************************************************************************/
//EC Mode 2
static int concealABS_ECMODE2(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition)
{
	imgpel *pred_ecmodeMB, *upper_pred_ecmodeMB, *lower_pred_ecmodeMB;

	int predMBNum = 0, numMBPerLine,
      compSplit1 = 0, compSplit2 = 0, compLeft = 1, comp = 0, compPred, order = 1,
      fInterNeighborExists, numIntraNeighbours,
      fZeroMotionChecked, predSplitted = 0,
      threshold = ERC_BLOCK_OK,
      minDist, currDist, i, k, j, p, bestDir;
	int32 regionSize;
	objectBuffer_t *currRegion;
	int32 mvBest[3] , mvPred[3], *mvptr;

	//Santosh
	imgpel *predMB_OBMC;
	
	//16x8 for luma and 8x4, 8x4 for chroma
	pred_ecmodeMB = (imgpel *) malloc ( (128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));
	upper_pred_ecmodeMB = (imgpel *) malloc ( (128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));
	lower_pred_ecmodeMB = (imgpel *) malloc ( (128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));

	if(OBMC)
		predMB_OBMC = (imgpel *) malloc ( (256 + (img->mb_cr_size_x*img->mb_cr_size_y)*2) * sizeof (imgpel));

	numMBPerLine = (int) (picSizeX>>4);
  	comp = 0;
	regionSize = 16;

	//UPPER HALF
	threshold = ERC_BLOCK_OK;
	comp=0;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 3 neighbours */
		for (i = 4; i < 8; i++) 
		{
   			if(i==6)
				continue;//Below does not exist for upper half

			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 4:
					predMBNum = currMBNum-numMBPerLine;
					compSplit1 = 2;
					compSplit2 = 3;
					break;              
				case 5:
					predMBNum = currMBNum-1;
					compSplit1 = 1;
					compSplit2 = 1;
					break;              
				case 7:
					predMBNum = currMBNum+1;
					compSplit1 = 0;
					compSplit2 = 0;
					break;
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE2(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE2(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE2(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<128+32+32;p++)
							{
								upper_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE2(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);

	  currDist = edgeDistortion_ECMODE2(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<128+32+32;p++)
		{
			upper_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];
    
	//LOWER HALF
	threshold = ERC_BLOCK_OK;
	comp=2;
	currRegion = object_list+(currMBNum<<2)+comp;

	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    
	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;

		/* loop the 3 neighbours */ //(No above for below half)
		for (i = 5; i < 8; i++) 
		{
			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 5:
					predMBNum = currMBNum-1;
					compSplit1 = 3;
					compSplit2 = 3;
					break;   
				case 6:
					predMBNum = currMBNum+numMBPerLine;
					compSplit1 = 0;
					compSplit2 = 1;
					break;
				case 7:
					predMBNum = currMBNum+1;
					compSplit1 = 2;
					compSplit2 = 2;
					break;
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE2(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE2(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE2(predBlocks,MBNum2YBlock(currMBNum,0,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);

						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode = 
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<128+32+32;p++)
							{
								lower_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
						}
              
						fInterNeighborExists = 1;
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE2(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);

	  currDist = edgeDistortion_ECMODE2(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);

      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<128+32+32;p++)
		{
			lower_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
      }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];

	//Whole MB concealed at this stage, copy to predMB

	for(j=0;j<8;j++)
	{
		for(i=0;i<16;i++)
		{
			predMB[j*16+i] = upper_pred_ecmodeMB[j*16+i];
			predMB[(j+8)*16+i] = lower_pred_ecmodeMB[j*16+i];
		}
	}

	for(j=0;j<4;j++)
	{
		for(i=0;i<8;i++)
		{
			predMB[256+j*8+i] = upper_pred_ecmodeMB[128+j*8+i];
			predMB[256+(j+4)*8+i] = lower_pred_ecmodeMB[128+j*8+i];
			predMB[256+64+j*8+i] = upper_pred_ecmodeMB[160+j*8+i];
			predMB[256+64+(j+4)*8+i] = lower_pred_ecmodeMB[160+j*8+i];
		}
	}

	if(OBMC)
	{
		OBMC_MB(predMB,predMB_OBMC,predBlocks,object_list,currMBNum,numMBPerLine,picSizeX);
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB_OBMC, recfr, picSizeX, regionSize);
	}
	else
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB, recfr,picSizeX, regionSize);

	yCondition[MBNum2YBlock(currMBNum,0,picSizeX)] = ERC_BLOCK_CONCEALED;

	free(pred_ecmodeMB);
	free(upper_pred_ecmodeMB);
	free(lower_pred_ecmodeMB);

	if(OBMC)
		free(predMB_OBMC);

	return 0;
}

static void buildOuterPredRegionYUV_ECMODE2(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos)
{
  int tmp_block[BLOCK_SIZE][BLOCK_SIZE];
  int i=0,j=0,ii=0,jj=0,i1=0,j1=0,j4=0,i4=0,k=0;
  int jf=0;
  int uv;
  int vec1_x=0,vec1_y=0;
  int ioff,joff;
  imgpel *pMB = predMB; //Note: predMB is actually pred_ecmodeMB; for ecmode2, it is upper/lower half, so 16x8 for Y and 8x4,8x4 for UV
  
  int ii0,jj0,ii1,jj1,if1,jf1,if0,jf0;
  int mv_mul;

  int above[4], left[4], below[4], right[4], index;
  
  //FRExt
  int f1_x, f1_y, f2_x, f2_y, f3, f4, ifx;
  int b8, b4;
  int yuv = dec_picture->chroma_format_idc - 1;
  
  int ref_frame = max (mv[2], 0); // !!KS: quick fix, we sometimes seem to get negative ref_pic here, so restrict to zero an above

  /* Update coordinates of the current concealed macroblock */
  img->mb_x = x/MB_BLOCK_SIZE;
  img->mb_y = y/MB_BLOCK_SIZE;
  img->block_y = img->mb_y * BLOCK_SIZE;
  img->pix_c_y = img->mb_y * img->mb_cr_size_y;
  img->block_x = img->mb_x * BLOCK_SIZE;
  img->pix_c_x = img->mb_x * img->mb_cr_size_x;

  mv_mul=4;

  // luma *******************************************************

  for(i=0;i<4;i++)
  {
	  above[i]=0;left[i]=0;below[i]=0;right[i]=0;
  }

  //pos = 0, means upper half, 1 means lower half	

  for(j=0;j<MB_BLOCK_SIZE/BLOCK_SIZE;j++)
  {
    joff=j*4;
    j4=img->block_y+j;
    for(i=0;i<MB_BLOCK_SIZE/BLOCK_SIZE;i++)
    {
      ioff=i*4;
      i4=img->block_x+i;

      vec1_x = i4*4*mv_mul + mv[0];
      vec1_y = j4*4*mv_mul + mv[1];

      get_block(ref_frame, listX[0], vec1_x,vec1_y,img,tmp_block);

	  index = i+4*j;
	  get_boundary(ref_frame, listX[0], vec1_x, vec1_y, img, index, above, left, below, right);
	  
	  if(j==0)
	  {
		  boundary[4*i+0]=above[0];boundary[4*i+1]=above[1];boundary[4*i+2]=above[2];boundary[4*i+3]=above[3];
	  }

	  if(i==0)
	  {
		  boundary[16+4*j+0]=left[0];boundary[16+4*j+1]=left[1];boundary[16+4*j+2]=left[2];boundary[16+4*j+3]=left[3];
	  }

	  if(j==3) //MB_BLOCK_SIZE/BLOCK_SIZE - 1 = 3
	  {
		  boundary[32+4*i+0]=below[0];boundary[32+4*i+1]=below[1];boundary[32+4*i+2]=below[2];boundary[32+4*i+3]=below[3];
	  }

	  if(i==3)
	  {
		  boundary[48+4*j+0]=right[0];boundary[48+4*j+1]=right[1];boundary[48+4*j+2]=right[2];boundary[48+4*j+3]=right[3];
	  }

      for(ii=0;ii<BLOCK_SIZE;ii++)
        for(jj=0;jj<MB_BLOCK_SIZE/BLOCK_SIZE;jj++)
          img->mpr[ii+ioff][jj+joff]=tmp_block[ii][jj];
    }
  }

  k=0;
  for(j=0+8*pos;j<8+8*pos;j++)
  {
	  for(i=0;i<16;i++)
	  {
		  pMB[k++] = img->mpr[i][j];
	  }
  }

  pMB += 128;

  if (dec_picture->chroma_format_idc != YUV400)
  {
    // chroma *******************************************************
    f1_x = 64/img->mb_cr_size_x;
    f2_x=f1_x-1;

    f1_y = 64/img->mb_cr_size_y;
    f2_y=f1_y-1;

    f3=f1_x*f1_y;
    f4=f3>>1;

    for(uv=0;uv<2;uv++)
    {
      for (b8=0;b8<(img->num_blk8x8_uv/2);b8++)
      {
        for(b4=0;b4<4;b4++)
		{
          joff = subblk_offset_y[yuv][b8][b4];
          j4=img->pix_c_y+joff;
          ioff = subblk_offset_x[yuv][b8][b4];
          i4=img->pix_c_x+ioff;

          for(jj=0;jj<4;jj++)
          {
            jf=(j4+jj)/(img->mb_cr_size_y/4);     // jf  = Subblock_y-coordinate
            for(ii=0;ii<4;ii++)
            {
              ifx=(i4+ii)/(img->mb_cr_size_x/4);  // ifx = Subblock_x-coordinate

              i1=(i4+ii)*f1_x + mv[0];
              j1=(j4+jj)*f1_y + mv[1];
            
              ii0=max (0, min (i1/f1_x,   dec_picture->size_x_cr-1));
              jj0=max (0, min (j1/f1_y,   dec_picture->size_y_cr-1));
              ii1=max (0, min ((i1+f2_x)/f1_x, dec_picture->size_x_cr-1));
              jj1=max (0, min ((j1+f2_y)/f1_y, dec_picture->size_y_cr-1));
            
              if1=(i1 & f2_x);
              jf1=(j1 & f2_y);
              if0=f1_x-if1;
              jf0=f1_y-jf1;
            
              img->mpr[ii+ioff][jj+joff]=(if0*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii0]+
                                          if1*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii1]+
                                          if0*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii0]+
                                          if1*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii1]+f4)/f3;
            }
          }
        }
      }

	  k=0;
	  for(j=0+4*pos;j<4+4*pos;j++)
	  {
		  for(i=0;i<8;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 32;

    }
  }
}

static int edgeDistortion_ECMODE2 (int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos)
{
  int i, j, distortion, numOfPredBlocks, threshold = ERC_BLOCK_OK;
  imgpel *currBlock = NULL, *neighbor = NULL;
  int32 currBlockOffset = 0;
  
  currBlock = recY + (yPosYBlock(currYBlockNum,picSizeX)<<3)*picSizeX + (xPosYBlock(currYBlockNum,picSizeX)<<3);
  
  do 
  {    
    distortion = 0; numOfPredBlocks = 0;
    
    /* loop the 4 neighbours */
    for (j = 4; j < 8; j++) 
    {
		if(pos==0 && j==6)
			continue;
		if(pos==1 && j==4)
			continue;

	  /* if reliable, count boundary pixel difference */
      if (predBlocks[j] >= threshold) 
      {        
        switch (j) 
        {
        case 4:
		  neighbor = currBlock - picSizeX;
		  if(OBMA)
		  {
			  for ( i = 0; i < regionSize; i++ ) 
				distortion += mabs((int)(boundary[i] - neighbor[i]));
          }
		  else
		  {
			  for ( i = 0; i < regionSize; i++ ) 
				distortion += mabs((int)(predMB[i] - neighbor[i]));
		  }
		  break;          
        case 5:
          neighbor = currBlock - 1;
		  if(OBMA)
		  {
			  for ( i = 0+8*pos; i < 8+8*pos; i++ ) 
				distortion += mabs((int)(boundary[16+i] - neighbor[i*picSizeX]));
		  }
		  else
		  {
			  for ( i = 0+8*pos; i < 8+8*pos; i++ ) 
	            distortion += mabs((int)(predMB[(i-8*pos)*16] - neighbor[i*picSizeX]));
		  }
          break;                
        case 6:
          neighbor = currBlock + regionSize*picSizeX;
          currBlockOffset = 16*7;
		  if(OBMA)
		  {
			  for ( i = 0; i < regionSize; i++ ) 
				distortion += mabs((int)(boundary[32+i] - neighbor[i]));
		  }
		  else
		  {
			  for ( i = 0; i < regionSize; i++ ) 
				distortion += mabs((int)(predMB[i+currBlockOffset] - neighbor[i]));
		  }
          break;                
        case 7:
          neighbor = currBlock + regionSize;
          currBlockOffset = regionSize-1;
		  if(OBMA)
		  {
			  for ( i = 0+8*pos; i < 8+8*pos; i++ ) 
				distortion += mabs((int)(boundary[48+i] - neighbor[i*picSizeX]));
		  }
		  else
		  {
			  for ( i = 0+8*pos; i < 8+8*pos; i++ ) 
				distortion += mabs((int)(predMB[(i-8*pos)*16+currBlockOffset] - neighbor[i*picSizeX]));
		  }
          break;
        }
        
        numOfPredBlocks++;
      }
    }
    
    threshold--;
    if (threshold < ERC_BLOCK_CONCEALED)
      break;
  } while (numOfPredBlocks == 0);
  
  if(numOfPredBlocks == 0)
    return 0;

  return (distortion/numOfPredBlocks);
}

/**********************************************************************************************************************/
//EC Mode 3
static int concealABS_ECMODE3(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition)
{
	imgpel *pred_ecmodeMB, *left_pred_ecmodeMB, *right_pred_ecmodeMB;

	int predMBNum = 0, numMBPerLine,
      compSplit1 = 0, compSplit2 = 0, compLeft = 1, comp = 0, compPred, order = 1,
      fInterNeighborExists, numIntraNeighbours,
      fZeroMotionChecked, predSplitted = 0,
      threshold = ERC_BLOCK_OK,
      minDist, currDist, i, k, j, p, bestDir;
	int32 regionSize;
	objectBuffer_t *currRegion;
	int32 mvBest[3] , mvPred[3], *mvptr;

	//Santosh
	imgpel *predMB_OBMC;
	
	//8x16 for luma and 4x8, 4x8 for chroma
	pred_ecmodeMB = (imgpel *) malloc ( (128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));
	left_pred_ecmodeMB = (imgpel *) malloc ( (128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));
	right_pred_ecmodeMB = (imgpel *) malloc ( (128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));

	if(OBMC)
		predMB_OBMC = (imgpel *) malloc ( (256 + (img->mb_cr_size_x*img->mb_cr_size_y)*2) * sizeof (imgpel));

	numMBPerLine = (int) (picSizeX>>4);
  	comp = 0;
	regionSize = 16;

	//LEFT HALF
	threshold = ERC_BLOCK_OK;
	comp=0;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 3 neighbours */
		for (i = 4; i < 7; i++) //No right nbr for left half
		{
			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 4:
					predMBNum = currMBNum-numMBPerLine;
					compSplit1 = 2;
					compSplit2 = 2;
					break;              
				case 5:
					predMBNum = currMBNum-1;
					compSplit1 = 1;
					compSplit2 = 3;
					break;              
				case 6:
					predMBNum = currMBNum+numMBPerLine;
					compSplit1 = 0;
					compSplit2 = 0;
					break;
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE3(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE3(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE3(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<128+32+32;p++)
							{
								left_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE3(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);

	  currDist = edgeDistortion_ECMODE3(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<128+32+32;p++)
		{
			left_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];
    
	//RIGHT HALF
	threshold = ERC_BLOCK_OK;
	comp=1;
	currRegion = object_list+(currMBNum<<2)+comp;

	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    
	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;

		/* loop the 3 neighbours */ 
		for (i = 4; i < 8; i++) 
		{
			if(i==5) //No left nbr for right half
				continue;

			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 4:
					predMBNum = currMBNum-numMBPerLine;
					compSplit1 = 3;
					compSplit2 = 3;
					break;   
				case 6:
					predMBNum = currMBNum+numMBPerLine;
					compSplit1 = 1;
					compSplit2 = 1;
					break;
				case 7:
					predMBNum = currMBNum+1;
					compSplit1 = 0;
					compSplit2 = 2;
					break;
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE3(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE3(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE3(predBlocks,MBNum2YBlock(currMBNum,0,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);

						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode = 
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<128+32+32;p++)
							{
								right_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}

							p=0;                
						}
              
						fInterNeighborExists = 1;
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE3(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);

	  currDist = edgeDistortion_ECMODE3(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);

      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<128+32+32;p++)
		{
			right_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
      }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];

	//Whole MB concealed at this stage, copy to predMB

	p=0;
	for(j=0;j<16;j++)
	{
		for(i=0;i<8;i++)
		{
			predMB[j*16+i] = left_pred_ecmodeMB[p];
			predMB[j*16+i+8] = right_pred_ecmodeMB[p];
			p++;
		}
	}

	p=0;
	for(j=0;j<8;j++)
	{
		for(i=0;i<4;i++)
		{
			predMB[256+j*8+i] = left_pred_ecmodeMB[128+p];
			predMB[256+j*8+i+4] = right_pred_ecmodeMB[128+p];
			predMB[256+64+j*8+i] = left_pred_ecmodeMB[160+p];
			predMB[256+64+j*8+i+4] = right_pred_ecmodeMB[160+p];
			p++;
		}
	}
	
	if(OBMC)
	{
		OBMC_MB(predMB,predMB_OBMC,predBlocks,object_list,currMBNum,numMBPerLine,picSizeX);
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB_OBMC, recfr, picSizeX, regionSize);
	}
	else
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB, recfr,picSizeX, regionSize);

	yCondition[MBNum2YBlock(currMBNum,0,picSizeX)] = ERC_BLOCK_CONCEALED;

	free(pred_ecmodeMB);
	free(left_pred_ecmodeMB);
	free(right_pred_ecmodeMB);

	if(OBMC)
		free(predMB_OBMC);

	return 0;
}

static void buildOuterPredRegionYUV_ECMODE3(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos)
{
  int tmp_block[BLOCK_SIZE][BLOCK_SIZE];
  int i=0,j=0,ii=0,jj=0,i1=0,j1=0,j4=0,i4=0,k=0;
  int jf=0;
  int uv;
  int vec1_x=0,vec1_y=0;
  int ioff,joff;
  imgpel *pMB = predMB; 
  
  int ii0,jj0,ii1,jj1,if1,jf1,if0,jf0;
  int mv_mul;

  int above[4], left[4], below[4], right[4], index;
  
  //FRExt
  int f1_x, f1_y, f2_x, f2_y, f3, f4, ifx;
  int b8, b4;
  int yuv = dec_picture->chroma_format_idc - 1;
  
  int ref_frame = max (mv[2], 0); // !!KS: quick fix, we sometimes seem to get negative ref_pic here, so restrict to zero an above

  /* Update coordinates of the current concealed macroblock */
  img->mb_x = x/MB_BLOCK_SIZE;
  img->mb_y = y/MB_BLOCK_SIZE;
  img->block_y = img->mb_y * BLOCK_SIZE;
  img->pix_c_y = img->mb_y * img->mb_cr_size_y;
  img->block_x = img->mb_x * BLOCK_SIZE;
  img->pix_c_x = img->mb_x * img->mb_cr_size_x;

  mv_mul=4;

  // luma *******************************************************

  for(i=0;i<4;i++)
  {
	  above[i]=0;left[i]=0;below[i]=0;right[i]=0;
  }

  for(j=0;j<MB_BLOCK_SIZE/BLOCK_SIZE;j++)
  {
    joff=j*4;
    j4=img->block_y+j;
    for(i=0;i<MB_BLOCK_SIZE/BLOCK_SIZE;i++)
    {
      ioff=i*4;
      i4=img->block_x+i;

      vec1_x = i4*4*mv_mul + mv[0];
      vec1_y = j4*4*mv_mul + mv[1];

      get_block(ref_frame, listX[0], vec1_x,vec1_y,img,tmp_block);

	  index = i+4*j;
	  get_boundary(ref_frame, listX[0], vec1_x, vec1_y, img, index, above, left, below, right);
	  
	  if(j==0)
	  {
		  boundary[4*i+0]=above[0];boundary[4*i+1]=above[1];boundary[4*i+2]=above[2];boundary[4*i+3]=above[3];
	  }

	  if(i==0)
	  {
		  boundary[16+4*j+0]=left[0];boundary[16+4*j+1]=left[1];boundary[16+4*j+2]=left[2];boundary[16+4*j+3]=left[3];
	  }

	  if(j==3) //MB_BLOCK_SIZE/BLOCK_SIZE - 1 = 3
	  {
		  boundary[32+4*i+0]=below[0];boundary[32+4*i+1]=below[1];boundary[32+4*i+2]=below[2];boundary[32+4*i+3]=below[3];
	  }

	  if(i==3)
	  {
		  boundary[48+4*j+0]=right[0];boundary[48+4*j+1]=right[1];boundary[48+4*j+2]=right[2];boundary[48+4*j+3]=right[3];
	  }

      for(ii=0;ii<BLOCK_SIZE;ii++)
        for(jj=0;jj<MB_BLOCK_SIZE/BLOCK_SIZE;jj++)
          img->mpr[ii+ioff][jj+joff]=tmp_block[ii][jj];
    }
  }

  k=0;
  for(j=0;j<16;j++)
  {
	  for(i=0+8*pos;i<8+8*pos;i++)
	  {
		  pMB[k++] = img->mpr[i][j];
	  }
  }

  pMB += 128;

  if (dec_picture->chroma_format_idc != YUV400)
  {
    // chroma *******************************************************
    f1_x = 64/img->mb_cr_size_x;
    f2_x=f1_x-1;

    f1_y = 64/img->mb_cr_size_y;
    f2_y=f1_y-1;

    f3=f1_x*f1_y;
    f4=f3>>1;

    for(uv=0;uv<2;uv++)
    {
      for (b8=0;b8<(img->num_blk8x8_uv/2);b8++)
      {
        for(b4=0;b4<4;b4++)
		{
          joff = subblk_offset_y[yuv][b8][b4];
          j4=img->pix_c_y+joff;
          ioff = subblk_offset_x[yuv][b8][b4];
          i4=img->pix_c_x+ioff;

          for(jj=0;jj<4;jj++)
          {
            jf=(j4+jj)/(img->mb_cr_size_y/4);     // jf  = Subblock_y-coordinate
            for(ii=0;ii<4;ii++)
            {
              ifx=(i4+ii)/(img->mb_cr_size_x/4);  // ifx = Subblock_x-coordinate

              i1=(i4+ii)*f1_x + mv[0];
              j1=(j4+jj)*f1_y + mv[1];
            
              ii0=max (0, min (i1/f1_x,   dec_picture->size_x_cr-1));
              jj0=max (0, min (j1/f1_y,   dec_picture->size_y_cr-1));
              ii1=max (0, min ((i1+f2_x)/f1_x, dec_picture->size_x_cr-1));
              jj1=max (0, min ((j1+f2_y)/f1_y, dec_picture->size_y_cr-1));
            
              if1=(i1 & f2_x);
              jf1=(j1 & f2_y);
              if0=f1_x-if1;
              jf0=f1_y-jf1;
            
              img->mpr[ii+ioff][jj+joff]=(if0*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii0]+
                                          if1*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii1]+
                                          if0*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii0]+
                                          if1*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii1]+f4)/f3;
            }
          }
        }
      }

	  k=0;
	  for(j=0;j<8;j++)
	  {
		  for(i=0+4*pos;i<4+4*pos;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 32;

    }
  }
}

static int edgeDistortion_ECMODE3 (int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos)
{
  int i, j, distortion, numOfPredBlocks, threshold = ERC_BLOCK_OK;
  imgpel *currBlock = NULL, *neighbor = NULL;
  int32 currBlockOffset = 0;
  
  currBlock = recY + (yPosYBlock(currYBlockNum,picSizeX)<<3)*picSizeX + (xPosYBlock(currYBlockNum,picSizeX)<<3);
  
  do 
  {    
    distortion = 0; numOfPredBlocks = 0;
    
    /* loop the 4 neighbours */
    for (j = 4; j < 8; j++) 
    {
		if(pos==0 && j==7)
			continue;
		if(pos==1 && j==5)
			continue;

	  /* if reliable, count boundary pixel difference */
      if (predBlocks[j] >= threshold) 
      {        
        switch (j) 
        {
        case 4:
		  neighbor = currBlock - picSizeX;
		  if(OBMA)
		  {
			  for ( i = 0+8*pos; i < 8+8*pos; i++ ) 
				distortion += mabs((int)(boundary[i] - neighbor[i]));
		  }
		  else
		  {
			  for ( i = 0+8*pos; i < 8+8*pos; i++ ) 
				distortion += mabs((int)(predMB[i-8*pos] - neighbor[i]));
		  }
		  break;          
        case 5:
          neighbor = currBlock - 1;
          if(OBMA)
		  {
			for ( i = 0; i < regionSize; i++ ) 
				distortion += mabs((int)(boundary[16+i] - neighbor[i*picSizeX]));
		  }
		  else
		  {
			  for ( i = 0; i < regionSize; i++ ) 
				distortion += mabs((int)(predMB[i*8] - neighbor[i*picSizeX]));
		  }
          break;                
        case 6:
          neighbor = currBlock + regionSize*picSizeX;
          currBlockOffset = 15*8;
          if(OBMA)
		  {
			for ( i = 0+8*pos; i < 8+8*pos; i++ ) 
				distortion += mabs((int)(boundary[32+i] - neighbor[i]));
		  }
		  else
		  {
			  for ( i = 0+8*pos; i < 8+8*pos; i++ ) 
				distortion += mabs((int)(predMB[(i-8*pos)+currBlockOffset] - neighbor[i]));
		  }
          break;                
        case 7:
          neighbor = currBlock + regionSize;
          currBlockOffset = 7;
          if(OBMA)
		  {
			for ( i = 0; i < regionSize; i++ ) 
				distortion += mabs((int)(boundary[48+i] - neighbor[i*picSizeX]));
		  }
		  else
		  {
			   for ( i = 0; i < regionSize; i++ ) 
				distortion += mabs((int)(predMB[i*8+currBlockOffset] - neighbor[i*picSizeX]));
		  }
          break;
        }
        
        numOfPredBlocks++;
      }
    }
    
    threshold--;
    if (threshold < ERC_BLOCK_CONCEALED)
      break;
  } while (numOfPredBlocks == 0);
  
  if(numOfPredBlocks == 0)
    return 0;
  
  return (distortion/numOfPredBlocks);
}

/**********************************************************************************************************************/
//EC Mode 4
static int concealABS_ECMODE4(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition)
{
	imgpel *pred_ecmodeMB, *topleft_pred_ecmodeMB, *topright_pred_ecmodeMB, *bottomleft_pred_ecmodeMB, *bottomright_pred_ecmodeMB;

	int predMBNum = 0, numMBPerLine,
      compSplit1 = 0, compSplit2 = 0, compLeft = 1, comp = 0, compPred, order = 1,
      fInterNeighborExists, numIntraNeighbours,
      fZeroMotionChecked, predSplitted = 0,
      threshold = ERC_BLOCK_OK,
      minDist, currDist, i, k, j, p, bestDir;
	int32 regionSize;
	objectBuffer_t *currRegion;
	int32 mvBest[3] , mvPred[3], *mvptr;

	//Santosh
 	imgpel *predMB_OBMC;
	
	if(OBMC)
		predMB_OBMC = (imgpel *) malloc ( (256 + (img->mb_cr_size_x*img->mb_cr_size_y)*2) * sizeof (imgpel));

	//8x8 for luma and 4x4, 4x4 for chroma
	pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	topleft_pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	topright_pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	bottomleft_pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	bottomright_pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));

	numMBPerLine = (int) (picSizeX>>4);
  	comp = 0;
	regionSize = 16;

	if(currMBNum==88)
		currMBNum=88;

	//TOP LEFT
	threshold = ERC_BLOCK_OK;
	comp=0;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 2 neighbours */
		for (i = 4; i < 6; i++) //only above and left nbrs
		{
			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 4:
					predMBNum = currMBNum-numMBPerLine;
					compSplit1 = 2;
					compSplit2 = 2;
					break;              
				case 5:
					predMBNum = currMBNum-1;
					compSplit1 = 1;
					compSplit2 = 1;
					break;              
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE4(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE4(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE4(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<64+16+16;p++)
							{
								topleft_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE4(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);

	  currDist = edgeDistortion_ECMODE4(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<64+16+16;p++)
		{
			topleft_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];


	//TOP RIGHT
	threshold = ERC_BLOCK_OK;
	comp=1;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 2 neighbours */
		for (i = 4; i < 8; i++) //only above and right nbrs
		{
			if(i==5 || i==6)
				continue;

			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 4:
					predMBNum = currMBNum-numMBPerLine;
					compSplit1 = 3;
					compSplit2 = 3;
					break;              
				case 7:
					predMBNum = currMBNum+1;
					compSplit1 = 0;
					compSplit2 = 0;
					break;              
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE4(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE4(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE4(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<64+16+16;p++)
							{
								topright_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE4(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);

	  currDist = edgeDistortion_ECMODE4(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<64+16+16;p++)
		{
			topright_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];
    

	//BOTTOM LEFT
	threshold = ERC_BLOCK_OK;
	comp=2;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 2 neighbours */
		for (i = 5; i < 7; i++) //only left and below nbrs
		{
			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 5:
					predMBNum = currMBNum-1;
					compSplit1 = 3;
					compSplit2 = 3;
					break;  
				case 6:
					predMBNum = currMBNum+numMBPerLine;
					compSplit1 = 0;
					compSplit2 = 0;
					break;
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE4(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,2);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE4(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,2);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE4(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 2);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<64+16+16;p++)
							{
								bottomleft_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE4(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,2);

	  currDist = edgeDistortion_ECMODE4(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 2);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<64+16+16;p++)
		{
			bottomleft_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];


	//BOTTOM RIGHT
	threshold = ERC_BLOCK_OK;
	comp=3;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 2 neighbours */
		for (i = 6; i < 8; i++) //only below and right nbrs
		{
			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 6:
					predMBNum = currMBNum+numMBPerLine;
					compSplit1 = 1;
					compSplit2 = 1;
					break;             
				case 7:
					predMBNum = currMBNum+1;
					compSplit1 = 2;
					compSplit2 = 2;
					break;              
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE4(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,3);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE4(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,3);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE4(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 3);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<64+16+16;p++)
							{
								bottomright_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE4(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,3);

	  currDist = edgeDistortion_ECMODE4(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 3);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<64+16+16;p++)
		{
			bottomright_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];


	//Whole MB concealed at this stage, copy to predMB

	p=0;
	for(j=0;j<8;j++)
	{
		for(i=0;i<8;i++)
		{
			predMB[j*16+i] = topleft_pred_ecmodeMB[p];
			predMB[j*16+i+8] = topright_pred_ecmodeMB[p];
			predMB[(j+8)*16+i] = bottomleft_pred_ecmodeMB[p];
			predMB[(j+8)*16+i+8] = bottomright_pred_ecmodeMB[p];
			p++;
		}
	}

	p=0;
	for(j=0;j<4;j++)
	{
		for(i=0;i<4;i++)
		{
			predMB[256+j*8+i] = topleft_pred_ecmodeMB[64+p];
			predMB[256+j*8+i+4] = topright_pred_ecmodeMB[64+p];
			predMB[256+(j+4)*8+i] = bottomleft_pred_ecmodeMB[64+p];
			predMB[256+(j+4)*8+i+4] = bottomright_pred_ecmodeMB[64+p];

			predMB[256+64+j*8+i] = topleft_pred_ecmodeMB[80+p];
			predMB[256+64+j*8+i+4] = topright_pred_ecmodeMB[80+p];
			predMB[256+64+(j+4)*8+i] = bottomleft_pred_ecmodeMB[80+p];
			predMB[256+64+(j+4)*8+i+4] = bottomright_pred_ecmodeMB[80+p];

			p++;
		}
	}

	if(OBMC)
	{
		OBMC_MB(predMB,predMB_OBMC,predBlocks,object_list,currMBNum,numMBPerLine,picSizeX);
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB_OBMC, recfr, picSizeX, regionSize);
	}
	else
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB, recfr,picSizeX, regionSize);
	
	yCondition[MBNum2YBlock(currMBNum,0,picSizeX)] = ERC_BLOCK_CONCEALED;

	free(pred_ecmodeMB);
	free(topleft_pred_ecmodeMB);
	free(topright_pred_ecmodeMB);
	free(bottomleft_pred_ecmodeMB);
	free(bottomright_pred_ecmodeMB);

	if(OBMC)
		free(predMB_OBMC);

	return 0;
}

static void buildOuterPredRegionYUV_ECMODE4(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos)
{
  int tmp_block[BLOCK_SIZE][BLOCK_SIZE];
  int i=0,j=0,ii=0,jj=0,i1=0,j1=0,j4=0,i4=0,k=0;
  int jf=0;
  int uv;
  int vec1_x=0,vec1_y=0;
  int ioff,joff;
  imgpel *pMB = predMB; //Note: predMB is actually pred_ecmodeMB; for ecmode2, it is left/right half, so 8x16 for Y and 4x8,4x8 for UV
  
  int ii0,jj0,ii1,jj1,if1,jf1,if0,jf0;
  int mv_mul;

  int above[4], left[4], below[4], right[4], index;
  
  //FRExt
  int f1_x, f1_y, f2_x, f2_y, f3, f4, ifx;
  int b8, b4;
  int yuv = dec_picture->chroma_format_idc - 1;
  
  int ref_frame = max (mv[2], 0); // !!KS: quick fix, we sometimes seem to get negative ref_pic here, so restrict to zero an above

  /* Update coordinates of the current concealed macroblock */
  img->mb_x = x/MB_BLOCK_SIZE;
  img->mb_y = y/MB_BLOCK_SIZE;
  img->block_y = img->mb_y * BLOCK_SIZE;
  img->pix_c_y = img->mb_y * img->mb_cr_size_y;
  img->block_x = img->mb_x * BLOCK_SIZE;
  img->pix_c_x = img->mb_x * img->mb_cr_size_x;

  mv_mul=4;

  // luma *******************************************************

  for(i=0;i<4;i++)
  {
	  above[i]=0;left[i]=0;below[i]=0;right[i]=0;
  }

  for(j=0;j<MB_BLOCK_SIZE/BLOCK_SIZE;j++)
  {
    joff=j*4;
    j4=img->block_y+j;
    for(i=0;i<MB_BLOCK_SIZE/BLOCK_SIZE;i++)
    {
      ioff=i*4;
      i4=img->block_x+i;

      vec1_x = i4*4*mv_mul + mv[0];
      vec1_y = j4*4*mv_mul + mv[1];

      get_block(ref_frame, listX[0], vec1_x,vec1_y,img,tmp_block);

	  index = i+4*j;
	  get_boundary(ref_frame, listX[0], vec1_x, vec1_y, img, index, above, left, below, right);
	  
	  if(j==0)
	  {
		  boundary[4*i+0]=above[0];boundary[4*i+1]=above[1];boundary[4*i+2]=above[2];boundary[4*i+3]=above[3];
	  }

	  if(i==0)
	  {
		  boundary[16+4*j+0]=left[0];boundary[16+4*j+1]=left[1];boundary[16+4*j+2]=left[2];boundary[16+4*j+3]=left[3];
	  }

	  if(j==3) //MB_BLOCK_SIZE/BLOCK_SIZE - 1 = 3
	  {
		  boundary[32+4*i+0]=below[0];boundary[32+4*i+1]=below[1];boundary[32+4*i+2]=below[2];boundary[32+4*i+3]=below[3];
	  }

	  if(i==3)
	  {
		  boundary[48+4*j+0]=right[0];boundary[48+4*j+1]=right[1];boundary[48+4*j+2]=right[2];boundary[48+4*j+3]=right[3];
	  }

      for(ii=0;ii<BLOCK_SIZE;ii++)
        for(jj=0;jj<MB_BLOCK_SIZE/BLOCK_SIZE;jj++)
          img->mpr[ii+ioff][jj+joff]=tmp_block[ii][jj];
    }
  }

  switch(pos)
  {
  case 0:
	  k=0;
	  for(j=0;j<8;j++)
	  {
		  for(i=0;i<8;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  break;
  case 1:
	  k=0;
	  for(j=0;j<8;j++)
	  {
		  for(i=8;i<16;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  break;
  case 2:
	  k=0;
	  for(j=8;j<16;j++)
	  {
		  for(i=0;i<8;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  break;
  case 3:
	  k=0;
	  for(j=8;j<16;j++)
	  {
		  for(i=8;i<16;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  break;
  }

  pMB += 64;

  if (dec_picture->chroma_format_idc != YUV400)
  {
    // chroma *******************************************************
    f1_x = 64/img->mb_cr_size_x;
    f2_x=f1_x-1;

    f1_y = 64/img->mb_cr_size_y;
    f2_y=f1_y-1;

    f3=f1_x*f1_y;
    f4=f3>>1;

    for(uv=0;uv<2;uv++)
    {
      for (b8=0;b8<(img->num_blk8x8_uv/2);b8++)
      {
        for(b4=0;b4<4;b4++)
		{
          joff = subblk_offset_y[yuv][b8][b4];
          j4=img->pix_c_y+joff;
          ioff = subblk_offset_x[yuv][b8][b4];
          i4=img->pix_c_x+ioff;

          for(jj=0;jj<4;jj++)
          {
            jf=(j4+jj)/(img->mb_cr_size_y/4);     // jf  = Subblock_y-coordinate
            for(ii=0;ii<4;ii++)
            {
              ifx=(i4+ii)/(img->mb_cr_size_x/4);  // ifx = Subblock_x-coordinate

              i1=(i4+ii)*f1_x + mv[0];
              j1=(j4+jj)*f1_y + mv[1];
            
              ii0=max (0, min (i1/f1_x,   dec_picture->size_x_cr-1));
              jj0=max (0, min (j1/f1_y,   dec_picture->size_y_cr-1));
              ii1=max (0, min ((i1+f2_x)/f1_x, dec_picture->size_x_cr-1));
              jj1=max (0, min ((j1+f2_y)/f1_y, dec_picture->size_y_cr-1));
            
              if1=(i1 & f2_x);
              jf1=(j1 & f2_y);
              if0=f1_x-if1;
              jf0=f1_y-jf1;
            
              img->mpr[ii+ioff][jj+joff]=(if0*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii0]+
                                          if1*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii1]+
                                          if0*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii0]+
                                          if1*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii1]+f4)/f3;
            }
          }
        }
      }

	  switch(pos)
	  {
	  case 0:
		  k=0;
		  for(j=0;j<4;j++)
		  {
			  for(i=0;i<4;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  break;
	  case 1:
		  k=0;
		  for(j=0;j<4;j++)
		  {
			  for(i=4;i<8;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  break;
	  case 2:
		  k=0;
		  for(j=4;j<8;j++)
		  {
			  for(i=0;i<4;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  break;
	  case 3:
		  k=0;
		  for(j=4;j<8;j++)
		  {
			  for(i=4;i<8;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  break;
	  }

	  pMB += 16;
    }
  }
}

static int edgeDistortion_ECMODE4(int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos)
{
  int i, j, distortion, numOfPredBlocks, threshold = ERC_BLOCK_OK;
  imgpel *currBlock = NULL, *neighbor = NULL;
  int32 currBlockOffset = 0;
  
  currBlock = recY + (yPosYBlock(currYBlockNum,picSizeX)<<3)*picSizeX + (xPosYBlock(currYBlockNum,picSizeX)<<3);
  
  do 
  {    
    distortion = 0; numOfPredBlocks = 0;
    
    /* loop the 2 neighbours */
    for (j = 4; j < 8; j++) 
    {
		if(pos==0 && (j==6 || j==7))
			continue;
		if(pos==1 && (j==5 || j==6))
			continue;
		if(pos==2 && (j==4 || j==7))
			continue;
		if(pos==3 && (j==4 || j==5))
			continue;

	  /* if reliable, count boundary pixel difference */
      if (predBlocks[j] >= threshold) 
      {
        
        switch (j) 
        {
        case 4:
		  neighbor = currBlock - picSizeX;
		  if(OBMA)
		  {
			for(i=0+8*pos;i<8+8*pos;i++) //pos = 0 or 1 for above
				distortion += mabs((int)(boundary[i] - neighbor[i]));
		  }
		  else
		  {
			  for ( i=0+8*pos;i<8+8*pos;i++ ) 
	            distortion += mabs((int)(predMB[i-8*pos] - neighbor[i]));
    	  }
		  break;          
        case 5:
          neighbor = currBlock - 1;
		  if(OBMA)
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(boundary[16+i] - neighbor[i*picSizeX]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(boundary[16+i] - neighbor[i*picSizeX]));
			  }
		  }
		  else
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(predMB[i*8] - neighbor[i*picSizeX]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(predMB[(i-8)*8] - neighbor[i*picSizeX]));
			  }
		  }		  
          break;                
        case 6:
          neighbor = currBlock + regionSize*picSizeX;
          currBlockOffset = 8*7;
		  if(OBMA)
		  {
			  if(pos==2)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(boundary[32+i] - neighbor[i]));
			  }
			  else if(pos==3)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(boundary[32+i] - neighbor[i]));
			  }
		  }
		  else
		  {
			  if(pos==2)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(predMB[i+currBlockOffset] - neighbor[i]));
			  }
			  else if(pos==3)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(predMB[i-8+currBlockOffset] - neighbor[i]));
			  }
		  }
          break;                
        case 7:
          neighbor = currBlock + regionSize;
          currBlockOffset = 7;
		  if(OBMA)
		  {
			  if(pos==1)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(boundary[48+i] - neighbor[i*picSizeX]));
			  }
			  else if(pos==3)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(boundary[48+i] - neighbor[i*picSizeX]));
			  }
		  }
		  else
		  {
			  if(pos==1)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(predMB[i*8+currBlockOffset] - neighbor[i*picSizeX]));
			  }
			  else if(pos==3)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(predMB[(i-8)*8+currBlockOffset] - neighbor[i*picSizeX]));
			  }
		  }
          break;
        }
        
        numOfPredBlocks++;
      }
    }
    
    threshold--;
    if (threshold < ERC_BLOCK_CONCEALED)
      break;
  } while (numOfPredBlocks == 0);
  
  if(numOfPredBlocks == 0)
    return 0;

  return (distortion/numOfPredBlocks);
}

/**********************************************************************************************************************/
//EC Mode 5
static int concealABS_ECMODE5(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition)
{
	imgpel *pred_above_ecmodeMB, *pred_ecmodeMB, *top_pred_ecmodeMB, *bottomleft_pred_ecmodeMB, *bottomright_pred_ecmodeMB;

	int predMBNum = 0, numMBPerLine,
      compSplit1 = 0, compSplit2 = 0, compLeft = 1, comp = 0, compPred, order = 1,
      fInterNeighborExists, numIntraNeighbours,
      fZeroMotionChecked, predSplitted = 0,
      threshold = ERC_BLOCK_OK,
      minDist, currDist, i, k, j, p, q, bestDir;
	int32 regionSize;
	objectBuffer_t *currRegion;
	int32 mvBest[3] , mvPred[3], *mvptr;

	//Santosh
 	imgpel *predMB_OBMC;
	
	if(OBMC)
		predMB_OBMC = (imgpel *) malloc ( (256 + (img->mb_cr_size_x*img->mb_cr_size_y)*2) * sizeof (imgpel));

	//16x8 for luma and 8x4, 8x4 for chroma
	pred_above_ecmodeMB = (imgpel *) malloc( ( 128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));
	top_pred_ecmodeMB = (imgpel *) malloc( ( 128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));

	//8x8 for luma and 4x4, 4x4 for chroma
	pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	bottomleft_pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	bottomright_pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));

	numMBPerLine = (int) (picSizeX>>4);
  	comp = 0;
	regionSize = 16;

	//UPPER HALF
	threshold = ERC_BLOCK_OK;
	comp=0;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 3 neighbours */
		for (i = 4; i < 8; i++) //no below nbr for upper half
		{
			if(i==6)
				continue;

			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 4:
					predMBNum = currMBNum-numMBPerLine;
					compSplit1 = 2;
					compSplit2 = 3;
					break;              
				case 5:
					predMBNum = currMBNum-1;
					compSplit1 = 1;
					compSplit2 = 1;
					break; 
				case 7:
					predMBNum = currMBNum+1;
					compSplit1 = 0;
					compSplit2 = 0;
					break;
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE5(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_above_ecmodeMB, boundary,0);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE5(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_above_ecmodeMB, boundary,0);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE5(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_above_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<128+32+32;p++)
							{
								top_pred_ecmodeMB[p] = pred_above_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE5(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_above_ecmodeMB, boundary,0);

	  currDist = edgeDistortion_ECMODE5(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_above_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<128+32+32;p++)
		{
			top_pred_ecmodeMB[p] = pred_above_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];
    

	//BOTTOM LEFT
	threshold = ERC_BLOCK_OK;
	comp=2;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 2 neighbours */
		for (i = 5; i < 7; i++) //only left and below nbrs
		{
			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 5:
					predMBNum = currMBNum-1;
					compSplit1 = 3;
					compSplit2 = 3;
					break;  
				case 6:
					predMBNum = currMBNum+numMBPerLine;
					compSplit1 = 0;
					compSplit2 = 0;
					break;
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE5(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE5(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE5(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<64+16+16;p++)
							{
								bottomleft_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE5(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);

	  currDist = edgeDistortion_ECMODE5(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<64+16+16;p++)
		{
			bottomleft_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];


	//BOTTOM RIGHT
	threshold = ERC_BLOCK_OK;
	comp=3;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 2 neighbours */
		for (i = 6; i < 8; i++) //only below and right nbrs
		{
			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 6:
					predMBNum = currMBNum+numMBPerLine;
					compSplit1 = 1;
					compSplit2 = 1;
					break;             
				case 7:
					predMBNum = currMBNum+1;
					compSplit1 = 2;
					compSplit2 = 2;
					break;              
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE5(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,2);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE5(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,2);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE5(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 2);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<64+16+16;p++)
							{
								bottomright_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE5(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,2);

	  currDist = edgeDistortion_ECMODE5(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 2);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<64+16+16;p++)
		{
			bottomright_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];


	//Whole MB concealed at this stage, copy to predMB

	p=0; q=0;
	for(j=0;j<8;j++)
	{
		for(i=0;i<16;i++)
		{
			predMB[j*16+i] = top_pred_ecmodeMB[p];
			p++;
		}
		for(i=0;i<8;i++)
		{
			predMB[(j+8)*16+i] = bottomleft_pred_ecmodeMB[q];
			predMB[(j+8)*16+i+8] = bottomright_pred_ecmodeMB[q];
			q++;
		}
	}

	p=0;q=0;
	for(j=0;j<4;j++)
	{
		for(i=0;i<8;i++)
		{
			predMB[256+j*8+i] = top_pred_ecmodeMB[128+p];
			predMB[256+64+j*8+i] = top_pred_ecmodeMB[128+32+p];
			p++;
		}
		for(i=0;i<4;i++)
		{
			predMB[256+(j+4)*8+i] = bottomleft_pred_ecmodeMB[64+q];
			predMB[256+(j+4)*8+i+4] = bottomright_pred_ecmodeMB[64+q];
			predMB[256+64+(j+4)*8+i] = bottomleft_pred_ecmodeMB[80+q];
			predMB[256+64+(j+4)*8+i+4] = bottomright_pred_ecmodeMB[80+q];
			q++;
		}
	}
	
	if(OBMC)
	{
		OBMC_MB(predMB,predMB_OBMC,predBlocks,object_list,currMBNum,numMBPerLine,picSizeX);
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB_OBMC, recfr, picSizeX, regionSize);
	}
	else
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB, recfr,picSizeX, regionSize);
	
	yCondition[MBNum2YBlock(currMBNum,0,picSizeX)] = ERC_BLOCK_CONCEALED;

	free(pred_ecmodeMB);
	free(pred_above_ecmodeMB);
	free(top_pred_ecmodeMB);
	free(bottomleft_pred_ecmodeMB);
	free(bottomright_pred_ecmodeMB);

	if(OBMC)
		free(predMB_OBMC);

	return 0;
}

static void buildOuterPredRegionYUV_ECMODE5(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos)
{
  int tmp_block[BLOCK_SIZE][BLOCK_SIZE];
  int i=0,j=0,ii=0,jj=0,i1=0,j1=0,j4=0,i4=0,k=0;
  int jf=0;
  int uv;
  int vec1_x=0,vec1_y=0;
  int ioff,joff;
  imgpel *pMB = predMB; //Note: predMB is actually pred_ecmodeMB; for ecmode2, it is left/right half, so 8x16 for Y and 4x8,4x8 for UV
  
  int ii0,jj0,ii1,jj1,if1,jf1,if0,jf0;
  int mv_mul;

  int above[4], left[4], below[4], right[4], index;
  
  //FRExt
  int f1_x, f1_y, f2_x, f2_y, f3, f4, ifx;
  int b8, b4;
  int yuv = dec_picture->chroma_format_idc - 1;
  
  int ref_frame = max (mv[2], 0); // !!KS: quick fix, we sometimes seem to get negative ref_pic here, so restrict to zero an above

  /* Update coordinates of the current concealed macroblock */
  img->mb_x = x/MB_BLOCK_SIZE;
  img->mb_y = y/MB_BLOCK_SIZE;
  img->block_y = img->mb_y * BLOCK_SIZE;
  img->pix_c_y = img->mb_y * img->mb_cr_size_y;
  img->block_x = img->mb_x * BLOCK_SIZE;
  img->pix_c_x = img->mb_x * img->mb_cr_size_x;

  mv_mul=4;

  // luma *******************************************************

  for(i=0;i<4;i++)
  {
	  above[i]=0;left[i]=0;below[i]=0;right[i]=0;
  }

  for(j=0;j<MB_BLOCK_SIZE/BLOCK_SIZE;j++)
  {
    joff=j*4;
    j4=img->block_y+j;
    for(i=0;i<MB_BLOCK_SIZE/BLOCK_SIZE;i++)
    {
      ioff=i*4;
      i4=img->block_x+i;

      vec1_x = i4*4*mv_mul + mv[0];
      vec1_y = j4*4*mv_mul + mv[1];

      get_block(ref_frame, listX[0], vec1_x,vec1_y,img,tmp_block);

	  index = i+4*j;
	  get_boundary(ref_frame, listX[0], vec1_x, vec1_y, img, index, above, left, below, right);
	  
	  if(j==0)
	  {
		  boundary[4*i+0]=above[0];boundary[4*i+1]=above[1];boundary[4*i+2]=above[2];boundary[4*i+3]=above[3];
	  }

	  if(i==0)
	  {
		  boundary[16+4*j+0]=left[0];boundary[16+4*j+1]=left[1];boundary[16+4*j+2]=left[2];boundary[16+4*j+3]=left[3];
	  }

	  if(j==3) //MB_BLOCK_SIZE/BLOCK_SIZE - 1 = 3
	  {
		  boundary[32+4*i+0]=below[0];boundary[32+4*i+1]=below[1];boundary[32+4*i+2]=below[2];boundary[32+4*i+3]=below[3];
	  }

	  if(i==3)
	  {
		  boundary[48+4*j+0]=right[0];boundary[48+4*j+1]=right[1];boundary[48+4*j+2]=right[2];boundary[48+4*j+3]=right[3];
	  }

      for(ii=0;ii<BLOCK_SIZE;ii++)
        for(jj=0;jj<MB_BLOCK_SIZE/BLOCK_SIZE;jj++)
          img->mpr[ii+ioff][jj+joff]=tmp_block[ii][jj];
    }
  }

  switch(pos)
  {
  case 0:
	  k=0;
	  for(j=0;j<8;j++)
	  {
		  for(i=0;i<16;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 128;
	  break;
  case 1:
	  k=0;
	  for(j=8;j<16;j++)
	  {
		  for(i=0;i<8;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 64;
	  break;
  case 2:
	  k=0;
	  for(j=8;j<16;j++)
	  {
		  for(i=8;i<16;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 64;
	  break;
  }

  if (dec_picture->chroma_format_idc != YUV400)
  {
    // chroma *******************************************************
    f1_x = 64/img->mb_cr_size_x;
    f2_x=f1_x-1;

    f1_y = 64/img->mb_cr_size_y;
    f2_y=f1_y-1;

    f3=f1_x*f1_y;
    f4=f3>>1;

    for(uv=0;uv<2;uv++)
    {
      for (b8=0;b8<(img->num_blk8x8_uv/2);b8++)
      {
        for(b4=0;b4<4;b4++)
		{
          joff = subblk_offset_y[yuv][b8][b4];
          j4=img->pix_c_y+joff;
          ioff = subblk_offset_x[yuv][b8][b4];
          i4=img->pix_c_x+ioff;

          for(jj=0;jj<4;jj++)
          {
            jf=(j4+jj)/(img->mb_cr_size_y/4);     // jf  = Subblock_y-coordinate
            for(ii=0;ii<4;ii++)
            {
              ifx=(i4+ii)/(img->mb_cr_size_x/4);  // ifx = Subblock_x-coordinate

              i1=(i4+ii)*f1_x + mv[0];
              j1=(j4+jj)*f1_y + mv[1];
            
              ii0=max (0, min (i1/f1_x,   dec_picture->size_x_cr-1));
              jj0=max (0, min (j1/f1_y,   dec_picture->size_y_cr-1));
              ii1=max (0, min ((i1+f2_x)/f1_x, dec_picture->size_x_cr-1));
              jj1=max (0, min ((j1+f2_y)/f1_y, dec_picture->size_y_cr-1));
            
              if1=(i1 & f2_x);
              jf1=(j1 & f2_y);
              if0=f1_x-if1;
              jf0=f1_y-jf1;
            
              img->mpr[ii+ioff][jj+joff]=(if0*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii0]+
                                          if1*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii1]+
                                          if0*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii0]+
                                          if1*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii1]+f4)/f3;
            }
          }
        }
      }

	  switch(pos)
	  {
	  case 0:
		  k=0;
		  for(j=0;j<4;j++)
		  {
			  for(i=0;i<8;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  pMB += 32;
		  break;
	  case 1:
		  k=0;
		  for(j=4;j<8;j++)
		  {
			  for(i=0;i<4;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  pMB += 16;
		  break;
	  case 2:
		  k=0;
		  for(j=4;j<8;j++)
		  {
			  for(i=4;i<8;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  pMB += 16;
		  break;
	  }
    }
  }
}

static int edgeDistortion_ECMODE5(int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos)
{
  int i, j, distortion, numOfPredBlocks, threshold = ERC_BLOCK_OK;
  imgpel *currBlock = NULL, *neighbor = NULL;
  int32 currBlockOffset = 0;
  
  currBlock = recY + (yPosYBlock(currYBlockNum,picSizeX)<<3)*picSizeX + (xPosYBlock(currYBlockNum,picSizeX)<<3);
  
  do 
  {    
    distortion = 0; numOfPredBlocks = 0;
    
    /* loop the 2/3 neighbours */
    for (j = 4; j < 8; j++) 
    {
		if(pos==0 && j==6)
			continue;
		if(pos==1 && (j==4 || j==7))
			continue;
		if(pos==2 && (j==4 || j==5))
			continue;

	  /* if reliable, count boundary pixel difference */
      if (predBlocks[j] >= threshold) 
      {        
        switch (j) 
        {
        case 4:
		  neighbor = currBlock - picSizeX;
		  if(OBMA)
		  {
			for ( i = 0; i < regionSize; i++ ) 
				distortion += mabs((int)(boundary[i] - neighbor[i]));
		  }
		  else
		  {
			  for ( i = 0; i < regionSize; i++ ) 
				distortion += mabs((int)(predMB[i] - neighbor[i]));
		  }
		  break;          
        case 5:
          neighbor = currBlock - 1;
		  if(OBMA)
		  {
			for ( i = 0+8*pos; i < 8+8*pos; i++ ) 
				distortion += mabs((int)(boundary[16+i] - neighbor[i*picSizeX]));
		  }
		  else
		  {
			  if(pos==0)
			  {
				  for ( i = 0; i < 8; i++ ) 
					distortion += mabs((int)(predMB[i*16] - neighbor[i*picSizeX]));
			  }
			  else if(pos==1)
			  {
				  for ( i = 8; i < 16; i++ ) 
					distortion += mabs((int)(predMB[(i-8)*8] - neighbor[i*picSizeX]));
			  }
		  }
          break;                
        case 6:
          neighbor = currBlock + regionSize*picSizeX;
          currBlockOffset = 7*8;
		  if(OBMA)
		  {
			  if(pos==1)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(boundary[32+i] - neighbor[i]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(boundary[32+i] - neighbor[i]));
			  }
		  }
		  else
		  {
			  if(pos==1)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(predMB[i+currBlockOffset] - neighbor[i]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(predMB[i-8+currBlockOffset] - neighbor[i]));
			  }
		  }
          break;                
        case 7:
          neighbor = currBlock + regionSize;
          currBlockOffset = 7;
		  if(OBMA)
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(boundary[48+i] - neighbor[i*picSizeX]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(boundary[48+i] - neighbor[i*picSizeX]));
			  }
		  }
		  else
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(predMB[i*8+currBlockOffset] - neighbor[i*picSizeX]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(predMB[(i-8)*8+currBlockOffset] - neighbor[i*picSizeX]));
			  }
		  }
          break;
        }
        
        numOfPredBlocks++;
      }
    }
    
    threshold--;
    if (threshold < ERC_BLOCK_CONCEALED)
      break;
  } while (numOfPredBlocks == 0);
  
  if(numOfPredBlocks == 0)
    return 0;

  return (distortion/numOfPredBlocks);
}

/**********************************************************************************************************************/
//EC Mode 6
static int concealABS_ECMODE6(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition)
{
	imgpel *pred_bottom_ecmodeMB, *pred_ecmodeMB, *bottom_pred_ecmodeMB, *topleft_pred_ecmodeMB, *topright_pred_ecmodeMB;

	int predMBNum = 0, numMBPerLine,
      compSplit1 = 0, compSplit2 = 0, compLeft = 1, comp = 0, compPred, order = 1,
      fInterNeighborExists, numIntraNeighbours,
      fZeroMotionChecked, predSplitted = 0,
      threshold = ERC_BLOCK_OK,
      minDist, currDist, i, k, j, p, q, bestDir;
	int32 regionSize;
	objectBuffer_t *currRegion;
	int32 mvBest[3] , mvPred[3], *mvptr;

 	imgpel *predMB_OBMC;
	
	if(OBMC)
		predMB_OBMC = (imgpel *) malloc ( (256 + (img->mb_cr_size_x*img->mb_cr_size_y)*2) * sizeof (imgpel));

	//16x8 for luma and 8x4, 8x4 for chroma
	pred_bottom_ecmodeMB = (imgpel *) malloc( ( 128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));
	bottom_pred_ecmodeMB = (imgpel *) malloc( ( 128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));

	//8x8 for luma and 4x4, 4x4 for chroma
	pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	topleft_pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	topright_pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));

	numMBPerLine = (int) (picSizeX>>4);
  	comp = 0;
	regionSize = 16;

	//TOP LEFT
	threshold = ERC_BLOCK_OK;
	comp=0;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 2 neighbours */
		for (i = 4; i < 6; i++) //only above and left nbrs
		{
			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 4:
					predMBNum = currMBNum-numMBPerLine;
					compSplit1 = 2;
					compSplit2 = 2;
					break;
				case 5:
					predMBNum = currMBNum-1;
					compSplit1 = 1;
					compSplit2 = 1;
					break;  
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE6(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE6(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE6(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<64+16+16;p++)
							{
								topleft_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE6(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);

	  currDist = edgeDistortion_ECMODE6(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<64+16+16;p++)
		{
			topleft_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];


	//TOP RIGHT
	threshold = ERC_BLOCK_OK;
	comp=1;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 2 neighbours */
		for (i = 4; i < 8; i++) //only above and right nbrs
		{
			if(i==5 || i==6)
				continue;

			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				 case 4:
					predMBNum = currMBNum-numMBPerLine;
					compSplit1 = 3;
					compSplit2 = 3;
					break;          
				case 7:
					predMBNum = currMBNum+1;
					compSplit1 = 0;
					compSplit2 = 0;
					break;              
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE6(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE6(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE6(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<64+16+16;p++)
							{
								topright_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE6(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);

	  currDist = edgeDistortion_ECMODE6(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<64+16+16;p++)
		{
			topright_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];


	//LOWER HALF
	threshold = ERC_BLOCK_OK;
	comp=2;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 3 neighbours */
		for (i = 5; i < 8; i++) //no above nbr for lower half
		{
			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 5:
					predMBNum = currMBNum-1;
					compSplit1 = 3;
					compSplit2 = 3;
					break; 
				case 6:
					predMBNum = currMBNum+numMBPerLine;
					compSplit1 = 0;
					compSplit2 = 1;
					break;
				case 7:
					predMBNum = currMBNum+1;
					compSplit1 = 2;
					compSplit2 = 2;
					break;
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE6(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_bottom_ecmodeMB, boundary,2);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE6(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_bottom_ecmodeMB, boundary,2);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE6(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_bottom_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 2);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<128+32+32;p++)
							{
								bottom_pred_ecmodeMB[p] = pred_bottom_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE6(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_bottom_ecmodeMB, boundary,2);

	  currDist = edgeDistortion_ECMODE6(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_bottom_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 2);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<128+32+32;p++)
		{
			bottom_pred_ecmodeMB[p] = pred_bottom_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];
    

	//Whole MB concealed at this stage, copy to predMB

	p=0; q=0;
	for(j=0;j<8;j++)
	{
		for(i=0;i<8;i++)
		{
			predMB[j*16+i] = topleft_pred_ecmodeMB[q];
			predMB[j*16+i+8] = topright_pred_ecmodeMB[q];
			q++;
		}
		for(i=0;i<16;i++)
		{
			predMB[(j+8)*16+i] = bottom_pred_ecmodeMB[p];
			p++;
		}		
	}

	p=0;q=0;
	for(j=0;j<4;j++)
	{
		for(i=0;i<4;i++)
		{
			predMB[256+j*8+i] = topleft_pred_ecmodeMB[64+q];
			predMB[256+j*8+i+4] = topright_pred_ecmodeMB[64+q];
			predMB[256+64+j*8+i] = topleft_pred_ecmodeMB[80+q];
			predMB[256+64+j*8+i+4] = topright_pred_ecmodeMB[80+q];
			q++;
		}
		for(i=0;i<8;i++)
		{
			predMB[256+(j+4)*8+i] = bottom_pred_ecmodeMB[128+p];
			predMB[256+64+(j+4)*8+i] = bottom_pred_ecmodeMB[128+32+p];
			p++;
		}		
	}
	
	if(OBMC)
	{
		OBMC_MB(predMB,predMB_OBMC,predBlocks,object_list,currMBNum,numMBPerLine,picSizeX);
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB_OBMC, recfr, picSizeX, regionSize);
	}
	else
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB, recfr,picSizeX, regionSize);

	yCondition[MBNum2YBlock(currMBNum,0,picSizeX)] = ERC_BLOCK_CONCEALED;

	free(pred_ecmodeMB);
	free(pred_bottom_ecmodeMB);
	free(bottom_pred_ecmodeMB);
	free(topleft_pred_ecmodeMB);
	free(topright_pred_ecmodeMB);

	if(OBMC)
		free(predMB_OBMC);

	return 0;
}

static void buildOuterPredRegionYUV_ECMODE6(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos)
{
  int tmp_block[BLOCK_SIZE][BLOCK_SIZE];
  int i=0,j=0,ii=0,jj=0,i1=0,j1=0,j4=0,i4=0,k=0;
  int jf=0;
  int uv;
  int vec1_x=0,vec1_y=0;
  int ioff,joff;
  imgpel *pMB = predMB; //Note: predMB is actually pred_ecmodeMB; for ecmode2, it is left/right half, so 8x16 for Y and 4x8,4x8 for UV
  
  int ii0,jj0,ii1,jj1,if1,jf1,if0,jf0;
  int mv_mul;

  int above[4], left[4], below[4], right[4], index;
  
  //FRExt
  int f1_x, f1_y, f2_x, f2_y, f3, f4, ifx;
  int b8, b4;
  int yuv = dec_picture->chroma_format_idc - 1;
  
  int ref_frame = max (mv[2], 0); // !!KS: quick fix, we sometimes seem to get negative ref_pic here, so restrict to zero an above

  /* Update coordinates of the current concealed macroblock */
  img->mb_x = x/MB_BLOCK_SIZE;
  img->mb_y = y/MB_BLOCK_SIZE;
  img->block_y = img->mb_y * BLOCK_SIZE;
  img->pix_c_y = img->mb_y * img->mb_cr_size_y;
  img->block_x = img->mb_x * BLOCK_SIZE;
  img->pix_c_x = img->mb_x * img->mb_cr_size_x;

  mv_mul=4;

  // luma *******************************************************

  for(i=0;i<4;i++)
  {
	  above[i]=0;left[i]=0;below[i]=0;right[i]=0;
  }

  for(j=0;j<MB_BLOCK_SIZE/BLOCK_SIZE;j++)
  {
    joff=j*4;
    j4=img->block_y+j;
    for(i=0;i<MB_BLOCK_SIZE/BLOCK_SIZE;i++)
    {
      ioff=i*4;
      i4=img->block_x+i;

      vec1_x = i4*4*mv_mul + mv[0];
      vec1_y = j4*4*mv_mul + mv[1];

      get_block(ref_frame, listX[0], vec1_x,vec1_y,img,tmp_block);

	  index = i+4*j;
	  get_boundary(ref_frame, listX[0], vec1_x, vec1_y, img, index, above, left, below, right);
	  
	  if(j==0)
	  {
		  boundary[4*i+0]=above[0];boundary[4*i+1]=above[1];boundary[4*i+2]=above[2];boundary[4*i+3]=above[3];
	  }

	  if(i==0)
	  {
		  boundary[16+4*j+0]=left[0];boundary[16+4*j+1]=left[1];boundary[16+4*j+2]=left[2];boundary[16+4*j+3]=left[3];
	  }

	  if(j==3) //MB_BLOCK_SIZE/BLOCK_SIZE - 1 = 3
	  {
		  boundary[32+4*i+0]=below[0];boundary[32+4*i+1]=below[1];boundary[32+4*i+2]=below[2];boundary[32+4*i+3]=below[3];
	  }

	  if(i==3)
	  {
		  boundary[48+4*j+0]=right[0];boundary[48+4*j+1]=right[1];boundary[48+4*j+2]=right[2];boundary[48+4*j+3]=right[3];
	  }

      for(ii=0;ii<BLOCK_SIZE;ii++)
        for(jj=0;jj<MB_BLOCK_SIZE/BLOCK_SIZE;jj++)
          img->mpr[ii+ioff][jj+joff]=tmp_block[ii][jj];
    }
  }

  switch(pos)
  {
  case 0:
	  k=0;
	  for(j=0;j<8;j++)
	  {
		  for(i=0;i<8;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 64;
	  break;
  case 1:
	  k=0;
	  for(j=0;j<8;j++)
	  {
		  for(i=8;i<16;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 64;
	  break;
  case 2:
	  k=0;
	  for(j=8;j<16;j++)
	  {
		  for(i=0;i<16;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 128;
	  break;
  }

  if (dec_picture->chroma_format_idc != YUV400)
  {
    // chroma *******************************************************
    f1_x = 64/img->mb_cr_size_x;
    f2_x=f1_x-1;

    f1_y = 64/img->mb_cr_size_y;
    f2_y=f1_y-1;

    f3=f1_x*f1_y;
    f4=f3>>1;

    for(uv=0;uv<2;uv++)
    {
      for (b8=0;b8<(img->num_blk8x8_uv/2);b8++)
      {
        for(b4=0;b4<4;b4++)
		{
          joff = subblk_offset_y[yuv][b8][b4];
          j4=img->pix_c_y+joff;
          ioff = subblk_offset_x[yuv][b8][b4];
          i4=img->pix_c_x+ioff;

          for(jj=0;jj<4;jj++)
          {
            jf=(j4+jj)/(img->mb_cr_size_y/4);     // jf  = Subblock_y-coordinate
            for(ii=0;ii<4;ii++)
            {
              ifx=(i4+ii)/(img->mb_cr_size_x/4);  // ifx = Subblock_x-coordinate

              i1=(i4+ii)*f1_x + mv[0];
              j1=(j4+jj)*f1_y + mv[1];
            
              ii0=max (0, min (i1/f1_x,   dec_picture->size_x_cr-1));
              jj0=max (0, min (j1/f1_y,   dec_picture->size_y_cr-1));
              ii1=max (0, min ((i1+f2_x)/f1_x, dec_picture->size_x_cr-1));
              jj1=max (0, min ((j1+f2_y)/f1_y, dec_picture->size_y_cr-1));
            
              if1=(i1 & f2_x);
              jf1=(j1 & f2_y);
              if0=f1_x-if1;
              jf0=f1_y-jf1;
            
              img->mpr[ii+ioff][jj+joff]=(if0*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii0]+
                                          if1*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii1]+
                                          if0*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii0]+
                                          if1*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii1]+f4)/f3;
            }
          }
        }
      }

	  switch(pos)
	  {
	  case 0:
		  k=0;
		  for(j=0;j<4;j++)
		  {
			  for(i=0;i<4;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  pMB += 16;
		  break;
	  case 1:
		  k=0;
		  for(j=0;j<4;j++)
		  {
			  for(i=4;i<8;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  pMB += 16;
		  break;
	  case 2:
		  k=0;
		  for(j=4;j<8;j++)
		  {
			  for(i=0;i<8;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  pMB += 32;
		  break;
	  }
    }
  }
}

static int edgeDistortion_ECMODE6(int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos)
{
  int i, j, distortion, numOfPredBlocks, threshold = ERC_BLOCK_OK;
  imgpel *currBlock = NULL, *neighbor = NULL;
  int32 currBlockOffset = 0;
  
  currBlock = recY + (yPosYBlock(currYBlockNum,picSizeX)<<3)*picSizeX + (xPosYBlock(currYBlockNum,picSizeX)<<3);
  
  do 
  {    
    distortion = 0; numOfPredBlocks = 0;
    
    /* loop the 2/3 neighbours */
    for (j = 4; j < 8; j++) 
    {
		if(pos==0 && (j==6 || j==7))
			continue;
		if(pos==1 && (j==5 || j==6))
			continue;
		if(pos==2 && j==4)
			continue;

	  /* if reliable, count boundary pixel difference */
      if (predBlocks[j] >= threshold) 
      {        
        switch (j) 
        {
        case 4:
		  neighbor = currBlock - picSizeX;
		  if(OBMA)
		  {
			  for ( i = 0+8*pos; i < 8+8*pos; i++ ) 
				distortion += mabs((int)(boundary[i] - neighbor[i]));
		  }
		  else
		  {
			  for ( i = 0+8*pos; i < 8+8*pos; i++ ) 
				distortion += mabs((int)(predMB[i-8*pos] - neighbor[i]));
		  }
		  break;          
        case 5:
          neighbor = currBlock - 1;
		  if(OBMA)
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(boundary[16+i] - neighbor[i*picSizeX]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(boundary[16+i] - neighbor[i*picSizeX]));
			  }
		  }
		  else
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(predMB[i*8] - neighbor[i*picSizeX]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(predMB[(i-8)*16] - neighbor[i*picSizeX]));
			  }
		  }
          break;                
        case 6:
          neighbor = currBlock + regionSize*picSizeX;
          currBlockOffset = 7*16;
		  if(OBMA)
		  {
			  for(i=0;i<regionSize;i++)
				  distortion += mabs((int)(boundary[32+i] - neighbor[i]));
		  }
		  else
		  {
			  for(i=0;i<regionSize;i++)
				  distortion += mabs((int)(predMB[i+currBlockOffset] - neighbor[i]));
		  }
		  break;                
        case 7:
          neighbor = currBlock + regionSize;
          currBlockOffset = regionSize-1;
		  if(OBMA)
		  {
			  if(pos==1)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(boundary[48+i] - neighbor[i*picSizeX]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(boundary[48+i] - neighbor[i*picSizeX]));
			  }
		  }
		  else
		  {
			  if(pos==1)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(predMB[i*8+7] - neighbor[i*picSizeX]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(predMB[(i-8)*16+currBlockOffset] - neighbor[i*picSizeX]));
			  }
		  }
          break;
        }
        
        numOfPredBlocks++;
      }
    }
    
    threshold--;
    if (threshold < ERC_BLOCK_CONCEALED)
      break;
  } while (numOfPredBlocks == 0);
  
  if(numOfPredBlocks == 0)
    return 0;

  return (distortion/numOfPredBlocks);
}

/**********************************************************************************************************************/
//EC Mode 7
static int concealABS_ECMODE7(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition)
{
	imgpel *pred_right_ecmodeMB, *pred_ecmodeMB, *right_pred_ecmodeMB, *topleft_pred_ecmodeMB, *bottomleft_pred_ecmodeMB;

	int predMBNum = 0, numMBPerLine,
      compSplit1 = 0, compSplit2 = 0, compLeft = 1, comp = 0, compPred, order = 1,
      fInterNeighborExists, numIntraNeighbours,
      fZeroMotionChecked, predSplitted = 0,
      threshold = ERC_BLOCK_OK,
      minDist, currDist, i, k, j, p, q, bestDir;
	int32 regionSize;
	objectBuffer_t *currRegion;
	int32 mvBest[3] , mvPred[3], *mvptr;
	imgpel *predMB_OBMC;
	
	if(OBMC)
		predMB_OBMC = (imgpel *) malloc ( (256 + (img->mb_cr_size_x*img->mb_cr_size_y)*2) * sizeof (imgpel));

	//8x8 for luma and 4x4, 4x4 for chroma
	pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	topleft_pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	bottomleft_pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));

	//8x16 for luma and 4x8, 4x8 for chroma
	pred_right_ecmodeMB = (imgpel *) malloc( ( 128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));
	right_pred_ecmodeMB = (imgpel *) malloc( ( 128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));
	
	numMBPerLine = (int) (picSizeX>>4);
  	comp = 0;
	regionSize = 16;

	//TOP LEFT
	threshold = ERC_BLOCK_OK;
	comp=0;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 2 neighbours */
		for (i = 4; i < 6; i++) //only above and left nbrs
		{
			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 4:
					predMBNum = currMBNum-numMBPerLine;
					compSplit1 = 2;
					compSplit2 = 2;
					break;
				case 5:
					predMBNum = currMBNum-1;
					compSplit1 = 1;
					compSplit2 = 1;
					break;  
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE7(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE7(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE7(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<64+16+16;p++)
							{
								topleft_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE7(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,0);

	  currDist = edgeDistortion_ECMODE7(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<64+16+16;p++)
		{
			topleft_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];


	//BOTTOM LEFT
	threshold = ERC_BLOCK_OK;
	comp=2;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 2 neighbours */
		for (i = 5; i < 7; i++) //only left and below nbrs
		{
			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 5:
						predMBNum = currMBNum-1;
						compSplit1 = 3;
						compSplit2 = 3;
						break;
				case 6:
						predMBNum = currMBNum+numMBPerLine;
						compSplit1 = 0;
						compSplit2 = 0;
						break;
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE7(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE7(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE7(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<64+16+16;p++)
							{
								bottomleft_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE7(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);

	  currDist = edgeDistortion_ECMODE7(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<64+16+16;p++)
		{
			bottomleft_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];


	//RIGHT HALF
	threshold = ERC_BLOCK_OK;
	comp=1;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 3 neighbours */
		for (i = 4; i < 8; i++) //no left nbr for right half
		{
			if(i==5)
				continue;

			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 4:
					predMBNum = currMBNum-numMBPerLine;
					compSplit1 = 3;
					compSplit2 = 3;
					break;
				case 6:
					predMBNum = currMBNum+numMBPerLine;
					compSplit1 = 1;
					compSplit2 = 1;
					break;
				case 7:
					predMBNum = currMBNum+1;
					compSplit1 = 0;
					compSplit2 = 2;
					break;
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE7(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_right_ecmodeMB, boundary,2);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE7(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_right_ecmodeMB, boundary,2);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE7(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_right_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 2);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<128+32+32;p++)
							{
								right_pred_ecmodeMB[p] = pred_right_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE7(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_right_ecmodeMB, boundary,2);

	  currDist = edgeDistortion_ECMODE7(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_right_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 2);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<128+32+32;p++)
		{
			right_pred_ecmodeMB[p] = pred_right_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];
    

	//Whole MB concealed at this stage, copy to predMB

	p=0;q=0;
	for(j=0;j<8;j++)
	{
		for(i=0;i<8;i++)
		{
			predMB[j*16+i] = topleft_pred_ecmodeMB[p];
			predMB[(j+8)*16+i] = bottomleft_pred_ecmodeMB[p];
			p++;
		}
	}
	for(j=0;j<16;j++)
	{
		for(i=8;i<16;i++)
		{
			predMB[j*16+i] = right_pred_ecmodeMB[q];
			q++;
		}
	}

	p=0;q=0;
	for(j=0;j<4;j++)
	{
		for(i=0;i<4;i++)
		{
			predMB[256+j*8+i] = topleft_pred_ecmodeMB[64+p];
			predMB[256+(j+4)*8+i] = bottomleft_pred_ecmodeMB[64+p];
			predMB[256+64+j*8+i] = topleft_pred_ecmodeMB[80+p];
			predMB[256+64+(j+4)*8+i] = bottomleft_pred_ecmodeMB[80+p];
			p++;
		}
	}
	for(j=0;j<8;j++)
	{
		for(i=4;i<8;i++)
		{
			predMB[256+j*8+i] = right_pred_ecmodeMB[128+q];
			predMB[256+64+j*8+i] = right_pred_ecmodeMB[128+32+q];
			q++;
		}
	}

	if(OBMC)
	{
		OBMC_MB(predMB,predMB_OBMC,predBlocks,object_list,currMBNum,numMBPerLine,picSizeX);
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB_OBMC, recfr, picSizeX, regionSize);
	}
	else
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB, recfr,picSizeX, regionSize);
	
	yCondition[MBNum2YBlock(currMBNum,0,picSizeX)] = ERC_BLOCK_CONCEALED;

	free(pred_ecmodeMB);
	free(pred_right_ecmodeMB);
	free(right_pred_ecmodeMB);
	free(topleft_pred_ecmodeMB);
	free(bottomleft_pred_ecmodeMB);
	if(OBMC)
		free(predMB_OBMC);

	return 0;
}

static void buildOuterPredRegionYUV_ECMODE7(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos)
{
  int tmp_block[BLOCK_SIZE][BLOCK_SIZE];
  int i=0,j=0,ii=0,jj=0,i1=0,j1=0,j4=0,i4=0,k=0;
  int jf=0;
  int uv;
  int vec1_x=0,vec1_y=0;
  int ioff,joff;
  imgpel *pMB = predMB; //Note: predMB is actually pred_ecmodeMB; for ecmode2, it is left/right half, so 8x16 for Y and 4x8,4x8 for UV
  
  int ii0,jj0,ii1,jj1,if1,jf1,if0,jf0;
  int mv_mul;

  int above[4], left[4], below[4], right[4], index;
  
  //FRExt
  int f1_x, f1_y, f2_x, f2_y, f3, f4, ifx;
  int b8, b4;
  int yuv = dec_picture->chroma_format_idc - 1;
  
  int ref_frame = max (mv[2], 0); // !!KS: quick fix, we sometimes seem to get negative ref_pic here, so restrict to zero an above

  /* Update coordinates of the current concealed macroblock */
  img->mb_x = x/MB_BLOCK_SIZE;
  img->mb_y = y/MB_BLOCK_SIZE;
  img->block_y = img->mb_y * BLOCK_SIZE;
  img->pix_c_y = img->mb_y * img->mb_cr_size_y;
  img->block_x = img->mb_x * BLOCK_SIZE;
  img->pix_c_x = img->mb_x * img->mb_cr_size_x;

  mv_mul=4;

  // luma *******************************************************

  for(i=0;i<4;i++)
  {
	  above[i]=0;left[i]=0;below[i]=0;right[i]=0;
  }

  for(j=0;j<MB_BLOCK_SIZE/BLOCK_SIZE;j++)
  {
    joff=j*4;
    j4=img->block_y+j;
    for(i=0;i<MB_BLOCK_SIZE/BLOCK_SIZE;i++)
    {
      ioff=i*4;
      i4=img->block_x+i;

      vec1_x = i4*4*mv_mul + mv[0];
      vec1_y = j4*4*mv_mul + mv[1];

      get_block(ref_frame, listX[0], vec1_x,vec1_y,img,tmp_block);

	  index = i+4*j;
	  get_boundary(ref_frame, listX[0], vec1_x, vec1_y, img, index, above, left, below, right);
	  
	  if(j==0)
	  {
		  boundary[4*i+0]=above[0];boundary[4*i+1]=above[1];boundary[4*i+2]=above[2];boundary[4*i+3]=above[3];
	  }

	  if(i==0)
	  {
		  boundary[16+4*j+0]=left[0];boundary[16+4*j+1]=left[1];boundary[16+4*j+2]=left[2];boundary[16+4*j+3]=left[3];
	  }

	  if(j==3) //MB_BLOCK_SIZE/BLOCK_SIZE - 1 = 3
	  {
		  boundary[32+4*i+0]=below[0];boundary[32+4*i+1]=below[1];boundary[32+4*i+2]=below[2];boundary[32+4*i+3]=below[3];
	  }

	  if(i==3)
	  {
		  boundary[48+4*j+0]=right[0];boundary[48+4*j+1]=right[1];boundary[48+4*j+2]=right[2];boundary[48+4*j+3]=right[3];
	  }

      for(ii=0;ii<BLOCK_SIZE;ii++)
        for(jj=0;jj<MB_BLOCK_SIZE/BLOCK_SIZE;jj++)
          img->mpr[ii+ioff][jj+joff]=tmp_block[ii][jj];
    }
  }

  switch(pos)
  {
  case 0:
	  k=0;
	  for(j=0;j<8;j++)
	  {
		  for(i=0;i<8;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 64;
	  break;
  case 1:
	  k=0;
	  for(j=8;j<16;j++)
	  {
		  for(i=0;i<8;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 64;
	  break;
  case 2:
	  k=0;
	  for(j=0;j<16;j++)
	  {
		  for(i=8;i<16;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 128;
	  break;
  }

  if (dec_picture->chroma_format_idc != YUV400)
  {
    // chroma *******************************************************
    f1_x = 64/img->mb_cr_size_x;
    f2_x=f1_x-1;

    f1_y = 64/img->mb_cr_size_y;
    f2_y=f1_y-1;

    f3=f1_x*f1_y;
    f4=f3>>1;

    for(uv=0;uv<2;uv++)
    {
      for (b8=0;b8<(img->num_blk8x8_uv/2);b8++)
      {
        for(b4=0;b4<4;b4++)
		{
          joff = subblk_offset_y[yuv][b8][b4];
          j4=img->pix_c_y+joff;
          ioff = subblk_offset_x[yuv][b8][b4];
          i4=img->pix_c_x+ioff;

          for(jj=0;jj<4;jj++)
          {
            jf=(j4+jj)/(img->mb_cr_size_y/4);     // jf  = Subblock_y-coordinate
            for(ii=0;ii<4;ii++)
            {
              ifx=(i4+ii)/(img->mb_cr_size_x/4);  // ifx = Subblock_x-coordinate

              i1=(i4+ii)*f1_x + mv[0];
              j1=(j4+jj)*f1_y + mv[1];
            
              ii0=max (0, min (i1/f1_x,   dec_picture->size_x_cr-1));
              jj0=max (0, min (j1/f1_y,   dec_picture->size_y_cr-1));
              ii1=max (0, min ((i1+f2_x)/f1_x, dec_picture->size_x_cr-1));
              jj1=max (0, min ((j1+f2_y)/f1_y, dec_picture->size_y_cr-1));
            
              if1=(i1 & f2_x);
              jf1=(j1 & f2_y);
              if0=f1_x-if1;
              jf0=f1_y-jf1;
            
              img->mpr[ii+ioff][jj+joff]=(if0*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii0]+
                                          if1*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii1]+
                                          if0*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii0]+
                                          if1*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii1]+f4)/f3;
            }
          }
        }
      }

	  switch(pos)
	  {
	  case 0:
		  k=0;
		  for(j=0;j<4;j++)
		  {
			  for(i=0;i<4;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  pMB += 16;
		  break;
	  case 1:
		  k=0;
		  for(j=4;j<8;j++)
		  {
			  for(i=0;i<4;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  pMB += 16;
		  break;
	  case 2:
		  k=0;
		  for(j=0;j<8;j++)
		  {
			  for(i=4;i<8;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  pMB += 32;
		  break;
	  }
    }
  }
}

static int edgeDistortion_ECMODE7(int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos)
{
  int i, j, distortion, numOfPredBlocks, threshold = ERC_BLOCK_OK;
  imgpel *currBlock = NULL, *neighbor = NULL;
  int32 currBlockOffset = 0;
  
  currBlock = recY + (yPosYBlock(currYBlockNum,picSizeX)<<3)*picSizeX + (xPosYBlock(currYBlockNum,picSizeX)<<3);
  
  do 
  {    
    distortion = 0; numOfPredBlocks = 0;
    
    /* loop the 2/3 neighbours */
    for (j = 4; j < 8; j++) 
    {
		if(pos==0 && (j==6 || j==7))
			continue;
		if(pos==1 && (j==4 || j==7))
			continue;
		if(pos==2 && j==5)
			continue;

	  /* if reliable, count boundary pixel difference */
      if (predBlocks[j] >= threshold) 
      {        
        switch (j) 
        {
        case 4:
		  neighbor = currBlock - picSizeX;
		  if(OBMA)
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(boundary[i] - neighbor[i]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(boundary[i] - neighbor[i]));
			  }
		  }
		  else
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(predMB[i] - neighbor[i]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(predMB[i-8] - neighbor[i]));
			  }
		  }
		  break;          
        case 5:
          neighbor = currBlock - 1;
		  if(OBMA)
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(boundary[16+i] - neighbor[i*picSizeX]));
			  }
			  else if(pos==1)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(boundary[16+i] - neighbor[i*picSizeX]));
			  }
		  }
		  else
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(predMB[i*8] - neighbor[i*picSizeX]));
			  }
			  else if(pos==1)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(predMB[(i-8)*8] - neighbor[i*picSizeX]));
			  }
		  }
          break;                
        case 6:
          neighbor = currBlock + regionSize*picSizeX;
          currBlockOffset = (regionSize-1)*16;
		  if(OBMA)
		  {
			  if(pos==1)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(boundary[32+i] - neighbor[i]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(boundary[32+i] - neighbor[i]));
			  }
		  }
		  else
		  {
			  if(pos==1)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(predMB[i+56] - neighbor[i]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(predMB[(i-8)+15*8] - neighbor[i]));
			  }
		  }
		  break;                
        case 7:
          neighbor = currBlock + regionSize;
          currBlockOffset = 7;
		  if(OBMA)
		  {
			  for(i=0;i<regionSize;i++)
				  distortion += mabs((int)(boundary[48+i] - neighbor[i*picSizeX]));
		  }
		  else
		  {
			  for(i=0;i<regionSize;i++)
				  distortion += mabs((int)(predMB[i*8+currBlockOffset] - neighbor[i*picSizeX]));
		  }
          break;
        }
        
        numOfPredBlocks++;
      }
    }
    
    threshold--;
    if (threshold < ERC_BLOCK_CONCEALED)
      break;
  } while (numOfPredBlocks == 0);
  
  if(numOfPredBlocks == 0)
    return 0;

  return (distortion/numOfPredBlocks);
}

/**********************************************************************************************************************/
//EC Mode 8
static int concealABS_ECMODE8(frame *recfr, imgpel *predMB,int currMBNum, objectBuffer_t *object_list, int predBlocks[], 
                      int32 picSizeX, int32 picSizeY, int *yCondition)
{
	imgpel *pred_left_ecmodeMB, *pred_ecmodeMB, *left_pred_ecmodeMB, *topright_pred_ecmodeMB, *bottomright_pred_ecmodeMB;

	int predMBNum = 0, numMBPerLine,
      compSplit1 = 0, compSplit2 = 0, compLeft = 1, comp = 0, compPred, order = 1,
      fInterNeighborExists, numIntraNeighbours,
      fZeroMotionChecked, predSplitted = 0,
      threshold = ERC_BLOCK_OK,
      minDist, currDist, i, k, j, p, q, bestDir;
	int32 regionSize;
	objectBuffer_t *currRegion;
	int32 mvBest[3] , mvPred[3], *mvptr;
	imgpel *predMB_OBMC;
	
	if(OBMC)
		predMB_OBMC = (imgpel *) malloc ( (256 + (img->mb_cr_size_x*img->mb_cr_size_y)*2) * sizeof (imgpel));

	//8x16 for luma and 4x8, 4x8 for chroma
	pred_left_ecmodeMB = (imgpel *) malloc( ( 128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));
	left_pred_ecmodeMB = (imgpel *) malloc( ( 128 + (img->mb_cr_size_x*img->mb_cr_size_y/2)*2) * sizeof (imgpel));
	
	//8x8 for luma and 4x4, 4x4 for chroma
	pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	topright_pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	bottomright_pred_ecmodeMB = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	
	numMBPerLine = (int) (picSizeX>>4);
  	comp = 0;
	regionSize = 16;

	//LEFT HALF
	threshold = ERC_BLOCK_OK;
	comp=0;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 3 neighbours */
		for (i = 4; i < 7; i++) //no right nbr for left half
		{
			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 4:
					predMBNum = currMBNum-numMBPerLine;
					compSplit1 = 2;
					compSplit2 = 2;
					break;
				case 5:
					predMBNum = currMBNum-1;
					compSplit1 = 1;
					compSplit2 = 3;
					break;
				case 6:
					predMBNum = currMBNum+numMBPerLine;
					compSplit1 = 0;
					compSplit2 = 0;
					break;
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE8(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_left_ecmodeMB, boundary,0);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE8(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_left_ecmodeMB, boundary,0);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE8(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_left_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<128+32+32;p++)
							{
								left_pred_ecmodeMB[p] = pred_left_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE8(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_left_ecmodeMB, boundary,0);

	  currDist = edgeDistortion_ECMODE8(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_left_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 0);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<128+32+32;p++)
		{
			left_pred_ecmodeMB[p] = pred_left_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];
    

	//TOP RIGHT
	threshold = ERC_BLOCK_OK;
	comp=1;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 2 neighbours */
		for (i = 4; i < 8; i++) //only above and right nbrs
		{
			if(i==5 || i==6)
				continue;

			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{
				case 4:
					predMBNum = currMBNum-numMBPerLine;
					compSplit1 = 3;
					compSplit2 = 3;
					break;
				case 7:
					predMBNum = currMBNum+1;
					compSplit1 = 0;
					compSplit2 = 0;
					break; 
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE8(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE8(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE8(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<64+16+16;p++)
							{
								topright_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE8(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,1);

	  currDist = edgeDistortion_ECMODE8(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 1);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<64+16+16;p++)
		{
			topright_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];


	//BOTTOM RIGHT
	threshold = ERC_BLOCK_OK;
	comp=3;
	currRegion = object_list+(currMBNum<<2)+comp;
	
	/* set the position of the region to be concealed */
    currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,comp,picSizeX),picSizeX)<<3);

	do
	{
		/* reliability loop */
		minDist = 0; 
		fInterNeighborExists = 0; 
		numIntraNeighbours = 0; 
		fZeroMotionChecked = 0;	

		/* loop the 2 neighbours */
		for (i = 6; i < 8; i++) //only right and below nbrs
		{
			/* if reliable, try it */
			if (predBlocks[i] >= threshold) 
			{
				switch (i) 
				{				
				case 6:
					predMBNum = currMBNum+numMBPerLine;
					compSplit1 = 1;
					compSplit2 = 1;
					break;
				case 7:
					predMBNum = currMBNum+1;
					compSplit1 = 2;
					compSplit2 = 2;
					break;
				}

				/* try the concealment with the Motion Info of the current neighbour
				only try if the neighbour is not Intra */
				if (isBlock(object_list,predMBNum,compSplit1,INTRA) || 
					isBlock(object_list,predMBNum,compSplit2,INTRA))
				{            
					numIntraNeighbours++;
				}
				else 
				{
					/* if neighbour MB is splitted, try the neighbour sub-blocks */
					for (predSplitted = isSplitted(object_list, predMBNum), compPred = compSplit1;
						 predSplitted >= 0; compPred = compSplit2, predSplitted -= ((compSplit1 == compSplit2) ? 2 : 1)) 
					{
						/* if Zero Motion Block, do the copying. This option is tried only once */
						if (isBlock(object_list, predMBNum, compPred, INTER_COPY)) 
						{
							if (fZeroMotionChecked) 
							{
								continue;
							}
							else 
							{
								fZeroMotionChecked = 1;
	  						    mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
								buildOuterPredRegionYUV_ECMODE8(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,2);
							}
						}
						/* build motion using the neighbour's Motion Parameters */
						else if (isBlock(object_list,predMBNum,compPred,INTRA)) 
						{
							continue;
						}
						else 
						{
							mvptr = getParam(object_list, predMBNum, compPred, mv);                
							mvPred[0] = mvptr[0]; mvPred[1] = mvptr[1]; mvPred[2] = mvptr[2];		

							buildOuterPredRegionYUV_ECMODE8(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,2);
						}

						/* measure absolute boundary pixel difference */
						currDist = edgeDistortion_ECMODE8(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 2);
						
						/* if so far best -> store the pixels as the best concealment */
						if (currDist < minDist || !fInterNeighborExists) 
						{
							minDist = currDist;
							bestDir = i;
                
							for (k=0;k<3;k++) 
								mvBest[k] = mvPred[k];
                
							currRegion->regionMode =
								(isBlock(object_list, predMBNum, compPred, INTER_COPY)) ? 
								((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8) : 
								((regionSize == 16) ? REGMODE_INTER_PRED : REGMODE_INTER_PRED_8x8);

							for(p=0;p<64+16+16;p++)
							{
								bottomright_pred_ecmodeMB[p] = pred_ecmodeMB[p];
							}
							
							p=0;
						}
              
						fInterNeighborExists = 1;    
					}
				}
			}
		}

		threshold--;      
	}while ((threshold >= ERC_BLOCK_CONCEALED) && (fInterNeighborExists == 0));
    
	/* always try zero motion */
    if (!fZeroMotionChecked) 
    {
      mvPred[0] = mvPred[1] = 0; mvPred[2] = 0;
	  buildOuterPredRegionYUV_ECMODE8(erc_img, mvPred, currRegion->xMin, currRegion->yMin, pred_ecmodeMB, boundary,2);

	  currDist = edgeDistortion_ECMODE8(predBlocks,MBNum2YBlock(currMBNum,comp,picSizeX),pred_ecmodeMB,recfr->yptr, picSizeX, regionSize, boundary, 2);
      
      if (currDist < minDist || !fInterNeighborExists) 
      {        
        minDist = currDist;            
        for (k=0;k<3;k++) 
          mvBest[k] = mvPred[k];
        
        currRegion->regionMode = ((regionSize == 16) ? REGMODE_INTER_COPY : REGMODE_INTER_COPY_8x8);

		for(p=0;p<64+16+16;p++)
		{
			bottomright_pred_ecmodeMB[p] = pred_ecmodeMB[p];
		}
	  }
    }

	for (i=0; i<3; i++)
      currRegion->mv[i] = mvBest[i];


	//Whole MB concealed at this stage, copy to predMB

	p=0;q=0;
	for(j=0;j<16;j++)
	{
		for(i=0;i<8;i++)
		{
			predMB[j*16+i] = left_pred_ecmodeMB[q];
			q++;
		}
	}
	for(j=0;j<8;j++)
	{
		for(i=8;i<16;i++)
		{
			predMB[j*16+i] = topright_pred_ecmodeMB[p];
			predMB[(j+8)*16+i] = bottomright_pred_ecmodeMB[p];
			p++;
		}
	}	

	p=0;q=0;
	for(j=0;j<8;j++)
	{
		for(i=0;i<4;i++)
		{
			predMB[256+j*8+i] = left_pred_ecmodeMB[128+q];
			predMB[256+64+j*8+i] = left_pred_ecmodeMB[128+32+q];
			q++;
		}
	}
	for(j=0;j<4;j++)
	{
		for(i=4;i<8;i++)
		{
			predMB[256+j*8+i] = topright_pred_ecmodeMB[64+p];
			predMB[256+(j+4)*8+i] = bottomright_pred_ecmodeMB[64+p];
			predMB[256+64+j*8+i] = topright_pred_ecmodeMB[80+p];
			predMB[256+64+(j+4)*8+i] = bottomright_pred_ecmodeMB[80+p];
			p++;
		}
	}
	
	if(OBMC)
	{
		OBMC_MB(predMB,predMB_OBMC,predBlocks,object_list,currMBNum,numMBPerLine,picSizeX);
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB_OBMC, recfr, picSizeX, regionSize);
	}
	else
		copyPredMB(MBNum2YBlock(currMBNum,0,picSizeX), predMB, recfr,picSizeX, regionSize);
	
	yCondition[MBNum2YBlock(currMBNum,0,picSizeX)] = ERC_BLOCK_CONCEALED;

	free(pred_ecmodeMB);
	free(pred_left_ecmodeMB);
	free(left_pred_ecmodeMB);
	free(topright_pred_ecmodeMB);
	free(bottomright_pred_ecmodeMB);
	if(OBMC)
		free(predMB_OBMC);

	return 0;
}

static void buildOuterPredRegionYUV_ECMODE8(struct img_par *img, int32 *mv, int x, int y, imgpel *predMB, imgpel *boundary, int pos)
{
  int tmp_block[BLOCK_SIZE][BLOCK_SIZE];
  int i=0,j=0,ii=0,jj=0,i1=0,j1=0,j4=0,i4=0,k=0;
  int jf=0;
  int uv;
  int vec1_x=0,vec1_y=0;
  int ioff,joff;
  imgpel *pMB = predMB; //Note: predMB is actually pred_ecmodeMB; for ecmode2, it is left/right half, so 8x16 for Y and 4x8,4x8 for UV
  
  int ii0,jj0,ii1,jj1,if1,jf1,if0,jf0;
  int mv_mul;

  int above[4], left[4], below[4], right[4], index;
  
  //FRExt
  int f1_x, f1_y, f2_x, f2_y, f3, f4, ifx;
  int b8, b4;
  int yuv = dec_picture->chroma_format_idc - 1;
  
  int ref_frame = max (mv[2], 0); // !!KS: quick fix, we sometimes seem to get negative ref_pic here, so restrict to zero an above

  /* Update coordinates of the current concealed macroblock */
  img->mb_x = x/MB_BLOCK_SIZE;
  img->mb_y = y/MB_BLOCK_SIZE;
  img->block_y = img->mb_y * BLOCK_SIZE;
  img->pix_c_y = img->mb_y * img->mb_cr_size_y;
  img->block_x = img->mb_x * BLOCK_SIZE;
  img->pix_c_x = img->mb_x * img->mb_cr_size_x;

  mv_mul=4;

  // luma *******************************************************

  for(i=0;i<4;i++)
  {
	  above[i]=0;left[i]=0;below[i]=0;right[i]=0;
  }

  for(j=0;j<MB_BLOCK_SIZE/BLOCK_SIZE;j++)
  {
    joff=j*4;
    j4=img->block_y+j;
    for(i=0;i<MB_BLOCK_SIZE/BLOCK_SIZE;i++)
    {
      ioff=i*4;
      i4=img->block_x+i;

      vec1_x = i4*4*mv_mul + mv[0];
      vec1_y = j4*4*mv_mul + mv[1];

      get_block(ref_frame, listX[0], vec1_x,vec1_y,img,tmp_block);

	  index = i+4*j;
	  get_boundary(ref_frame, listX[0], vec1_x, vec1_y, img, index, above, left, below, right);
	  
	  if(j==0)
	  {
		  boundary[4*i+0]=above[0];boundary[4*i+1]=above[1];boundary[4*i+2]=above[2];boundary[4*i+3]=above[3];
	  }

	  if(i==0)
	  {
		  boundary[16+4*j+0]=left[0];boundary[16+4*j+1]=left[1];boundary[16+4*j+2]=left[2];boundary[16+4*j+3]=left[3];
	  }

	  if(j==3) //MB_BLOCK_SIZE/BLOCK_SIZE - 1 = 3
	  {
		  boundary[32+4*i+0]=below[0];boundary[32+4*i+1]=below[1];boundary[32+4*i+2]=below[2];boundary[32+4*i+3]=below[3];
	  }

	  if(i==3)
	  {
		  boundary[48+4*j+0]=right[0];boundary[48+4*j+1]=right[1];boundary[48+4*j+2]=right[2];boundary[48+4*j+3]=right[3];
	  }

      for(ii=0;ii<BLOCK_SIZE;ii++)
        for(jj=0;jj<MB_BLOCK_SIZE/BLOCK_SIZE;jj++)
          img->mpr[ii+ioff][jj+joff]=tmp_block[ii][jj];
    }
  }

  switch(pos)
  {
  case 0:
	  k=0;
	  for(j=0;j<16;j++)
	  {
		  for(i=0;i<8;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 128;
	  break;
  case 1:
	  k=0;
	  for(j=0;j<8;j++)
	  {
		  for(i=8;i<16;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 64;
	  break;
  case 2:
	  k=0;
	  for(j=8;j<16;j++)
	  {
		  for(i=8;i<16;i++)
		  {
			  pMB[k++] = img->mpr[i][j];
		  }
	  }
	  pMB += 64;
	  break;
  }

  if (dec_picture->chroma_format_idc != YUV400)
  {
    // chroma *******************************************************
    f1_x = 64/img->mb_cr_size_x;
    f2_x=f1_x-1;

    f1_y = 64/img->mb_cr_size_y;
    f2_y=f1_y-1;

    f3=f1_x*f1_y;
    f4=f3>>1;

    for(uv=0;uv<2;uv++)
    {
      for (b8=0;b8<(img->num_blk8x8_uv/2);b8++)
      {
        for(b4=0;b4<4;b4++)
		{
          joff = subblk_offset_y[yuv][b8][b4];
          j4=img->pix_c_y+joff;
          ioff = subblk_offset_x[yuv][b8][b4];
          i4=img->pix_c_x+ioff;

          for(jj=0;jj<4;jj++)
          {
            jf=(j4+jj)/(img->mb_cr_size_y/4);     // jf  = Subblock_y-coordinate
            for(ii=0;ii<4;ii++)
            {
              ifx=(i4+ii)/(img->mb_cr_size_x/4);  // ifx = Subblock_x-coordinate

              i1=(i4+ii)*f1_x + mv[0];
              j1=(j4+jj)*f1_y + mv[1];
            
              ii0=max (0, min (i1/f1_x,   dec_picture->size_x_cr-1));
              jj0=max (0, min (j1/f1_y,   dec_picture->size_y_cr-1));
              ii1=max (0, min ((i1+f2_x)/f1_x, dec_picture->size_x_cr-1));
              jj1=max (0, min ((j1+f2_y)/f1_y, dec_picture->size_y_cr-1));
            
              if1=(i1 & f2_x);
              jf1=(j1 & f2_y);
              if0=f1_x-if1;
              jf0=f1_y-jf1;
            
              img->mpr[ii+ioff][jj+joff]=(if0*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii0]+
                                          if1*jf0*listX[0][ref_frame]->imgUV[uv][jj0][ii1]+
                                          if0*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii0]+
                                          if1*jf1*listX[0][ref_frame]->imgUV[uv][jj1][ii1]+f4)/f3;
            }
          }
        }
      }

	  switch(pos)
	  {
	  case 0:
		  k=0;
		  for(j=0;j<8;j++)
		  {
			  for(i=0;i<4;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  pMB += 32;
		  break;
	  case 1:
		  k=0;
		  for(j=0;j<4;j++)
		  {
			  for(i=4;i<8;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  pMB += 16;
		  break;
	  case 2:
		  k=0;
		  for(j=4;j<8;j++)
		  {
			  for(i=4;i<8;i++)
			  {
				  pMB[k++] = img->mpr[i][j];
			  }
		  }
		  pMB += 16;
		  break;
	  }
    }
  }
}

static int edgeDistortion_ECMODE8(int predBlocks[], int currYBlockNum, imgpel *predMB, imgpel *recY, int32 picSizeX, int32 regionSize, imgpel *boundary, int pos)
{
  int i, j, distortion, numOfPredBlocks, threshold = ERC_BLOCK_OK;
  imgpel *currBlock = NULL, *neighbor = NULL;
  int32 currBlockOffset = 0;
  
  currBlock = recY + (yPosYBlock(currYBlockNum,picSizeX)<<3)*picSizeX + (xPosYBlock(currYBlockNum,picSizeX)<<3);
  
  do 
  {    
    distortion = 0; numOfPredBlocks = 0;
    
    /* loop the 2/3 neighbours */
    for (j = 4; j < 8; j++) 
    {
		if(pos==0 && j==7)
			continue;
		if(pos==2 && (j==4 || j==5))
			continue;
		if(pos==1 && (j==5 || j==6))
			continue;

	  /* if reliable, count boundary pixel difference */
      if (predBlocks[j] >= threshold) 
      {        
        switch (j) 
        {
        case 4:
		  neighbor = currBlock - picSizeX;
		  if(OBMA)
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(boundary[i] - neighbor[i]));
			  }
			  else if(pos==1)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(boundary[i] - neighbor[i]));
			  }
		  }
		  else
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(predMB[i] - neighbor[i]));
			  }
			  else if(pos==1)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(predMB[i-8] - neighbor[i]));
			  }
		  }
		  break;          
        case 5:
          neighbor = currBlock - 1;
		  if(OBMA)
		  {
			  for(i=0;i<regionSize;i++)
				  distortion += mabs((int)(boundary[16+i] - neighbor[i*picSizeX]));
		  }
		  else
		  {
			  for(i=0;i<regionSize;i++)
				  distortion += mabs((int)(predMB[i*8] - neighbor[i*picSizeX]));
		  }
          break;                
        case 6:
          neighbor = currBlock + regionSize*picSizeX;
          currBlockOffset = (regionSize-1)*16;
		  if(OBMA)
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(boundary[32+i] - neighbor[i]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(boundary[32+i] - neighbor[i]));
			  }
		  }
		  else
		  {
			  if(pos==0)
			  {
				  for(i=0;i<8;i++)
					  distortion += mabs((int)(predMB[i+15*8] - neighbor[i]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(predMB[(i-8)+56] - neighbor[i]));
			  }
		  }
		  break;                
        case 7:
          neighbor = currBlock + regionSize;
          currBlockOffset = 7;
		  if(OBMA)
		  {
			  if(pos==1)
			  {
				   for(i=0;i<8;i++)
					   distortion += mabs((int)(boundary[48+i] - neighbor[i*picSizeX]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(boundary[48+i] - neighbor[i*picSizeX]));
			  }
		  }
		  else
		  {
			  if(pos==1)
			  {
				   for(i=0;i<8;i++)
					   distortion += mabs((int)(predMB[i*8+currBlockOffset] - neighbor[i*picSizeX]));
			  }
			  else if(pos==2)
			  {
				  for(i=8;i<16;i++)
					  distortion += mabs((int)(predMB[(i-8)*8+currBlockOffset] - neighbor[i*picSizeX]));
			  }
		  }
          break;
        }
        
        numOfPredBlocks++;
      }
    }
    
    threshold--;
    if (threshold < ERC_BLOCK_CONCEALED)
      break;
  } while (numOfPredBlocks == 0);
  
  if(numOfPredBlocks == 0)
    return 0;

  return (distortion/numOfPredBlocks);
}


static void OBMC_MB(imgpel *predMB, imgpel *predMB_OBMC, int predBlocks[], objectBuffer_t *object_list, int currMBNum, int numMBPerLine, int picSizeX)
{
	int32 mvLR[3], mvTD[3], *mvptr;
	int i, j, predMBNum, lower, upper;
	imgpel *predMB_LR, *predMB_TD;
	objectBuffer_t *currRegion;

	predMB_LR = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));
	predMB_TD = (imgpel *) malloc ( (64 + (img->mb_cr_size_x*img->mb_cr_size_y/4)*2) * sizeof (imgpel));

	currRegion = object_list+(currMBNum<<2); 
	currRegion->xMin = (xPosYBlock(MBNum2YBlock(currMBNum,0,picSizeX),picSizeX)<<3);
    currRegion->yMin = (yPosYBlock(MBNum2YBlock(currMBNum,0,picSizeX),picSizeX)<<3);

	//Comp = 0
	for(i=0;i<2;i++)
	{
		 mvLR[i] = NIL; mvTD[i] = NIL;
	}
	mvLR[2] = 0; mvTD[2] = 0;

	//Above
	if(predBlocks[4]>=ERC_BLOCK_CONCEALED)
	{
		predMBNum = currMBNum-numMBPerLine;
		if(!(isBlock(object_list, predMBNum, 2, INTER_COPY) || isBlock(object_list,predMBNum,2,INTRA))) 
		{
			mvptr = getParam(object_list, predMBNum, 2, mv);
			mvTD[0]=mvptr[0]; mvTD[1]=mvptr[1];
		}				
	}

	//Left
	if(predBlocks[5]>=ERC_BLOCK_CONCEALED)
	{
		predMBNum = currMBNum-1;
		if(!(isBlock(object_list, predMBNum,1, INTER_COPY) || isBlock(object_list,predMBNum,1,INTRA))) 
		{
			mvptr = getParam(object_list, predMBNum, 1, mv);
			mvLR[0]=mvptr[0]; mvLR[1]=mvptr[1];
		}
	}
	
	//construct MB with mvTD
	if(mvTD[0]!=NIL)
	{
		buildOuterPredRegionYUV_ECMODE4(erc_img,mvTD,currRegion->xMin,currRegion->yMin,predMB_TD,boundary,0);
	}
	else
	{
		for(i=0;i<8;i++)
		{
			for(j=0;j<8;j++)
			{
				predMB_TD[i*8+j] = predMB[i*16+j];
			}
		}

		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
			{
				predMB_TD[64+i*4+j] = predMB[256+i*8+j];
				predMB_TD[64+16+i*4+j] = predMB[256+64+i*8+j];
			}
		}
	}

	//construct MB with mvLR
	if(mvLR[0]!=NIL)
	{
		buildOuterPredRegionYUV_ECMODE4(erc_img,mvLR,currRegion->xMin,currRegion->yMin,predMB_LR,boundary,0);
	}
	else
	{
		for(i=0;i<8;i++)
		{
			for(j=0;j<8;j++)
			{
				predMB_LR[i*8+j] = predMB[i*16+j];
			}
		}

		for(i=0;i<4;i++)
		{
			for(j=0;j<4;j++)
			{
				predMB_LR[64+i*4+j] = predMB[256+i*8+j];
				predMB_LR[64+16+i*4+j] = predMB[256+64+i*8+j];
			}
		}
	}
	
	//apply obmc averaging with weights 
	for(i=0;i<8;i++)
	{
		for(j=0;j<8;j++)
		{
			predMB_OBMC[i*16+j] = (predMB[i*16+j]*H_E_8x8[i][j] + predMB_LR[i*8+j]*H_LR_8x8[i][j] + predMB_TD[i*8+j]*H_TD_8x8[i][j]);
			predMB_OBMC[i*16+j] = (int)floor(predMB_OBMC[i*16+j]/8.0 + 0.5);

			upper = predMB[i*16+j] + (predMB[i*16+j]*OBMC_TR/100);
			lower = predMB[i*16+j] - (predMB[i*16+j]*OBMC_TR/100);
			if(OBMC_THRESHOLD && !(predMB_OBMC[i*16+j]>=lower && predMB_OBMC[i*16+j]<=upper))
				predMB_OBMC[i*16+j] = predMB[i*16+j];
		}
	}

	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			predMB_OBMC[256+i*8+j] = (predMB[256+i*8+j]*H_E_8x8[i][j] + predMB_LR[64+i*4+j]*H_LR_8x8[i][j] + predMB_TD[64+i*4+j]*H_TD_8x8[i][j]);
			predMB_OBMC[256+i*8+j] = (int)floor(predMB_OBMC[256+i*8+j]/8.0 + 0.5);

			upper = predMB[256+i*8+j] + (predMB[256+i*8+j]*OBMC_TR/100);
			lower = predMB[256+i*8+j] - (predMB[256+i*8+j]*OBMC_TR/100);
			if(OBMC_THRESHOLD && !(predMB_OBMC[256+i*8+j]>=lower && predMB_OBMC[256+i*8+j]<=upper))
				predMB_OBMC[256+i*8+j] = predMB[256+i*8+j];

			predMB_OBMC[256+64+i*8+j] = (predMB[256+64+i*8+j]*H_E_8x8[i][j] + predMB_LR[64+16+i*4+j]*H_LR_8x8[i][j] + predMB_TD[64+16+i*4+j]*H_TD_8x8[i][j]);
			predMB_OBMC[256+64+i*8+j] = (int)floor(predMB_OBMC[256+64+i*8+j]/8.0 + 0.5);

			upper = predMB[256+64+i*8+j] + (predMB[256+64+i*8+j]*OBMC_TR/100);
			lower = predMB[256+64+i*8+j] - (predMB[256+64+i*8+j]*OBMC_TR/100);
			if(OBMC_THRESHOLD && !(predMB_OBMC[256+64+i*8+j]>=lower && predMB_OBMC[256+64+i*8+j]<=upper))
				predMB_OBMC[256+64+i*8+j] = predMB[256+64+i*8+j];
		}
	}


	//Comp = 1
	for(i=0;i<2;i++)
	{
		 mvLR[i] = NIL; mvTD[i] = NIL;
	}
	mvLR[2] = 0; mvTD[2] = 0;

	//Above
	if(predBlocks[4]>=ERC_BLOCK_CONCEALED)
	{
		predMBNum = currMBNum-numMBPerLine;
		if(!(isBlock(object_list, predMBNum, 3, INTER_COPY) || isBlock(object_list,predMBNum,3,INTRA))) 
		{
			mvptr = getParam(object_list, predMBNum, 3, mv);
			mvTD[0]=mvptr[0]; mvTD[1]=mvptr[1];
		}				
	}

	//Right
	if(predBlocks[7]>=ERC_BLOCK_CONCEALED)
	{
		predMBNum = currMBNum+1;
		if(!(isBlock(object_list, predMBNum, 0, INTER_COPY) || isBlock(object_list,predMBNum,0,INTRA))) 
		{
			mvptr = getParam(object_list, predMBNum, 0, mv);
			mvLR[0]=mvptr[0]; mvLR[1]=mvptr[1];
		}
	}
	
	//construct MB with mvTD
	if(mvTD[0]!=NIL)
	{
		buildOuterPredRegionYUV_ECMODE4(erc_img,mvTD,currRegion->xMin,currRegion->yMin,predMB_TD,boundary,1);
	}
	else
	{
		for(i=0;i<8;i++)
		{
			for(j=8;j<16;j++)
			{
				predMB_TD[i*8+(j-8)] = predMB[i*16+j];
			}
		}

		for(i=0;i<4;i++)
		{
			for(j=4;j<8;j++)
			{
				predMB_TD[64+i*4+(j-4)] = predMB[256+i*8+j];
				predMB_TD[64+16+i*4+(j-4)] = predMB[256+64+i*8+j];
			}
		}
	}

	//construct MB with mvLR
	if(mvLR[0]!=NIL)
	{
		buildOuterPredRegionYUV_ECMODE4(erc_img,mvLR,currRegion->xMin,currRegion->yMin,predMB_LR,boundary,1);
	}
	else
	{
		for(i=0;i<8;i++)
		{
			for(j=8;j<16;j++)
			{
				predMB_LR[i*8+(j-8)] = predMB[i*16+j];
			}
		}

		for(i=0;i<4;i++)
		{
			for(j=4;j<8;j++)
			{
				predMB_LR[64+i*4+(j-4)] = predMB[256+i*8+j];
				predMB_LR[64+16+i*4+(j-4)] = predMB[256+64+i*8+j];
			}
		}
	}
	
	//apply obmc averaging with weights 
	for(i=0;i<8;i++)
	{
		for(j=8;j<16;j++)
		{
			predMB_OBMC[i*16+j] = (predMB[i*16+j]*H_E_8x8[i][j-8] + predMB_LR[i*8+(j-8)]*H_LR_8x8[i][j-8] + predMB_TD[i*8+(j-8)]*H_TD_8x8[i][j-8]);
			predMB_OBMC[i*16+j] = (int)floor(predMB_OBMC[i*16+j]/8.0 + 0.5);

			upper = predMB[i*16+j] + (predMB[i*16+j]*OBMC_TR/100);
			lower = predMB[i*16+j] - (predMB[i*16+j]*OBMC_TR/100);
			if(OBMC_THRESHOLD && !(predMB_OBMC[i*16+j]>=lower && predMB_OBMC[i*16+j]<=upper))
				predMB_OBMC[i*16+j] = predMB[i*16+j];
		}
	}

	for(i=0;i<4;i++)
	{
		for(j=4;j<8;j++)
		{
			predMB_OBMC[256+i*8+j] = (predMB[256+i*8+j]*H_E_8x8[i][j-4] + predMB_LR[64+i*4+(j-4)]*H_LR_8x8[i][j-4] + predMB_TD[64+i*4+(j-4)]*H_TD_8x8[i][j-4]);
			predMB_OBMC[256+i*8+j] = (int)floor(predMB_OBMC[256+i*8+j]/8.0 + 0.5);

			upper = predMB[256+i*8+j] + (predMB[256+i*8+j]*OBMC_TR/100);
			lower = predMB[256+i*8+j] - (predMB[256+i*8+j]*OBMC_TR/100);
			if(OBMC_THRESHOLD && !(predMB_OBMC[256+i*8+j]>=lower && predMB_OBMC[256+i*8+j]<=upper))
				predMB_OBMC[256+i*8+j] = predMB[256+i*8+j];

			predMB_OBMC[256+64+i*8+j] = (predMB[256+64+i*8+j]*H_E_8x8[i][j-4] + predMB_LR[64+16+i*4+(j-4)]*H_LR_8x8[i][j-4] + predMB_TD[64+16+i*4+(j-4)]*H_TD_8x8[i][j-4]);
			predMB_OBMC[256+64+i*8+j] = (int)floor(predMB_OBMC[256+64+i*8+j]/8.0 + 0.5);

			upper = predMB[256+64+i*8+j] + (predMB[256+64+i*8+j]*OBMC_TR/100);
			lower = predMB[256+64+i*8+j] - (predMB[256+64+i*8+j]*OBMC_TR/100);
			if(OBMC_THRESHOLD && !(predMB_OBMC[256+64+i*8+j]>=lower && predMB_OBMC[256+64+i*8+j]<=upper))
				predMB_OBMC[256+64+i*8+j] = predMB[256+64+i*8+j];
		}
	}

	//Comp = 2
	for(i=0;i<2;i++)
	{
		 mvLR[i] = NIL; mvTD[i] = NIL;
	}
	mvLR[2] = 0; mvTD[2] = 0;

	//Left
	if(predBlocks[5]>=ERC_BLOCK_CONCEALED)
	{
		predMBNum = currMBNum-1;
		if(!(isBlock(object_list, predMBNum,3, INTER_COPY) || isBlock(object_list,predMBNum,3,INTRA))) 
		{
			mvptr = getParam(object_list, predMBNum, 3, mv);
			mvLR[0]=mvptr[0]; mvLR[1]=mvptr[1];
		}
	}

	//Below
	if(predBlocks[6]>=ERC_BLOCK_CONCEALED)
	{
		predMBNum = currMBNum+numMBPerLine;
		if(!(isBlock(object_list, predMBNum, 0, INTER_COPY) || isBlock(object_list,predMBNum,0,INTRA))) 
		{
			mvptr = getParam(object_list, predMBNum, 0, mv);
			mvTD[0]=mvptr[0]; mvTD[1]=mvptr[1];
		}
	}
	
	//construct MB with mvTD
	if(mvTD[0]!=NIL)
	{
		buildOuterPredRegionYUV_ECMODE4(erc_img,mvTD,currRegion->xMin,currRegion->yMin,predMB_TD,boundary,2);
	}
	else
	{
		for(i=8;i<16;i++)
		{
			for(j=0;j<8;j++)
			{
				predMB_TD[(i-8)*8+j] = predMB[i*16+j];
			}
		}

		for(i=4;i<8;i++)
		{
			for(j=0;j<4;j++)
			{
				predMB_TD[64+(i-4)*4+j] = predMB[256+i*8+j];
				predMB_TD[64+16+(i-4)*4+j] = predMB[256+64+i*8+j];
			}
		}
	}

	//construct MB with mvLR
	if(mvLR[0]!=NIL)
	{
		buildOuterPredRegionYUV_ECMODE4(erc_img,mvLR,currRegion->xMin,currRegion->yMin,predMB_LR,boundary,2);
	}
	else
	{
		for(i=8;i<16;i++)
		{
			for(j=0;j<8;j++)
			{
				predMB_LR[(i-8)*8+j] = predMB[i*16+j];
			}
		}

		for(i=4;i<8;i++)
		{
			for(j=0;j<4;j++)
			{
				predMB_LR[64+(i-4)*4+j] = predMB[256+i*8+j];
				predMB_LR[64+16+(i-4)*4+j] = predMB[256+64+i*8+j];
			}
		}
	}
	
	//apply obmc averaging with weights 
	for(i=8;i<16;i++)
	{
		for(j=0;j<8;j++)
		{
			predMB_OBMC[i*16+j] = (predMB[i*16+j]*H_E_8x8[i-8][j] + predMB_LR[(i-8)*8+j]*H_LR_8x8[i-8][j] + predMB_TD[(i-8)*8+j]*H_TD_8x8[i-8][j]);
			predMB_OBMC[i*16+j] = (int)floor(predMB_OBMC[i*16+j]/8.0 + 0.5);

			upper = predMB[i*16+j] + (predMB[i*16+j]*OBMC_TR/100);
			lower = predMB[i*16+j] - (predMB[i*16+j]*OBMC_TR/100);
			if(OBMC_THRESHOLD && !(predMB_OBMC[i*16+j]>=lower && predMB_OBMC[i*16+j]<=upper))
				predMB_OBMC[i*16+j] = predMB[i*16+j];
		}
	}

	for(i=4;i<8;i++)
	{
		for(j=0;j<4;j++)
		{
			predMB_OBMC[256+i*8+j] = (predMB[256+i*8+j]*H_E_8x8[i-4][j] + predMB_LR[64+(i-4)*4+j]*H_LR_8x8[i-4][j] + predMB_TD[64+(i-4)*4+j]*H_TD_8x8[i-4][j]);
			predMB_OBMC[256+i*8+j] = (int)floor(predMB_OBMC[256+i*8+j]/8.0 + 0.5);

			upper = predMB[256+i*8+j] + (predMB[256+i*8+j]*OBMC_TR/100);
			lower = predMB[256+i*8+j] - (predMB[256+i*8+j]*OBMC_TR/100);
			if(OBMC_THRESHOLD && !(predMB_OBMC[256+i*8+j]>=lower && predMB_OBMC[256+i*8+j]<=upper))
				predMB_OBMC[256+i*8+j] = predMB[256+i*8+j];

			predMB_OBMC[256+64+i*8+j] = (predMB[256+64+i*8+j]*H_E_8x8[i-4][j] + predMB_LR[64+16+(i-4)*4+j]*H_LR_8x8[i-4][j] + predMB_TD[64+16+(i-4)*4+j]*H_TD_8x8[i-4][j]);
			predMB_OBMC[256+64+i*8+j] = (int)floor(predMB_OBMC[256+64+i*8+j]/8.0 + 0.5);

			upper = predMB[256+64+i*8+j] + (predMB[256+64+i*8+j]*OBMC_TR/100);
			lower = predMB[256+64+i*8+j] - (predMB[256+64+i*8+j]*OBMC_TR/100);
			if(OBMC_THRESHOLD && !(predMB_OBMC[256+64+i*8+j]>=lower && predMB_OBMC[256+64+i*8+j]<=upper))
				predMB_OBMC[256+64+i*8+j] = predMB[256+64+i*8+j];
		}
	}

	//Comp = 3
	for(i=0;i<2;i++)
	{
		 mvLR[i] = NIL; mvTD[i] = NIL;
	}
	mvLR[2] = 0; mvTD[2] = 0;

	//Below
	if(predBlocks[6]>=ERC_BLOCK_CONCEALED)
	{
		predMBNum = currMBNum+numMBPerLine;
		if(!(isBlock(object_list, predMBNum, 1, INTER_COPY) || isBlock(object_list,predMBNum,1,INTRA))) 
		{
			mvptr = getParam(object_list, predMBNum, 1, mv);
			mvTD[0]=mvptr[0]; mvTD[1]=mvptr[1];
		}
	}

	//Right
	if(predBlocks[7]>=ERC_BLOCK_CONCEALED)
	{
		predMBNum = currMBNum+1;
		if(!(isBlock(object_list, predMBNum, 2, INTER_COPY) || isBlock(object_list,predMBNum,2,INTRA))) 
		{
			mvptr = getParam(object_list, predMBNum, 2, mv);
			mvLR[0]=mvptr[0]; mvLR[1]=mvptr[1];
		}
	}
	
	//construct MB with mvTD
	if(mvTD[0]!=NIL)
	{
		buildOuterPredRegionYUV_ECMODE4(erc_img,mvTD,currRegion->xMin,currRegion->yMin,predMB_TD,boundary,3);
	}
	else
	{
		for(i=8;i<16;i++)
		{
			for(j=8;j<16;j++)
			{
				predMB_TD[(i-8)*8+(j-8)] = predMB[i*16+j];
			}
		}

		for(i=4;i<8;i++)
		{
			for(j=4;j<8;j++)
			{
				predMB_TD[64+(i-4)*4+(j-4)] = predMB[256+i*8+j];
				predMB_TD[64+16+(i-4)*4+(j-4)] = predMB[256+64+i*8+j];
			}
		}
	}

	//construct MB with mvLR
	if(mvLR[0]!=NIL)
	{
		buildOuterPredRegionYUV_ECMODE4(erc_img,mvLR,currRegion->xMin,currRegion->yMin,predMB_LR,boundary,3);
	}
	else
	{
		for(i=8;i<16;i++)
		{
			for(j=8;j<16;j++)
			{
				predMB_LR[(i-8)*8+(j-8)] = predMB[i*16+j];
			}
		}

		for(i=4;i<8;i++)
		{
			for(j=4;j<8;j++)
			{
				predMB_LR[64+(i-4)*4+(j-4)] = predMB[256+i*8+j];
				predMB_LR[64+16+(i-4)*4+(j-4)] = predMB[256+64+i*8+j];
			}
		}
	}
	
	//apply obmc averaging with weights 
	for(i=8;i<16;i++)
	{
		for(j=8;j<16;j++)
		{
			predMB_OBMC[i*16+j] = (predMB[i*16+j]*H_E_8x8[i-8][j-8] + predMB_LR[(i-8)*8+(j-8)]*H_LR_8x8[i-8][j-8] + predMB_TD[(i-8)*8+(j-8)]*H_TD_8x8[i-8][j-8]);
			predMB_OBMC[i*16+j] = (int)floor(predMB_OBMC[i*16+j]/8.0 + 0.5);

			upper = predMB[i*16+j] + (predMB[i*16+j]*OBMC_TR/100);
			lower = predMB[i*16+j] - (predMB[i*16+j]*OBMC_TR/100);
			if(OBMC_THRESHOLD && !(predMB_OBMC[i*16+j]>=lower && predMB_OBMC[i*16+j]<=upper))
				predMB_OBMC[i*16+j] = predMB[i*16+j];
		}
	}

	for(i=4;i<8;i++)
	{
		for(j=4;j<8;j++)
		{
			predMB_OBMC[256+i*8+j] = (predMB[256+i*8+j]*H_E_8x8[i-4][j-4] + predMB_LR[64+(i-4)*4+(j-4)]*H_LR_8x8[i-4][j-4] + predMB_TD[64+(i-4)*4+(j-4)]*H_TD_8x8[i-4][j-4]);
			predMB_OBMC[256+i*8+j] = (int)floor(predMB_OBMC[256+i*8+j]/8.0 + 0.5);

			upper = predMB[256+i*8+j] + (predMB[256+i*8+j]*OBMC_TR/100);
			lower = predMB[256+i*8+j] - (predMB[256+i*8+j]*OBMC_TR/100);
			if(OBMC_THRESHOLD && !(predMB_OBMC[256+i*8+j]>=lower && predMB_OBMC[256+i*8+j]<=upper))
				predMB_OBMC[256+i*8+j] = predMB[256+i*8+j];

			predMB_OBMC[256+64+i*8+j] = (predMB[256+64+i*8+j]*H_E_8x8[i-4][j-4] + predMB_LR[64+16+(i-4)*4+(j-4)]*H_LR_8x8[i-4][j-4] + predMB_TD[64+16+(i-4)*4+(j-4)]*H_TD_8x8[i-4][j-4]);
			predMB_OBMC[256+64+i*8+j] = (int)floor(predMB_OBMC[256+64+i*8+j]/8.0 + 0.5);

			upper = predMB[256+64+i*8+j] + (predMB[256+64+i*8+j]*OBMC_TR/100);
			lower = predMB[256+64+i*8+j] - (predMB[256+64+i*8+j]*OBMC_TR/100);
			if(OBMC_THRESHOLD && !(predMB_OBMC[256+64+i*8+j]>=lower && predMB_OBMC[256+64+i*8+j]<=upper))
				predMB_OBMC[256+64+i*8+j] = predMB[256+64+i*8+j];
		}
	}

	free(predMB_LR); free(predMB_TD);
}


int fourmodes_find_mb_ecmode(int predBlocks[],int numMBPerLine,int currMBNum)
{
	int i, predMBNum=0, numIntraNeighbours=0, threshold=ERC_BLOCK_CONCEALED, mb_ecmode=ECMODE1; //default mode
	int mode_A=1, mode_L=1, mode_B=1, mode_R=1;
	int mode_A_available=0, mode_L_available=0, mode_B_available=0, mode_R_available=0;

	for (i = 4; i < 8; i++) 
    {
        if (predBlocks[i] >= threshold) 
        {
			switch (i) 
			{
			case 4:
				predMBNum = currMBNum-numMBPerLine;
				mode_A = img->mb_data[predMBNum].mb_type;
				mode_A_available = 1;
				break;              
			case 5:
				predMBNum = currMBNum-1;
				mode_L = img->mb_data[predMBNum].mb_type;
				mode_L_available = 1;
				break;              
			case 6:
				predMBNum = currMBNum+numMBPerLine;
				mode_B = img->mb_data[predMBNum].mb_type;
				mode_B_available = 1;
				break;              
			case 7:
				predMBNum = currMBNum+1;
				mode_R = img->mb_data[predMBNum].mb_type;
				mode_R_available = 1;
				break;
			}
		}
	}

	//Determine if this MB is to be concealed using SEC or TEC
	if(mode_A_available && mode_A>=9)
		numIntraNeighbours++;
	if(mode_L_available && mode_L>=9)
		numIntraNeighbours++;
	if(mode_B_available && mode_B>=9)
		numIntraNeighbours++;
	if(mode_R_available && mode_R>=9)
		numIntraNeighbours++;

	//DISCUSS - all 4 should be INTRA? 3 is doubtful, even if one nbr has motion, we should use TEC?
	if(numIntraNeighbours>=3) 
	{
		mb_ecmode = SEC;
		//return mb_ecmode;
		return ECMODE1; //change this later...TODO
	}

	//Else, we are now sure that Temporal Error Concealment (TEC) is to be used
	
	//Determine adaptively the block size for this MB to conceal using the nbr mb_type

	if(currMBNum==59)
		currMBNum=59;
    
	if( ((mode_A_available && (mode_A!=8 || mode_A!=3)) || !mode_A_available) &&
		((mode_B_available && (mode_B!=8 || mode_B!=3)) || !mode_B_available) &&
		(mode_L_available && (mode_L==8 || mode_L==2)) &&
		(mode_R_available && (mode_R==8 || mode_R==2)) )
	{
		mb_ecmode = ECMODE2;
	}
	else if( (mode_A_available && (mode_A==8 || mode_A==3)) &&
			 (mode_B_available && (mode_B==8 || mode_B==3)) &&
			 ((mode_L_available && (mode_L!=8 || mode_L!=2)) || !mode_L_available) &&
			 ((mode_R_available && (mode_R!=8 || mode_R!=2)) || !mode_R_available) )
	{
		mb_ecmode = ECMODE3;
	}
	else if( (mode_A_available && (mode_A==8 || mode_A==3)) &&
			 (mode_B_available && (mode_B==8 || mode_B==3)) &&
			 (mode_L_available && (mode_L==8 || mode_L==2)) &&
			 (mode_R_available && (mode_R==8 || mode_R==2)) )
	{
		mb_ecmode = ECMODE4;
	}

	return mb_ecmode;
}
