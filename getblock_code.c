void get_block(int ref_frame, StorablePicture **list, int x_pos, int y_pos, struct img_par *img, int block[BLOCK_SIZE][BLOCK_SIZE])
{

  int dx, dy;
  int x, y;
  int i, j;
  int maxold_x,maxold_y;
  int result;
  int pres_x;
  int pres_y;
  int tmp_res[4][9];
  static const int COEF[6] = {    1, -5, 20, 20, -5, 1  };

  if (list[ref_frame] == no_reference_picture && img->framepoc < img->recovery_poc)
  {
      printf("list[ref_frame] is equal to 'no reference picture' before RAP\n");

      /* fill the block with sample value 128 */
      for (j = 0; j < BLOCK_SIZE; j++)
        for (i = 0; i < BLOCK_SIZE; i++)
          block[i][j] = 128;
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
    for (j = 0; j < BLOCK_SIZE; j++)
      for (i = 0; i < BLOCK_SIZE; i++)
        block[i][j] = list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j))][max(0,min(maxold_x,x_pos+i))];
  }
  else
  { /* other positions */

    if (dy == 0)
    { /* No vertical interpolation */

      for (j = 0; j < BLOCK_SIZE; j++)
      {
        for (i = 0; i < BLOCK_SIZE; i++)
        {
          for (result = 0, x = -2; x < 4; x++)
            result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j))][max(0,min(maxold_x,x_pos+i+x))]*COEF[x+2];
          block[i][j] = max(0, min(img->max_imgpel_value, (result+16)/32));
        }
      }

      if ((dx&1) == 1)
      {
        for (j = 0; j < BLOCK_SIZE; j++)
          for (i = 0; i < BLOCK_SIZE; i++)
            block[i][j] = (block[i][j] + list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j))][max(0,min(maxold_x,x_pos+i+dx/2))] +1 )/2;
      }
    }
    else if (dx == 0)
    {  /* No horizontal interpolation */

      for (j = 0; j < BLOCK_SIZE; j++)
      {
        for (i = 0; i < BLOCK_SIZE; i++)
        {
          for (result = 0, y = -2; y < 4; y++)
            result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j+y))][max(0,min(maxold_x,x_pos+i))]*COEF[y+2];
          block[i][j] = max(0, min(img->max_imgpel_value, (result+16)/32));
        }
      }

      if ((dy&1) == 1)
      {
        for (j = 0; j < BLOCK_SIZE; j++)
          for (i = 0; i < BLOCK_SIZE; i++)
           block[i][j] = (block[i][j] + list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j+dy/2))][max(0,min(maxold_x,x_pos+i))] +1 )/2;
      }
    }
    else if (dx == 2)
    {  /* Vertical & horizontal interpolation */

      for (j = -2; j < BLOCK_SIZE+3; j++)
      {
        for (i = 0; i < BLOCK_SIZE; i++)
          for (tmp_res[i][j+2] = 0, x = -2; x < 4; x++)
            tmp_res[i][j+2] += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j))][max(0,min(maxold_x,x_pos+i+x))]*COEF[x+2];
      }

      for (j = 0; j < BLOCK_SIZE; j++)
      {
        for (i = 0; i < BLOCK_SIZE; i++)
        {
          for (result = 0, y = -2; y < 4; y++)
            result += tmp_res[i][j+y+2]*COEF[y+2];
          block[i][j] = max(0, min(img->max_imgpel_value, (result+512)/1024));
        }
      }

      if ((dy&1) == 1)
      {
        for (j = 0; j < BLOCK_SIZE; j++)
          for (i = 0; i < BLOCK_SIZE; i++)
            block[i][j] = (block[i][j] + max(0, min(img->max_imgpel_value, (tmp_res[i][j+2+dy/2]+16)/32)) +1 )/2;
      }
    }
    else if (dy == 2)
    {  /* Horizontal & vertical interpolation */

      for (j = 0; j < BLOCK_SIZE; j++)
      {
        for (i = -2; i < BLOCK_SIZE+3; i++)
          for (tmp_res[j][i+2] = 0, y = -2; y < 4; y++)
            tmp_res[j][i+2] += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j+y))][max(0,min(maxold_x,x_pos+i))]*COEF[y+2];
      }

      for (j = 0; j < BLOCK_SIZE; j++)
      {
        for (i = 0; i < BLOCK_SIZE; i++)
        {
          for (result = 0, x = -2; x < 4; x++)
            result += tmp_res[j][i+x+2]*COEF[x+2];
          block[i][j] = max(0, min(img->max_imgpel_value, (result+512)/1024));
        }
      }

      if ((dx&1) == 1)
      {
        for (j = 0; j < BLOCK_SIZE; j++)
          for (i = 0; i < BLOCK_SIZE; i++)
            block[i][j] = (block[i][j] + max(0, min(img->max_imgpel_value, (tmp_res[j][i+2+dx/2]+16)/32))+1)/2;
      }
    }
    else
    {  /* Diagonal interpolation */

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
          pres_x = dx == 1 ? x_pos+i : x_pos+i+1;
          pres_x = max(0,min(maxold_x,pres_x));
          for (result = 0, y = -2; y < 4; y++)
            result += list[ref_frame]->imgY[max(0,min(maxold_y,y_pos+j+y))][pres_x]*COEF[y+2];
          block[i][j] = (block[i][j] + max(0, min(img->max_imgpel_value, (result+16)/32)) +1 ) / 2;
        }
      }

    }
  }
}



	if(index==0||index==1||index==2||index==3)
	{
		//Above
	}
	if(index==0||index==4||index==8||index==12)
	{
		//Left
	}

	if(index==12||index==13||index==14||index==15)
	{
		//Below
	}

	if(index==3||index==7||index==11||index==15)
	{
		//Right
	}