/*
 * C_H.c
 *
 *  Created on: 2024��3��4��
 *      Author: xiaoming
 */

#include "zf_common_headfile.h"
#include "stdlib.h"
#include "C_H.h"
#include <math.h>
#include "motor.h"
#include "Fuzzy.h"
//#include "steer.h"
//extern int adcsum;
extern uint16 adcleft;
extern uint16 adcright;

uint8 Stop_car_Flag = 0;

int ImageScanInterval;                         //ɨ�߷�Χ    ��һ�еı߽�+-ImageScanInterval
int ImageScanInterval_Cross;                   //270��������ʮ�ֵ�ɨ�߷�Χ
uint8 Image_Use[LCDH][LCDW];          //�Ҷ�ͼ��
uint8 Pixle[LCDH][LCDW];              //���ڴ���Ķ�ֵ��ͼ��
//uint8 uPixle[uLCDH][uLCDW];                      //������ʾ��ѹ��Ķ�ֵ��ͼ��
//uint8 UImage_Use[uLCDH][uLCDW];                  //������ʾ��ѹ��ĻҶ�ͼ��
static int Ysite = 0, Xsite = 0;                   //Y����=��
static uint8* PicTemp;                             //���浥��ͼ��
static int IntervalLow = 0, IntervalHigh = 0;      //����ߵ�ɨ������
static int ytemp = 0;                              //�����
static int TFSite = 0, FTSite = 0;                 //�����
static float DetR = 0, DetL = 0;                   //���б��
static int BottomBorderRight = 79,                 //59���ұ߽�
BottomBorderLeft = 0,                              //59����߽�
BottomCenter = 0;                                  //59���е�
ImageDealDatatypedef ImageDeal[60];                //��¼���е���Ϣ
ImageStatustypedef ImageStatus;                    //ͼ���ȫ�ֱ���
ImageStatustypedef ImageData;             ///////��Ҫ�޸ĵ�ͼ����ֵ����
SystemDatatypdef SystemData;
ImageFlagtypedef ImageFlag;
uint8 Ring_Help_Flag = 0;                      //����������־
int Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite;   //��Բ���жϵ�����������
int Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite; //��Բ���жϵ�����������
int Point_Xsite,Point_Ysite;                   //�յ��������
int Repair_Point_Xsite,Repair_Point_Ysite;     //���ߵ��������
int forklenth;
int barnlenth;
int ramplenth;
float Weighting[10] = {0.96, 0.92, 0.88, 0.83, 0.77,0.71, 0.65, 0.59, 0.53, 0.47};//10��Ȩ�ز�����������ģ�������Ӱ�죬���°�����̬�ֲ�����
uint8 ExtenLFlag = 0;  //�Ƿ����ӳ���־
uint8 ExtenRFlag = 0;  //�Ƿ����ӳ���־
float ang_l;
int ycircle=0;
int xcircle=0;
int axcircle=0;
int aycircle=0;
int bxcircle=0;
int bycircle=0;
float K;
uint8 Half_Road_Wide[60] =                      //ֱ���������
{  4, 5, 5, 6, 6, 6, 7, 7, 8, 8,
        9, 9,10,10,10,11,12,12,13,13,
       13,14,14,15,15,16,16,17,17,17,
       18,18,19,19,20,20,20,21,21,22,
       23,23,23,24,24,25,25,25,26,26,
       27,28,28,28,29,30,31,31,31,32,
};

uint8 Half_Bend_Wide[60] =                      //����������
{   33,33,33,33,33,33,33,33,33,33,
    33,33,32,32,30,30,29,29,28,27,
    28,27,27,26,26,25,25,24,24,23,
    22,21,21,22,22,22,23,24,24,24,
    25,25,25,26,26,26,27,27,28,28,
    28,29,29,30,30,31,31,32,32,33,
};

uint8 buxianwide[60]={6,7,8,9,10,11,12,13,14,15,
                      16,17,18,19,20,21,22,23,24,25,
                      26,27,28,29,30,31,32,33,34,35,
                      36,37,38,39,40,41,42,43,44,45,
                      46,47,48,49,50,51,52,53,54,55,
                      56,57,58,59,60,61,62,63,64,65};

int adczongzhi;
int tuoluoyijifen;//�������

int Limit(int num, int numH, int numL) {
  if (num > numH)
    num = numH;
  if (num < numL)
    num = numL;
  return num;
}

/*****************ֱ���ж�******************/
float Straight_Judge(uint8 dir, uint8 start, uint8 end)     //���ؽ��С��1��Ϊֱ��
{
    int i;
    float S = 0, Sum = 0, Err = 0, k = 0;
    switch (dir)
    {
    case 1:k = (float)(ImageDeal[start].LeftBorder - ImageDeal[end].LeftBorder) / (start - end);
        for (i = 0; i < end - start; i++)
        {
            Err = (ImageDeal[start].LeftBorder + k * i - ImageDeal[i + start].LeftBorder) * (ImageDeal[start].LeftBorder + k * i - ImageDeal[i + start].LeftBorder);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    case 2:k = (float)(ImageDeal[start].RightBorder - ImageDeal[end].RightBorder) / (start - end);
        for (i = 0; i < end - start; i++)
        {
            Err = (ImageDeal[start].RightBorder + k * i - ImageDeal[i + start].RightBorder) * (ImageDeal[start].RightBorder + k * i - ImageDeal[i + start].RightBorder);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    }
    return S;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��С���˷��������
//  @param      start    ��ʼ��
//  @param      num      ���ݸ���
//  @return     void
//  @since      v1.0
//  Sample usage:  Fit1(119,20);
//-------------------------------------------------------------------------------------------------------------------
void Fit1(int start,int num)
{
    float sum_x = 0.0;
    float sum_y = 0.0;
    float sum_x2 = 0.0;
    float sum_xy = 0.0;
    float a,b;
    float y = 0;

    for (int i = 0; i < num; ++i)
    {
        sum_x += y++;
        sum_y += ImageDeal[start-num+i].Center;
        sum_x2 += (y-1) * (y-1);
        sum_xy += (y-1) * ImageDeal[start-num+i].Center;
    }

    sum_x /= num;
    sum_y /= num;
    sum_x2 /= num;
    sum_xy /= num;

    a = (sum_xy - sum_x * sum_y) / (sum_x2 - sum_x * sum_x);
    b = (sum_x2 * sum_y - sum_x * sum_xy) / (sum_x2 - sum_x * sum_x);
    for(int i = 0;i<num;i++)
    {
     ImageDeal[start-num+i].Center = a*i+b;
    }
}




float Get_angle(float Ax, float Ay, float Bx, float By, float Cx, float Cy)
{

    float BA = 0.00;//����BA��ģ
    float BC = 0.00;
    float SBA_BC = 0.00;//������˵�ֵ
    float angle = 0.00;

    BA = sqrt((Ax-Bx)*(Ax-Bx)+(Ay-By)*(Ay-By));
    BC = sqrt((Cx-Bx)*(Cx-Bx)+(Cy-By)*(Cy-By));

    SBA_BC = (Ax-Bx)*(Cx-Bx)+(Ay-By)*(Cy-By);

    angle =  acos(SBA_BC*1.00/(BA*BC));

    return angle*57.3;
}


float Fit1_k(int start,int num)
{
    float sum_x = 0.0;
    float sum_y = 0.0;
    float sum_x2 = 0.0;
    float sum_xy = 0.0;
    float a,b;
    float y = 0;
    uint16 middle_NO[60] = {0};

    for (int i = 0; i < num; ++i)
    {
        sum_x += y++;
        sum_y += ImageDeal[start-num+i].Center;
        sum_x2 += (y-1) * (y-1);
        sum_xy += (y-1) * ImageDeal[start-num+i].Center;
    }

    sum_x /= num;
    sum_y /= num;
    sum_x2 /= num;
    sum_xy /= num;

    a = (sum_xy - sum_x * sum_y) / (sum_x2 - sum_x * sum_x);
    b = (sum_x2 * sum_y - sum_x * sum_xy) / (sum_x2 - sum_x * sum_x);
    for(int i = 0;i<num;i++)
    {
        middle_NO[start-num+i] = a*i+b;
    }

    float Y = 59-ImageStatus.OFFLine+10-1 - 59-ImageStatus.OFFLine+1;                          //��Y�᳤��
    float X = middle_NO[59-ImageStatus.OFFLine+10-1] - middle_NO[59-ImageStatus.OFFLine+1];    //��X�᳤��(������)
    float k = 1.00*(X/Y);                             //��б��(��y��Ϊ��)
    if(k<0)  k = k*-1;
    return k;
}

float Mh = MT9V03X_H;
float Lh = LCDH;
float Mw = MT9V03X_W;
float Lw = LCDW;

void compressimage() {
  int i, j, row, line;
  const float div_h = Mh / Lh, div_w = Mw / Lw;

  for (i = 0; i < LCDH; i++) {
    row = i * div_h + 0.5;
    for (j = 0; j < LCDW; j++) {
      line = j * div_w + 0.5;
      Image_Use[i][j] =mt9v03x_image[row][line];
    }
  }
  mt9v03x_finish_flag = 0;  //ʹ����һ֡DMA�����ͼ��ͼ��  ���Կ�ʼ������һ֡
}


int HD_thre;  //��ʱ�۲����
//��ֵ��
void Get01change() {
  uint8 thre;
  uint8 i, j;
  for (i = 0; i < LCDH; i++) {
    for (j = 0; j < LCDW; j++) {
      if (j <= 15)
        thre = ImageStatus.Threshold_static - 10;
      else if ((j > 70 && j <= 75))
        thre = ImageStatus.Threshold_static - 15;
      else if (j >= 65)
        thre = ImageStatus.Threshold_static - 15;
      else
        thre = ImageStatus.Threshold_static;

      if (Image_Use[i][j] >
          (thre))  //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
        Pixle[i][j] = 1;  //��
      else
        Pixle[i][j] = 0;  //��
    }
  }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      �Ż��Ĵ��
//  @param      image  ͼ������
//  @param      clo    ��
//  @param      row    ��
//  @param      pixel_threshold ��ֵ����
//  @return     uint8
//  @since      2021.6.23
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8 Threshold_deal(uint8* image,
                     uint16 col,
                     uint16 row,
                     uint32 pixel_threshold) {
#define GrayScale 256
  uint16 width = col;
  uint16 height = row;
  int pixelCount[GrayScale];
  float pixelPro[GrayScale];
  int i, j, pixelSum = width * height;
  uint8 threshold = 0;
  uint8* data = image;  //ָ���������ݵ�ָ��
  for (i = 0; i < GrayScale; i++) {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32 gray_sum = 0;
  //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
  for (i = 0; i < height; i += 1) {
    for (j = 0; j < width; j += 1) {
      // if((sun_mode&&data[i*width+j]<pixel_threshold)||(!sun_mode))
      //{
      pixelCount[(
          int)data[i * width + j]]++;  //����ǰ�ĵ������ֵ��Ϊ����������±�
      gray_sum += (int)data[i * width + j];  //�Ҷ�ֵ�ܺ�
      //}
    }
  }

  //����ÿ������ֵ�ĵ�������ͼ���еı���
  for (i = 0; i < GrayScale; i++) {
    pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }


  //�����Ҷȼ�[0,255]
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (j = 0; j < pixel_threshold; j++) {
    w0 +=
        pixelPro[j];  //��������ÿ���Ҷ�ֵ�����ص���ռ����֮�� ���������ֵı���
    u0tmp += j * pixelPro[j];  //�������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;    //����ƽ���Ҷ�
    u1 = u1tmp / w1;    //ǰ��ƽ���Ҷ�
    u = u0tmp + u1tmp;  //ȫ��ƽ���Ҷ�
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax) {
      deltaMax = deltaTmp;
      threshold = j;
    }
    if (deltaTmp < deltaMax) break;
  }
  return threshold;
}

void Get01change_dajin() {
  ImageStatus.Threshold = Threshold_deal(Image_Use[0], LCDW, LCDH, ImageStatus.Threshold_detach);
  if (ImageStatus.Threshold < ImageStatus.Threshold_static)
    ImageStatus.Threshold = ImageStatus.Threshold_static;
  uint8 i, j = 0;
  uint8 thre;
  for (i = 0; i < LCDH; i++) {
    for (j = 0; j < LCDW; j++) {
      if (j <= 15)
        thre = ImageStatus.Threshold - 10;
      else if ((j > 70 && j <= 75))
        thre = ImageStatus.Threshold - 10;
      else if (j >= 65)
        thre = ImageStatus.Threshold - 10;
      else
        thre = ImageStatus.Threshold;

      if (Image_Use[i][j] >(thre))         //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
        Pixle[i][j] = 1;  //��
      else
        Pixle[i][j] = 0;  //��
    }
  }
}


//�����˲�
void Pixle_Filter() {
  int nr;  //��
  int nc;  //��

  for (nr = 10; nr < 40; nr++) {
    for (nc = 10; nc < 70; nc = nc + 1) {
      if ((Pixle[nr][nc] == 0) && (Pixle[nr - 1][nc] + Pixle[nr + 1][nc] +
                                       Pixle[nr][nc + 1] + Pixle[nr][nc - 1] >=
                                   3)) {
        Pixle[nr][nc] = 1;
      }
    }
  }
}

void GetJumpPointFromDet(uint8* p,uint8 type,int L,int H,JumpPointtypedef* Q)  //��һ��������Ҫ���ҵ����飨80���㣩
                                                                               //�ڶ���ɨ����߻���ɨ�ұ���
{                                                                              //�����ǿ�ʼ�ͽ�����
  int i = 0;
  if (type == 'L')                              //ɨ�������
  {
    for (i = H; i >= L; i--) {
      if (*(p + i) == 1 && *(p + i - 1) != 1)   //�ɺڱ��
      {
        Q->point = i;                           //��¼�����
        Q->type = 'T';                          //��ȷ����
        break;
      } else if (i == (L + 1))                  //����ɨ�����Ҳû�ҵ�
      {
        if (*(p + (L + H) / 2) != 0)            //����м��ǰ׵�
        {
          Q->point = (L + H) / 2;               //��Ϊ��������е�
          Q->type = 'W';                        //����ȷ�������м�Ϊ�ף���Ϊû�б�
          break;
        } else                                  //����ȷ�������м�Ϊ��
        {
          Q->point = H;                         //����м��Ǻڵ�
          Q->type = 'H';                        //�����ֱ�����ֵ����Ϊ�Ǵ�����
          break;
        }
      }
    }
  } else if (type == 'R')                       //ɨ���ұ���
  {
    for (i = L; i <= H; i++)                    //��������ɨ
    {
      if (*(p + i) == 1 && *(p + i + 1) != 1)   //���ɺڵ��׵�����
      {
        Q->point = i;                           //��¼
        Q->type = 'T';
        break;
      } else if (i == (H - 1))                  //����ɨ�����Ҳû�ҵ�
      {
        if (*(p + (L + H) / 2) != 0)            //����м��ǰ׵�
        {
          Q->point = (L + H) / 2;               //�ұ������е�
          Q->type = 'W';
          break;
        } else                                  //����е��Ǻڵ�
        {
          Q->point = L;                         //�����ֱ�����ֵ
          Q->type = 'H';
          break;
        }
      }
    }
  }
}





static uint8 DrawLinesFirst(void) {
  PicTemp = Pixle[59];
  if (*(PicTemp + ImageSensorMid) == 0)                 //����ױ�ͼ���е�Ϊ�ڣ��쳣���
  {
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++)    //�����ұ���
    {
      if (*(PicTemp + ImageSensorMid - Xsite) != 0)     //һ���ҵ���������������ľ��룬��break
        break;                                          //���Ҽ�¼Xsite
      if (*(PicTemp + ImageSensorMid + Xsite) != 0)
        break;
    }

    if (*(PicTemp + ImageSensorMid - Xsite) != 0)       //�����������ߵĻ�
    {
      BottomBorderRight = ImageSensorMid - Xsite + 1;   // 59���ұ�������
      for (Xsite = BottomBorderRight; Xsite > 0; Xsite--)  //��ʼ��59�������
      {
        if (*(PicTemp + Xsite) == 0 &&
            *(PicTemp + Xsite - 1) == 0)                //���������ڵ㣬�˲�
        {
          BottomBorderLeft = Xsite;                     //������ҵ�
          break;
        } else if (Xsite == 1) {
          BottomBorderLeft = 0;                         //����������ˣ�����������ߣ��������Ϊ��0
          break;
        }
      }
    } else if (*(PicTemp + ImageSensorMid + Xsite) != 0)  //����������ұߵĻ�
    {
      BottomBorderLeft = ImageSensorMid + Xsite - 1;    // 59�����������
      for (Xsite = BottomBorderLeft; Xsite < 79; Xsite++)  //��ʼ��59�������
      {
        if (  *(PicTemp + Xsite) == 0
            &&*(PicTemp + Xsite + 1) == 0)              //���������ڵ㣬�˲�
        {
          BottomBorderRight = Xsite;                    //�ұ����ҵ�
          break;
        } else if (Xsite == 78) {
          BottomBorderRight = 79;                       //����������ˣ��������ұ��ߣ��������Ϊ��79
          break;
        }
      }
    }
  }
  else                                                //������е��ǰ׵ģ��Ƚ����������
  {
    for (Xsite = 79; Xsite >ImageSensorMid; Xsite--)   //һ����һ����������ұ���
    {
      if (  *(PicTemp + Xsite) == 1
          &&*(PicTemp + Xsite - 1) == 1)                //���������ڵ㣬�˲�     //�����׵�
      {
        BottomBorderRight = Xsite;                      //�ҵ��ͼ�¼
        break;
      } else if (Xsite == 40) {
        BottomBorderRight = 39;                         //�Ҳ�����Ϊ79
        break;
      }
    }
    for (Xsite = 0; Xsite < ImageSensorMid; Xsite++)    //һ����һ��������������
    {
      if (  *(PicTemp + Xsite) == 1
          &&*(PicTemp + Xsite + 1) == 1)                //���������ڵ㣬�˲�
      {
        BottomBorderLeft = Xsite;                       //�ҵ��ͼ�¼
        break;
      } else if (Xsite == 38) {
        BottomBorderLeft = 39;                           //�Ҳ�����Ϊ0
        break;
      }
    }
  }
  BottomCenter =(BottomBorderLeft + BottomBorderRight) / 2;   // 59���е�ֱ��ȡƽ��
  ImageDeal[59].LeftBorder = BottomBorderLeft;                //�����������¼һ����Ϣ����һ������һ�����
  ImageDeal[59].RightBorder = BottomBorderRight;
  ImageDeal[59].Center = BottomCenter;                        //ȷ����ױ�
  ImageDeal[59].Wide = BottomBorderRight - BottomBorderLeft;  //�洢�����Ϣ
  ImageDeal[59].IsLeftFind = 'T';
  ImageDeal[59].IsRightFind = 'T';
  for (Ysite = 58; Ysite > 54; Ysite--)                       //���м�������ȷ���ױ�����
  {
    PicTemp = Pixle[Ysite];
    for (Xsite = 79; Xsite > ImageDeal[Ysite + 1].Center;Xsite--)                                             //��ǰ��һ��������
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite - 1) == 1) {
        ImageDeal[Ysite].RightBorder = Xsite;
        break;
      } else if (Xsite == (ImageDeal[Ysite + 1].Center+1)) {
        ImageDeal[Ysite].RightBorder = ImageDeal[Ysite + 1].Center;
        break;
      }
    }
    for (Xsite = 0; Xsite < ImageDeal[Ysite + 1].Center;Xsite++)                                             //��ǰ��һ��������
    {
      if (*(PicTemp + Xsite) == 1 && *(PicTemp + Xsite + 1) == 1) {
        ImageDeal[Ysite].LeftBorder = Xsite;
        break;
      } else if (Xsite == (ImageDeal[Ysite + 1].Center-1)) {
        ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite + 1].Center;
        break;
      }
    }
    ImageDeal[Ysite].IsLeftFind = 'T';                        //��Щ��Ϣ�洢��������
    ImageDeal[Ysite].IsRightFind = 'T';
    ImageDeal[Ysite].Center =
        (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) /2; //�洢�е�
    ImageDeal[Ysite].Wide =
        ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;      //�洢���
  }
  return 'T';
}                                                             //�������Ҫ��������������Ȳ����ܵ����ţ�����Ҫ�ڰ�װ��ʱ���������ͷ���ӽ�

/*����׷����µõ�ȫ������*/
static void DrawLinesProcess(void)  //////���ø���
{
  uint8 L_Found_T = 'F';  //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־
  uint8 Get_L_line = 'F';  //�ҵ���һ֡ͼ��Ļ�׼��б��
  uint8 R_Found_T = 'F';  //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־
  uint8 Get_R_line = 'F';  //�ҵ���һ֡ͼ��Ļ�׼��б��
  float D_L = 0;           //�ӳ��������б��
  float D_R = 0;           //�ӳ����ұ���б��
  int ytemp_W_L;           //��ס�״��󶪱���
  int ytemp_W_R;           //��ס�״��Ҷ�����
  ExtenRFlag = 0;          //��־λ��0
  ExtenLFlag = 0;
   ImageStatus.Left_Line = 0;
   ImageStatus.WhiteLine = 0;
   ImageStatus.Right_Line = 0;
  for (Ysite = 54 ; Ysite > ImageStatus.OFFLine; Ysite--)            //ǰ5�д�����ˣ������55�е����趨�Ĳ��������OFFLine��
  {                        //̫Զ��ͼ���ȶ���OFFLine�Ժ�Ĳ�����
    PicTemp = Pixle[Ysite];
    JumpPointtypedef JumpPoint[2];                                          // 0��1��
    if (ImageStatus.Road_type != Cross_ture
           /* &&SystemData.SpeedData.Length*OX>500*/) {
      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval;             //����һ���ұ���-Interval�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
      IntervalHigh =ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;           //����һ���ұ���+Interval�ĵ������ȷ��ɨ������㣩
    } else {
      IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval_Cross;       //����һ���ұ���-Interval_Cross�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
      IntervalHigh = ImageDeal[Ysite + 1].RightBorder + ImageScanInterval_Cross;    //����һ���ұ���+Interval_Cross�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
    }

    LimitL(IntervalLow);   //ȷ����ɨ�����䲢��������
    LimitH(IntervalHigh);  //ȷ����ɨ�����䲢��������
    GetJumpPointFromDet(PicTemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);     //ɨ�ұ���

    IntervalLow =ImageDeal[Ysite + 1].LeftBorder -ImageScanInterval;                //����һ�������-5�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
    IntervalHigh =ImageDeal[Ysite + 1].LeftBorder +ImageScanInterval;               //����һ�������+5�ĵ������ȷ��ɨ������㣩

    LimitL(IntervalLow);   //ȷ����ɨ�����䲢��������
    LimitH(IntervalHigh);  //ȷ����ɨ�����䲢��������
    GetJumpPointFromDet(PicTemp, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);

    if (JumpPoint[0].type =='W')                                                    //�����������߲��������䣬����10���㶼�ǰ׵�
    {
      ImageDeal[Ysite].LeftBorder =ImageDeal[Ysite + 1].LeftBorder;                 //�������������һ�е���ֵ
    } else                                                                          //���������
    {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;                             //��¼������
    }

    if (JumpPoint[1].type == 'W')                                                   //��������ұ��߲���������
    {
      ImageDeal[Ysite].RightBorder =ImageDeal[Ysite + 1].RightBorder;               //�����ұ�������һ�е���ֵ
    } else                                                                          //�ұ�������
    {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point;                            //��¼������
    }

    ImageDeal[Ysite].IsLeftFind =JumpPoint[0].type;                                 //��¼�����Ƿ��ҵ����ߣ�����������
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;

    //����ȷ����Щ������ı�Ե
    if (( ImageDeal[Ysite].IsLeftFind == 'H'
         ||ImageDeal[Ysite].IsRightFind == 'H')) {
      if (ImageDeal[Ysite].IsLeftFind == 'H')                                   //�������ߴ�����
        for (Xsite = (ImageDeal[Ysite].LeftBorder + 1);
             Xsite <= (ImageDeal[Ysite].RightBorder - 1);
             Xsite++)                                                           //���ұ���֮������ɨ��
        {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite + 1) != 0)) {
            ImageDeal[Ysite].LeftBorder =Xsite;                                 //�����һ������ߵ��ұ��кڰ�������Ϊ���Ա���ֱ��ȡ��
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          } else if (*(PicTemp + Xsite) != 0)                                   //һ�����ְ׵���ֱ������
            break;
          else if (Xsite ==(ImageDeal[Ysite].RightBorder - 1))
          {
             ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          }
        }
      if ((ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder) <=
          7)                              //ͼ�����޶�
      {
        ImageStatus.OFFLine = Ysite + 1;  //������б�7С�˺���ֱ�Ӳ�Ҫ��
        break;
      }
      if (ImageDeal[Ysite].IsRightFind == 'H')
        for (Xsite = (ImageDeal[Ysite].RightBorder - 1);
             Xsite >= (ImageDeal[Ysite].LeftBorder + 1); Xsite--) {
          if ((*(PicTemp + Xsite) == 0) && (*(PicTemp + Xsite - 1) != 0)) {
            ImageDeal[Ysite].RightBorder =
                Xsite;                    //����ұ��ߵ���߻��кڰ�������Ϊ���Ա���ֱ��ȡ��
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          } else if (*(PicTemp + Xsite) != 0)
            break;
          else if (Xsite == (ImageDeal[Ysite].LeftBorder + 1))
          {
            ImageDeal[Ysite].RightBorder = Xsite;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
        }
    }

 /***********����ȷ���ޱ���************/
    int ysite = 0;
    uint8 L_found_point = 0;
    uint8 R_found_point = 0;


    if(  ImageStatus.Road_type != Ramp)
    {
    if (    ImageDeal[Ysite].IsRightFind == 'W'
          &&Ysite > 10
          &&Ysite < 50
          &&ImageStatus.Road_type!=Barn_in
          )                     //������ֵ��ޱ���
    {
      if (Get_R_line == 'F')    //��һ֡ͼ��û���ܹ�����һ�׼�ߵĴ���β�����
      {
        Get_R_line = 'T';       //����  һ֡ͼ��ֻ��һ�� ��ΪT
        ytemp_W_R = Ysite + 2;
        for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
          if (ImageDeal[ysite].IsRightFind =='T')  //���ޱ�����������  һ�㶼���бߵ�
            R_found_point++;
        }
        if (R_found_point >8)                      //�ҵ���׼б�ʱ�  ���ӳ�������ȷ���ޱ�   ���бߵĵ�������8
        {
          D_R = ((float)(ImageDeal[Ysite + R_found_point].RightBorder - ImageDeal[Ysite + 3].RightBorder)) /((float)(R_found_point - 3));
                                                  //��������Щ����������б��
                                                  //�ø��ޱ������ӳ��������׼
          if (D_R > 0) {
            R_Found_T ='T';                       //���б�ʴ���0  ��ô�ҵ��������׼��  ��Ϊ���λ���
                                                  //����һ���������б�ʴ���0  С��0�����Ҳ�����ӳ� û��Ҫ
          } else {
            R_Found_T = 'F';                      //û���ҵ������׼��
            if (D_R < 0)
              ExtenRFlag = 'F';                   //�����־λ����ʮ�ֽǵ㲹��  ��ֹͼ�����õ�
          }
        }
      }
      if (R_Found_T == 'T')
        ImageDeal[Ysite].RightBorder =ImageDeal[ytemp_W_R].RightBorder -D_R * (ytemp_W_R - Ysite);  //����ҵ��� ��ô�Ի�׼�����ӳ���

      LimitL(ImageDeal[Ysite].RightBorder);  //�޷�
      LimitH(ImageDeal[Ysite].RightBorder);  //�޷�
    }

    if (ImageDeal[Ysite].IsLeftFind == 'W' && Ysite > 10 && Ysite < 50 &&
        ImageStatus.Road_type != Barn_in)    //����ͬ��  ��߽�
    {
      if (Get_L_line == 'F') {
        Get_L_line = 'T';
        ytemp_W_L = Ysite + 2;
        for (ysite = Ysite + 1; ysite < Ysite + 15; ysite++) {
          if (ImageDeal[ysite].IsLeftFind == 'T')
            L_found_point++;
        }
        if (L_found_point > 8)              //�ҵ���׼б�ʱ�  ���ӳ�������ȷ���ޱ�
        {
          D_L = ((float)(ImageDeal[Ysite + 3].LeftBorder -ImageDeal[Ysite + L_found_point].LeftBorder)) /((float)(L_found_point - 3));
          if (D_L > 0) {
            L_Found_T = 'T';

          } else {
            L_Found_T = 'F';
            if (D_L < 0)
              ExtenLFlag = 'F';
          }
        }
      }

      if (L_Found_T == 'T')
        ImageDeal[Ysite].LeftBorder =ImageDeal[ytemp_W_L].LeftBorder + D_L * (ytemp_W_L - Ysite);

      LimitL(ImageDeal[Ysite].LeftBorder);  //�޷�
      LimitH(ImageDeal[Ysite].LeftBorder);  //�޷�
    }
}
    if (ImageDeal[Ysite].IsLeftFind == 'W'&&ImageDeal[Ysite].IsRightFind == 'W')
         {
             ImageStatus.WhiteLine++;  //Ҫ�����Ҷ��ޱߣ�������+1
         }
        if (ImageDeal[Ysite].IsLeftFind == 'W'&&Ysite<55)
        {
             ImageStatus.Left_Line++;
        }
        if (ImageDeal[Ysite].IsRightFind == 'W'&&Ysite<55)
        {
             ImageStatus.Right_Line++;
        }


      LimitL(ImageDeal[Ysite].LeftBorder);   //�޷�
      LimitH(ImageDeal[Ysite].LeftBorder);   //�޷�
      LimitL(ImageDeal[Ysite].RightBorder);  //�޷�
      LimitH(ImageDeal[Ysite].RightBorder);  //�޷�

      ImageDeal[Ysite].Wide =ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;

    if (ImageDeal[Ysite].Wide <= 7)         //����ȷ�����Ӿ���
    {
      ImageStatus.OFFLine = Ysite + 1;
      break;
    }

    else if (  ImageDeal[Ysite].RightBorder <= 10
             ||ImageDeal[Ysite].LeftBorder >= 70) {
              ImageStatus.OFFLine = Ysite + 1;
              break;
    }                                        //��ͼ����С��0�������ұߴﵽһ��������ʱ������ֹѲ��
  }


  return;
}

//�ӳ��߻��ƣ���������˵�Ǻ�׼ȷ��
static void DrawExtensionLine(void)        //�����ӳ��߲�����ȷ������ ���Ѳ��߲���б��
{
  if ((
        ImageStatus.Road_type != Barn_in
        &&ImageStatus.Road_type != Ramp)
//        &&ImageStatus.pansancha_Lenth* OX==0
        &&ImageStatus.Road_type !=LeftCirque
        &&ImageStatus.Road_type !=RightCirque
        )                                  // g5.22  6.22����ע��  �ǵøĻ���
  {
    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
      TFSite = 55;
//    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == LeftCirque)
//      TFSite = 55;
    if (ExtenLFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--)                       //�ӵ����п�ʼ����ɨɨ��������������   ��β���
                                          //������ֻ��һ��
      {
        PicTemp = Pixle[Ysite];           //�浱ǰ��
        if (ImageDeal[Ysite].IsLeftFind =='W')                          //���������߽�ûɨ����ɨ�����ǰ�ɫ��˵������û����߽��
        {
          //**************************************************//**************************************************
          if (ImageDeal[Ysite + 1].LeftBorder >= 70)                    //�����߽�ʵ����̫�ұ�
          {
            ImageStatus.OFFLine = Ysite + 1;
            break;                        //ֱ�����������������
          }
          //************************************************//*************************************************

          while (Ysite >= (ImageStatus.OFFLine + 4))                    //��ʱ��ûɨ������
          {
            Ysite--;                      //��������ɨ
            if (  ImageDeal[Ysite].IsLeftFind == 'T'
                &&ImageDeal[Ysite - 1].IsLeftFind == 'T'
                &&ImageDeal[Ysite - 2].IsLeftFind == 'T'
                &&ImageDeal[Ysite - 2].LeftBorder > 0
                &&ImageDeal[Ysite - 2].LeftBorder <70
                )                                                       //���ɨ�����г����˲��ұ��������������ж�����߽�㣨��߽��ڿհ��Ϸ���
            {
              FTSite = Ysite - 2;          //�ѱ�������ĵڶ��д���FTsite
              break;
            }
          }

          DetL =
              ((float)(ImageDeal[FTSite].LeftBorder -
                       ImageDeal[TFSite].LeftBorder)) /
              ((float)(FTSite - TFSite));  //��߽��б�ʣ��е������/�е������
          if (FTSite > ImageStatus.OFFLine)
            for (
                ytemp = TFSite; ytemp >= FTSite; ytemp--)               //�ӵ�һ��ɨ������߽������ڶ��е����꿪ʼ����ɨֱ���հ��Ϸ�����߽��������ֵ
            {
              ImageDeal[ytemp].LeftBorder =
                  (int)(DetL * ((float)(ytemp - TFSite))) +
                  ImageDeal[TFSite]
                      .LeftBorder;                                      //�����ڼ�Ŀհ״����ߣ���б�ߣ���Ŀ���Ƿ���ͼ����
            }
        } else
          TFSite = Ysite + 2;                                           //���ɨ���˱��е���߽磬���д��������棬����б�ʣ�
      }

    if (ImageStatus.WhiteLine >= ImageStatus.TowPoint_True - 15)
      TFSite = 55;
    // g5.22
    if (ImageStatus.CirqueOff == 'T' && ImageStatus.Road_type == RightCirque)
      TFSite = 55;
    if (ExtenRFlag != 'F')
      for (Ysite = 54; Ysite >= (ImageStatus.OFFLine + 4);
           Ysite--)               //�ӵ����п�ʼ����ɨɨ��������������
      {
        PicTemp = Pixle[Ysite];  //�浱ǰ��

        if (ImageDeal[Ysite].IsRightFind =='W')                       //��������ұ߽�ûɨ����ɨ�����ǰ�ɫ��˵������û���ұ߽�㣬���Ǵ��������ڵ�
        {
          if (ImageDeal[Ysite + 1].RightBorder <= 10)                 //����ұ߽�ʵ����̫���
          {
            ImageStatus.OFFLine =Ysite + 1;                           //ֱ��������˵�����������������������
            break;
          }
          while (Ysite >= (ImageStatus.OFFLine + 4))                  //��ʱ��ûɨ��������������
          {
            Ysite--;
            if (  ImageDeal[Ysite].IsRightFind == 'T'
                &&ImageDeal[Ysite - 1].IsRightFind == 'T'
                &&ImageDeal[Ysite - 2].IsRightFind == 'T'
                &&ImageDeal[Ysite - 2].RightBorder < 70
                &&ImageDeal[Ysite - 2].RightBorder > 10
                )                                                      //���ɨ�����г����˲��ұ��������������ж�����߽�㣨��߽��ڿհ��Ϸ���
            {
              FTSite = Ysite - 2;                                      // �ѱ�������ĵڶ��д���FTsite
              break;
            }
          }

          DetR =((float)(ImageDeal[FTSite].RightBorder -ImageDeal[TFSite].RightBorder)) /((float)(FTSite - TFSite));         //�ұ߽��б�ʣ��е������/�е������
          if (FTSite > ImageStatus.OFFLine)
            for (ytemp = TFSite; ytemp >= FTSite;ytemp--)              //�ӵ�һ��ɨ�����ұ߽������ڶ��е����꿪ʼ����ɨֱ���հ��Ϸ����ұ߽��������ֵ
            {
              ImageDeal[ytemp].RightBorder =(int)(DetR * ((float)(ytemp - TFSite))) +ImageDeal[TFSite].RightBorder;          //�����ڼ�Ŀհ״����ߣ���б�ߣ���Ŀ���Ƿ���ͼ����
            }
        } else
          TFSite =Ysite +2;                                           //������е��ұ߽��ҵ��ˣ���Ѹ�������ڶ��������͸�TFsite
      }
  }
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) {
    ImageDeal[Ysite].Center =(ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) /2;                                //ɨ�����������һ�龭�Ż�֮����м�ֵ����
    ImageDeal[Ysite].Wide =-ImageDeal[Ysite].LeftBorder +ImageDeal[Ysite].RightBorder;                                       //���Ż�֮��Ŀ�ȴ���
  }
}
/*�Ͻ��������ַ���ɨ�ߣ���Ϊ����Բ�����ж�Ԫ�صĵڶ�����*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Bottom_Line_OTSU
//  @brief          ��ȡ�ײ����ұ���
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        �����ͼ������
//  @param          Row                                     ͼ���Ysite
//  @param          Col                                     ͼ���Xsite
//  @return         Bottonline                              �ױ���ѡ��
//  @time           2022��10��9��
//  @Author
//  Sample usage:   Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Bottom_Line_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{

    //Ѱ����߽߱�
    for (int Xsite = Col / 2-2; Xsite > 1; Xsite--)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite - 1] == 0)
        {
            ImageDeal[Bottonline].LeftBoundary = Xsite;//��ȡ�ױ������
            break;
        }
    }
    for (int Xsite = Col / 2+2; Xsite < LCDW-1; Xsite++)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite + 1] == 0)
        {
            ImageDeal[Bottonline].RightBoundary = Xsite;//��ȡ�ױ��ұ���
            break;
        }
    }

}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Left_and_Right_Lines
//  @brief          ͨ��sobel��ȡ���ұ���
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        �����ͼ������
//  @param          Row                                     ͼ���Ysite
//  @param          Col                                     ͼ���Xsite
//  @param          Bottonline                              �ױ���ѡ��
//  @return         ��
//  @time           2022��10��7��
//  @Author
//  Sample usage:   Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Left_and_Right_Lines(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    //����С�˵ĵ�ǰ����״̬λ��Ϊ �� �� �� �� һ��Ҫ�� �ϣ����Ϊ��ɫ ���ϱ�Ϊ��ɫ �£��ұ�Ϊɫ  �ң������к�ɫ
/*  ǰ�������壺
                *   0
                * 3   1
                *   2
*/
/*Ѱ�����������*/
    uint8 Left_Rule[2][8] = {
                                  {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},  (x,y )
                                  {-1,-1,1,-1,1,1,-1,1} //{-1,-1},{1,-1},{1,1},{-1,1}
    };
    /*Ѱ�����������*/
    int Right_Rule[2][8] = {
                              {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},
                              {1,-1,1,1,-1,1,-1,-1} //{1,-1},{1,1},{-1,1},{-1,-1}
    };
      int num=0;
    uint8 Left_Ysite = Bottonline;
    uint8 Left_Xsite = ImageDeal[Bottonline].LeftBoundary;
    uint8 Left_Rirection = 0;//��߷���
    uint8 Pixel_Left_Ysite = Bottonline;
    uint8 Pixel_Left_Xsite = 0;

    uint8 Right_Ysite = Bottonline;
    uint8 Right_Xsite = ImageDeal[Bottonline].RightBoundary;
    uint8 Right_Rirection = 0;//�ұ߷���
    uint8 Pixel_Right_Ysite = Bottonline;
    uint8 Pixel_Right_Xsite = 0;
    uint8 Ysite = Bottonline;
    ImageStatus.OFFLineBoundary = 5;
    while (1)
    {
            num++;
            if(num>400)
            {
                 ImageStatus.OFFLineBoundary = Ysite;
                break;
            }
        if (Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite)
        {
            if (Ysite < ImageStatus.OFFLineBoundary)
            {
                ImageStatus.OFFLineBoundary = Ysite;
                break;
            }
            else
            {
                Ysite--;
            }
        }
        /*********���Ѳ��*******/
        if ((Pixel_Left_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//�ұ�ɨ��
        {
            /*����ǰ������*/
            Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
            Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];

            if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//ǰ���Ǻ�ɫ
            {
                //˳ʱ����ת90
                if (Left_Rirection == 3)
                    Left_Rirection = 0;
                else
                    Left_Rirection++;
            }
            else//ǰ���ǰ�ɫ
            {
                /*������ǰ������*/
                Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];

                if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//��ǰ��Ϊ��ɫ
                {
                    //���򲻱�  Left_Rirection
                    Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0){
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                        ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                    }
                }
                else//��ǰ��Ϊ��ɫ
                {
                    // �������ı� Left_Rirection  ��ʱ��90��
                    Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0 )
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                    if (Left_Rirection == 0)
                        Left_Rirection = 3;
                    else
                        Left_Rirection--;
                }

            }
        }
        /*********ʱ��-����һ�ſ�Դ*******/
        /*********�ұ�Ѳ��*******/
        if ((Pixel_Right_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//�ұ�ɨ��
        {
            /*����ǰ������*/
            Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
            Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];

            if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//ǰ���Ǻ�ɫ
            {
                //��ʱ����ת90
                if (Right_Rirection == 0)
                    Right_Rirection = 3;
                else
                    Right_Rirection--;
            }
            else//ǰ���ǰ�ɫ
            {
                /*������ǰ������*/
                Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];

                if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//��ǰ��Ϊ��ɫ
                {
                    //���򲻱�  Right_Rirection
                    Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == 79 )
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                }
                else//��ǰ��Ϊ��ɫ
                {
                    // �������ı� Right_Rirection  ��ʱ��90��
                    Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == 79)
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                    if (Right_Rirection == 3)
                        Right_Rirection = 0;
                    else
                        Right_Rirection++;
                }

            }
        }

        if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 3)//Ysite<80��Ϊ�˷��ڵײ��ǰ�����ɨ�����  3 && Ysite < 30
        {

            ImageStatus.OFFLineBoundary = Ysite;
            break;
        }

    }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------
//  @name           Search_Border_OTSU
//  @brief          ͨ��OTSU��ȡ���� ����Ϣ
//  @param          imageInput[IMAGE_ROW][IMAGE_COL]        �����ͼ������
//  @param          Row                                     ͼ���Ysite
//  @param          Col                                     ͼ���Xsite
//  @param          Bottonline                              �ױ���ѡ��
//  @return         ��
//  @time           2022��10��7��
//  @Author
//  Sample usage:   Search_Border_OTSU(mt9v03x_image, IMAGE_ROW, IMAGE_COL, IMAGE_ROW-8);
//--------------------------------------------------------------------------------------------------------------------------------------------

void Search_Border_OTSU(uint8 imageInput[LCDH][LCDW], uint8 Row, uint8 Col, uint8 Bottonline)
{
    ImageStatus.WhiteLine_L = 0;
    ImageStatus.WhiteLine_R = 0;
    //ImageStatus.OFFLine = 1;
    /*�����±߽紦��*/
    for (int Xsite = 0; Xsite < LCDW; Xsite++)
    {
        imageInput[0][Xsite] = 0;
        imageInput[Bottonline + 1][Xsite] = 0;
    }
    /*�����ұ߽紦��*/
    for (int Ysite = 0; Ysite < LCDH; Ysite++)
    {
            ImageDeal[Ysite].LeftBoundary_First = 0;
            ImageDeal[Ysite].RightBoundary_First = 79;

            imageInput[Ysite][0] = 0;
            imageInput[Ysite][LCDW - 1] = 0;
    }
    /********��ȡ�ײ�����*********/
    Search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);
    /********��ȡ���ұ���*********/
    Search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);



    for (int Ysite = Bottonline; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary < 3)
        {
            ImageStatus.WhiteLine_L++;
        }
        if (ImageDeal[Ysite].RightBoundary > LCDW - 3)
        {
            ImageStatus.WhiteLine_R++;
        }
    }
}

//���ֶ��ߵ�ʱ��  ����ȷ���ޱ��е�����
static void RouteFilter(void) {
  for (Ysite = 58; Ysite >= (ImageStatus.OFFLine + 5);
       Ysite--)                                     //�ӿ�ʼλ��ֹͣλ
  {
    if (   ImageDeal[Ysite].IsLeftFind == 'W'
         &&ImageDeal[Ysite].IsRightFind == 'W'
         &&Ysite <= 45
         &&ImageDeal[Ysite - 1].IsLeftFind == 'W'
         &&ImageDeal[Ysite - 1].IsRightFind =='W')  //��ǰ�����Ҷ��ޱߣ�������ǰ45��   �˲�
    {
      ytemp = Ysite;
      while (ytemp >= (ImageStatus.OFFLine +5))     // �ĸ����ԣ�-6Ч����һЩ
      {
        ytemp--;
        if (  ImageDeal[ytemp].IsLeftFind == 'T'
            &&ImageDeal[ytemp].IsRightFind == 'T')  //Ѱ�����߶������ģ��ҵ��뱾������ľͲ�����
        {
          DetR = (float)(ImageDeal[ytemp - 1].Center - ImageDeal[Ysite + 2].Center) /(float)(ytemp - 1 - Ysite - 2);          //��б��
          int CenterTemp = ImageDeal[Ysite + 2].Center;
          int LineTemp = Ysite + 2;
          while (Ysite >= ytemp) {
            ImageDeal[Ysite].Center =(int)(CenterTemp +DetR * (float)(Ysite - LineTemp));                                     //��б�ʲ�
            Ysite--;
          }
          break;
        }
      }
    }
    ImageDeal[Ysite].Center =(ImageDeal[Ysite - 1].Center + 2 * ImageDeal[Ysite].Center) /3;                                  //��ƽ����Ӧ�û�Ƚϻ�  ��������������ƽ��
  }
}

int icm_start_test_cross = 0;  //����icm���ֱ�־λ

/****ʮ�ּ��*****/  //������㷨�Ѿ��˳���ʮ��  �൱�ڲ�Ҫ����
void Cross_Test() {
  if (ImageStatus.OFFLine > 15)                       //�����Ӿ���ȽϽ���ʱ�� �������ִ���  �������⴦��270�����֮���ʮ��
    icm_start_test_cross = 1;
  else
    icm_start_test_cross = 0;

  if (abs(tuoluoyijifen) > 100)
    ImageStatus.Road_type = Cross;
//  if (ImageStatus.Road_type == Cross && ImageStatus.Cross_Lenth * OX > 100)
//    ImageStatus.Road_type = 0;
}

/****Բ�����***/
uint8 Pass_flag = 'F';
//uint8 Left_Less_Num = 0;
//�������
int ceshi_flag;
//--------------------------------------------------------------
//  @name           Element_Judgment_Left_Rings()
//  @brief          ����ͼ���жϵ��Ӻ����������ж���Բ������.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Judgment_Left_Rings();
//--------------------------------------------------------------
void Element_Judgment_Left_Rings()
{
//    Disf = 0;
    if (
              ImageStatus.Right_Line > 7
            ||ImageStatus.Left_Line < 13 // 13
            ||ImageStatus.OFFLine > 10
            //||variance_acc>50
            || Straight_Judge(2, 25, 45) > 50
            || ImageStatus.WhiteLine>15
            || ImageDeal[52].IsLeftFind == 'W'
            || ImageDeal[53].IsLeftFind == 'W'
            || ImageDeal[54].IsLeftFind == 'W'
            || ImageDeal[55].IsLeftFind == 'W'
            || ImageDeal[56].IsLeftFind == 'W'
            || ImageDeal[57].IsLeftFind == 'W'
            || ImageDeal[58].IsLeftFind == 'W'
        )
            return;
    int ring_ysite = 25;
  //  uint8 Left_Less_Num = 0;
    Left_RingsFlag_Point1_Ysite = 0;
    Left_RingsFlag_Point2_Ysite = 0;
 //   ceshi_flag = 1;
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary_First - ImageDeal[Ysite - 1].LeftBoundary_First > 4)
        {
            Left_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 4)
        {
            Left_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

    for (int Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine; Ysite--)
    {
        if (   ImageDeal[Ysite + 6].LeftBorder < ImageDeal[Ysite+3].LeftBorder
            && ImageDeal[Ysite + 5].LeftBorder < ImageDeal[Ysite+3].LeftBorder
            && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 2].LeftBorder
            && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 1].LeftBorder
            )
        {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if(Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 0 )
    {
        if(ImageStatus.Left_Line > 13)//13
            Ring_Help_Flag = 1;
    }
    if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag ==0)
    {

        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small=1;
        ImageStatus.Road_type = LeftCirque;
    }
    Ring_Help_Flag = 0;
}

//--------------------------------------------------------------
//  @name           Element_Judgment_Right_Rings()
//  @brief          ����ͼ���жϵ��Ӻ����������ж���Բ������.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Judgment_Right_Rings();
//--------------------------------------------------------------
void Element_Judgment_Right_Rings()
{
    if (   ImageStatus.Left_Line > 7
        || ImageStatus.Right_Line < 13 //13
        || ImageStatus.OFFLine > 10
       || Straight_Judge(1, 25, 45) > 50
        || ImageStatus.WhiteLine>15
        || ImageDeal[52].IsRightFind == 'W'
        || ImageDeal[53].IsRightFind == 'W'
        || ImageDeal[54].IsRightFind == 'W'
        || ImageDeal[55].IsRightFind == 'W'
        || ImageDeal[56].IsRightFind == 'W'
        || ImageDeal[57].IsRightFind == 'W'
        || ImageDeal[58].IsRightFind == 'W'
        )
        {return;}
    int ring_ysite = 25;
    Right_RingsFlag_Point1_Ysite = 0;
    Right_RingsFlag_Point2_Ysite = 0;
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
        {
            Right_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
        {
            Right_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = Right_RingsFlag_Point1_Ysite; Ysite > 10; Ysite--)
    {
        if (   ImageDeal[Ysite + 6].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 5].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 2].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 1].RightBorder
           )
        {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if(Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 0)
    {
        if(ImageStatus.Right_Line>7)
            Ring_Help_Flag = 1;
    }
    if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag == 0)
    {

        ImageFlag.image_element_rings = 2;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small=1;     //С��
        ImageStatus.Road_type = RightCirque;
//        gpio_set_level(Beep, 1);
    }
    Ring_Help_Flag = 0;
}

//��Բ���ж�
void Element_Handle_Left_Rings()
{
    /***************************************�ж�**************************************/
    int num = 0;
    for (int Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsLeftFind == 'W')
            num++;
        if(    ImageDeal[Ysite+3].IsLeftFind == 'W' && ImageDeal[Ysite+2].IsLeftFind == 'W'
            && ImageDeal[Ysite+1].IsLeftFind == 'W' && ImageDeal[Ysite].IsLeftFind == 'T')
            break;
    }

        //׼������
    if (ImageFlag.image_element_rings_flag == 1 && num>10)
    {
        ImageFlag.image_element_rings_flag = 2;
        //wireless_uart_send_byte(2);
    }
    if (ImageFlag.image_element_rings_flag == 2 && num<8)
    {

        ImageFlag.image_element_rings_flag = 5;
        //wireless_uart_send_byte(5);
    }
        //����
    if(ImageFlag.image_element_rings_flag == 5 && /*num>15)*/ImageStatus.Right_Line>15)
    {
        ImageFlag.image_element_rings_flag = 6;
     //   ImageStatus.Road_type = LeftCirque;
        //wireless_uart_send_byte(6);
    }
        //����СԲ��
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Right_Line<3)
    {
        //Stop = 1;
        ImageFlag.image_element_rings_flag = 7;
        //wireless_uart_send_byte(8);
    }
        //���� ��Բ���ж�
    if (ImageFlag.ring_big_small == 1 && ImageFlag.image_element_rings_flag == 7)
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 50; Ysite > ImageStatus.OFFLine + 3; Ysite--)
        {
            if (       ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 2].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 2].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 1].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 1].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 4].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 4].RightBorder
               )
            {
                Point_Xsite = ImageDeal[Ysite].RightBorder;
                Point_Ysite = Ysite;
                break;
            }
        }
        if (Point_Ysite > 24)
        {
            ImageFlag.image_element_rings_flag = 8;
           // wireless_uart_send_byte(8);
            //Stop = 1;
        }
    }
    //������
        if (ImageFlag.image_element_rings_flag == 8)
        {
             if (
                     //Straight_Judge(2, ImageStatus.OFFLine+15, 50) < 1
                 ImageStatus.Right_Line < 9
                 && ImageStatus.OFFLine < 10)    //�ұ�Ϊֱ���ҽ�ֹ�У�ǰհֵ����С
               {ImageFlag.image_element_rings_flag = 9;
                //wireless_uart_send_byte(9);
               }
        }

        //����Բ������
        if (ImageFlag.image_element_rings_flag == 9)
        {
            int num=0;
            for (int Ysite = 40; Ysite > 10; Ysite--)
            {
                if(ImageDeal[Ysite].IsLeftFind == 'W' )
                    num++;
            }
            if(num < 5)
            {
                ImageStatus.Road_type = 0;   //�����������·������0
                ImageFlag.image_element_rings_flag = 0;
                ImageFlag.image_element_rings = 0;
                ImageFlag.ring_big_small = 0;
                ImageStatus.Road_type = Normol;
                //wireless_uart_send_byte(0);
//                gpio_set_level(Beep, 0);
            }
    }



    /***************************************����**************************************/
        //׼������  �����
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite]-5;
        }
    }
        //����  ����
    if  ( ImageFlag.image_element_rings_flag == 5
        ||ImageFlag.image_element_rings_flag == 6
        )
    {
        int  flag_Xsite_1=0;
        int flag_Ysite_1=0;
        float Slope_Rings=0;
        for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)//���满��
        {
            for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
            {
                if(  Pixle[Ysite][Xsite] == 1 && Pixle[Ysite][Xsite + 1] == 0)
                 {
                   flag_Ysite_1 = Ysite;
                   flag_Xsite_1 = Xsite;
                   Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                   break;
                 }
            }
            if(flag_Ysite_1 != 0)
            {
                break;
            }
        }
        if(flag_Ysite_1 == 0)
        {
            for(Ysite=ImageStatus.OFFLine+1;Ysite<30;Ysite++)
            {
                if(ImageDeal[Ysite].IsLeftFind=='T'&&ImageDeal[Ysite+1].IsLeftFind=='T'&&ImageDeal[Ysite+2].IsLeftFind=='W'
                    &&abs(ImageDeal[Ysite].LeftBorder-ImageDeal[Ysite+2].LeftBorder)>10
                  )
                {
                    flag_Ysite_1=Ysite;
                    flag_Xsite_1=ImageDeal[flag_Ysite_1].LeftBorder;
                    ImageStatus.OFFLine=Ysite;
                    Slope_Rings=(float)(79-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                    break;
                }

            }
        }
        //����
        if(flag_Ysite_1 != 0)
        {
            for(Ysite=flag_Ysite_1;Ysite<60;Ysite++)
            {
                ImageDeal[Ysite].RightBorder=flag_Xsite_1+Slope_Rings*(Ysite-flag_Ysite_1);
                //if(ImageFlag.ring_big_small==1)//��Բ���������
                    ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2);
                //else//СԲ�������
                //    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
                if(ImageDeal[Ysite].Center<4)
                    ImageDeal[Ysite].Center = 4;
            }
            ImageDeal[flag_Ysite_1].RightBorder=flag_Xsite_1;
            for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A���Ϸ�����ɨ��
            {
                for(Xsite=ImageDeal[Ysite+1].RightBorder-10;Xsite<ImageDeal[Ysite+1].RightBorder+2;Xsite++)
                {
                    if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite+1]==0)
                    {
                        ImageDeal[Ysite].RightBorder=Xsite;
                        //if(ImageFlag.ring_big_small==1)//��Բ���������
                            ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2);
                        //else//СԲ�������
                        //    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
                        if(ImageDeal[Ysite].Center<4)
                            ImageDeal[Ysite].Center = 4;
                        ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
                        break;
                    }
                }

                if(ImageDeal[Ysite].Wide>8 &&ImageDeal[Ysite].RightBorder< ImageDeal[Ysite+2].RightBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite+2;
                    break;
                }
            }
        }
    }
        //���� С���������� �󻷲���
    if (ImageFlag.image_element_rings_flag == 7)
    {

    }
        //��Բ������ ����
    if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 1)    //��Բ��
    {
        Repair_Point_Xsite = 20;
        Repair_Point_Ysite = 0;
        for (int Ysite = 40; Ysite > 5; Ysite--)
        {
            if (Pixle[Ysite][28] == 1 && Pixle[Ysite-1][28] == 0)//28
            {
                Repair_Point_Xsite = 28;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  //��ֹ�����¹滮
                break;
            }
        }
        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //����
        {
            ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].RightBorder;
            ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2);
        }
    }
        //�ѳ��� �����
    if (ImageFlag.image_element_rings_flag == 9 || ImageFlag.image_element_rings_flag == 10)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];
        }
    }
}
//--------------------------------------------------------------
//  @name           Element_Handle_Right_Rings()
//  @brief          ����ͼ������Ӻ���������������Բ������.
//  @parameter      void
//  @time
//  @Author         MRCHEN
//  Sample usage:   Element_Handle_Right_Rings();
//-------------------------------------------------------------
void Element_Handle_Right_Rings()
{
    /****************�ж�*****************/
    int num =0 ;
    for (int Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsRightFind == 'W')
        {
            num++;
        }
        if(    ImageDeal[Ysite+3].IsRightFind == 'W' && ImageDeal[Ysite+2].IsRightFind == 'W'
            && ImageDeal[Ysite+1].IsRightFind == 'W' && ImageDeal[Ysite].IsRightFind == 'T' )
            break;
    }
        //׼������
    if (ImageFlag.image_element_rings_flag == 1 && num>10)
    {
        ImageFlag.image_element_rings_flag = 2;
    }
    if (ImageFlag.image_element_rings_flag == 2 && num<8)
    {
        ImageFlag.image_element_rings_flag = 5;
    }
        //����
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.Left_Line>15)
    {
        ImageFlag.image_element_rings_flag = 6;
       // ImageStatus.Road_type = RightCirque;
    }
        //����СԲ��
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Left_Line<4)
    {
        ImageFlag.image_element_rings_flag = 7;
        //Stop=1;
    }
    if (ImageFlag.image_element_rings_flag == 7)
    {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for (int Ysite = 45; Ysite > ImageStatus.OFFLine+3; Ysite--)
        {
            if (    ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 2].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 2].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 1].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 1].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 4].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 4].LeftBorder
                )

            {
                        Point_Xsite = ImageDeal[Ysite].LeftBorder;
                        Point_Ysite = Ysite;
                        break;
            }
        }
        if (Point_Ysite > 22)
        {
            ImageFlag.image_element_rings_flag = 8;
        }
    }
    if (ImageFlag.image_element_rings_flag == 8)
    {
         if (   Straight_Judge(1, ImageStatus.OFFLine+10, 45) < 1
             && ImageStatus.Left_Line < 9
             && ImageStatus.OFFLine < 20)    //�ұ�Ϊֱ���ҽ�ֹ�У�ǰհֵ����С
            {ImageFlag.image_element_rings_flag = 9;}

    }
    if(ImageFlag.image_element_rings_flag == 9 )
    {
        int num=0;
        for (int Ysite = 40; Ysite > 10; Ysite--)
        {
            if(ImageDeal[Ysite].IsRightFind == 'W' )
            {
                num++;
            }
        }
        if(num < 5)
        {
            ImageStatus.Road_type = 0;   //�����������·������0
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            ImageStatus.Road_type = Normol;
//            Front_Ring_Continue_Count++;
//            gpio_set_level(Beep, 0);
        }
    }
    /***************************************����**************************************/
         //׼������  �����
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }

        //����  ����
    if (   ImageFlag.image_element_rings_flag == 5
        || ImageFlag.image_element_rings_flag == 6
       )
    {
        int flag_Xsite_1=0;
        int  flag_Ysite_1=0;
        float Slope_Right_Rings = 0;
        for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)
        {
            for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
            {
                if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite+1]==0)
                {
                    flag_Ysite_1=Ysite;
                    flag_Xsite_1=Xsite;
                    Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
                    break;
                }
            }
            if(flag_Ysite_1!=0)
            {
              break;
            }
        }
        if(flag_Ysite_1==0)
        {
        for(Ysite=ImageStatus.OFFLine+5;Ysite<30;Ysite++)
        {
         if(ImageDeal[Ysite].IsRightFind=='T'&&ImageDeal[Ysite+1].IsRightFind=='T'&&ImageDeal[Ysite+2].IsRightFind=='W'
               &&abs(ImageDeal[Ysite].RightBorder-ImageDeal[Ysite+2].RightBorder)>10
         )
         {
             flag_Ysite_1=Ysite;
             flag_Xsite_1=ImageDeal[flag_Ysite_1].RightBorder;
             ImageStatus.OFFLine=Ysite;
             Slope_Right_Rings=(float)(0-flag_Xsite_1)/(float)(59-flag_Ysite_1);
             break;
         }

        }

        }
        //����
        if(flag_Ysite_1!=0)
        {
            for(Ysite=flag_Ysite_1;Ysite<58;Ysite++)
            {
                ImageDeal[Ysite].LeftBorder=flag_Xsite_1+Slope_Right_Rings*(Ysite-flag_Ysite_1);
//                if(ImageFlag.ring_big_small==2)//СԲ���Ӱ��
//                    ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//���
//                else//��Բ�����Ӱ��
                    ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;//���
                if(ImageDeal[Ysite].Center>79)
                    ImageDeal[Ysite].Center=79;
            }
            ImageDeal[flag_Ysite_1].LeftBorder=flag_Xsite_1;
            for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) //A���Ϸ�����ɨ��
            {
                for(Xsite=ImageDeal[Ysite+1].LeftBorder+8;Xsite>ImageDeal[Ysite+1].LeftBorder-4;Xsite--)
                {
                    if(Pixle[Ysite][Xsite]==1 && Pixle[Ysite][Xsite-1]==0)
                    {
                     ImageDeal[Ysite].LeftBorder=Xsite;
                     ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
//                     if(ImageFlag.ring_big_small==2)//СԲ���Ӱ��
//                         ImageDeal[Ysite].Center=ImageDeal[Ysite].LeftBorder+Half_Bend_Wide[Ysite];//���
//                     else//��Բ�����Ӱ��
                         ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;//���
                     if(ImageDeal[Ysite].Center>79)
                         ImageDeal[Ysite].Center=79;
                     if(ImageDeal[Ysite].Center<5)
                         ImageDeal[Ysite].Center=5;
                     break;
                    }
                }
                if(ImageDeal[Ysite].Wide>8 && ImageDeal[Ysite].LeftBorder>  ImageDeal[Ysite+2].LeftBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite+2;
                    break;
                }
            }
        }
    }
        //���ڲ�����
    if (ImageFlag.image_element_rings_flag == 7)
    {

    }
        //��Բ������ ����
    if (ImageFlag.image_element_rings_flag == 8)  //��Բ��
    {
        Repair_Point_Xsite = 20;
        Repair_Point_Ysite = 0;
        for (int Ysite = 40; Ysite > 8; Ysite--)
        {
            if (Pixle[Ysite][28] == 1 && Pixle[Ysite-1][28] == 0)
            {
                Repair_Point_Xsite = 28;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  //��ֹ�����¹滮
                break;
            }
        }
        for (int Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         //����
        {
            ImageDeal[Ysite].LeftBorder = (ImageDeal[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].LeftBorder;
            //if(ImageDeal[Ysite].LeftBorder<3){ImageDeal[Ysite].LeftBorder = 3;}
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
    }
        //�ѳ��� �����
    if (ImageFlag.image_element_rings_flag == 9)
    {
        for (int Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }
}


//���ڼ��ٵ�ֱ�����
float variance, variance_acc;  //����
void Straightacc_Test(void) {
  int sum = 0;
  for (Ysite = 55; Ysite > ImageStatus.OFFLine + 1; Ysite--) {
    sum += (ImageDeal[Ysite].Center - ImageStatus.MiddleLine) *(ImageDeal[Ysite].Center - ImageStatus.MiddleLine);
  }
  variance_acc = (float)sum / (54 - ImageStatus.OFFLine);
  if ( variance_acc < ImageStatus.variance_acc
    && ImageStatus.OFFLine <= 7
    && ImageStatus.Left_Line < 2
    && ImageStatus.Right_Line < 2 //  && ImageStatus.Road_type == 0
   // &&SystemData.SpeedData.Length*OX>350
    ) {
      ImageStatus.straight_acc = 1;
  } else
      ImageStatus.straight_acc = 0;
}


/****�µ����***/
int ramptestflag=1;//�µ��жϱ�־λ
int rampnum=0;     //�µ�����
void Ramp_Test() {
  int i = 0;
  if (ImageStatus.OFFLine == 2 && ramptestflag==1) {
    for (Ysite = ImageStatus.OFFLine; Ysite < 7; Ysite++) {
      if (       ImageDeal[Ysite].Wide > 15
              &&ImageDeal[Ysite].IsRightFind == 'T'
              &&ImageDeal[Ysite].IsLeftFind == 'T'
              &&ImageDeal[Ysite].LeftBorder < 40
              &&ImageDeal[Ysite].RightBorder > 40)
                i++;
      if (i >= 4) {
        ImageStatus.Road_type = Ramp;
        rampnum++;
        ramptestflag=0;
        break;
      }
    }
  }
}

void Cross_Test2(){
    int leftlowlen=0;
    int leftmiddlen=0;
    int lefthighlen=0;
    int rightlowlen=0;
    int rightmiddlen=0;
    int righthighlen=0;
        //�����ʮ������
    for (Ysite = 54; Ysite > ImageStatus.OFFLine+2; Ysite--) {

        //ʮ�ֵױߵ��б�
        if(ImageDeal[Ysite].IsLeftFind=='T'&&ImageDeal[Ysite-1].IsLeftFind=='T'&&leftlowlen==0){
            while(ImageDeal[Ysite].IsLeftFind=='T'){
                leftlowlen++;
                Ysite--;
                if(ImageDeal[Ysite].LeftBorder<ImageDeal[Ysite+5].LeftBorder-2){
                    leftlowlen=1;
                    break;
                }

            }
        }

        //ʮ���м�Ķ���
        if(ImageDeal[Ysite].IsLeftFind=='W'&&ImageDeal[Ysite-1].IsLeftFind=='W'&&leftmiddlen==0){
            while(ImageDeal[Ysite].IsLeftFind=='W'){
                leftmiddlen++;
                Ysite--;
            }
        }

        //ʮ���ϱߵ��б�
        if(ImageDeal[Ysite].IsRightFind=='T'&&ImageDeal[Ysite-1].IsRightFind=='T'&&lefthighlen==0){
            while(ImageDeal[Ysite].IsRightFind=='T'){
                lefthighlen++;
                Ysite--;
            }
        }
    }
        //���ұ�ʮ������
    for (Ysite = 54; Ysite > ImageStatus.OFFLine+2; Ysite--) {
        //ʮ�ֵױߵ��б�
        if(ImageDeal[Ysite].IsRightFind=='T'&&ImageDeal[Ysite-1].IsRightFind=='T'&&rightlowlen==0){
            while(ImageDeal[Ysite].IsRightFind=='T'){
                rightlowlen++;
                Ysite--;
                if(ImageDeal[Ysite].RightBorder>ImageDeal[Ysite+5].RightBorder+2){
                    rightlowlen=1;
                    break;
                }
            }
        }

        //ʮ�ֵױߵĶ���
        if(ImageDeal[Ysite].IsRightFind=='W'&&ImageDeal[Ysite-1].IsRightFind=='W'&&rightmiddlen==0){
            while(ImageDeal[Ysite].IsRightFind=='W'){
                rightmiddlen++;
                Ysite--;
            }
        }

        //ʮ���ϱߵ��б�
        if(ImageDeal[Ysite].IsRightFind=='T'&&ImageDeal[Ysite-1].IsRightFind=='T'&&righthighlen==0){
            while(ImageDeal[Ysite].IsRightFind=='T'){
                righthighlen++;
                Ysite--;
            }
        }
    }
    if(     leftlowlen > 5 &&leftmiddlen > 5
          &&lefthighlen>4
          &&rightlowlen>5
          &&rightmiddlen>5
          &&righthighlen>4
          )
        ImageStatus.Road_type=Cross_ture;

//     if(ImageStatus.Cross_ture_lenth*OX>80)
//        ImageStatus.Road_type=0;
    // else
    //       Fork_dowm=0;


}

void Pcir_Handle() {
  if (ImageStatus.Road_type==Cross_ture)   //�ڶ�Ȧ����
  {
      for (Ysite = 58; Ysite > ImageStatus.OFFLine; Ysite--)
      {
      ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - buxianwide[Ysite]*0.5;
      ImageDeal[Ysite].LeftBorder=ImageDeal[Ysite].RightBorder - buxianwide[Ysite];
      }
      Fit1(59,50);
   }

}


/****Ԫ�ؼ��*****/  //Բ�� ʮ�� ���� ��Ҫͨ�������������Լ�����Ԫ�ر�־  ������ֻ��һ˲��
                     //����û��⵽������ֱ�Ӵ˴�����
void Element_Test(void) {
  if (  ImageStatus.Road_type != Cross  //����������0
      &&ImageStatus.Road_type != LeftCirque
      &&ImageStatus.Road_type != RightCirque
      &&ImageStatus.Road_type != Barn_in
      &&ImageStatus.Road_type != Ramp
      &&ImageStatus.Road_type != Cross_ture
      )
    ImageStatus.Road_type = 0;

//  if (
//      ImageStatus.Road_type != Cross  //����������0
//      &&ImageStatus.Road_type != LeftCirque
//      &&ImageStatus.Road_type != RightCirque
//      &&ImageStatus.Road_type != Barn_in
//      &&ImageStatus.Road_type != Ramp
//    //  &&ImageStatus.Road_type != Cross_ture
//      )
    Straightacc_Test();


  if (ImageStatus.Road_type != Ramp
     &&ImageStatus.Road_type != LeftCirque
     &&ImageStatus.Road_type != RightCirque/*&&SystemData.SpeedData.Length >SystemData.ramp_lenth_start*/
         )//1800
  {Ramp_Test();}                     //�µ����





  if (      ImageStatus.Road_type != LeftCirque
          &&ImageStatus.Road_type != RightCirque
//          &&ImageStatus.Road_type != Ramp
//          &&ImageStatus.Road_type != Cross
          &&ImageStatus.Road_type != Barn_in
          //&&ImageStatus.Barn_Lenth!=0
//          &&ImageStatus.Road_type != Straight
          )  //Բ�����ƽǶ����
    {Cross_Test2();}

  if (   ImageStatus.Road_type != Barn_in  //Բ�����
      && ImageStatus.Road_type != Cross_ture
      && ImageStatus.Road_type != Barn_out
//      && ImageStatus.Road_type != Ramp
      )
  {
      Element_Judgment_Left_Rings();           //��Բ�����
      Element_Judgment_Right_Rings();          //��Բ�����
  }
}

void Element_Handle() {
  if (ImageFlag.image_element_rings == 1)
      Element_Handle_Left_Rings();
  else if(ImageFlag.image_element_rings == 2)
      Element_Handle_Right_Rings();
//      Cross_Handle();
}
uint8 all_stop_car[] = "9";
//extern int Sanchacha;
//extern int rukubz;
/****���紦��********/
void Stop_Test() {
 if (SystemData.SpeedData.Length * OX > 20) {//ǿ����

//      if (ImageStatus.OFFLine >= 55)           //�����ź�С���ҿ��Ӿ������û�оͱ�ʾ����
//           SystemData.Stop = 1;
     // else SystemData.Stop = 0;
     if (ImageStatus.OFFLine >= 55)           //�����ź�С���ҿ��Ӿ������û�оͱ�ʾ����
           SystemData.Stop = 1;

  }
}

void Stop_Test2() {                            //������
  if (  ImageStatus.OFFLine >= 55 && SystemData.Stop == 0
      &&SystemData.SpeedData.Length * OX > 150)
        SystemData.Stop = 2;

  if (      SystemData.Stop == 2
          &&ImageStatus.Stop_lenth * OX > 80
          &&ImageStatus.OFFLine >= 55)
    SystemData.Stop = 1;
  else if ( SystemData.Stop == 2
          &&ImageStatus.Stop_lenth * OX > 80
          &&ImageStatus.OFFLine < 55)
    SystemData.Stop = 0;
}

void Stop_Test3()
{
    uint8 whitepoint = 0 ;
    for(uint8 i = 0; i < 80 ; i++)
    {

        if( Pixle[59][i] )
        {
            whitepoint ++;
            tft180_show_int(30,125,whitepoint,3);
            if(whitepoint  < whilepoint_protect)
            {
                Stop_car_Flag = 1;
            }
        }
    }

}


void DrawLine()  //���߽�  �õ���
{
  uint8 i;
  for (i = 59; i > ImageStatus.OFFLine; i--) {
    Pixle[i][ImageDeal[i].LeftBorder + 2] = 0;  //�ƶ���λ���ڹ۲�
    Pixle[i][ImageDeal[i].RightBorder - 2] = 0;
    Pixle[i][ImageDeal[i].Center] = 0;
  }
}

/*****************��Ȩ����������**********************/
void GetDet() {
  float DetTemp = 0;
  int TowPoint = 0;
  float SpeedGain = 0;
  float UnitAll = 0;

  SpeedGain=(SystemData.SpeedData.nowspeed - SystemData.SpeedData.MinSpeed) * 0.2 +0.5 ;//�����ٶȵ���ǰհ������   �ٶȸ���80  ����Ϊ����  ǰհ�ӳ�

    if (SpeedGain >= 3)
      SpeedGain = 3;
    else if (SpeedGain <= -1)
      SpeedGain = -1;

    if ((ImageStatus.Road_type == RightCirque ||ImageStatus.Road_type == LeftCirque)&&ImageStatus.CirqueOff == 'F')
    TowPoint = 30;                                                                      //Բ��ǰհ

    else if (ImageStatus.Road_type == Straight)
    TowPoint = SystemData.straighet_towpoint;

  else if(ImageStatus.Road_type ==Cross_ture)
  {
      TowPoint=22;
  }
  else if(ImageFlag.image_element_rings_flag == 1 || ImageFlag.image_element_rings_flag == 2)
  {
      TowPoint=30;
  }
else
    TowPoint = ImageStatus.TowPoint-SpeedGain;                                          //�ٶ�Խ��ǰհԽ��

  if (TowPoint < ImageStatus.OFFLine)
    TowPoint = ImageStatus.OFFLine + 1;

  if (TowPoint >= 49)
    TowPoint = 49;

  if ((TowPoint - 5) >= ImageStatus.OFFLine) {                                          //ǰհȡ�趨ǰհ���ǿ��Ӿ���  ��Ҫ���������
    for (int Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) {
      DetTemp = DetTemp + Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1);

  } else if (TowPoint > ImageStatus.OFFLine) {
    for (Ysite = ImageStatus.OFFLine; Ysite < TowPoint; Ysite++) {
      DetTemp += Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[TowPoint - Ysite - 1];
    }
    for (Ysite = (TowPoint + TowPoint - ImageStatus.OFFLine); Ysite > TowPoint;
         Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[Ysite].Center + DetTemp) / (UnitAll + 1);
  } else if (ImageStatus.OFFLine < 49) {
    for (Ysite = (ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine;
         Ysite--) {
      DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
      UnitAll += Weighting[-TowPoint + Ysite - 1];
    }
    DetTemp = (ImageDeal[ImageStatus.OFFLine].Center + DetTemp) / (UnitAll + 1);

  } else
    DetTemp =ImageStatus.Det_True;                                                     //����ǳ���OFFLine>50�����������һ�ε�ƫ��ֵ

  ImageStatus.Det_True = DetTemp;                                                      //��ʱ�Ľ��������ƽ��ͼ��ƫ��

  ImageStatus.TowPoint_True = TowPoint;                                                //��ʱ��ǰհ
}


int sanchachazhi;

float Det = 0;
void ImageProcess(void) {
    compressimage();          //ͼ��ѹ�� 0.6ms
    ImageStatus.OFFLine = 2;  //���ֵ������ʵ����õ��������������
    ImageStatus.WhiteLine = 0;
  for (Ysite = 59; Ysite >= ImageStatus.OFFLine; Ysite--) {
    ImageDeal[Ysite].IsLeftFind = 'F';
    ImageDeal[Ysite].IsRightFind = 'F';
    ImageDeal[Ysite].LeftBorder = 0;
    ImageDeal[Ysite].RightBorder = 79;
    ImageDeal[Ysite].LeftTemp = 0;
    ImageDeal[Ysite].RightTemp = 79;

    // g  5.12
    ImageDeal[Ysite].close_LeftBorder = 0;
    ImageDeal[Ysite].close_RightBorder = 79;
  }

  Get01change_dajin();  //ͼ���ֵ��    2.7ms

  Pixle_Filter();
  DrawLinesFirst();     //���Ƶױ�      30us
  DrawLinesProcess();   //�õ���������  8us
  Search_Border_OTSU(Pixle, LCDH, LCDW, LCDH - 2);//58��λ����

  Element_Test();                   //5us
  DrawExtensionLine();
  RouteFilter();        //�����˲�ƽ�� 2us
  Element_Handle();  // 3us
  Stop_Test();           //
  GetDet();               //
}



