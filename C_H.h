/*
 * C_H.h
 *
 *  Created on: 2024��2��27��
 *      Author: xiaoming
 */

#ifndef C_H_H_
#define C_H_H_
#include "motor.h"
#include "zf_common_headfile.h"
#define uLCDH 120                            //����ͼ����ʾ��ͼ��߶�
#define uLCDW 160                            //����ͼ����ʾ��ͼ����
#define LCDH 60
#define LCDW 80
#define HMAX (LCDH - 1)                      //û��
#define WMAX (LCDW - 1)                      //û��
#define LimitL(L) (L = ((L < 1) ? 1 : L))    //���Ʒ���
#define LimitH(H) (H = ((H > 78) ? 78 : H))  //���Ʒ���
#define ImageSensorMid 39                    //ͼ����Ļ�е�
extern int ImageScanInterval;  //ɨ�߷�Χ    ��һ�еı߽�+-ImageScanInterval
extern int ImageScanInterval_Cross;  //ʮ��ɨ�߷�Χ
extern uint8 Image_Use[LCDH][LCDW];
extern uint8 Pixle[LCDH][LCDW];
extern uint8 UImage_Use[uLCDH][uLCDW];
extern float variance;  //����
extern int ForkLinePointy;
extern float variance_acc;
extern int Fork_dowm;
extern int circle_r;
extern float circlk_k;
extern int circleoff_lenth;//
extern int forklenth;
extern int barnlenth;
extern int ramplenth;
extern int ramptestflag;
extern int rampnum;

#define whilepoint_protect 50
extern uint8 Stop_car_Flag;;

typedef struct {
  int point;
  uint8 type;
} JumpPointtypedef;

typedef struct {
  float nowspeed;     // pulse��ʾnowspeed
  int expectspeed;    // speed��ʾexpectspeed
  int motor_duty;     //���ռ�ձ�
  float Length;       //�߹�·��
  int Circle_OUT_th;
  int MinSpeed;             //����ٶ�
  int MaxSpeed;             //����ٶ�
  float expect_True_speed;  //ʵ�������ٶ�
  int straight_speed;       //ֱ���ٶ�
} SpeedDatatypedef;

typedef struct {
  /*���ұ߽߱��־    TΪ���������    WΪ�ޱ�   PΪ�ϰ�������ߵ��ڱ�*/
  uint8 IsRightFind;      //�ұ��б߱�־
  uint8 IsLeftFind;       //����б߱�־
  uint8 isBlackFind;      //�����
  int Wide;               //�߽���
  int LeftBorder;         //��߽�
  int RightBorder;        //�ұ߽�
  int close_LeftBorder;   //���߽߱�
  int close_RightBorder;  //���߽߱�
  int opp_LeftBorder;     //����߽�
  int opp_RightBorder;    //����߽�
  int Center;             //����
  int RightTemp;          //�ұ���ʱֵ
  int LeftTemp;           //�����ʱֵ
  int CenterTemp;         //������ʱֵ
  int Black_Point;        //���кڵ�����

  //�����ַ���ɨ������
  int LeftBoundary_First;  //��߽� �����һ������
  int RightBoundary_First; //�ұ߽� �����һ������
  int LeftBoundary;        //��߽� �������һ������
  int RightBoundary;       //�ұ߽� �������һ������

  // fork   ����ı������ڲ�·�ڵļ��
} ImageDealDatatypedef;


//Ԫ������
typedef enum {
  Normol,       //���κ�����
  Straight,     ////ֱ��
  Cross,        ////ʮ��
  Ramp,         //�µ�
  LeftCirque,   ////��Բ��
  RightCirque,  ////��Բ��
  Forkin,       //��·����
  Forkout,      //��·����
  Barn_out,     //����
  Barn_in,      //���
  Cross_ture,
} RoadType_e;

typedef struct {
  /*���¹���ȫ��ͼ����������*/

  //ͼ����Ϣ
  int TowPoint;             //��ʼ�Ƚϵ�           ����ǰհ
  int TowPointAdjust_v;     //��ʼ�Ƚϵ���Ӧ�ٶ�    û��
  int TowPoint_True;        //ʵ�ʱȽϵ�           û��
  int TowPoint_Gain;        //����ϵ��             û��
  int TowPoint_Offset_Max;  //���ƫ��ϵ��         û��
  int TowPoint_Offset_Min;  //��Сƫ��ϵ��         û��
  int Det_True;             //��GetDet()�������������ƽ��ͼ��ƫ��
  int Det_all;              //ͼ��Ľ��˵�Զ�˵���ƫ��
  float Det_all_k;          //б��

  uint8 Threshold;          //��ֵ����ֵ
  uint32 Threshold_static;  //��ֵ����̬����
  uint8 Threshold_detach;   //�����㷨�ָ���ֵ
  uint8 MiddleLine;         //��Ļ����
  int Foresight;            //ƫ���ٶ�����   Զ�˵�ƫ��  ���ڿ���
  uint8 Left_Line;          //��߶�����
  uint8 Right_Line;         //�ұ߶�����
  uint8 OFFLine;            /////ͼ�񶥱�
  uint8 WhiteLine;          ////˫�߶�����
  float ExpectCur;          /////ͼ����������    û��
  float White_Ritio;        //��Ч�а׵����     û��
  int Black_Pro_ALL;        //����ڵ����%      û��

  // PID
  float Piriod_P;  //�ֶ�P   ֵ     û��
  float MU_P;

  RoadType_e Road_type;  //Ԫ������

  /****Բ��***/
  uint8 IsCinqueOutIn;  //����Բ��
  uint8 CirquePass;     //Բ����
  uint8 CirqueOut;      //��Բ��
  uint8 CirqueOff;      //Բ������

  //�����ַ���ɨ������

  int16 WhiteLine_L;        //��߶�����
  int16 WhiteLine_R;        //�ұ߶�����
  int16 OFFLineBoundary;   //�������ֹ��

  int Pass_Lenth;       //�뻷����  ���ڷ���
                        /****Բ��***/
  int Cirque1lenth;        //tly1
  int Cirque2lenth;        //tly2
  int Out_Lenth;     //
  int Fork_Out_Len;  //����ڷ���
  int Dowm_lenth;    //������پ���  //g5.13
  int Cross_Lenth;  // 270��ת��֮��һ����ʮ��·��   ��Ϊ����ڵ�����
  int Cross_ture_lenth;
  int Sita;  //�ݴ��ж��ڹ�ȥһ��·����ת�����ٽǶ�
  int pansancha_Lenth;

  //����
  int Barn_Flag;   //�жϿ�Ĵ���
  int Barn_Lenth;  //���ֹͣ����
  int sanchaju;

  //����
  int Stop_lenth;  //���籣�������о���
  //�µ�����
  int Ramp_lenth;

  int variance;  //ֱ�������ֵ����

  int straight_acc;  //ֱ�����ٱ�־λ
  int  variance_acc;    //���ڼ��ٵ���ֵ����

  int ramptestlenth;//�µ������

  int rukuwait_lenth;
  int rukuwait_flag;
 // int Blue_lenth;
  int newblue_flag;

} ImageStatustypedef;

typedef struct{

  uint8 SteerOK;      //���������־
  uint8 CameraOK;     //����ͷ������ʾ��־
  uint8 OldCameraOK;  //�Ҷȴ����־
  uint8 MotorOK;      //�������
  uint8 Point;        //����ʵ������
  uint8 UpdataOK;     //���ݸ���
  uint8 Stop;         //ֹͣ��־
  uint8 GO_OK;        //����
  int Model;          //����ģʽ

  //ͼ�����
  int OutCicle_line;  //�����ж�ͼ���ֹ��  Խ������ж�Խ�ϸ�Խ��
  int L_T_R_W;  // �������б����ޱ�����  Խ���������Խ�ϸ�
  int Circleoff_offline;  //����Զ���ľ����жϳ�������  ԽСԽ�ϸ�
  int CircleP;            //����P


//�����о���
  int fork_start_lenth;  //��·�ڼ����ʼ
  int fork_off_lenth;//��·�ڼ���ֹ

  int circle_off_lenth;//���������о���

  //СԲ��
  int circles_pl;
  int circles_pr;
  int circles_off_lenth;//СԲ�����������о���

   //��Բ��
  int circlem_pl;
  int circlem_pr;
  int circlem_off_lenth;//СԲ�����������о���

   //��Բ��
  int circlel_pl;
  int circlel_pr;
  int circlel_off_lenth;//СԲ�����������о���
  int clrcle_priority[3];//Բ������
  int clrcle_num;// �ڵڼ���Բ��


  int circle_kin;//�뻷���߰뾶
  float circle_kout;//��������б��
  int circle_max_ang;//���ڴ���޷�


  //ֱ��
  float straight_p;//ֱ��P
  float straight_d;//ֱ��D
  int   straighet_towpoint;//ֱ��ǰհ


  int debug_lenth;//���Ծ���

  //����ͷ����
  int exp_time; //�ع�ʱ��
  int mtv_lroffset;//����ͷ����ƫ��
  int mtv_gain;//����ͷ����

  int ramp_lenth_start;//�µ�����
  int fork_lenth_start;//�������
  int barn_lenth;//Բ������

  int outbent_acc;//�������


  int rounds; // Ȧ��
  int speed_per_round; // ÿȦ���ٶ���

  SpeedDatatypedef SpeedData;
} SystemDatatypdef;

typedef struct {
//    int16 Bend_Road;                            /*0 :��               1 :�����     2 :�����*/
    int16 image_element_rings;                  /*0 :��Բ��          1 :��Բ��       2 :��Բ��*/
    int16 ring_big_small;                       /*0:��                     1 :��Բ��       2 :СԲ��*/
    int16 image_element_rings_flag;             /*Բ������*/
    int16 straight_long;                        /*��ֱ����־λ*/
//    int16 Garage_Location;                      /*0 :�޳���          1 :�󳵿�       2 :�ҳ���*/
//    int16 Zebra_Flag;                           /*0 :�ް�����       1 �󳵿�       2 :�ҳ���*/
//    int16 Ramp;                                  /*0 :���µ�          1���µ�*/
//    int16 RoadBlock_Flag;                        /*0 :��·��            1 :·��*/
//    int16 Out_Road;                               /*0 :�޶�·      1 :��·*/
} ImageFlagtypedef;
void camera_display(void);  //ͼ����ʾ
void compressimage();       ////ѹ��ͼ��
void Get01change();         ////��ֵ��ͼ��
void Pixle_Filter();        ////�����˲���Ч������
void Draw_Road(void);       ///���������߽�
void ImageProcess(void);    ////ͼ����������
void uncompressimage();     //ͼ���ѹ
//void ForkTest();            //��·�ڼ��
void Cross_Test();          //Բ�����
void Cross_Handle();        //Բ������
void Element_Test();        //Ԫ�ؼ��
void Ramp_Test();           //�µ����
void Barn_test_in();        //�����
void Element_Handle();      //Ԫ�ش���
//void Fork_Handle();         //���洦��
//void Barn_in_Handle();      //û��
//void GetLenSita();          //û����
//void Pcir_Handle(void);
int Limit(int num, int numH, int numL);
extern ImageStatustypedef ImageStatus;                //ͼ���ȫ�ֱ���
extern SystemDatatypdef SystemData;
extern ImageDealDatatypedef ImageDeal[60];                //��¼���е���Ϣ
extern ImageFlagtypedef ImageFlag;
extern ImageStatustypedef ImageData;             ///////��Ҫ�޸ĵ�ͼ����ֵ����
#endif /* C_H_H_ */
