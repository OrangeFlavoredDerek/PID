//
//  main.c
//  PID
//
//  Created by Derek Chan on 2020/8/13.
//

#include <stdio.h>

typedef unsigned char      uChar8;
typedef unsigned int       uInt16;
typedef unsigned long int  uInt32;

typedef struct PIDValue {               //定义结构体来存储PID算法中要用到的变量
    uInt32 differenceValue[3];          //差值保存，给定和反馈的差值
    uChar8 sign[3];                     //符号，1则对应的为负数，0为对应的为正数
    uChar8 KP;                          //比例系数, P
    uChar8 KI;                          //积分常数, I
    uChar8 KD;                          //微分常数, D
    uInt16 previousValue;               //上一时刻值
    uInt16 setValue;                    //设定值
    uInt16 actualValue;                 //实际值
} PIDValueStr;

PIDValueStr PID;

//MARK: PID = Uk + KP·[E(k)-E(k-1)] + KI·E(k) + KD·[E(k)-2E(k-1)+E(k-2)]
void PIDOperation(void) {
    uInt32 temp[3] = {0};        //中间临时变量
    uInt32 positiveSum = 0;          //正数和
    uInt32 negativeSum = 0;           //负数和
    
    if (PID.setValue > PID.actualValue) { //设定值是否大于实际值
        //MARK: 计算偏差是否大于10
            if (PID.setValue - PID.actualValue > 10) { //若偏差大于10，则上限幅值输出
                PID.previousValue = 100;
            } else {
                temp[0] = PID.setValue - PID.actualValue; //偏差小于等于10，计算E(k)
                PID.sign[1] = 0; //符号为正，E(k)为正数，因为设定值大于实际值
                
                PID.differenceValue[2] = PID.differenceValue[1];
                PID.differenceValue[1] = PID.differenceValue[0];
                PID.differenceValue[0] = temp[0];
                
                if (PID.differenceValue[0] > PID.differenceValue[1]) {                //E(k)是否大于E(k-1)
                    temp[0] = PID.differenceValue[0] - PID.differenceValue[1];        //E(k)>E(k-1)
                    PID.sign[0] = 0;                               //E(k)-E(k-1)为正数
                } else {
                    temp[0] = PID.differenceValue[1] - PID.differenceValue[0];        //E(k)<E(k-1)
                    PID.sign[0] = 1;                               //E(k)-E(k-1)为负数
                }
                
                temp[2] = PID.differenceValue[1] * 2;                         //2倍的E(k-1)
                if ((PID.differenceValue[0] + PID.differenceValue[2]) > temp[2]) {    //E(k-2)+E(k)是否大于2E(k-1)
                    temp[2] = (PID.differenceValue[0] + PID.differenceValue[2]) - temp[2];
                    PID.sign[2] = 0;                               //E(k-2)+E(k)-2E(k-1)为正数
                } else {
                    temp[2] = temp[2] - (PID.differenceValue[0] + PID.differenceValue[2]);
                    PID.sign[2] = 1;                               ////E(k-2)+E(k)-2E(k-1)为负数
                }

                //MARK: 计算
                temp[0] = (uInt32)PID.KP * temp[0];        //KP*[E(k)-E(k-1)]
                temp[1] = (uInt32)PID.KI * PID.differenceValue[0]; //KI*E(k)
                temp[2] = (uInt32)PID.KD * temp[2];        //KD*[E(k-2)+E(k)-2E(k-1)]
                
                //MARK: 计算KP·[E(k)-E(k-1)]的值
                if (PID.sign[0] == 0) {
                    positiveSum += temp[0]; //正数和
                } else{
                    negativeSum += temp[0]; //负数和
                }
                
                //MARK: 计算KI·E(k)的值
                if (PID.sign[1] == 0) {
                    positiveSum += temp[1]; //正数和
                } else {
                    ; //空操作，因为PID.setValue > PID.acualValue（即E(K)>0）才进入if，所以不可能为负
                }
                
                //MARK: 计算KD·[E(k)-2E(k-1)+E(k-2)]的值
                if (PID.sign[2] == 0) {
                    positiveSum += temp[2]; //正数和
                } else {
                    negativeSum += temp[2]; //负数和
                }
                
                //MARK: 计算Uk
                positiveSum += (uInt32)PID.previousValue;
                if (positiveSum > negativeSum) { //是否控制量为正数
                    temp[0] = positiveSum - negativeSum;
                    if (temp[0] < 100) { //小于上限幅值则为计算值输出
                        PID.previousValue = (uInt16)temp[0];
                    } else { //否则为上限幅值输出，输出100（上限幅值输出）
                        PID.previousValue = 100;
                    }
                } else {
                    PID.previousValue = 0; //控制量输出为负数，则输出0(下限幅值输出)
                }
            }
    } else {
        PID.previousValue = 0; //若设定值小于实际值，则输出0(下限幅值输出)
    }
}


//int main(int argc, const char * argv[]) {
//    // insert code here...
//    printf("Hello, World!\n");
//    return 0;
//}
