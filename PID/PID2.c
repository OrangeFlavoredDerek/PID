//
//  PID2.c
//  PID
//
//  Created by Derek Chan on 2020/8/18.
//

//via https://github.com/2XUzhiweiro/stm32

#include <stdio.h>

//MARK: 增量式与位置式PID算法.未考虑死区，未设定上下限
struct _pid
{
    float ExpectedValue;        //预期值
    float ActualValue;        //实际值
    float err;            //当前偏差值
    float err_last;            //上一个偏差值
    float err_prev;            //上上个的偏差值
    float Kp, Ki, Kd;        //比例、积分、微分系数
    //float integral;
    //定义积分值
}pid;
 
void PID_Init()                //初始化
{
    pid.ExpectedValue = 0.0;
    pid.ActualValue = 0.0;
    pid.err = 0.0;
    pid.err_last = 0.0;
    pid.err_prev = 0.0;
    pid.Kp = 0.4;
    pid.Ki = 0.2;
    pid.Kd = 0.0;
    //pid.integral=0.0;
    //积分值初始化
}
 
float PID_Realize(float speed) {
    pid.ExpectedValue = speed;
    pid.err = pid.ExpectedValue - pid.ActualValue;
    
    //    MARK: 增量式PID算法
    float incrementValue = pid.Kp*(pid.err - pid.err_last) + pid.Ki*pid.err + pid.Kd*(pid.err - 2 * pid.err_last + pid.err_prev);//增量式PID公式
    pid.ActualValue += incrementValue;
    pid.err_prev = pid.err_last;
    pid.err_last = pid.err;
    
    //    MARK： 位置式PID算法
    pid.integral+=pid.err;
    pid.ActualValue=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last); //位置时PID公式
    pid.err_last=pid.err;
    return pid.ActualValue;
}



#PID算法优化：

//MARK: PID算法优化1.1：积分分离
//基本思路：当被控量与设定值偏差较大时，取消积分作用;
//        当被控量接近给定值时，引入积分控制，以消除静差，提高精度。
if(abs(pid.err)>200)
    {
    index=0;
    }else{
    index=1;
    pid.integral+=pid.err;
    }
   pid.ActualValue=pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);


//MARK: PID算法优化1.2：变积分（积分分离升级版)
         2020.8.14
/*问题1：普通PID控制算法中，Ki是常数，积分增量不变
基本思路：系统对于积分项的要求：系统偏差大时，积分作用应该减弱甚至是全无；
而在偏差小时，则应该加强。
问题2：积分系数取大了会产生超调，甚至积分饱和；
取小了又不能短时间内消除静差。
基本思路：根据系统的偏差大小改变积分速度
偏差越大，积分越慢;
偏差越小，积分越快。
解决给积分系数前加上一个比例值index：
当abs(err)<180时，index=1;
当180<abs(err)<200时，index=（200-abs(err)）/20;
当abs(err)>200时，index=0;
最终的比例环节的比例系数值为ki*index;*/





//MARK: PID算法优化2：抗积分饱和法
/*问题：积分饱和现象或积分失控现象。（类似于开关量）
基本思路：在计算u(k)时，首先判断上一时刻的控制量u(k-1)是否已经超出了极限范围：
如果u(k-1)>umax，则只累加负偏差;
如果u(k-1)<umin，则只累加正偏差。
从而避免控制量长时间停留在饱和区。*/




//MARK: PID算法优化3：梯形积分法
/*基本思路：积分项作用是消除余差;
提高积分项运算精度→→→减小余差
可以将矩形积分改为梯形积分*/

//MARK: 梯形积分公式
pid.ActualValue=pid.Kp*pid.err+index*pid.Ki*pid.integral/2+pid.Kd*(pid.err-pid.err_last);
