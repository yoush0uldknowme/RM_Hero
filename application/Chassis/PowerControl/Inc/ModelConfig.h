//
// Created by Pazfic on 2025/4/30.
//

#ifndef MODELCONFIG_H
#define MODELCONFIG_H
// Based On Model: P_predict = m_0*torq*rpm + m_1*abs(rpm) + m_2*torq*torq + m_3

//5.6RMUC_TEST2进化版
#define m_0_3508  0.00415919708521f
#define m_1_3508  0.00184076527471f
#define m_2_3508  3.12493747059f
#define m_3_3508  -0.0636489118594f

#define m_0_6020  0.00758228431176f
#define m_1_6020  0.0797845190155f
#define m_2_6020  5.86877481322f
#define m_3_6020  0.689279996528f

//5.6RMUC_TEST2,代码
// #define m_0_3508  0.00415919708521f

// #define m_1_3508  0.00184076527471f
// #define m_2_3508  3.12493747059f
// #define m_3_3508  -0.0636489118594f
//
// #define m_0_6020  0.00758228431176f
// #define m_1_6020  0.0797845190155f
// #define m_2_6020  5.86877481322f
// #define m_3_6020  0.689279996528f

//5.6RMUL代码
// #define m_0_3508 = (-0.000275f)
// #define m_1_3508 = 0.002721f
// #define m_2_3508 = 2.409288f
// #define m_3_3508 = 1.349725f
//
// #define m_0_6020 = 0.097991f
// #define m_1_6020 = (-0.001976f)
// #define m_2_6020 = 8.602107f
// #define m_3_6020 = 3.185897f

//5.5.RMUL代码  超非常多，正常移动都死
// #define m_0_3508  0.003570f
// #define m_1_3508  0.000629f
// #define m_2_3508  (-0.209229f)
// #define m_3_3508  3.506837f

// #define m_0_6020  0.116354f
// #define m_1_6020  (-0.008394f)
// #define m_2_6020  8.619794f
// #define m_3_6020  3.298242f


//5.5 RMUC_TSET2 非常慢，离设定功率差了15-20W
// #define m_0_3508  0.000681073714275f
// #define m_1_3508  0.00519521860983f
// #define m_2_3508  3.47034556408f
// #define m_3_3508  -1.93492198626f

// #define m_0_6020  0.0376099448511f
// #define m_1_6020  0.0595352817094f
// #define m_2_6020  6.4440787841f
// #define m_3_6020  0.829287071531f

//TODO:联盟赛使用，可上场
//2.1参数，效果尚可，会超几W，正常跑不会扣血，但是在某些急转情况会控不住，抵住会扣血
// #define m_0_3508  0.006693f
// #define m_1_3508  0.000472f
// #define m_2_3508  0.628765f
// #define m_3_3508  0.123292f
//
// #define m_0_6020  0.181430f
// #define m_1_6020  (-0.014875f)
// #define m_2_6020  1.481659f
// #define m_3_6020  2.980139f


#endif
