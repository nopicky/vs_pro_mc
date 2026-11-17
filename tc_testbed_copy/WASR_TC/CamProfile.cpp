#include "CamProfile.h"

/* 凸轮固定段参数（来自 FAE 凸轮表：加速/减速段均为 300mm 主轴 / 120mm 从轴） */
static const double c_fSectX = 300.0;   /* 加速/减速段主轴行程 mm */
static const double c_fSectY = 120.0;   /* 加速/减速段从轴行程 mm */

bool makeQuintic(double fX0, double fY0, double fV0, double fA0,
                 double fX1, double fY1, double fV1, double fA1,
                 QUINTIC_SEG_T* pOut) {

    double fL = fX1 - fX0;   /* 段长（主轴 mm） */
    double fH = fY1 - fY0;   /* 位移（从轴 mm） */
    if (pOut == 0 || fL <= 0.0 || fH == 0.0) {
        return false;
    }

    /* 边界条件归一化：p(0)=0, p(1)=1
       p'(u) = (L/H)*dY/dX，p''(u) = (L^2/H)*d2Y/dX2 */
    double fV0n = fV0 * fL / fH;
    double fV1n = fV1 * fL / fH;
    double fA0n = fA0 * fL * fL / fH;
    double fA1n = fA1 * fL * fL / fH;

    /* c0=0, c1=p'(0), c2=p''(0)/2；c3~c5 由 p(1)=1、p'(1)=fV1n、p''(1)=fA1n 解方程组 */
    double c1 = fV0n;
    double c2 = 0.5 * fA0n;
    double d = 1.0 - (c1 + c2);
    double e = fV1n - (c1 + 2.0 * c2);
    double f = fA1n - 2.0 * c2;

    pOut->a[0] = 0.0;
    pOut->a[1] = c1;
    pOut->a[2] = c2;
    pOut->a[3] = 10.0 * d - 4.0 * e + 0.5 * f;
    pOut->a[4] = -15.0 * d + 7.0 * e - f;
    pOut->a[5] = 6.0 * d - 3.0 * e + 0.5 * f;
    pOut->fX0 = fX0;
    pOut->fX1 = fX1;
    pOut->fY0 = fY0;
    pOut->fY1 = fY1;
    return true;
}

/* 段内位置求值：Horner 法多项式 + 映射回实际坐标，相位越界钳位 */
static double evalSegPos(const QUINTIC_SEG_T* pSeg, double fX) {

    double fL = pSeg->fX1 - pSeg->fX0;
    double fH = pSeg->fY1 - pSeg->fY0;
    double u = (fX - pSeg->fX0) / fL;
    if (u < 0.0) {
        u = 0.0;
    }
    if (u > 1.0) {
        u = 1.0;
    }
    const double* a = pSeg->a;
    double p = a[0] + u * (a[1] + u * (a[2] + u * (a[3] + u * (a[4] + u * a[5]))));
    return pSeg->fY0 + fH * p;
}

/* 段内速比 dY/dX = (H/L)*p'(u) */
static double evalSegVel(const QUINTIC_SEG_T* pSeg, double fX) {

    double fL = pSeg->fX1 - pSeg->fX0;
    double fH = pSeg->fY1 - pSeg->fY0;
    double u = (fX - pSeg->fX0) / fL;
    if (u < 0.0 || u > 1.0) {
        return 0.0;
    }
    const double* a = pSeg->a;
    double dp = a[1] + u * (2.0 * a[2] + u * (3.0 * a[3] + u * (4.0 * a[4] + u * 5.0 * a[5])));
    return (fH / fL) * dp;
}

bool CCamFeeder::build(double fSyncLen, double fOvertravel) {

    if (fSyncLen <= 0.0) {
        return false;
    }
    /* v1：过送补偿并入同步段（轴1 +10mm 使纸尾脱离），后续按实测优化叠加方式 */
    m_fSyncLen = fSyncLen + fOvertravel;
    m_fTotalLen = c_fSectX + m_fSyncLen + c_fSectX;

    /* 加速段：速比 0→1，加速度 0→0 */
    if (!makeQuintic(0.0, 0.0, 0.0, 0.0,
                     c_fSectX, c_fSectY, 1.0, 0.0, &m_accelSeg)) {
        return false;
    }
    /* 减速段：速比 1→0，加速度 0→0（局部坐标 0→300mm） */
    if (!makeQuintic(0.0, 0.0, 1.0, 0.0,
                     c_fSectX, c_fSectY, 0.0, 0.0, &m_decelSeg)) {
        return false;
    }
    return true;
}

double CCamFeeder::evalPos(double fX) const {

    if (fX <= 0.0) {
        return 0.0;
    }
    if (fX < c_fSectX) {
        return evalSegPos(&m_accelSeg, fX);            /* 加速段 */
    }
    if (fX < c_fSectX + m_fSyncLen) {
        return c_fSectY + (fX - c_fSectX);             /* 同步段：速比 1 */
    }
    if (fX < m_fTotalLen) {
        /* 减速段：映射到局部坐标 */
        return c_fSectY + m_fSyncLen + evalSegPos(&m_decelSeg, fX - c_fSectX - m_fSyncLen);
    }
    return 2.0 * c_fSectY + m_fSyncLen;                /* 走完保持终值 */
}

double CCamFeeder::evalVel(double fX) const {

    if (fX <= 0.0 || fX >= m_fTotalLen) {
        return 0.0;
    }
    if (fX < c_fSectX) {
        return evalSegVel(&m_accelSeg, fX);
    }
    if (fX < c_fSectX + m_fSyncLen) {
        return 1.0;
    }
    return evalSegVel(&m_decelSeg, fX - c_fSectX - m_fSyncLen);
}

double CCamFeeder::getLength() const {

    return m_fTotalLen;
}
