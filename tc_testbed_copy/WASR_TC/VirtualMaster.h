#ifndef VIRTUAL_MASTER_H__20261117
#define VIRTUAL_MASTER_H__20261117

#include <cmath>

/* 虚拟主轴：相位积分器（mm），对拖阶段替代印刷滚筒编码器
   纯数学，零硬件依赖；现场可整体替换为真实编码器轴相位输入 */
class CVirtualMaster {
public:
    CVirtualMaster() {

        m_fPhase = 0.0;
        m_fVelocity = 0.0;
        m_bRunning = false;
        m_fDistAmp = 0.0;
        m_fDistFreq = 0.0;
        m_fDistTime = 0.0;
    }

    /* 设定线速度：350 m/min → 5833.33 mm/s */
    void setVelocity(double fMmPerSec) {

        m_fVelocity = fMmPerSec;
    }
    void start() {

        m_bRunning = true;
    }
    void stop() {

        m_bRunning = false;
    }

    /* 每周期调用（125us）：相位 += 速度*dt，可叠加正弦速度扰动（启动补偿/DOB 实验用） */
    void tick(double fDtSec) {

        if (!m_bRunning) {
            return;
        }
        double fVel = m_fVelocity;
        if (m_fDistAmp != 0.0) {
            m_fDistTime += fDtSec;
            fVel += m_fDistAmp * sin(2.0 * 3.14159265358979 * m_fDistFreq * m_fDistTime);
        }
        m_fPhase += fVel * fDtSec;
    }

    double getPhase() const {

        return m_fPhase;
    }

    /* 扰动设置：幅值 mm/s、频率 Hz；幅值 0 = 关闭 */
    void setDisturbance(double fAmpMmPerSec, double fFreqHz) {

        m_fDistAmp = fAmpMmPerSec;
        m_fDistFreq = fFreqHz;
    }

private:
    double m_fPhase;      /* 当前相位 mm */
    double m_fVelocity;   /* 设定速度 mm/s */
    bool   m_bRunning;    /* 运行标志 */
    double m_fDistAmp;    /* 扰动幅值 mm/s */
    double m_fDistFreq;   /* 扰动频率 Hz */
    double m_fDistTime;   /* 扰动累计时间 s */
};

#endif /* VIRTUAL_MASTER_H__20261117 */
