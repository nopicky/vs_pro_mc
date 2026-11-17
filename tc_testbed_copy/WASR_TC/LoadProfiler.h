#ifndef LOAD_PROFILER_H__20261117
#define LOAD_PROFILER_H__20261117

/* 负载侧转矩谱（4 个 CST 负载伺服）
   v1：恒定转矩占位；后续按主轴相位/送纸状态切换负载（模拟纸板进出各辊）
   注意：仅当 g_cfgFeed.bLoadSideEnable = true 时挂载层才调用本类，
         关闭时驱动层完全不碰负载伺服（首测与实际机台均无负载伺服） */
class CLoadProfiler {
public:
    CLoadProfiler() {

        for (int i = 0; i < 4; i++) {
            m_afTorque[i] = 0.0;
        }
    }

    /* 设定各对拖组的恒定负载转矩（单位以下发层约定为准：千分比额定或 N·m） */
    void setConstTorque(const double afTorque[4]) {

        for (int i = 0; i < 4; i++) {
            m_afTorque[i] = afTorque[i];
        }
    }

    /* 每周期输出转矩指令；v1 恒值，相位/状态参数预留 */
    void tick(double fMasterPhase, int nFeedState, double afTorqueOut[4]) {

        (void)fMasterPhase;   /* v1 未用，预留 */
        (void)nFeedState;     /* v1 未用，预留 */
        for (int i = 0; i < 4; i++) {
            afTorqueOut[i] = m_afTorque[i];
        }
    }

private:
    double m_afTorque[4];   /* 各组恒定负载转矩 */
};

#endif /* LOAD_PROFILER_H__20261117 */
