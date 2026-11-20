#ifndef CAM_PROFILE_H__20261117
#define CAM_PROFILE_H__20261117

/* 一段五次曲线：归一化 u=(x-fX0)/(fX1-fX0) ∈ [0,1]，Y(u)=fY0+(fY1-fY0)*p(u)
   p(u)=a[0]+a[1]u+...+a[5]u^5，系数构建期预计算，运行期只做多项式求值 */
typedef struct tagQUINTIC_SEG_T {
    double a[6];    /* 归一化多项式系数 */
    double fX0;     /* 段起点（主轴 mm） */
    double fX1;     /* 段终点（主轴 mm） */
    double fY0;     /* 起点位移（从轴 mm） */
    double fY1;     /* 终点位移（从轴 mm） */
} QUINTIC_SEG_T;

/* 按 6 个边界条件解析求解五次曲线系数
   fV*：端点速比 dY/dX；fA*：端点速比导数 d2Y/dX2
   返回 false = 参数非法（pOut 为空、fX1<=fX0 或位移为 0） */
bool makeQuintic(double fX0, double fY0, double fV0, double fA0,
                 double fX1, double fY1, double fV1, double fA1,
                 QUINTIC_SEG_T* pOut);

/* 三段式送纸凸轮：Poly5 加速段 + 匀速同步段 + Poly5 减速段
   纯算法，零硬件依赖（仅 <cmath>），可原样移植到量产控制器 */
class CCamFeeder {
public:
    /* 构造: 全部归零 -> 未 build 时 evalPos/evalVel 恒返回 0（安全空凸轮），
       避免"忘记 build"变成读未初始化成员（NaN 进实时环最麻烦） */
    CCamFeeder();

    /* 构建凸轮：fSyncLen 同步段长 mm；fOvertravel 过送补偿 mm（v1：并入同步段长）
       fAccelX/fAccelY：加速段主轴/从轴行程 mm；fDecelX/fDecelY：减速段同理
       内部拼接三段并做边界自校验，不合格返回 false（坏曲线不进实时环） */
    bool   build(double fSyncLen, double fOvertravel,
                 double fAccelX, double fAccelY, double fDecelX, double fDecelY);

    /* 主轴相位 fX(mm) → 从轴位置(mm)；越界自动钳位（前 0、后保持终值） */
    double evalPos(double fX) const;
    /* 主轴相位 fX(mm) → 速比 dY/dX（速度前馈用） */
    double evalVel(double fX) const;
    /* 主轴相位 fX(mm) → 曲率 d2Y/dX2（加速度前馈用；同步段为 0） */
    double evalAcc(double fX) const;
    /* 凸轮总长（主轴 mm） */
    double getLength() const;

private:
    QUINTIC_SEG_T m_accelSeg;   /* 加速段：速比 0→1 */
    QUINTIC_SEG_T m_decelSeg;   /* 减速段：速比 1→0（局部坐标） */
    double        m_fSyncLen;   /* 有效同步段长 mm（含过送补偿） */
    double        m_fAccelX;    /* 加速段主轴行程 mm（参数化，默认 300=FAE 值） */
    double        m_fDecelX;    /* 减速段主轴行程 mm */
    double        m_fTotalLen;  /* 凸轮总长 mm = 加速段 + 同步段 + 减速段 */
};

#endif /* CAM_PROFILE_H__20261117 */
