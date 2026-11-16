///////////////////////////////////////////////////////////////////////////////
// EncoderDebounce.h
// @brief  位置实际值 → 去抖 / 回绕修正 / 差分测速 [rpm]。
//
// 回绕模数 = 每转指令单位数 = 编码器每转计数 × 齿轮比分母 / 分子。
// ±1 个指令单位的抖动需连续 2 拍才确认。
//
// 用法:
//   EncoderSpeedFilter filt;
//   filt.Init(pos, unitsPerRev, spdGain);
//   float spdFb = filt.Update(rawPos);   // 每周期
///////////////////////////////////////////////////////////////////////////////
#pragma once

struct EncoderSpeedFilter
{
    // 当前接受的位置 [指令单位]
    LONG  acceptedPos;

    // 上一拍接受的位置 (差分用)
    LONG  prevAcceptedPos;

    // 待确认的位置
    LONG  pendingPos;

    // rawPos 连续匹配 pendingPos 的拍数
    int   pendingCount;

    // 首拍标志 (两拍之后才输出速度)
    bool  isFirst;

    // 回绕模数 [指令单位/转]
    LONG  wrapUnits;

    // (指令单位/拍) → rpm 增益 = 60 / (wrapUnits × dtSec)
    float speedGain;

    /**
     * 初始化
     * @param pos         首拍位置 [指令单位]
     * @param unitsPerRev 每转指令单位数
     * @param spdGain     速度增益 = 60 / (unitsPerRev × dtSec)
     */
    void Init(LONG pos, LONG unitsPerRev, float spdGain)
    {
        acceptedPos     = pos;
        prevAcceptedPos = pos;
        pendingPos      = pos;
        pendingCount    = 0;
        isFirst         = true;
        wrapUnits       = unitsPerRev;
        speedGain       = spdGain;
    }

    /**
     * 处理一拍位置, 返回去抖后的速度
     * @param rawPos 位置实际值 [指令单位]
     * @return 速度 [rpm], 首拍返回 0
     */
    float Update(LONG rawPos)
    {
        // 首拍只记录位置
        if (isFirst) {
            acceptedPos     = rawPos;
            prevAcceptedPos = rawPos;
            pendingPos      = rawPos;
            pendingCount    = 0;
            isFirst         = false;
            return 0.0f;
        }

        // 去抖: ±1 单位需连续 2 拍确认
        LONG delta = DeltaWrapped(rawPos, acceptedPos);

        if (delta == 0) {
            pendingCount = 0;
        } else if (delta == 1 || delta == -1) {
            if (rawPos == pendingPos) {
                pendingCount++;
                if (pendingCount >= 2) {
                    // 确认是真实移动
                    acceptedPos  = rawPos;
                    pendingCount = 0;
                }
            } else {
                // 新方向或新的抖动
                pendingPos   = rawPos;
                pendingCount = 1;
            }
        } else {
            // |delta| >= 2: 直接接受
            acceptedPos  = rawPos;
            pendingPos   = rawPos;
            pendingCount = 0;
        }

        // 差分求速
        LONG cleanDelta = DeltaWrapped(acceptedPos, prevAcceptedPos);
        float speed = (float)cleanDelta * speedGain;
        prevAcceptedPos = acceptedPos;

        return speed;
    }

private:
    /**
     * 回绕差分, 归一化到 [-wrapUnits/2, +wrapUnits/2]
     * @param cur  当前位置
     * @param prev 上一拍位置
     * @return 差分
     */
    LONG DeltaWrapped(LONG cur, LONG prev)
    {
        LONG d = cur - prev;
        LONG half = (LONG)(wrapUnits >> 1);
        if (d > half) {
            d -= wrapUnits;
        } else if (d < -half) {
            d += wrapUnits;
        }
        return d;
    }
};
