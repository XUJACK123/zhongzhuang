//
// Created by 2b superman on 2026/5/3.
//

#ifndef EGADAPTER_MC02_UITILS_H
#define EGADAPTER_MC02_UITILS_H

namespace
{
    constexpr float kHalfRotationDeg = 180.0f;
    constexpr float kFullRotationDeg = 360.0f;
    constexpr float kTwoPi = 6.28318530717958647692f;
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kDegToRad = kPi / 180.0f;

    // wrap 到 [0, 360)
    inline float wrap360(float x)
    {
        if (!std::isfinite(x))
            return 0.0f;

        x = fmodf(x, kFullRotationDeg);
        if (x < 0.0f)
            x += kFullRotationDeg;

        return x;
    }

    // wrap 到 [-180, 180)
    inline float wrap180(float x)
    {
        if (!std::isfinite(x))
            return 0.0f;

        x = fmodf(x + kHalfRotationDeg, kFullRotationDeg);
        if (x < 0.0f)
            x += kFullRotationDeg;

        return x - kHalfRotationDeg;
    }

    inline float wrap90(float x)
    {
        if (!std::isfinite(x))
            return 0.0f;

        x = fmodf(x + 90.0f, 180.0f);
        if (x < 0.0f)
            x += 180.0f;

        return x - 90.0f;
    }

    inline float wrapTargetNearMeasure180(float target, float measure)
    {
        if (!std::isfinite(target) || !std::isfinite(measure))
            return measure;

        return measure + wrap90(target - measure);
    }

    inline float wrapTargetNearMeasure(float target, float measure)
    {
        if (!std::isfinite(target) || !std::isfinite(measure))
            return measure;

        return measure + wrap180(target - measure);
    }


    float wrapToPi(float angle)
    {
        angle = std::fmod(angle + kPi, 2.0f * kPi);
        if (angle < 0.0f)
        {
            angle += 2.0f * kPi;
        }
        return angle - kPi;
    }

    float wrapTo2Pi(float angle)
    {
        angle = fmodf(angle, 2.0f * kPi);
        if (angle < 0.0f)
        {
            angle += 2.0f * kPi;
        }
        return angle;
    }


    float clampSymmetric(const float value, const float abs_limit)
    {
        if (value > abs_limit)
        {
            return abs_limit;
        }
        if (value < -abs_limit)
        {
            return -abs_limit;
        }
        return value;
    }


}


#endif // EGADAPTER_MC02_UITILS_H
