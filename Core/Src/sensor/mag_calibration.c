#include "sensor/mag_calibration.h"

void MagCal_Update(MagCal_Simple_t *c, MAG_RAW_DATA_t *raw, MAG_DATA_t *out)
{
    int i;

    switch (c->state) {
    case MAG_CAL_IDLE:
        c->offset[0] = 4.7840004f;
        c->offset[1] = -3.45000076f;
        c->offset[2] = 7.36000061f;
        c->scale[0] = 0.937098265f;
        c->scale[1] = 0.954184234f;
        c->scale[2] = 1.13012183f;
        c->state = MAG_CAL_DONE;
        break;

    case MAG_CAL_START:
        for (i = 0; i < 3; i++) {
            c->min[i] = 1e9f;
            c->max[i] = -1e9f;
        }
        c->samples = 0;
        c->state = MAG_CAL_COLLECTING;
        break;

    case MAG_CAL_COLLECTING:
        for (i = 0; i < 3; i++) {
            if (raw->mag[i] < c->min[i]) {
                c->min[i] = raw->mag[i];
            }
            if (raw->mag[i] > c->max[i]) {
                c->max[i] = raw->mag[i];
            }
        }
        c->samples++;
        if (c->samples >= c->samples_target) {
            c->state = MAG_CAL_COMPUTE;
        }
        break;

    case MAG_CAL_COMPUTE:
    {
        float radius[3];
        float avg_radius;

        for (i = 0; i < 3; i++) {
            c->offset[i] = (c->max[i] + c->min[i]) * 0.5f;
            radius[i] = (c->max[i] - c->min[i]) * 0.5f;
        }
        avg_radius = (radius[0] + radius[1] + radius[2]) / 3.0f;
        for (i = 0; i < 3; i++) {
            c->scale[i] = avg_radius / radius[i];
        }
        c->state = MAG_CAL_DONE;
        break;
    }

    case MAG_CAL_DONE:
        Mag_ApplyCalibration(c, raw, out);
        break;
    }

    if (c->state != MAG_CAL_DONE) {
        for (i = 0; i < 3; i++) {
            out->mag_uT[i] = 0;
        }
    }
}

void Mag_ApplyCalibration(MagCal_Simple_t *c, MAG_RAW_DATA_t *raw, MAG_DATA_t *out)
{
    int i;

    for (i = 0; i < 3; i++) {
        float x = raw->mag[i] - c->offset[i];

        x *= c->scale[i];
        out->mag_uT[i] = x * (1.0f / c->S);
    }
}
