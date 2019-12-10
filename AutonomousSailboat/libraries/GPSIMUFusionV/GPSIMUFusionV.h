#ifndef GPSIMUFV_SENSOR_H
#define GPSIMUFV_SENSOR_H

#include <MatrixMath.h>
class GPSIMUFusionV{
	public:
		GPSIMUFusionV(float var, float noise, float dt);
		void predict();
		void update(float* meas);

	private:
		mtx_type x[4];
		mtx_type P[4][4];
		mtx_type Q[4][4];
		mtx_type R[4][4];
		mtx_type K[4][4];
		float dt;
		float noise;
};

#endif
