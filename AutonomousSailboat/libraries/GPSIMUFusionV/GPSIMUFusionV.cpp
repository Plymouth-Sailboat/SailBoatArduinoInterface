#include <GPSIMUFusionV.h>

GPSIMUFusionV::GPSIMUFusionV(float var, float noise, float dt):noise(noise),dt(dt){
	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
			P[i][j] = 0;

	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
			R[i][j] = 0;

	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
			Q[i][j] = 0;

	for(int i = 0; i < 4; i++)
		P[i][i] = var;
	for(int i = 0; i < 4; i++)
		Q[i][i] = noise;
	for(int i = 0; i < 4; i++)
		R[i][i] = noise;
	for(int i = 0; i < 4; i++)
		x[i] = 0;
}
void GPSIMUFusionV::predict(){
	x[0] = x[0] + dt*x[2];
	x[1] = x[1] + dt*x[3];
	//x[2] = x[2];
	//x[3] = x[3];
	mtx_type F[4][4];
	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
			F[i][j] = 0;
	F[0][0] = 1;
	F[0][2] = dt;
	F[1][1] = 1;
	F[1][3] = dt;
	F[2][2] = 1;
	F[3][3] = 1;
	mtx_type FT[4][4];
	Matrix.Transpose((mtx_type*) F, 4, 4, (mtx_type*) FT);

	mtx_type res[4][4];
	Matrix.Multiply((mtx_type*)F, (mtx_type*)P, 4, 4, 4, (mtx_type*)res);
	mtx_type res2[4][4];
	Matrix.Multiply((mtx_type*)P, (mtx_type*)FT, 4, 4, 4, (mtx_type*)res2);
	Matrix.Add((mtx_type*)res2, (mtx_type*)Q, 4, 4, (mtx_type*)P);

	/*for(int i = 0; i < 4; i++)
		res[i] = P[i]+dt*P[8+i];
	for(int i = 0; i < 4; i++)
		res[i+4] = P[i+4]+dt*P[12+i];
	for(int i = 0; i < 4; i++)
		res[i+8] = P[i+8];
	for(int i = 0; i < 4; i++)
		res[i+12] = P[i+12];

	float res2[4*4];
	for(int i = 0; i < 4; i++)
		res2[i*4] = res[i*4]+dt*res[i*4+2] + Q[i*4];
	for(int i = 0; i < 4; i++)
		res2[i*4+1] = res[i*4+1]+dt*res[i*4+3] + Q[i*4];
	for(int i = 0; i < 4; i++)
		res2[i*4+2] = res[i*4+2] + Q[i*4];
	for(int i = 0; i < 4; i++)
		res2[i*4+3] = res[i*4+3] + Q[i*4];
	for(int i = 0; i < 16; i++)
		P[i] = res2[i];*/
}

void GPSIMUFusionV::update(float* meas){
	mtx_type y[4];
	for(int i =0; i < 4; i++){
		y[i] = meas[i]-x[i];
	}

	mtx_type S[4][4];
	//Matrix.Print((mtx_type*)P,4,4,"P");
	//Matrix.Print((mtx_type*)R,4,4,"R");
	Matrix.Add((mtx_type*)P,(mtx_type*)R,4,4,(mtx_type*)S);

	//Matrix.Print((mtx_type*)S,4,4,"S");
	Matrix.Invert((mtx_type*) S, 4);
	Matrix.Multiply((mtx_type*)P, (mtx_type*)S, 4, 4, 4, (mtx_type*)K);

	mtx_type y2[4];
	Matrix.Multiply((mtx_type*)K, (mtx_type*)y, 4, 4, 1, (mtx_type*)y2);

	for(int i =0; i < 4; i++){
		/*Serial.print(meas[i]);
		Serial.print(", ");
		Serial.print(y[i]);
		Serial.print(" ");*/
		x[i] = x[i] + y2[i];
		meas[i] = x[i];
	}
	//Serial.println(" ");

	mtx_type I[4][4];
	mtx_type KI[4][4];
	mtx_type P2[4][4];
	Matrix.Identity((mtx_type*)I,4);
	Matrix.Subtract((mtx_type*)I,(mtx_type*)K,4,4,(mtx_type*)KI);
	Matrix.Multiply((mtx_type*)KI,(mtx_type*)P,4,4,4,(mtx_type*)P2);
	Matrix.Copy((mtx_type*)P2,4,4,(mtx_type*)P);
}
