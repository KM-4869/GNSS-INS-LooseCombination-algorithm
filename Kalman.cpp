
#include"Kalman.h"
/*
   x - Nx1 mean state estimate of previous step
   P - NxN state covariance of previous step
   A - Transition matrix of discrete model 
   q - Process noise of discrete model     
   B - Input effect matrix                 (optional)
   U - Constant input                      (optional)
*/
void kf_predict(Matrix& x, Matrix& P, Matrix& A, Matrix& q, Matrix& B, Matrix& u, Matrix& x_predict, Matrix& P_predict)
{
	Matrix Tx(x.getrow(), x.getcol());
	Matrix TP(P.getrow(), P.getcol());
	Tx = x;
	TP = P;

	x_predict = A * Tx + B * u;
	P_predict = A * TP * A.T() + q;
}

void kf_predict(Matrix& x, Matrix& P, Matrix& A, Matrix& q, Matrix& x_predict, Matrix& P_predict)
{
	x_predict = A * x;
	P_predict = A * P * A.T() + q;
}

/*
   x_predict - Nx1 mean state estimate after prediction step
   P_predict - NxN state covariance after prediction step
   Y - Dx1 measurement vector.
   H - Measurement matrix.
   R - Measurement noise covariance.
*/
void kf_update(Matrix& x_predict, Matrix& P_predict, Matrix& y, Matrix& H, Matrix& R, Matrix& x, Matrix& P)
{
	Matrix K(x_predict.getrow(), H.getrow());
	Matrix I(1, 1);
	I.ToE(x_predict.getrow());

	K = P_predict * H.T() * (H * P_predict * H.T() + R).inv();
	x = x_predict + K * (y - H * x_predict);
	P = (I - K * H) * P_predict * (I - K * H).T() + K * R * K.T();

}