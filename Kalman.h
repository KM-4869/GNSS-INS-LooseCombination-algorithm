#pragma once
#include"Matrix2.h"

void kf_predict(Matrix& x, Matrix& P, Matrix& A, Matrix& q, Matrix& B, Matrix& u, Matrix& x_predict, Matrix& P_predict);
void kf_predict(Matrix& x, Matrix& P, Matrix& A, Matrix& q, Matrix& x_predict, Matrix& P_predict);
void kf_update(Matrix& x_predict, Matrix& P_predict, Matrix& y, Matrix& H, Matrix& R, Matrix& x, Matrix& P);