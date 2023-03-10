#pragma once
#include<stdarg.h>
class Vector;
class Matrix
{
protected:
	int MatrixRows;//��������
	int MatrixCols;//��������
	double* M;//��һά�����ʾ����
	void Initialize();//��ʼ������Ϊ�������ռ�

public:
	Matrix(const Matrix& m);//�ֶ���д���ƹ��캯��
	Matrix(int, int, double = 0.0);//Ĭ�Ͼ�������Ԫ�ظ�ֵΪ0.����ע�⣺������ֵΪ��������Ҫ����ں���ӡ�.0������ʽ����
	Matrix(int, int, char, ...);//�����������ÿ��Ԫ�ظ���ֵ�����е���������ΪΪ�������غ������裬Ҳ�������־��������������Ҫ����ֵ���Ƽ�ʹ�á�|��������ע�⣺������ֵΪ��������Ҫ����ں���ӡ�.0������ʽ����
	virtual ~Matrix();//��������

	Matrix& operator=(const Matrix&);//����Ծ���ֵ
	Matrix& operator=(double Array[]);//����Ծ���ֵ(Ҫ����������������Ĵ�С)
	void assign(int i, int j, double value);//�Ե�i�е�j�е�Ԫ�ظ�ֵ
	Matrix operator+(const Matrix& m)const;//����ӷ�
	Matrix operator-(const Matrix& m)const;//�������
	Matrix operator*(const Matrix& m)const;//����˷�
	Matrix operator,(const Matrix& m)const;//��������������Һϲ�
	Matrix operator&(const Matrix& m)const;//��������������ºϲ�
	Matrix SubMatrix(int topleft_row, int topleft_col, int submatrixrow, int submatrixcol)const;//ȡ�Ӿ���
	friend Matrix operator*(const double C,const Matrix&m);//��������
	friend Matrix operator*(const Matrix& m, const double C);//��������
	Matrix T()const;//����ת��
	double det()const;//������ʽ
	Matrix A()const;//�������
	Matrix inv()const;//��������
	void ToE(int n);//�������Ϊnά��λ��
	void ToZero();//��һ������ȫ������
	void Show()const;

	int getrow()const;
	int getcol()const;
	int getsize()const;
	double getelement(int row,int col)const;


/*******************************************************************************
����ά�ռ�������������ת�� �;�������˳�򣬵õ�һ����ת����order��1-12�ֱ��Ӧ
  XYZ   XZY   YXZ   YZX   ZXY   ZYX   XYX   XZX   YXY   YZY   ZXZ   ZYZ
  ������������˳���������ת˳���෴����������ת˳��ΪZ��->X��->Y�ᣬ��ѡ��order=3��
********************************************************************************/
	friend Matrix RotationMatrix(double x_angle, double y_angle, double z_angle, int order);
};

Matrix RotationMatrix(double, double, double, int);