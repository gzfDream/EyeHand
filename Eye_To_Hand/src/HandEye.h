#pragma once

#include <opencv2\opencv.hpp>
#include <vector>
#include <iostream>
#include <fstream>  
#include <string>
#include "quaternion.h"
using namespace cv;
using namespace std;

//////////////////////////////////////////////////////////////////////////
void matrixMulti(double a[][4], double b[][4], double c[][4])
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			double num = 0;
			for (int k = 0; k < 4; k++)
			{
				num += a[i][k] * b[k][j];
			}
			c[i][j] = num;
		}
	}
}

Mat R_to_k(Mat &R)
{
	//行向量
	double theta = acos((R.at<double>(1, 1) + R.at<double>(2, 2) + R.at<double>(3, 3) - 1.) / 2);
	double c = 2 * sin(theta);
	double k_vector[3] = {};
	k_vector[0] = (R.at<double>(3, 2) - R.at<double>(2, 3)) / c;
	k_vector[1] = (R.at<double>(1, 3) - R.at<double>(3, 1)) / c;
	k_vector[2] = (R.at<double>(2, 1) - R.at<double>(1, 2)) / c;
	Mat k(3, 1, CV_64FC1, k_vector);
	return k;
}

Mat zhiji_Multiply(Mat&A, Mat&B)
{
	//矩阵直积
	int height = A.rows*B.rows;
	int width = A.cols*B.cols;
	Mat dst(height, width, CV_64FC1);
	for (int i = 0; i < A.rows; i++)
		for (int j = 0; j < A.cols; j++)
			for (int k = 0; k < B.rows; k++)
				for (int m = 0; m < B.cols; m++)
					dst.at<double>(i*B.rows + k, j*B.cols + m)
					= A.at<double>(i, j)*B.at<double>(k, m);
	return dst;
}

Mat vector_To_Mat(Mat&A)
{
	Mat dst(3, 3, CV_64FC1);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			dst.at<double>(i, j) = A.at<double>(i * 3 + j, 0);
	return dst;
}

//判断是否为单位正交矩阵
void judgement(const Mat &A)
{
	Mat b(3, 3, CV_64FC1);
	A(Rect(0, 0, 3, 3)).copyTo(b);
	double a[3][3] = {};
	for (int i = 0; i < 3; i++)
	{
		double *data = b.ptr<double>(i);
		for (int j = 0; j < 3; j++)
			a[i][j] = data[j];
	}

	//行列和是否各为1
	double m1 = a[0][0] * a[0][0] + a[0][1] * a[0][1] + a[0][2] * a[0][2];
	double m2 = a[1][0] * a[1][0] + a[1][1] * a[1][1] + a[1][2] * a[1][2];
	double m3 = a[2][0] * a[2][0] + a[2][1] * a[2][1] + a[2][2] * a[2][2];
	double m4 = a[0][0] * a[0][0] + a[1][0] * a[1][0] + a[2][0] * a[2][0];
	double m5 = a[0][1] * a[0][1] + a[1][1] * a[1][1] + a[2][1] * a[2][1];
	double m6 = a[0][2] * a[0][2] + a[1][2] * a[1][2] + a[2][2] * a[2][2];
	cout << m1 << "\t" << m2 << "\t" << m3 << "\t" << m4 << "\t" << m5 << "\t" << m6 << endl;
	//不同行列相乘是否为0
	double n1 = a[0][0] * a[0][1] + a[1][0] * a[1][1] + a[2][0] * a[2][1];
	double n2 = a[0][0] * a[0][2] + a[1][0] * a[1][2] + a[2][0] * a[2][2];
	double n3 = a[1][0] * a[2][0] + a[1][1] * a[2][1] + a[1][2] * a[2][2];
	double n4 = a[0][0] * a[1][0] + a[0][1] * a[1][1] + a[0][2] * a[1][2];
	double n5 = a[0][0] * a[2][0] + a[0][1] * a[2][1] + a[0][2] * a[2][2];
	double n6 = a[1][0] * a[2][0] + a[1][1] * a[2][1] + a[1][2] * a[2][2];
	cout << n1 << "\t" << n2 << "\t" << n3 << "\t" << n4 << "\t" << n5 << "\t" << n6 << endl;
	//行列式是否为1
	double detValue = a[0][0] * (a[1][1] * a[2][2] - a[2][1] * a[1][2])
		- a[0][1] * (a[1][0] * a[2][2] - a[2][0] * a[1][2])
		+ a[0][2] * (a[1][0] * a[2][1] - a[2][0] * a[1][1]);
	cout << detValue << endl << endl;
}

void errorCal(const vector<Mat>&Hgij, const vector<Mat>&Hcij, const Mat& handEye)
{
	int nRobotState = Hgij.size();

	vector<Mat>temps;
	vector<double> averageError;
	for (int i = 0; i < nRobotState; i++)
	{
		Mat temp = Hgij[i] * handEye - handEye * Hcij[i];
		Mat squareMat = temp.t()*temp;
		double count = 0;
		for (int i = 0; i < squareMat.rows; i++)
		{
			double *data = squareMat.ptr<double>(i);
			for (int j = 0; j < squareMat.cols; j++)
			{
				count += data[j];
			}
		}
		count /= (squareMat.cols*squareMat.rows);
		averageError.push_back(count);
	}
	cout << "平均误差：" << endl;
	for (int i = 0; i < averageError.size(); i++)
	{
		cout << averageError[i] << "\t";
		if ((i % 10) == 9)
			cout << endl;
	}
	cout << endl << endl;
}

void errorWithHcg(const Mat HcgCalculate, const Mat Hcg)
{
	Mat temp = HcgCalculate - Hcg;
	Mat tempMat = temp*temp.t();
	double count = 0;
	for (int i = 0; i < tempMat.rows; i++)
	{
		double*data = tempMat.ptr<double>(i);
		for (int j = 0; j < tempMat.cols; j++)
		{
			count += data[j];
		}
	}
	count /= (tempMat.cols*tempMat.rows);
	cout << "两个Hcg的平均误差为：\t" << count << endl << endl;
}

//////////////////////////////////////////////////////////////////////////

Mat skew(Mat A)
{
	CV_Assert(A.cols == 1 && A.rows == 3);
	Mat B(3, 3, CV_64FC1);

	B.at<double>(0, 0) = 0.0;
	B.at<double>(0, 1) = -A.at<double>(2, 0);
	B.at<double>(0, 2) = A.at<double>(1, 0);

	B.at<double>(1, 0) = A.at<double>(2, 0);
	B.at<double>(1, 1) = 0.0;
	B.at<double>(1, 2) = -A.at<double>(0, 0);

	B.at<double>(2, 0) = -A.at<double>(1, 0);
	B.at<double>(2, 1) = A.at<double>(0, 0);
	B.at<double>(2, 2) = 0.0;

	return B;
}

Mat readfcv(string str){
	Mat mat(4, 4, CV_64FC1);
	float x;
	ifstream  file(str);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			//file >> mat(i, j);
			file >> x;
			mat.at<double>(i, j) = x;
		}
	}

	file.close();
	//cout << mat << endl;
	return mat;
}

int getlineNum(string str){
	ifstream  file(str);
	int lineNum = 0;
	string temp;
	if (!file.fail())
		while (getline(file, temp))
			lineNum++;
	file.close();
	return lineNum;
}

std::vector<Mat> readf_vec(string str){
	std::vector<Mat> vec_mat;
	Mat mat(4, 4, CV_64FC1);
	float x;
	int lineNum = getlineNum(str);
	ifstream  file(str);
	for (int i = 0; i < lineNum; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			file >> x;
			mat.at<double>((i%4), j) = x;
		}

		if (i % 4 == 3){
			vec_mat.push_back(mat);
			mat.release();
			mat = Mat::ones(4, 4, CV_64FC1);
		}

	}
	file.close();
	//cout << mat << endl;
	return vec_mat;
}

/**
* Creates a dual quaternion from a rotation matrix and a translation vector.
*
* @Returns  void
* @param q [out] q
* @param qprime [out] q'
* @param R [in] Rotation
* @param t [in] Translation
*/
void getDualQ(Mat q, Mat qprime, Mat R, Mat t)
{
	Mat r(3, 1, CV_64FC1);
	Mat l(3, 1, CV_64FC1);
	double theta;
	Mat tempd(1, 1, CV_64FC1);
	double d;
	Mat c(3, 1, CV_64FC1);
	Mat m(3, 1, CV_64FC1);
	Mat templ(3, 1, CV_64FC1);
	Mat tempqt(1, 1, CV_64FC1);
	double qt;
	Mat tempml(3, 1, CV_64FC1);

	Rodrigues(R, r);
	theta = norm(r);
	l = r / theta;
	tempd = l.t()*t;
	d = tempd.at<double>(0, 0);

	c = 0.5*(t - d*l) + cos(theta / 2) / sin(theta / 2)*l.cross(t);
	m = c.cross(l);

	q.at<double>(0, 0) = cos(theta / 2);
	templ = sin(theta / 2)*l;
	templ.copyTo(q(Rect(0, 1, 1, 3)));

	tempqt = -0.5*templ.t()*t;
	qt = tempqt.at<double>(0, 0);
	tempml = 0.5*(q.at<double>(0, 0)*t + t.cross(templ));

	qprime.at<double>(0, 0) = qt;
	tempml.copyTo(qprime(Rect(0, 1, 1, 3)));

}

/**
* Compute the Kronecker tensor product of matrix A and B.
*
* @Returns  cv::Mat (MP)x(NQ) matrix
* @param A [in] MxN matrix
* @param B [in] PxQ matrix
*/
Mat kron(Mat A, Mat B)
{
	Mat C(A.rows*B.rows, A.cols*B.cols, CV_64FC1, Scalar(0));

	for (int i = 0; i < A.rows; i++)
		for (int j = 0; j < A.cols; j++)
			C(Rect(B.cols * j, B.rows * i, B.cols, B.rows)) = A.at<double>(i, j)*B;

	return C;
}

/**
* Signum function.
* For each element of X, SIGN(X) returns 1 if the element is greater than zero,
* return 0 if it equals zero and -1 if it is less than zero.
*
* @Returns  double
* @param a [in]
*/
double sign(double a)
{
	if (a > 0)
		return 1;
	else if (a < 0)
		return -1;
	else
		return 0;
}

/**
* Hand/Eye calibration using Tsai' method.
* Read the paper "A New Technique for Fully Autonomous and Efficient 3D Robotics
* Hand-Eye Calibration, Tsai, 1989" for further details.
*
* Solving AX=XB
* @Returns  void
* @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
* @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
* @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
*/
void Tsai_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	Mat Rgij(3, 3, CV_64FC1);
	Mat Rcij(3, 3, CV_64FC1);

	Mat rgij(3, 1, CV_64FC1);
	Mat rcij(3, 1, CV_64FC1);

	double theta_gij;
	double theta_cij;

	Mat rngij(3, 1, CV_64FC1);
	Mat rncij(3, 1, CV_64FC1);

	Mat Pgij(3, 1, CV_64FC1);
	Mat Pcij(3, 1, CV_64FC1);

	Mat tempA(3, 3, CV_64FC1);
	Mat tempb(3, 1, CV_64FC1);

	Mat A;
	Mat b;
	Mat pinA;

	Mat Pcg_prime(3, 1, CV_64FC1);
	Mat Pcg(3, 1, CV_64FC1);
	Mat PcgTrs(1, 3, CV_64FC1);

	Mat Rcg(3, 3, CV_64FC1);
	Mat eyeM = Mat::eye(3, 3, CV_64FC1);

	Mat Tgij(3, 1, CV_64FC1);
	Mat Tcij(3, 1, CV_64FC1);

	Mat tempAA(3, 3, CV_64FC1);
	Mat tempbb(3, 1, CV_64FC1);

	Mat AA;
	Mat bb;
	Mat pinAA;

	Mat Tcg(3, 1, CV_64FC1);

	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);

		Rodrigues(Rgij, rgij);
		Rodrigues(Rcij, rcij);

		theta_gij = norm(rgij);
		theta_cij = norm(rcij);

		rngij = rgij / theta_gij;
		rncij = rcij / theta_cij;

		Pgij = 2 * sin(theta_gij / 2)*rngij;
		Pcij = 2 * sin(theta_cij / 2)*rncij;

		tempA = skew(Pgij + Pcij);
		tempb = Pcij - Pgij;

		A.push_back(tempA);
		b.push_back(tempb);
	}

	//Compute rotation
	invert(A, pinA, DECOMP_SVD);

	Pcg_prime = pinA * b;
	Pcg = 2 * Pcg_prime / sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime));
	PcgTrs = Pcg.t();
	Rcg = (1 - norm(Pcg) * norm(Pcg) / 2) * eyeM + 0.5 * (Pcg * PcgTrs + sqrt(4 - norm(Pcg)*norm(Pcg))*skew(Pcg));

	//Computer Translation 
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

		tempAA = Rgij - eyeM;
		tempbb = Rcg * Tcij - Tgij;

		AA.push_back(tempAA);
		bb.push_back(tempbb);
	}

	invert(AA, pinAA, DECOMP_SVD);
	Tcg = pinAA * bb;

	Rcg.copyTo(Hcg(Rect(0, 0, 3, 3)));
	Tcg.copyTo(Hcg(Rect(3, 0, 1, 3)));
	Hcg.at<double>(3, 0) = 0.0;
	Hcg.at<double>(3, 1) = 0.0;
	Hcg.at<double>(3, 2) = 0.0;
	Hcg.at<double>(3, 3) = 1.0;

}
/**
* Hand/Eye calibration using Park' method(NAVY).
* Read the paper "Robot Sensor Calibration: Solving AX = XB on the Euclidean Group, 1994" for further details.
*
* @Returns  void
* @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
* @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
* @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
*/
void Navy_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	Mat Rgij(3, 3, CV_64FC1);
	Mat Rcij(3, 3, CV_64FC1);

	Mat alpha1(3, 1, CV_64FC1);
	Mat beta1(3, 1, CV_64FC1);
	Mat alpha2(3, 1, CV_64FC1);
	Mat beta2(3, 1, CV_64FC1);
	Mat A(3, 3, CV_64FC1);
	Mat B(3, 3, CV_64FC1);

	Mat alpha(3, 1, CV_64FC1);
	Mat beta(3, 1, CV_64FC1);
	Mat M(3, 3, CV_64FC1, Scalar(0));

	Mat MtM(3, 3, CV_64FC1);
	Mat veMtM(3, 3, CV_64FC1);
	Mat vaMtM(3, 1, CV_64FC1);
	Mat pvaM(3, 3, CV_64FC1, Scalar(0));

	Mat Rx(3, 3, CV_64FC1);

	Mat Tgij(3, 1, CV_64FC1);
	Mat Tcij(3, 1, CV_64FC1);

	Mat eyeM = Mat::eye(3, 3, CV_64FC1);

	Mat tempCC(3, 3, CV_64FC1);
	Mat tempdd(3, 1, CV_64FC1);

	Mat C;
	Mat d;
	Mat Tx(3, 1, CV_64FC1);

	//Compute rotation
	if (Hgij.size() == 2) // Two (Ai,Bi) pairs
	{
		Rodrigues(Hgij[0](Rect(0, 0, 3, 3)), alpha1);
		Rodrigues(Hgij[1](Rect(0, 0, 3, 3)), alpha2);
		Rodrigues(Hcij[0](Rect(0, 0, 3, 3)), beta1);
		Rodrigues(Hcij[1](Rect(0, 0, 3, 3)), beta2);

		alpha1.copyTo(A.col(0));
		alpha2.copyTo(A.col(1));
		(alpha1.cross(alpha2)).copyTo(A.col(2));

		beta1.copyTo(B.col(0));
		beta2.copyTo(B.col(1));
		(beta1.cross(beta2)).copyTo(B.col(2));

		Rx = A*B.inv();

	}
	else // More than two (Ai,Bi) pairs
	{
		for (int i = 0; i < nStatus; i++)
		{
			Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
			Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);

			Rodrigues(Rgij, alpha);
			Rodrigues(Rcij, beta);

			M = M + beta*alpha.t();
		}

		MtM = M.t()*M;
		eigen(MtM, vaMtM, veMtM);

		pvaM.at<double>(0, 0) = 1 / sqrt(vaMtM.at<double>(0, 0));
		pvaM.at<double>(1, 1) = 1 / sqrt(vaMtM.at<double>(1, 0));
		pvaM.at<double>(2, 2) = 1 / sqrt(vaMtM.at<double>(2, 0));

		Rx = veMtM*pvaM*veMtM.inv()*M.t();
	}

	//Computer Translation 
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

		tempCC = eyeM - Rgij;
		tempdd = Tgij - Rx * Tcij;

		C.push_back(tempCC);
		d.push_back(tempdd);
	}

	Tx = (C.t()*C).inv()*(C.t()*d);

	Rx.copyTo(Hcg(Rect(0, 0, 3, 3)));
	Tx.copyTo(Hcg(Rect(3, 0, 1, 3)));
	Hcg.at<double>(3, 0) = 0.0;
	Hcg.at<double>(3, 1) = 0.0;
	Hcg.at<double>(3, 2) = 0.0;
	Hcg.at<double>(3, 3) = 1.0;

}

/**
* Hand/Eye calibration using Daniilidis' method.
* Read the paper "Hand-Eye Calibration Using Dual Quaternions, Konstantinos Daniilidis, 1999" for further details.
*
* @Returns  void
* @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
* @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
* @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
*/
void DualQ_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	Mat Rgij(3, 3, CV_64FC1);
	Mat Rcij(3, 3, CV_64FC1);
	Mat Tgij(3, 1, CV_64FC1);
	Mat Tcij(3, 1, CV_64FC1);

	Mat Qa(4, 1, CV_64FC1);
	Mat Qaprime(4, 1, CV_64FC1);
	Mat Qb(4, 1, CV_64FC1);
	Mat Qbprime(4, 1, CV_64FC1);

	Mat a(3, 1, CV_64FC1);
	Mat aprime(3, 1, CV_64FC1);
	Mat b(3, 1, CV_64FC1);
	Mat bprime(3, 1, CV_64FC1);

	Mat a_b(3, 1, CV_64FC1);
	Mat ap_bp(3, 1, CV_64FC1);
	Mat axb(3, 3, CV_64FC1);
	Mat apxbp(3, 3, CV_64FC1);

	Mat zero1 = Mat::zeros(3, 1, CV_64FC1);
	Mat zero3 = Mat::zeros(3, 3, CV_64FC1);

	Mat S(6, 8, CV_64FC1);
	Mat T;

	Mat w, u, vt, v;

	Mat v7, v8;
	Mat u1, v1, u2, v2;

	Mat coeffs(3, 1, CV_64FC1);
	Mat s;

	double val1, val2, fs, val;
	double lamda1, lamda2;

	Mat Qresult, Q, Qprime;
	Mat Rresult(3, 3, CV_64FC1);
	Mat Tresult(3, 1, CV_64FC1);

	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

		getDualQ(Qa, Qaprime, Rgij, Tgij);
		getDualQ(Qb, Qbprime, Rcij, Tcij);

		Qa(Rect(0, 1, 1, 3)).copyTo(a);
		Qaprime(Rect(0, 1, 1, 3)).copyTo(aprime);
		Qb(Rect(0, 1, 1, 3)).copyTo(b);
		Qbprime(Rect(0, 1, 1, 3)).copyTo(bprime);

		a_b = a - b;
		ap_bp = aprime - bprime;

		axb = skew(a + b);
		apxbp = skew(aprime + bprime);

		a_b.copyTo(S(Rect(0, 0, 1, 3)));
		axb.copyTo(S(Rect(1, 0, 3, 3)));
		zero1.copyTo(S(Rect(4, 0, 1, 3)));
		zero3.copyTo(S(Rect(5, 0, 3, 3)));

		ap_bp.copyTo(S(Rect(0, 3, 1, 3)));
		apxbp.copyTo(S(Rect(1, 3, 3, 3)));
		a_b.copyTo(S(Rect(4, 3, 1, 3)));
		axb.copyTo(S(Rect(5, 3, 3, 3)));

		T.push_back(S);
	}

	SVD::compute(T, w, u, vt);
	v = vt.t();

	v(Rect(6, 0, 1, 8)).copyTo(v7);
	v(Rect(7, 0, 1, 8)).copyTo(v8);

	v7(Rect(0, 0, 1, 4)).copyTo(u1);
	v7(Rect(0, 4, 1, 4)).copyTo(v1);
	v8(Rect(0, 0, 1, 4)).copyTo(u2);
	v8(Rect(0, 4, 1, 4)).copyTo(v2);

	coeffs.at<double>(0, 0) = u1.dot(v1);
	coeffs.at<double>(1, 0) = u1.dot(v2) + u2.dot(v1);
	coeffs.at<double>(2, 0) = u2.dot(v2);

	solvePoly(coeffs, s);

	val1 = s.at<double>(0, 0)*s.at<double>(0, 0)*u1.dot(u1) + 2 * s.at<double>(0, 0)*u1.dot(u2) + u2.dot(u2);
	val2 = s.at<double>(1, 0)*s.at<double>(1, 0)*u1.dot(u1) + 2 * s.at<double>(1, 0)*u1.dot(u2) + u2.dot(u2);

	if (val1 > val2)
	{
		fs = s.at<double>(0, 0);
		val = val1;

	}
	else
	{
		fs = s.at<double>(1, 0);
		val = val2;
	}

	lamda2 = sqrt(1 / val);
	lamda1 = fs*lamda2;

	Qresult = lamda1*v7 + lamda2*v8;
	Qresult(Rect(0, 0, 1, 4)).copyTo(Q);
	Qresult(Rect(0, 4, 1, 4)).copyTo(Qprime);

	Rresult = q2dcm(Q);
	Tresult = 2 * qmult(Qprime, qconj(Q))(Rect(0, 1, 1, 3));

	Rresult.copyTo(Hcg(Rect(0, 0, 3, 3)));
	Tresult.copyTo(Hcg(Rect(3, 0, 1, 3)));

	Hcg.at<double>(3, 0) = 0.0;
	Hcg.at<double>(3, 1) = 0.0;
	Hcg.at<double>(3, 2) = 0.0;
	Hcg.at<double>(3, 3) = 1.0;

}

/**
* Hand/Eye calibration using Andreff' method.
* Read the paper "Robot Hand-Eye Calibration Using Structure-from-Motion, Andreff N, Horaud R, 2001" for further details.
*
* @Returns  void
* @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
* @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
* @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
*/
void Kron_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	Mat Rgij(3, 3, CV_64FC1);
	Mat Rcij(3, 3, CV_64FC1);
	Mat eyeM9 = Mat::eye(9, 9, CV_64FC1);

	Mat tempLi(9, 9, CV_64FC1);
	Mat Li;

	Mat w, u, vt, v;
	Mat vi(9, 1, CV_64FC1);
	Mat Vi(3, 3, CV_64FC1);
	Mat Rcg(3, 3, CV_64FC1);

	Mat tempCC(3, 3, CV_64FC1);
	Mat tempdd(3, 1, CV_64FC1);

	Mat Tgij(3, 1, CV_64FC1);
	Mat Tcij(3, 1, CV_64FC1);

	Mat eyeM3 = Mat::eye(3, 3, CV_64FC1);
	Mat C;
	Mat d;
	Mat Tcg(3, 1, CV_64FC1);

	// Compute rotation
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);

		tempLi = eyeM9 - kron(Rgij, Rcij);
		Li.push_back(tempLi);
	}

	SVD::compute(Li, w, u, vt, SVD::FULL_UV);
	v = vt.t();
	v(Rect(8, 0, 1, 9)).copyTo(vi);
	Vi = vi.reshape(0, 3);
	Rcg = sign(determinant(Vi))*pow(abs(determinant(Vi)), -1.0 / 3)*Vi; // -1.0/3 NOT -1/3

	//Computer Translation 
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

		tempCC = eyeM3 - Rgij;
		tempdd = Tgij - Rcg * Tcij;

		C.push_back(tempCC);
		d.push_back(tempdd);
	}

	Tcg = (C.t()*C).inv()*(C.t()*d);

	Rcg.copyTo(Hcg(Rect(0, 0, 3, 3)));
	Tcg.copyTo(Hcg(Rect(3, 0, 1, 3)));
	Hcg.at<double>(3, 0) = 0.0;
	Hcg.at<double>(3, 1) = 0.0;
	Hcg.at<double>(3, 2) = 0.0;
	Hcg.at<double>(3, 3) = 1.0;

}

/**
* Hand/Eye calibration using Horaud' method(INRIA).
* Read the paper "Hand-Eye Calibration, Radu Horaud and Fadi Dornaika, 1995" for further details.
*
* @Returns  void
* @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
* @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
* @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
*/
void Inria_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	Mat Rgij(3, 3, CV_64FC1);
	Mat Rcij(3, 3, CV_64FC1);
	Mat Tgij(3, 1, CV_64FC1);
	Mat Tcij(3, 1, CV_64FC1);

	Mat Qgij(4, 1, CV_64FC1);
	Mat Qcij(4, 1, CV_64FC1);
	Mat Lqij(4, 4, CV_64FC1);
	Mat Rqij(4, 4, CV_64FC1);

	Mat tempA(4, 4, CV_64FC1);
	Mat A;
	Mat w, u, vt, v;
	Mat qoff(4, 1, CV_64FC1);
	Mat Roff(3, 3, CV_64FC1);

	Mat eyeM = Mat::eye(3, 3, CV_64FC1);
	Mat tempAA(3, 3, CV_64FC1);
	Mat tempbb(3, 1, CV_64FC1);
	Mat AA;
	Mat bb;
	Mat pinAA;

	Mat Toff(3, 1, CV_64FC1);

	// Compute rotation
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

		Qgij = dcm2q(Rgij);
		Qcij = dcm2q(Rcij);

		// qA*qX=qX*qB<=>(RqA-LqB)*qx=0
		Lqij = qskewL(Qgij);
		Rqij = qskewR(Qcij);

		tempA = Lqij - Rqij;
		A.push_back(tempA);
	}

	SVD::compute(A, w, u, vt, SVD::FULL_UV);
	v = vt.t();
	v(Rect(3, 0, 1, 4)).copyTo(qoff);
	Roff = q2dcm(qoff);

	// Computer Translation 
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

		tempAA = Rgij - eyeM;
		tempbb = Roff * Tcij - Tgij;

		AA.push_back(tempAA);
		bb.push_back(tempbb);
	}

	invert(AA, pinAA, DECOMP_SVD);
	Toff = pinAA * bb;

	Roff.copyTo(Hcg(Rect(0, 0, 3, 3)));
	Toff.copyTo(Hcg(Rect(3, 0, 1, 3)));

	Hcg.at<double>(3, 0) = 0.0;
	Hcg.at<double>(3, 1) = 0.0;
	Hcg.at<double>(3, 2) = 0.0;
	Hcg.at<double>(3, 3) = 1.0;

}

//优化的手眼标定1
Mat developed_HandEye1(const vector<Mat>Hgij, const vector<Mat>Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nRobotState = Hgij.size();

	Mat I = Mat::eye(3, 3, CV_64FC1);
	Mat RxleftMat;
	for (int i = 0; i < nRobotState; i++)
	{
		Mat Rgij(3, 3, CV_64FC1);
		Mat Rcij(3, 3, CV_64FC1);
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Mat RcijT = Rcij.t();
		Mat tempMat = zhiji_Multiply(Rgij, I) - zhiji_Multiply(I, RcijT);
		RxleftMat.push_back(tempMat);
	}
	int rows = nRobotState / 2 * 9;
	Mat lamda(9, 1, CV_64FC1);         //奇异值
	Mat U(rows, 9, CV_64FC1);          //左矩阵
	Mat VT(9, 9, CV_64FC1);         //右矩阵的转置
	//cout << temp << endl;
	//temp = temp.t();  //结果一样？！
	SVD::compute(RxleftMat, lamda, U, VT);
	//cout << "左矩阵：\n" << U << endl;
	//cout << "特征值：\n" << lamda << endl;
	//cout << "右矩阵：\n" << VT << endl;
	Mat V(9, 9, CV_64FC1);
	V = VT.t();
	Mat vector_x(9, 1, CV_64FC1);
	V(Rect(8, 0, 1, 9)).copyTo(vector_x);
	Mat Rx(3, 3, CV_64FC1);
	Rx = vector_To_Mat(vector_x);
	//cout << Rx << endl;
	Mat leftMat;
	Mat rightMat;
	for (int i = 0; i < nRobotState; i++)
	{
		Mat Rgij(3, 3, CV_64FC1);
		Mat Tgij(3, 1, CV_64FC1);
		Mat Tcij(3, 1, CV_64FC1);
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);
		Mat tempL = Rgij - I;
		leftMat.push_back(tempL);
		Mat tempR = Rx*Tcij - Tgij;
		rightMat.push_back(tempR);
	}
	Mat leftMat_invert;
	invert(leftMat, leftMat_invert, DECOMP_SVD);
	Mat Tx = leftMat_invert * rightMat;
	Mat Hcg;
	hconcat(Rx, Tx, Hcg);
	//cout << Temp << endl;
	double homogeneous[4] = { 0, 0, 0, 1 };
	Mat h(1, 4, CV_64FC1, homogeneous);
	Hcg.push_back(h);
	//vconcat(dst, h, dst);
	return Hcg;
}

//优化的手眼标定2
Mat developMethod2(const vector<Mat>Hgij, const vector<Mat>Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nRobotState = Hgij.size();

	Mat I = Mat::eye(3, 3, CV_64FC1);
	Mat leftMat, rigthMat;
	for (int i = 0; i < nRobotState; i++)
	{
		Mat Rgij(3, 3, CV_64FC1);
		Mat Tgij(3, 1, CV_64FC1);
		Mat Rcij(3, 3, CV_64FC1);
		Mat Tcij(3, 1, CV_64FC1);
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);
		Mat RcijT = Rcij.t();
		Mat tempMatLU = zhiji_Multiply(Rgij, I) - zhiji_Multiply(I, RcijT);
		Mat TcijT = Tcij.t();
		Mat tempMatLD = zhiji_Multiply(I, TcijT);
		Mat tempMatRD = I - Rgij;
		Mat tempLeftMat(12, 12, CV_64FC1, Scalar(0));
		tempMatLU.copyTo(tempLeftMat(Rect(0, 0, 9, 9)));
		tempMatLD.copyTo(tempLeftMat(Rect(0, 9, 9, 3)));
		tempMatRD.copyTo(tempLeftMat(Rect(9, 9, 3, 3)));
		leftMat.push_back(tempLeftMat);
		Mat tempRightMat(12, 1, CV_64FC1, Scalar(0));
		Tgij.copyTo(tempRightMat(Rect(0, 9, 1, 3)));
		rigthMat.push_back(tempRightMat);
	}

	int rows = nRobotState * 12;
	Mat lamda(12, 1, CV_64FC1);         //奇异值
	Mat U(rows, 12, CV_64FC1);          //左矩阵  (不是方阵！！！)
	Mat VT(12, 12, CV_64FC1);         //右矩阵的转置
	SVD::compute(leftMat, lamda, U, VT);
	//cout << "左矩阵：\n" << U << endl << endl;       //Ut*U=I
	//cout << "特征值：\n" << lamda << endl;           //Vt*V=I
	//cout << "右矩阵：\n" << VT << endl << endl;
	Mat Ut = U.t();
	Mat rightMat_pian = Ut*rigthMat;
	Mat tempYfactor(12, 1, CV_64FC1);
	for (int i = 0; i < 12; i++)
	{
		double*dataY = tempYfactor.ptr<double>(i);
		double*datalamda = lamda.ptr<double>(i);
		double*datarightMat_pian = rightMat_pian.ptr<double>(i);
		dataY[0] = datarightMat_pian[0] / datalamda[0];
	}
	Mat XXX = VT.t()*tempYfactor;
	Mat vector_x(9, 1, CV_64FC1);
	XXX(Rect(0, 0, 1, 9)).copyTo(vector_x);
	Mat Rx = vector_To_Mat(vector_x);
	Mat Tx(1, 3, CV_64FC1);
	XXX(Rect(0, 9, 1, 3)).copyTo(Tx);

	Mat Hcg;
	hconcat(Rx, Tx, Hcg);
	//cout << Temp << endl;
	double homogeneous[4] = { 0, 0, 0, 1 };
	Mat h(1, 4, CV_64FC1, homogeneous);
	Hcg.push_back(h);
	//vconcat(dst, h, dst);
	return Hcg;
}

//优化的手眼标定3
Mat developMethod3(const vector<Mat>Hgij, const vector<Mat>Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nRobotState = Hgij.size();

	Mat I = Mat::eye(3, 3, CV_64FC1);
	Mat leftMat, rigthMat;
	for (int i = 0; i < nRobotState; i++)
	{
		Mat Rgij(3, 3, CV_64FC1);
		Mat Tgij(3, 1, CV_64FC1);
		Mat Rcij(3, 3, CV_64FC1);
		Mat Tcij(3, 1, CV_64FC1);
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);
		Mat RcijT = Rcij.t();
		Mat tempMatLU = zhiji_Multiply(Rgij, I) - zhiji_Multiply(I, RcijT);
		Mat TbT = Tcij.t();
		Mat tempMatLD = zhiji_Multiply(I, TbT);
		Mat tempMatRD = I - Rgij;
		Mat tempLeftMat(12, 12, CV_64FC1, Scalar(0));
		tempMatLU.copyTo(tempLeftMat(Rect(0, 0, 9, 9)));
		tempMatLD.copyTo(tempLeftMat(Rect(0, 9, 9, 3)));
		tempMatRD.copyTo(tempLeftMat(Rect(9, 9, 3, 3)));
		leftMat.push_back(tempLeftMat);
		Mat tempRightMat(12, 1, CV_64FC1, Scalar(0));
		Tgij.copyTo(tempRightMat(Rect(0, 9, 1, 3)));
		rigthMat.push_back(tempRightMat);
	}
	Mat mat;
	hconcat(leftMat, rigthMat, mat);
	int rows = nRobotState * 12;
	Mat lamda(12, 1, CV_64FC1);         //奇异值
	Mat U(rows, 12, CV_64FC1);          //左矩阵  (不是方阵！！！)
	Mat VT(12, 12, CV_64FC1);         //右矩阵的转置
	SVD::compute(mat, lamda, U, VT);
	//cout << "左矩阵：\n" << U << endl << endl;       //Ut*U=I
	//cout << "特征值：\n" << lamda << endl;           //Vt*V=I
	//cout << "右矩阵：\n" << VT << endl << endl;
	Mat V = VT.t();
	Mat mm;
	V(Rect(12, 12, 1, 1)).copyTo(mm);
	double mmmm = mm.at<double>(0, 0);
	Mat vector_x(9, 1, CV_64FC1);
	V(Rect(12, 0, 1, 9)).copyTo(vector_x);
	Mat Rx = vector_To_Mat(vector_x) / mmmm*(-1);
	Mat Tx(1, 3, CV_64FC1);
	V(Rect(12, 9, 1, 3)).copyTo(Tx);
	Tx /= (-mmmm);

	Mat dst;
	hconcat(Rx, Tx, dst);
	//cout << Temp << endl;
	double homogeneous[4] = { 0, 0, 0, 1 };
	Mat h(1, 4, CV_64FC1, homogeneous);
	dst.push_back(h);
	//vconcat(dst, h, dst);
	return dst;
}

void processes(){
	Mat Hcg(4, 4, CV_64FC1);
	vector<Mat> Hgij;
	vector<Mat> Hcij;
	std::vector<Mat> basHtool;
	std::vector<Mat> calHcam;
	Mat matA(4, 4, CV_64FC1), matB(4, 4, CV_64FC1);

	basHtool = readf_vec("..//data//calibration//robot.txt");
	calHcam = readf_vec("..//data//calibration//cal_cam_robot.txt");

	if (basHtool.size() == calHcam.size())
		for (int i = 0; i < basHtool.size()-1; i++)
		{
			//calHcam*camHbase*baseHtool
			matB = basHtool[i + 1] * basHtool[i].inv();
			matA = calHcam[i + 1].inv() * calHcam[i];

			Hgij.push_back(matA);
			Hcij.push_back(matB);
			
			cout << "matA" << matA << endl;
			cout << "matB" << matB << endl;

			matA.release();
			matB.release();
		}
	/*
	*手眼标定
	*/
	//Tsai_HandEye(Hcg, Hgij, Hcij);
	//DualQ_HandEye(Hcg, Hgij, Hcij);
	//Inria_HandEye(Hcg, Hgij, Hcij);
	//Kron_HandEye(Hcg, Hgij, Hcij);
	Hcg = developMethod3(Hgij, Hcij);

	cout << "Hcg: \n" << Hcg << endl;

	cv::Mat m_result = Hcg.inv();
	cout << "result: \n" << m_result << endl;
	std::ofstream ofsCalib("..//data//calibration//result.txt");
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++)
		{
			ofsCalib << m_result.at<double>(i, j) << " ";
		}
		ofsCalib << std::endl;
	}

	ofsCalib.close();

}
