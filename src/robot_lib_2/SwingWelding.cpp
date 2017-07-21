/*
 * SwingWelding.cpp
 *
 *  Created on: 2017Äê7ÔÂ20ÈÕ
 *      Author: deng
 */

#include <SwingWelding.h>
#include <vector>
SwingWelding::SwingWelding() {
	// TODO Auto-generated constructor stub

}

SwingWelding::~SwingWelding() {
	// TODO Auto-generated destructor stub
}

vector<Quaterniond> SwingWelding::Position3DInterpolationTri(const Vector3d& pa,const Vector3d& pb,const Vector3d& pc,const Vector3d& pd,const Vector3d& pe,const Vector3d& pf,vector<Vector3d> & pMN,int n = 1, Quaterniond * qa = NULL, Quaterniond * qc = NULL)
{
	if(n <= 0)
	  {
		cerr << "warning: n mast >= 1, now set n = 1 !" << endl;
		n = 1;
	  }
	  pMN.clear();
	  vector<Quaterniond> qMN;
	  qMN.clear();
	  vector<double> T;
	  T.clear();
	  if(qa != NULL && qc != NULL)
	  {
		  for(int i = 0; i < 2 * n + 1;i++)
			  T.push_back( i / (double)(2 * n));
		   for (double e : T)
	       qMN.push_back(Slerp(*qa,*qc,e));
	  }
	  T.clear();
	  for(int i = 0; i < 2 * n;i++)
	 	T.push_back(i / (double)(2 * n));
	  Vector3d temp(0,0,0);
	  for (int i = 0; i < n; i++)
	  {
		  temp = Position3DInterpolation(pa, pb, pc, T[i*2]);
		  pMN.push_back(temp);
		  temp = Position3DInterpolation(pd, pe, pf, T[i*2 + 1]);
		  pMN.push_back(temp);
	  }
	  pMN.push_back(pc);
	  return qMN;

}
vector<Quaterniond> SwingWelding::PositionLinearInterpolationTri(const Vector3d& pa,const Vector3d& pb,const Vector3d& pc,const Vector3d& pd,vector<Vector3d> & pMN,int n, Quaterniond * qa, Quaterniond * qd)
{
	if(n <= 0)
	  {
		cerr << "warning: n mast >= 1, now set n = 1 !" << endl;
		n = 1;
	  }
	  pMN.clear();
	  vector<Quaterniond> qMN;
	  qMN.clear();
	  vector<double> T;
	  T.clear();
	  if(qa != NULL && qd != NULL)
	  {
		  for(int i = 0; i < 2 * n + 1;i++)
			  T.push_back(i / (double)(2 * n));
		   for (double e : T)
		   qMN.push_back(Lerp(*qa,*qd,e));
	  }
	  T.clear();
	  for(int i = 0; i < 2 * n;i++)
		T.push_back(i / (double)(2 * n));
	  Vector3d temp(0,0,0);
	  for (int i = 0; i < n; i++)
	  {
		  temp = PositionLinearInterpolation(pa, pd,T[i*2]);
		  pMN.push_back(temp);
		  temp = PositionLinearInterpolation(pb, pc,T[i*2 + 1]);
		  pMN.push_back(temp);
	  }
	  pMN.push_back(pc);
	  return qMN;
}

list<MJCoint> SwingWelding::PositionAttitudeLinearInterpolationTri(const Matrix4d& A, const Matrix4d& B,const Matrix4d& C, const Matrix4d& D,int n)
{
	Vector3d pa = Vector3d(A(0,3),A(1,3),A(2,3));
	Vector3d pb = Vector3d(B(0,3),B(1,3),B(2,3));
	Vector3d pc = Vector3d(C(0,3),C(1,3),C(2,3));
	Vector3d pd = Vector3d(D(0,3),D(1,3),D(2,3));
	Quaterniond qa = pose_2_quaternion(A);
	Quaterniond qd = pose_2_quaternion(D);
	vector<Vector3d> pMN;
	vector<Quaterniond> qMN;
	pMN.clear();
	qMN.clear();
	qMN = PositionLinearInterpolationTri(pa,pb,pc,pd,pMN,n,&qa,&qd);
	vector<Matrix4d> triM;
	triM.clear();
	Matrix4d m;
    for(int i = 0; i < (int)pMN.size();i++)
    {
    	m = quaternion_2_pose(qMN[i]);
	    m(0,3) = pMN[i][0];
	    m(1,3) = pMN[i][1];
	    m(2,3) = pMN[i][2];
	    triM.push_back(m);
    }
    list<MJCoint> mjclist;
    mjclist.clear();
    for(int i = 0; i < (int)triM.size() - 1; i++)
    {
    	list<MJCoint> list1;
    	list1.clear();
    	list1 = PositionAttitudeLinearInterpolation(triM[i],triM[i + 1]);
    	for(list<MJCoint>::iterator it = list1.begin(); it != list1.end();it++)
    		mjclist.push_back(*it);
    }
    return mjclist;
}

list<MJCoint> SwingWelding::PositionAttitude3DInterpolationTri(const Matrix4d& A, const Matrix4d& B,const Matrix4d& C, const Matrix4d& D,const Matrix4d& E, const Matrix4d&F ,int n)
{
	Vector3d pa = Vector3d(A(0,3),A(1,3),A(2,3));
	Vector3d pb = Vector3d(B(0,3),B(1,3),B(2,3));
	Vector3d pc = Vector3d(C(0,3),C(1,3),C(2,3));
	Vector3d pd = Vector3d(D(0,3),D(1,3),D(2,3));
	Vector3d pe = Vector3d(E(0,3),E(1,3),E(2,3));
	Vector3d pf = Vector3d(F(0,3),F(1,3),F(2,3));
	Quaterniond qa = pose_2_quaternion(A);
	Quaterniond qc = pose_2_quaternion(C);
	vector<Vector3d> pMN;
	vector<Quaterniond> qMN;
	pMN.clear();
	qMN.clear();
	qMN = Position3DInterpolationTri(pa,pb,pc,pd,pe,pf,pMN,n,&qa,&qc);
	vector<Matrix4d> triM;
	triM.clear();
	Matrix4d m;
	for(int i = 0; i < (int)pMN.size();i++)
	{
		m = quaternion_2_pose(qMN[i]);
		m(0,3) = pMN[i][0];
		m(1,3) = pMN[i][1];
		m(2,3) = pMN[i][2];
		triM.push_back(m);
	}
	list<MJCoint> mjclist;
	mjclist.clear();
	for(int i = 0; i < (int)triM.size() - 1; i++)
	{
		list<MJCoint> list1;
		list1.clear();
		list1 = PositionAttitudeLinearInterpolation(triM[i],triM[i + 1]);
		for(list<MJCoint>::iterator it = list1.begin(); it != list1.end();it++)
			mjclist.push_back(*it);
	}
	return mjclist;
}
