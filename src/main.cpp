#include <iostream>
#include <Eigen/Dense>
#include <numeric>
#include <cmath>

// computes the center of a circle from 3 points on its boundary
Eigen::Vector2d circleCenter(const Eigen::Vector2d &P1, const Eigen::Vector2d &P2, const Eigen::Vector2d &P3)
{
	Eigen::Matrix<double,3,2> mat;
	mat.block<1,2>(0,0) = (P2 - P1).transpose();
	mat.block<1,2>(1,0) = (P3 - P2).transpose();
	mat.block<1,2>(2,0) = (P1 - P3).transpose();
	
	Eigen::Vector3d vec;
	vec.x() = 0.5*(P2 - P1).squaredNorm() + (P2 - P1).dot(P1);
	vec.y() = 0.5*(P3 - P2).squaredNorm() + (P3 - P2).dot(P2);
	vec.z() = 0.5*(P1 - P3).squaredNorm() + (P1 - P3).dot(P3);
	
	return mat.colPivHouseholderQr().solve(vec);
}


int main(int argc, char** argv)
{
	double t = 0.21111111111111111111111111111111111111111111111111111111111;
	Eigen::Vector2d P1( 0.0, 0.0),
					P2( 1.0, 0.0),
					P3( 1.0, 1.0),
					P4( 0.0, 1.0);
	Eigen::Matrix2d mat;
	mat <<  1.0, 0.0,
		    0.0, 1.0;
	Eigen::Vector2d tr(0.8,0.6);
	P1 = t*mat*P1 + tr;
	P2 = t*mat*P2 + tr;
	P3 = t*mat*P3 + tr;
	P4 = t*mat*P4 + tr;
	
	Eigen::Vector2d C = circleCenter(P1,P2,P3);

	double d1 = (C - P1).norm(),
		   d2 = (C - P2).norm(),
		   d3 = (C - P3).norm(),
		   d4 = (C - P4).norm();
	
	std::cout.precision(100);
	std::cout << "d1 " << d1 << std::endl;
	std::cout << "d2 " << d2 << std::endl;
	std::cout << "d3 " << d3 << std::endl;
	std::cout << "d4 " << d4 << std::endl;
	
	std::cout << std::endl;
	std::cout << "Test d4 == d1" << std::endl;
	if(d4 == d1)
		std::cout << "Distances égales" << std::endl;
	else
		std::cout << "Distances non égales" << std::endl;
	
	std::cout << std::endl;
	std::cout << "Test std::abs(d1 - d4) <= std::min(d1,d4)*std::numeric_limits<double>::epsilon()" << std::endl;
	if( std::abs(d1 - d4) <= std::min(d1,d4)*std::numeric_limits<double>::epsilon() )
		std::cout << "Distances égales" << std::endl;
	else
		std::cout << "Distances non égales" << std::endl;
	
	std::cout << std::endl;
	std::cout << "Test std::abs(d1 - d4) <= 6.0*std::min(d1,d4)*std::numeric_limits<double>::epsilon()" << std::endl;
	if( std::abs(d1 - d4) <= 6.0*std::min(d1,d4)*std::numeric_limits<double>::epsilon() )
		std::cout << "Distances égales" << std::endl;
	else
		std::cout << "Distances non égales" << std::endl;
	
	std::cout << std::endl;
	std::cout << "Test std::abs(d1 - d4) <= 7.0*std::min(d1,d4)*std::numeric_limits<double>::epsilon()" << std::endl;
	if( std::abs(d1 - d4) <= 7.0*std::min(d1,d4)*std::numeric_limits<double>::epsilon() )
		std::cout << "Distances égales" << std::endl;
	else
		std::cout << "Distances non égales" << std::endl;
	
	std::cout << std::endl;
	std::cout << "Test (float)d4 == (float)d1" << std::endl;
	if((float)d4 == (float)d1)
		std::cout << "Distances égales" << std::endl;
	else
		std::cout << "Distances non égales" << std::endl;
	
	return 0;
}
