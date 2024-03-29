/**
 *	\file src/helpers.cpp
 *	\author Mike Timmerman
 *	\version 1.0
 *	\date 2022
 */

#include "../header.h"    // #include header



MatrixXf loadFromFile(std::string FileName, int row, int col)
{
    MatrixXf Matrix(row, col);
	std::ifstream File(FileName);
	for (int k = 0; k < row; ++k) {
		std::string line;
		std::getline(File, line, '\n');
		for (int j = 0; j < col; ++j) {
			std::string entry = line.substr(0, line.find(','));
			line.erase(0, line.find(',') + 1);
			Matrix(k, j) = std::stod(entry);
		}
	}
    return Matrix;
}


void saveToFile(MatrixXf &data, int rows, int cols, std::string FileName)
{
    std::ofstream File; File.open(FileName);
    for (int k = 0; k < rows; ++k) {
        std::string line = "";
        for (int j = 0; j < cols; ++j) {
            line.append(std::to_string(data(k, j)));
            line.append(",");
        }
        line.pop_back();
        File << line << "\n";
    }
    File.close();
}


VectorXf toQuaternion( VectorXf& EAngles )
{   
    VectorXf temp(4);

    float roll = EAngles(0); float pitch = EAngles(1); float yaw = EAngles(2);

    float cy = cos( yaw * 0.5 );
    float sy = sin( yaw * 0.5);
    float cp = cos( pitch * 0.5 );
    float sp = sin( pitch * 0.5 );
    float cr = cos( roll * 0.5 );
    float sr = sin( roll * 0.5 );

    temp(0) = cr * cp * cy + sr * sp * sy;
    temp(1) = sr * cp * cy - cr * sp * sy;
    temp(2) = cr * sp * cy + sr * cp * sy;
    temp(3) = cr * cp * sy - sr * sp * cy;

    return temp;
}


VectorXf BFRtoNED( VectorXf& EulerAngles, VectorXf& Vector )
{
    float phi = EulerAngles(0); float theta = EulerAngles(1); float psi = EulerAngles(2);

    MatrixXf Mnb(3,3);
    Mnb <<  cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi),
            sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi),-cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi),
            -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi);
    
    return Mnb*Vector;
}