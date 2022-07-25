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