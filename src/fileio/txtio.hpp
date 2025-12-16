#ifndef TXTIO_HPP_
#define TXTIO_HPP_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>



class TxtIO
{
    public:
        // 建構子
        TxtIO() = default;
        TxtIO(const std::string & filePath);

        // 函數定義
        void read();
        void print() const; 
        int getRowInfo() const;
        int getColInfo() const;
        void splitStringSpace(std::string & str);

        // 運算子重載
        double operator()(const int &rowindex, const int & colindex) const;

    private:
        // define fstream
        std::fstream fin;

        // define the row and col variable
        int row = 0;
        int col = 0;
        int num = 1;

        // date
        std::vector<std::vector<double>> data;

};

#endif 