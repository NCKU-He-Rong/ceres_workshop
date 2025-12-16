#include "txtio.hpp"
#include <vector>

TxtIO::TxtIO(const std::string & filePath)
{
    // initial the fin
    // 不用open the file
    fin = std::fstream(filePath);

}

void TxtIO::read()
{
    // check the fin is open
    if (!fin.is_open())
    {
        std::cerr << "[ERROR] Can't Open the File" << std::endl;
        return;
    }

    // print the content
    while((!fin.eof()) )
    {   
        // 得到行
        std::string line;

        std::getline(fin, line);

        // 跳過空行
        if (line.empty()) continue;

        // 去除line前後的空白
        splitStringSpace(line);

        // 針對處理過後的line 建立stringstream
        std::stringstream ss(line);
        
        std::vector<double> rowData;
        // 針對這個stringstream進行>>運算
        for (num; !ss.eof(); num++)
        {   
            // >>運算
            double temp = 0;
            ss >> temp;
            rowData.push_back(temp);
            
        }

        // 存放資料
        data.push_back(rowData);
        row += 1;
    }
    col = num / row;

}

void TxtIO::print() const
{
    for(int i=0;i<row;i++)
    {
        for(int j=0;j<col;j++)
        {
            std::cout << this->operator()(i,j) << " ";
        }
        std::cout << std::endl;
    }
}

int TxtIO::getRowInfo() const
{
    return row;
}

int TxtIO::getColInfo() const
{
    return col;
}

double TxtIO::operator()(const int &rowindex, const int & colindex) const
{
    return data[rowindex][colindex];
}


void TxtIO::splitStringSpace(std::string & str)
{
    // 去除尾巴的空白
    while(!str.empty() && std::isspace(str.back()))
    {
        str.pop_back();
    }

    // 去除開頭的空白
    while(!str.empty() && std::isspace(str[0]))
    {
        str.erase(str.begin(), str.begin()+1);
    }
}

