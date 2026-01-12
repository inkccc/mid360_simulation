/**
 * @file csv_reader.hpp
 * @brief CSV 文件读取工具类
 *
 * 用于读取 Mid-360 激光雷达扫描模式配置文件
 * CSV 文件格式: Time/s, Azimuth/deg, Zenith/deg
 */

#ifndef MID360_SIMULATION_CSV_READER_HPP
#define MID360_SIMULATION_CSV_READER_HPP

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

namespace mid360_simulation
{

/**
 * @class CsvReader
 * @brief CSV 文件读取器
 *
 * 静态工具类，用于解析扫描模式 CSV 文件
 */
class CsvReader
{
public:
    /**
     * @brief 读取 CSV 文件并解析数据
     *
     * @param file_name CSV 文件路径
     * @param datas 输出数据，每行为一个 double 向量 [time, azimuth, zenith]
     * @return true 读取成功
     * @return false 读取失败
     */
    static bool ReadCsvFile(const std::string& file_name,
                            std::vector<std::vector<double>>& datas)
    {
        std::fstream file_stream;
        file_stream.open(file_name, std::ios::in);

        if (!file_stream.is_open())
        {
            std::cerr << "[CsvReader] 无法打开文件: " << file_name << std::endl;
            return false;
        }

        // 跳过表头行
        std::string header;
        std::getline(file_stream, header, '\n');

        // 逐行读取数据
        while (!file_stream.eof())
        {
            std::string line_str;
            std::getline(file_stream, line_str, '\n');

            if (line_str.empty()) continue;

            std::stringstream line_stream;
            line_stream << line_str;
            std::vector<double> data;

            try
            {
                while (!line_stream.eof())
                {
                    std::string value;
                    std::getline(line_stream, value, ',');
                    if (!value.empty())
                    {
                        data.push_back(std::stod(value));
                    }
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << "[CsvReader] 解析错误: " << line_str << std::endl;
                continue;
            }

            if (!data.empty())
            {
                datas.push_back(data);
            }
        }

        std::cout << "[CsvReader] 成功读取 " << datas.size() << " 条扫描数据" << std::endl;
        return true;
    }
};

} // namespace mid360_simulation

#endif // MID360_SIMULATION_CSV_READER_HPP
