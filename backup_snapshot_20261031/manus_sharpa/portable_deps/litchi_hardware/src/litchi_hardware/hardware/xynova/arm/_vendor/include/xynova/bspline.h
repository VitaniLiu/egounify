#pragma once
#include<vector>
#include<iostream>
#include<fstream>
#include <cmath>
#include<string>
#include<regex>
#include<fstream>
#include<sstream>
#include "base.h"

namespace xynova {

#if defined(XYNOVASDK_SUPPRESS_DLL_WARNING)
#pragma warning(push)
#pragma warning(disable : 4251)
#endif

/**
 * @brief B样条曲线
 */
 class XYNOVA_API Bspline {

  public:
   Bspline() {}
   void bspline_set(int p); //设置参数

   void Control_point_set(std::vector<std::vector<double> > &curpoint);   //设置n+1个控制点

   void knot_vector_cal(double max_value = 1.0); //计算节点向量

   void knot_vector_show(); //在控制台输出knot_vector

   void Bspline_N_func(int i, double ut);  //计算基函数

   int find_span_func(double ut); //返回ut所作的节点区间编号

   void Bspline_generate_func(); //B样条整体曲线生成

   void Bspline_show();  //显示B样条的曲线点

   /**
    * @brief 插值调用函数
    * @param file_text 需要插值的点位
    * @param speed_rate 插值后点位速度控制因子，速度越快，速度因子越大，默认值为1
    * @return time_sum 返回值：运行所有点位所需时间
   */
   double BsplineFunctionCall(std::vector<std::vector<double> > &file_text, double speed_rate = 1);

   std::vector<std::vector<double> > pTrack;           //轨迹点

  private:
   int p;                          //阶数
   int n;                          //控制点数-1
   int m;                          //节点个数-1 ==n+p+1;
   std::vector<double> u;   //自变量
   double delta_u = 0.001;         //自变量间隔
   std::vector<std::vector<double> > ControlPoint;     //控制点
   std::vector<double> knot_vector;      //节点向量
   std::vector<double> N_cofi;          //基函数系数
   std::vector<std::vector<double> > delta_pTrack;
   std::vector<std::vector<double> > dpTrack;           //一阶微分
   std::vector<std::vector<double> > ddpTrack;         //二阶微分
 };

 /**
  * @brief 读取文件file_name，返回文件内容file_text
  */
 XYNOVA_API bool fileReadFunc(const std::string &file_name, std::vector<std::vector<double> > &file_text);

#if defined(XYNOVASDK_SUPPRESS_DLL_WARNING)
#pragma warning(pop)
#endif // if defined(XYNOVASDK_SUPPRESS_DLL_WARNING)

}  // namespace xynova
