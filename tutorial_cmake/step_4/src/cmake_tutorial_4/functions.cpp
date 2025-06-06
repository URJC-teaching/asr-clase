#include "cmake_tutorial_4/functions.hpp"

namespace cmake_tutorial_4
{

double duplica(double value_in)
{
  return value_in * 2.0;
}

void show_image(cv::Mat & image)
{
  cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
  cv::imshow("Display Image", image);
  cv::waitKey(0);
}

void fill_msg(std_msgs::msg::String & msg_in)
{
  msg_in.data = "Hola CMake";
}

}  // namespace cmake_tutorial_4
