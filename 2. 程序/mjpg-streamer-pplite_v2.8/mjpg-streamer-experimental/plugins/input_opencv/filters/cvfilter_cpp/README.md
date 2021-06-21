<!--
 * @Author: Ken Kaneki
 * @Date: 2021-05-31 06:10:27
 * @LastEditTime: 2021-06-03 22:37:07
 * @Description: README
 * @FilePath: \undefinedd:\Learn\毕设资料\终期答辩\2. 程序\mjpg-streamer-pplite_v2.8\mjpg-streamer-experimental\plugins\input_opencv\filters\cvfilter_cpp\README.md
-->
input_opencv filter plugin: cvfilter_cpp
========================================

This is just a demonstration plugin that shows how to create a bare minimum
filter plugin for use with the mjpg-streamer input_opencv plugin.

To create your own filter plugin, just copy filter_cpp.cpp to your own project,
and compile/link it with the same build of OpenCV that mjpg-streamer is linked
to.

CMakeLists.txt is specific to the mjpg-streamer build tree, and won't be useful
outside of it.

预测工程
=======================
* model[预测模型]
* CMakeLists.txt[C++编译文件]
* filter_cpp.cpp[预测程序]
