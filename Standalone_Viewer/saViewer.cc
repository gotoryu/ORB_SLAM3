/**
 * mono_s20fe.cc
 * Custom ORB-SLAM3 runner for Samsung S20 FE datasets
 */

#include <System.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <unistd.h>

#include "StandaloneViewer.h"

using namespace std;

void LoadCustomImages(const string &strImagePath, vector<string> &vstrImages, vector<double> &vTimestamps, int fps);

int main(int argc, char **argv)
{
     if (argc < 4)
     {
          cerr << endl
               << "Usage: ./SA_Viewer path_to_vocabulary path_to_settings path_to_map" << endl;
          return 1;
     }

     cout << "Starting the Standalone Map Viewer" << endl;

     ORB_SLAM3::StandaloneViewer SAViewer(argv[3], argv[1], argv[2]);

     std::cout << "\n----------------------------------" << std::endl;
     std::cout << "Map is loaded." << std::endl;
     std::cout << "You can safely explore it in the viewer." << std::endl;
     std::cout << "Press ENTER in this terminal to exit..." << std::endl;
     std::cout << "----------------------------------\n"
               << std::endl;

     std::cin.get();

     SAViewer.Shutdown();

     std::cout << "Exiting application." << std::endl;

     return 0;
}

void LoadCustomImages(const string &strImagePath, vector<string> &vstrImages, vector<double> &vTimestamps, int fps)
{
     int index = 1;
     double frame_time = 1.0 / fps;

     while (true)
     {
          stringstream ss;
          ss << setfill('0') << setw(5) << index;
          string filename = strImagePath + "/" + ss.str() + ".png";

          ifstream f(filename.c_str());
          if (!f.good())
          {
               break;
          }

          vstrImages.push_back(filename);

          double current_time = (index - 1) * frame_time;
          vTimestamps.push_back(current_time);

          index++;
     }
}