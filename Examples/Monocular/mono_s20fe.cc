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

using namespace std;

void LoadCustomImages(const string &strImagePath, vector<string> &vstrImages, vector<double> &vTimestamps, int fps);

int main(int argc, char **argv)
{
     if (argc < 5)
     {
          cerr << endl
               << "Usage: ./mono_s20fe path_to_vocabulary path_to_settings fps path_to_image_folder_1 (path_to_image_folder_2 ... path_to_image_folder_n)" << endl;
          return 1;
     }

     const int num_seq = argc - 4;
     const int fps = atoi(argv[3]);
     cout << "fps = " << fps << endl;
     cout << "num_seq = " << num_seq << endl;
     vector<vector<string>> vstrImages;
     vector<vector<double>> vTimestamps;
     vector<int> nImages;

     vstrImages.resize(num_seq);
     vTimestamps.resize(num_seq);
     nImages.resize(num_seq);

     int tot_images = 0;
     int seq;
     for (seq = 0; seq < num_seq; seq++)
     {
          cout << "Loading images for sequence " << seq << "..." << endl;

          string pathSeq(argv[seq + 4]);
          string pathCam0 = pathSeq + "/cam0";

          LoadCustomImages(pathCam0, vstrImages[seq], vTimestamps[seq], fps);
          cout << "LOADED!" << endl;

          nImages[seq] = vstrImages[seq].size();
          tot_images += nImages[seq];
     }

     ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
     float imageScale = SLAM.GetImageScale();

     vector<float> vTimesTrack;
     vTimesTrack.resize(tot_images);
     int global_image_index = 0;

     cv::Mat im;
     for (seq = 0; seq < num_seq; seq++)
     {
          cout << endl
               << "-------" << endl;
          cout << "Start processing sequence " << seq << " ..." << endl;
          cout << "Images in the sequence: " << nImages[seq] << endl
               << endl;

          for (int ni = 0; ni < nImages[seq]; ni++)
          {
               im = cv::imread(vstrImages[seq][ni], cv::IMREAD_UNCHANGED);
               double tframe = vTimestamps[seq][ni];

               if (im.empty())
               {
                    cerr << endl
                         << "Failed to load image at: "
                         << string(vstrImages[seq][ni]) << endl;
                    return 1;
               }

               if (imageScale != 1.f)
               {
                    int width = im.cols * imageScale;
                    int height = im.rows * imageScale;
                    cv::resize(im, im, cv::Size(width, height));
               }

               std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

               SLAM.TrackMonocular(im, tframe);

               std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

               double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
               vTimesTrack[global_image_index++] = ttrack;

               double T = 0;
               if (ni < nImages[seq] - 1)
               {
                    T = vTimestamps[seq][ni + 1] - tframe;
               }
               else if (ni > 0)
               {
                    T = tframe - vTimestamps[seq][ni - 1];
               }

               if (ttrack < T)
               {
                    usleep((T - ttrack) * 1e6);
               }
          }
     }

     std::cout << "\n----------------------------------" << std::endl;
     std::cout << "Video processing finished. The map is now stabilizing." << std::endl;
     std::cout << "You can safely explore the map in the viewer." << std::endl;
     std::cout << "Press ENTER in this terminal to save the map and exit..." << std::endl;
     std::cout << "----------------------------------\n"
               << std::endl;

     std::cin.get();

     SLAM.Shutdown();

     SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_S20FE.txt");

     sort(vTimesTrack.begin(), vTimesTrack.end());
     float totaltime = 0;
     for (int ni = 0; ni < tot_images; ni++)
     {
          totaltime += vTimesTrack[ni];
     }

     cout << "\n-------" << endl
          << endl;
     cout << "median tracking time: " << vTimesTrack[tot_images / 2] << endl;
     cout << "mean tracking time: " << totaltime / tot_images << endl;

     std::cout << "Exiting application..." << std::endl;

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