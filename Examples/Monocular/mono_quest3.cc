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

void LoadImages(const string &path, vector<string> &vstrImage, vector<double> &vTimeStamps, int fps = 20);

int main(int argc, char **argv)
{
     if (argc < 5)
     {
          cerr << endl
               << "Usage: ./mono_quest3 path_to_vocabulary path_to_settings fps path_to_image_folder_1 (path_to_image_folder_2 ... path_to_image_folder_n)" << endl;
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

          LoadImages(pathCam0, vstrImages[seq], vTimestamps[seq], fps);
          cout << "LOADED!" << endl;

          nImages[seq] = vstrImages[seq].size();
          tot_images += nImages[seq];
     }

     ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
     float imageScale = SLAM.GetImageScale();

     vector<float> vTimesTrack, vTimesTrackTotal, vTimesLoad, vTimesWaiting;
     vTimesTrack.resize(tot_images);
     vTimesTrackTotal.resize(tot_images);
     vTimesLoad.resize(tot_images);
     vTimesWaiting.resize(tot_images);

     cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
     cv::Mat im;
     int global_image_index = 0;

     for (seq = 0; seq < num_seq; seq++)
     {
          cout << endl
               << "-------" << endl;
          cout << "Start processing sequence " << seq << " ..." << endl;
          cout << "Images in the sequence: " << nImages[seq] << endl
               << endl;

          for (int ni = 0; ni < nImages[seq]; ni++)
          {
               std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
               im = cv::imread(vstrImages[seq][ni], cv::IMREAD_GRAYSCALE);
               std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

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

               // clahe->apply(im, im);

               std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

               SLAM.TrackMonocular(im, tframe);

               std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();

               double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count();
               vTimesTrack[global_image_index] = ttrack;

               double T = 0;
               if (ni < nImages[seq] - 1)
               {
                    T = vTimestamps[seq][ni + 1] - tframe;
               }
               else if (ni > 0)
               {
                    T = tframe - vTimestamps[seq][ni - 1];
               }

               std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
               double t_current_image = std::chrono::duration_cast<std::chrono::duration<double>>(t5 - t1).count();
               if (t_current_image < T)
               {
                    usleep((T - t_current_image) * 1e6);
               }

               std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
               vTimesTrackTotal[global_image_index] = std::chrono::duration_cast<std::chrono::duration<double>>(t6 - t1).count();
               vTimesWaiting[global_image_index] = std::chrono::duration_cast<std::chrono::duration<double>>(t6 - t5).count();
               vTimesLoad[global_image_index++] = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
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

     SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_Quest3.txt");

     sort(vTimesTrack.begin(), vTimesTrack.end());
     float total_slam_time = 0;
     float total_time = 0;
     float total_load_time = 0;
     float total_waiting_time = 0;

     for (int ni = 0; ni < tot_images; ni++)
     {
          total_slam_time += vTimesTrack[ni];
          total_time += vTimesTrackTotal[ni];
          total_load_time += vTimesLoad[ni];
          total_waiting_time += vTimesWaiting[ni];
     }

     cout << "\n-------" << endl
          << endl;
     cout << "median tracking time: " << vTimesTrack[tot_images / 2] << endl;
     cout << "mean tracking time: " << total_slam_time / tot_images << endl;
     cout << "mean total image processing time: " << total_time / tot_images << endl;
     cout << "mean loading time: " << total_load_time / tot_images << endl;
     cout << "mean waiting time: " << total_waiting_time / tot_images << endl;
     cout << "total time: " << total_time << endl;
     cout << "total SLAM time: " << total_slam_time << endl;
     cout << "total load time: " << total_load_time << endl;
     cout << "total waiting time: " << total_waiting_time << endl;

     std::cout << "Exiting application..." << std::endl;

     return 0;
}

void LoadImages(const string &path, vector<string> &vstrImage, vector<double> &vTimeStamps, int fps)
{
     vector<cv::String> imagePaths;
     cv::glob(path + "/*.png", imagePaths, false);
     int index = 1;

     // Sort to ensure chronological order
     std::sort(imagePaths.begin(), imagePaths.end());

     stringstream fst;
     fst << setfill('0') << setw(5) << index;
     string firstFile = path + "/" + fst.str() + ".png";
     ifstream ff(firstFile.c_str());

     if (ff.good())
     {
          double frame_time = 1.0 / fps;
          std::cout << "Image name format: 12345.png (framerate is fixed at " << fps << " FPS)" << std::endl;

          while (true)
          {
               stringstream ss;
               ss << setfill('0') << setw(5) << index;
               string filename = path + "/" + ss.str() + ".png";

               ifstream f(filename.c_str());
               if (!f.good())
               {
                    break;
               }

               vstrImage.push_back(filename);

               double current_time = (index - 1) * frame_time;
               vTimeStamps.push_back(current_time);

               index++;
          }
     }
     else
     {
          std::cout << "Image name format: 1234567891011121314.png (name represents timestamp)" << std::endl;

          for (size_t i = 0; i < imagePaths.size(); i++)
          {
               string leftPath = imagePaths[i];

               // Extract filename without path and extension
               size_t lastSlash = leftPath.find_last_of("/\\");
               size_t lastDot = leftPath.find_last_of(".");
               string filename = leftPath.substr(lastSlash + 1, lastDot - lastSlash - 1);

               vstrImage.push_back(leftPath);

               // Convert filename (nanoseconds) to seconds
               double t = std::stod(filename) / 1e9;
               vTimeStamps.push_back(t);
          }
     }
}