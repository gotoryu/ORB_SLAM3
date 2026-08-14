#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <thread>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <System.h>
#include <Viewer.h>

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);
// ---------------------------------------------------------
// Background thread for processing images
// ---------------------------------------------------------
void TrackingThread(ORB_SLAM3::System *pSLAM, const int num_seq, const vector<int> &nImages, const vector<vector<string>> &vstrImageLeft, const vector<vector<string>> &vstrImageRight, const vector<vector<double>> &vTimestampsCam)
{
    cv::Mat imLeft, imRight;

    for (int seq = 0; seq < num_seq; seq++)
    {
        for (int ni = 0; ni < nImages[seq]; ni++)
        {
            imLeft = cv::imread(vstrImageLeft[seq][ni], cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[seq][ni], cv::IMREAD_UNCHANGED);
            double tframe = vTimestampsCam[seq][ni];

            if (imLeft.empty() || imRight.empty())
            {
                cerr << endl
                     << "Failed to load image at index " << ni << endl;
                continue;
            }

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            pSLAM->TrackStereo(imLeft, imRight, tframe, vector<ORB_SLAM3::IMU::Point>(), vstrImageLeft[seq][ni]);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

            double T = 0;
            if (ni < nImages[seq] - 1)
                T = vTimestampsCam[seq][ni + 1] - tframe;
            else if (ni > 0)
                T = tframe - vTimestampsCam[seq][ni - 1];

            if (ttrack < T)
            {
                usleep((T - ttrack) * 1e6);
            }
        }

        if (seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;
            pSLAM->ChangeDataset();
        }
    }

    pSLAM->Shutdown();
}

// ---------------------------------------------------------
// Main function running on the Main Thread (macOS safe)
// ---------------------------------------------------------
int main(int argc, char **argv)
{
    if (argc < 4)
    {
        cerr << endl
             << "Usage: ./stereo_quest3 path_to_vocabulary path_to_settings path_to_sequence_folder_1 (path_to_sequence_folder_2 ... path_to_sequence_folder_N)" << endl;
        return 1;
    }

    // Number of sequence folders passed via CLI arguments
    const int num_seq = argc - 3;
    cout << "Number of sequences to process: " << num_seq << endl;

    vector<vector<string>> vstrImageLeft(num_seq);
    vector<vector<string>> vstrImageRight(num_seq);
    vector<vector<double>> vTimestampsCam(num_seq);
    vector<int> nImages(num_seq);

    for (int seq = 0; seq < num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[seq + 3]);

        string pathCam0 = pathSeq + "/cam0";
        string pathCam1 = pathSeq + "/cam1";

        LoadImages(pathCam0, pathCam1, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageLeft[seq].size();
    }

    // Initialize SLAM with TRUE so Viewer objects are created in memory
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);

    // Start background tracking thread
    std::thread trackingWorker(TrackingThread, &SLAM, num_seq, nImages, vstrImageLeft, vstrImageRight, vTimestampsCam);

    // Run Pangolin Viewer strictly on the MAIN thread
    if (SLAM.mpViewer)
    {
        SLAM.mpViewer->Run();
    }

    // Wait for tracking to finish before saving
    trackingWorker.join();

    // Save final combined trajectories
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    vector<cv::String> imagePaths;
    cv::glob(strPathLeft + "/*.png", imagePaths, false);
    
    // Sort to ensure chronological order
    std::sort(imagePaths.begin(), imagePaths.end());

    for (size_t i = 0; i < imagePaths.size(); i++)
    {
        string leftPath = imagePaths[i];
        
        // Extract filename without path and extension
        size_t lastSlash = leftPath.find_last_of("/\\");
        size_t lastDot = leftPath.find_last_of(".");
        string filename = leftPath.substr(lastSlash + 1, lastDot - lastSlash - 1);
        
        string rightPath = strPathRight + "/" + filename + ".png";
        
        vstrImageLeft.push_back(leftPath);
        vstrImageRight.push_back(rightPath);
        
        // Convert filename (nanoseconds) to seconds
        double t = std::stod(filename) / 1e9;
        vTimeStamps.push_back(t);
    }
}