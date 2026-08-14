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

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
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
    if (argc < 5)
    {
        cerr << endl
             << "Usage: ./stereo_euroc_pangolin path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1" << endl;
        return 1;
    }

    const int num_seq = (argc - 3) / 2;
    cout << "num_seq = " << num_seq << endl;

    bool bFileName = (((argc - 3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc - 1]);
    }

    vector<vector<string>> vstrImageLeft(num_seq);
    vector<vector<string>> vstrImageRight(num_seq);
    vector<vector<double>> vTimestampsCam(num_seq);
    vector<int> nImages(num_seq);

    for (int seq = 0; seq < num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2 * seq) + 3]);
        string pathTimeStamps(argv[(2 * seq) + 4]);
        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathCam1 = pathSeq + "/mav0/cam1/data";

        LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
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

    if (bFileName)
    {
        SLAM.SaveTrajectoryEuRoC("f_" + file_name + ".txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("kf_" + file_name + ".txt");
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes(strPathTimes.c_str());
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t / 1e9);
        }
    }
}