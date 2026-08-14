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

void LoadImages(const string &strPath, vector<string> &vstrImage, vector<double> &vTimeStamps);
// ---------------------------------------------------------
// Background thread for processing images
// ---------------------------------------------------------
void TrackingThread(ORB_SLAM3::System *pSLAM, const int num_seq, const vector<int> &nImages, const vector<vector<string>> &vstrImage, const vector<vector<double>> &vTimestampsCam)
{
    cv::Mat im;

    for (int seq = 0; seq < num_seq; seq++)
    {
        for (int ni = 0; ni < nImages[seq]; ni++)
        {
            im = cv::imread(vstrImage[seq][ni], cv::IMREAD_UNCHANGED);
            double tframe = vTimestampsCam[seq][ni];

            if (im.empty())
            {
                cerr << endl
                     << "Failed to load image at index " << ni << endl;
                continue;
            }

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            pSLAM->TrackMonocular(im, tframe, vector<ORB_SLAM3::IMU::Point>(), vstrImage[seq][ni]);

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

    // pSLAM->Shutdown();
}

// ---------------------------------------------------------
// Main function running on the Main Thread (macOS safe)
// ---------------------------------------------------------
int main(int argc, char **argv)
{
    if (argc < 4)
    {
        cerr << endl
             << "Usage: ./stereo_s20fe path_to_vocabulary path_to_settings path_to_sequence_folder_1 (path_to_sequence_folder_2 ... path_to_sequence_folder_N)" << endl;
        return 1;
    }

    // Number of sequence folders passed via CLI arguments
    const int num_seq = argc - 3;
    cout << "Number of sequences to process: " << num_seq << endl;

    vector<vector<string>> vstrImage(num_seq);
    vector<vector<double>> vTimestampsCam(num_seq);
    vector<int> nImages(num_seq);

    for (int seq = 0; seq < num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[seq + 3]);

        LoadImages(pathSeq, vstrImage[seq], vTimestampsCam[seq]);

        if (vstrImage[seq].empty())
        {
            cerr << endl << "CRITICAL ERROR: No images found in " << pathSeq << endl;
            cerr << "Make sure the files are strictly named 0001.png, 0002.png, etc." << endl;
            return 1;
        }

        cout << "LOADED!" << endl;

        nImages[seq] = vstrImage[seq].size();
    }

    // Initialize SLAM with TRUE so Viewer objects are created in memory
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    // Start background tracking thread
    std::thread trackingWorker(TrackingThread, &SLAM, num_seq, nImages, vstrImage, vTimestampsCam);

    // Run Pangolin Viewer strictly on the MAIN thread
    if (SLAM.mpViewer)
    {
        SLAM.mpViewer->Run();
    }

    // Wait for tracking to finish before saving
    trackingWorker.join();

    SLAM.Shutdown();

    // Save final combined trajectories
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPath, vector<string> &vstrImage, vector<double> &vTimeStamps)
{
    int index = 1;
    double fps = 30.0;
    double frame_time = 1.0 / fps;

    while(true)
    {
        stringstream ss;
        ss << setfill('0') << setw(4) << index;
        string filename = strPath + "/" + ss.str() + ".png";

        ifstream f(filename.c_str());
        if(!f.good())
        {
            break; 
        }

        vstrImage.push_back(filename);
        
        double current_time = (index - 1) * frame_time;
        vTimeStamps.push_back(current_time);

        index++;
    }
}