#include "StandaloneViewer.h"
#include <pangolin/pangolin.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <openssl/md5.h>
#include <thread>

namespace ORB_SLAM3 {
    StandaloneViewer::StandaloneViewer(const string &pStrLoadAtlasFromFile,
                                       const string &strVocFile,
                                       const string &strSettingsFile) : 
                                        mStrLoadAtlasFromFile(pStrLoadAtlasFromFile),
                                        mStrVocabularyFilePath(strVocFile), mptViewer(nullptr), 
                                        mpViewer(nullptr)
    {
        cout << endl
                << "=====================================================================" << endl
                << "ORB-SLAM3 Standalone Viewer." << endl
                << "INITIALIZING..." << endl
                << "=====================================================================" << endl
                << endl;

        if (mStrLoadAtlasFromFile.empty()) {
            cout << endl
                    << "ERROR: No input file was provided..." << endl;

            exit(-1);
        }

        // Load ORB Vocabulary
        cout << endl
                << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(mStrVocabularyFilePath);
        if (!bVocLoad) {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Failed to open at: " << mStrVocabularyFilePath << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl
                << endl;

        // Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        cout << "Load File" << endl;

        // Load the file with an earlier session
        cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << endl;
        bool isRead = LoadAtlas(FileType::BINARY_FILE);

        if (!isRead) {
            cout << "Error loading the file, please try with other session file or vocabulary file" << endl;
            exit(-1);
        }

        // set active map to the one with the most keyframes
        if (!mpAtlas->GetAllMaps().empty()) {
            Map* pBestMap = nullptr;
            int maxKFs = -1;

            for (Map* pMap : mpAtlas->GetAllMaps()) {
                if ((int)pMap->GetAllKeyFrames().size() > maxKFs) {
                    maxKFs = (int)pMap->GetAllKeyFrames().size();
                    pBestMap = pMap;
                }
            }

            if (pBestMap) {
                cout << "Setting map with ID " << pBestMap->GetId() << " as active map." << endl;
                mpAtlas->ChangeMap(pBestMap);
            } else {
                cout << "No maps found in the atlas." << endl;
            }
        }
        
        mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, nullptr);

        mpViewer = new Viewer(mpMapDrawer, strSettingsFile);
        mptViewer = new std::thread(&Viewer::Run, mpViewer);

        // Fix verbosity
        Verbose::SetTh(Verbose::VERBOSITY_QUIET);
    }

    bool StandaloneViewer::LoadAtlas(int type) {
        string strFileVoc, strVocChecksum;
        bool isRead = false;

        string pathLoadFileName = "./";
        pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);

        if (pathLoadFileName.find(".osa") == string::npos) {
            pathLoadFileName += ".osa";
        }

        if (type == BINARY_FILE) // File binary
        {
            cout << "Starting to read the save binary file" << endl;
            std::ifstream ifs(pathLoadFileName, std::ios::binary);
            if (!ifs.good()) {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::binary_iarchive ia(ifs);
            ia >> strFileVoc;
            ia >> strVocChecksum;
            ia >> mpAtlas;
            cout << "End to load the save binary file" << endl;
            isRead = true;
        }
        else // File text
        {
            cout << "Please select a BINARY file to load" << endl;
        } 

        if (isRead) {
            // Check if the vocabulary is the same
            string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);

            if (strInputVocabularyChecksum.compare(strVocChecksum) != 0) {
                cout << "The vocabulary load isn't the same which the load session was created " << endl;
                cout << "-Vocabulary name: " << strFileVoc << endl;
                return false; // Both are differents
            }

            mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
            mpAtlas->SetORBVocabulary(mpVocabulary);
            mpAtlas->PostLoad();

            return true;
        }
        return false;
    }

    void StandaloneViewer::Shutdown()
    {
        cout << "Waiting for Pangolin-Viewer to finish ..." << endl;

        if (mpViewer)
        {
            mpViewer->RequestFinish();
            while (!mpViewer->isFinished())
                usleep(5000);
        }

        if (mptViewer && mptViewer->joinable()) {
            mptViewer->join();
        }
    }

    string StandaloneViewer::CalculateCheckSum(string filename, int type) {
        string checksum = "";

        unsigned char c[MD5_DIGEST_LENGTH];

        std::ios_base::openmode flags = std::ios::in;
        if (type == BINARY_FILE) // Binary file
            flags = std::ios::in | std::ios::binary;

        ifstream f(filename.c_str(), flags);
        if (!f.is_open()) {
            cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << endl;
            return checksum;
        }

        MD5_CTX md5Context;
        char buffer[1024];

        MD5_Init(&md5Context);
        while (int count = f.readsome(buffer, sizeof(buffer))) {
            MD5_Update(&md5Context, buffer, count);
        }

        f.close();

        MD5_Final(c, &md5Context);

        for (int i = 0; i < MD5_DIGEST_LENGTH; i++) {
            char aux[10];
            sprintf(aux, "%02x", c[i]);
            checksum = checksum + aux;
        }

        return checksum;
    }
}
