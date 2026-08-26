#ifndef STANDALONEVIEWER_H
#define STANDALONEVIEWER_H

#include "Viewer.h"
#include "Atlas.h"
#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>
#include <thread>
#include <vector>

namespace ORB_SLAM3 {
    class Atlas;
    class Viewer;

    class StandaloneViewer {
    public:
        StandaloneViewer(const string &pStrLoadAtlasFromFile, const string &strVocFile, const string &strSettingsFile);

        Map *GetCurrentMap();

        int GetCurrentMapIdx();

        int GetNumberOfMaps();

        void ChangeMap(int mapIdx);

        enum FileType{
            TEXT_FILE=0,
            BINARY_FILE=1,
        };

        void Shutdown();

    private:
        string mStrLoadAtlasFromFile;
        string mStrVocabularyFilePath;
        MapDrawer *mpMapDrawer;
        ORBVocabulary *mpVocabulary;
        KeyFrameDatabase* mpKeyFrameDatabase;
        Atlas* mpAtlas;
        Viewer* mpViewer;
        std::thread* mptViewer;
        Map* pCurrentMap = nullptr;
        int pCurrentMapIdx = -1;
        vector<Map*> mvpMaps;

        bool LoadAtlas(int type);

        string CalculateCheckSum(string filename, int type);
    };
}

#endif // STANDALONEVIEWER_H
