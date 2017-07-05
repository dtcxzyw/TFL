#include "common.h"

bool listDirs(const char* dirPath, std::vector<std::string>& dirs) {
#ifdef WIN32
    std::string path(FileSystem::getResourcePath());
    if (dirPath && strlen(dirPath) > 0) {
        path.append(dirPath);
    }
    path.append("/*");
    // Convert char to wchar
    std::basic_string<TCHAR> wPath;
    wPath.assign(path.begin(), path.end());

    WIN32_FIND_DATA FindFileData;
    HANDLE hFind = FindFirstFile(wPath.c_str(), &FindFileData);
    if (hFind == INVALID_HANDLE_VALUE) {
        return false;
    }
    do {
        // Add to the list if this is a directory
        if (FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY &&
            FindFileData.cFileName[0] != L'.') {
            // Convert wchar to char
            std::basic_string<TCHAR> wfilename(FindFileData.cFileName);
            std::string filename;
            filename.assign(wfilename.begin(), wfilename.end());
            dirs.push_back(filename);
        }
    }
    while (FindNextFile(hFind, &FindFileData) != 0);

    FindClose(hFind);
    return true;
#else
    std::string path(FileSystem::getResourcePath());
    if (dirPath && strlen(dirPath) > 0) {
        path.append(dirPath);
    }
    path.append("/.");
    bool result = false;

    struct dirent* dp;
    DIR* dir = opendir(path.c_str());
    if (dir != NULL) {
        while ((dp = readdir(dir)) != NULL) {
            if (dp->d_name[0] == '.')continue;

            std::string filepath(path);
            filepath.append("/");
            filepath.append(dp->d_name);

            struct stat buf;
            if (!stat(filepath.c_str(), &buf)) {
                // Add to the list if this is not a directory
                if (S_ISDIR(buf.st_mode)) {
                    dirs.push_back(dp->d_name);
                }
            }
        }
        closedir(dir);
        result = true;
    }

    return result;
#endif
}

void removeAll(const std::string & path) {
    {
        std::vector<std::string> files;
        FileSystem::listFiles(path.c_str(), files);
        for (auto && file : files)
            remove((path + '/' + file).c_str());
    }
    {
        std::vector<std::string> dirs;
        listDirs(path.c_str(), dirs);
        for (auto&& dir : dirs) {
            removeAll(path+"/"+dir);
        }
    }
    rmdir(path.c_str());
}

void correctVector(Node * node, Vector3(Node::* sampler)() const, Vector3 dest
    , float X, float Y, float Z) {
    auto unit = 1.0f;
    auto dot = [&] {
        auto vec = (node->*sampler)();
        return vec.dot(dest);
    };
    auto cd = dot();
    if (cd > 0.999f)return;
    float cntX=0.0f, cntY=0.0f, cntZ=0.0f;
    for (auto unit = std::max({ X,Y,Z }); unit > 0.0005f; unit /= 2.0f) {
#define TEST(a,b) \
        while (std::abs(cnt##a+(b))<a) {\
            node->rotate##a((b));\
            auto nd = dot();\
            if (cd < nd)cd = nd,cnt##a+=(b);\
            else {\
                  node->rotate##a(-(b));\
                  break;\
            }\
        }\

        TEST(X, unit);
        TEST(X, -unit);
        TEST(Y, unit);
        TEST(Y, -unit);
        TEST(Z, unit);
        TEST(Z, -unit);
#undef TEST
    }
}

std::mt19937_64 mt(std::chrono::high_resolution_clock::now().time_since_epoch().count());

uint16_t shadowSize=1;
bool enableParticle = false;
