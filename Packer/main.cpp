#include <filesystem>
#include <iostream>
#include <fstream>
#include <set>
#include<random>
using namespace std;
using namespace std::experimental::filesystem;
ofstream out;
struct Path {
    char name[60] = {};
    uint32_t size=0;
};
const std::set<path> exts =
{ ".config", ".form",".gpb",".theme",".png",
".data",".info",".terrain",".material",".frag",
".vert",".lua",".scene",".physics"};
void find(const path& p) {
    for (auto&& child : directory_iterator(p)) {
        cout << child.path();
        if (child.status().type() == file_type::regular 
            && exts.find(child.path().extension())!=exts.cend()) {
            Path info;
            auto abs = child.path().string();
            for (auto& i : abs)
                if (i == '\\')
                    i = '/';
            strcpy(info.name,abs.c_str());
            info.size = file_size(child.path());
            out.write(reinterpret_cast<const char*>(&info), sizeof(info));
            if (info.size) {
                ifstream in(child.path(), ios::binary);
                if (in.is_open())
                    out << in.rdbuf();
                else {
                    cout << "Error: Cannot open the file." << endl;
                    throw;
                }
            }
            cout << " size:"<< info.size;
        }
        else cout << "   ignored";
        cout << endl;
        if (child.status().type() == file_type::directory) {
            find(child.path());
        }
    }
}
int main(int argc,char** argv) {
    out.open(argv[2], ios::trunc | ios::binary);
    if (!out.is_open())return -1;
    std::mt19937_64 mt(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    bool local = argv[3] == "local"s;
    out.write(reinterpret_cast<const char*>(&local), sizeof(local));
    auto key = mt();
    out.write(reinterpret_cast<const char*>(&key), sizeof(key));
    uint8_t depNum=argc - 4;
    out.write(reinterpret_cast<const char*>(&depNum), sizeof(depNum));
    char depName[32];
    for (uint8_t i = 0; i < depNum; ++i) {
        strcpy(depName, argv[i+4]);
        out.write(depName, 32);
    }
    find(argv[1]);
    out.close();
    cin.get();
    cin.get();
    return 0;
}
