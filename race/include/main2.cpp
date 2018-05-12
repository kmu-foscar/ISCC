#include "Lane_Detector.hpp"

using namespace std;

int main(int argc, char* argv[]) {
    Lane_Detector* ld = new Lane_Detector();
    ld->init();
    while(true) {
        ld->operate();
    }
    delete ld;
}
