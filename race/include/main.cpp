#include "Look_Ahead.hpp"

using namespace std;

int main(int argc, char* argv[]) {
    Lane_Detector* ld = new Lane_Detector();
    Look_Ahead* la = new Look_Ahead();
    ld->init();
    while(true) {
        ld->operate();
        la->operate(ld->originImg_left, ld->originImg_right);
    }
    delete la;
    delete ld;
}