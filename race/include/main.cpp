#include "Look_Ahead.hpp"

using namespace std;

int main(int argc, char* argv[]) {
    Look_Ahead* la = new Look_Ahead();
    la->init();
    while(true) {
        la->operate();
    }
    delete la;
}