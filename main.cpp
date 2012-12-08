#include <iostream>
#include <rw/rw.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::models;

void ass_i();

WorkCell::Ptr wc;
Device::Ptr device;

int main(int argc, char** argv) {
    cout << "Program startet." << endl;
    
    cout << "Loading scene." << endl;
    string wcFile = "/Users/tamen/Documents/Archive/Skole/SDU/7Semester/ROB/Exercises/Mandatory2/KUKA_KR120_scene/KukaKr120_CDH_downMill.wc.xml";
    wc = WorkCellLoader::load(wcFile);
    
    string deviceName = "KukaKr120";
    cout << "Finding device: " << deviceName << "." << endl;
    device = wc->findDevice(deviceName);
    if (device == NULL) {
		cout << "Device: " << deviceName << " not found!" << endl;
        return 1;
	}
    
    ass_i();
    
	cout << "Program done." << endl;
	return 0;
}

void ass_i() {
    cout << "Running assignment i." << endl;
    // Set Q's
    Q qa(6, 0.0, -(Pi/2.0), Pi/2.0, 0.0, -0.1745, 0.0);
    Q qb(6, 0.0, -(Pi/4.0), Pi/8.0, 0.0, 1.0, 0.0);
    Q qc(6, 0.65, -1.0, 1.0, 0.1, -0.1745, 0.1);
    cout << "Finished running assignment i." << endl;
}