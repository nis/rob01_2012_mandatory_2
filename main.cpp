#include <iostream>
#include <rw/rw.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::models;

int main(int argc, char** argv) {
    cout << "Program startet." << endl;
    
    cout << "Loading scene." << endl;
    string wcFile = "/Users/tamen/Documents/Archive/Skole/SDU/7Semester/ROB/Exercises/Mandatory2/KUKA_KR120_scene/KukaKr120_CDH_downMill.wc.xml";
    WorkCell::Ptr wc = WorkCellLoader::load(wcFile);
    
    string deviceName = "KukaKr120";
    cout << "Finding device: " << deviceName << "." << endl;
    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
		cout << "Device: " << deviceName << " not found!" << endl;
        return 1;
	}
    
	cout << "Program done." << endl;
	return 0;
}

