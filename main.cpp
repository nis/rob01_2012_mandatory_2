#include <iostream>
#include <rw/rw.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::kinematics;

void ass_i();

// Utility functions
void print_xyzrpy(Transform3D<>& transform);

WorkCell::Ptr wc;
Device::Ptr device;
Frame* tool;

// Defines:
#define SCENE_FILE "/Users/tamen/Documents/Archive/Skole/SDU/7Semester/ROB/Exercises/Mandatory2/KUKA_KR120_scene/Rob01MillingSceneKR120.wc.xml"
#define DEVICE_NAME "KukaKr120"
#define TOOL_FRAME "ENDMILL4"
#define TRANSFORM_FILE "/Users/tamen/Documents/Archive/Skole/SDU/7Semester/ROB/Exercises/Mandatory2/transforms_v4.dat"

int main(int argc, char** argv) {
    cout << "Program startet." << endl;
    
    cout << "Loading scene." << endl;
    string wcFile = SCENE_FILE;
    wc = WorkCellLoader::load(wcFile);
    
    string deviceName = DEVICE_NAME;
    cout << "Finding device: " << deviceName << "." << endl;
    device = wc->findDevice(deviceName);
    if (device == NULL) {
		cout << "Device: " << deviceName << " not found!" << endl;
        return 1;
	}
    
    string toolFramName = TOOL_FRAME;
    cout << "Finding frame for tool: " << toolFramName << "." << endl;
    tool = wc->findFrame(toolFramName);
    if (tool == NULL) {
        cout << "Toolframe not found!" << endl;
        return 1;
    }
    
    ass_i();
    
	cout << "Program done." << endl;
	return 0;
}

void ass_i() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment i." << endl << endl;
    // Set Q's
    Q qa(6, 0.0, -Pi/2.0, Pi/2.0, 0.0, -0.1745, 0.0);
    Q qb(6, 0.0, -(Pi/4.0), Pi/8.0, 0.0, 1.0, 0.0);
    Q qc(6, 0.65, -1.0, 1.0, 0.1, -0.1745, 0.1);
    
    State state = wc->getDefaultState();
    Transform3D<> current_transform;
    
    cout << "Setting Q to qa: \t" << qa << endl;
    device->setQ(qa, state);
    current_transform = device->baseTframe(tool, state);
    print_xyzrpy(current_transform);
    cout << endl;
    
    cout << "Setting Q to qb: \t" << qb << endl;
    device->setQ(qb, state);
    current_transform = device->baseTframe(tool, state);
    print_xyzrpy(current_transform);
    cout << endl;
    
    cout << "Setting Q to qc: \t" << qc << endl;
    device->setQ(qc, state);
    current_transform = device->baseTframe(tool, state);
    print_xyzrpy(current_transform);
    cout << endl;
    
    cout << "Finished running assignment i." << endl;
    cout << "------------------------------------------------------------------------" << endl;
}

void print_xyzrpy(Transform3D<>& transform) {
    cout << "XYZRPY: \t\t\t{";
    
    cout << transform.P()[0] << ", " << transform.P()[1] << ", " << transform.P()[2];
    
    cout << "}" << endl;
}