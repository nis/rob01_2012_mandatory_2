#include <iostream>
#include <fstream>
#include <sstream>
#include <rw/rw.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::kinematics;

void ass_i();
void ass_ii(vector<Transform3D<> >& t_world_desired, vector<Transform3D<> >& t_desired);

// Utility functions
void print_xyzrpy(Transform3D<>& transform);
void import_transforms_from_file(string file, vector<Transform3D<> >& result);

WorkCell::Ptr wc;
Device::Ptr device;
Frame* tool;
State start_state;

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
    
    // Save the starting state
    start_state = wc->getDefaultState();
    
    string toolFramName = TOOL_FRAME;
    cout << "Finding frame for tool: " << toolFramName << "." << endl;
    tool = wc->findFrame(toolFramName);
    if (tool == NULL) {
        cout << "Toolframe not found!" << endl;
        return 1;
    }
    
    ass_i();
    
    // Import the transforms from the data file
    vector<Transform3D<> > t_world_desired;
    import_transforms_from_file(TRANSFORM_FILE, t_world_desired);
    
    // Calculate the desired transfrom relative to the base
    vector<Transform3D<> > t_desired;
    ass_ii(t_world_desired, t_desired);
    
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

// Only position part needs modyfying.
// Add the position part of the transform from world >> base to each position part in t_worl_desired
void ass_ii(vector<Transform3D<> >& t_world_desired, vector<Transform3D<> >& t_desired) {
    Transform3D<> world_to_base = device->worldTbase(start_state);
    
    for (int i = 0; i < t_world_desired.size(); i++) {
        t_desired.push_back(t_world_desired[i]);
        t_desired[i].P() = t_desired[i].P() + world_to_base.P();
    }
}

void import_transforms_from_file(string file, vector<Transform3D<> >& result) {
    result.erase(result.begin(), result.end());
    
    ifstream infile(file.c_str());
    string line;
    Vector3D<> p(0.0, 0.0, 0.0);
    Rotation3D<> r(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    int count = 0;
    
    
    if (!infile) {
        cout << "Transforms file not found!" << endl;
    } else {
        cout << "Importing transforms." << endl;
        
        while (!infile.eof()) {
            getline( infile , line );
            if ( line != "") {
                stringstream ss (stringstream::in | stringstream::out);
                ss << line;
                ss >> r(0, 0);
                ss >> r(0, 1);
                ss >> r(0, 2);
                ss >> p(0);
                ss >> r(1, 0);
                ss >> r(1, 1);
                ss >> r(1, 2);
                ss >> p(1);
                ss >> r(2, 0);
                ss >> r(2, 1);
                ss >> r(2, 2);
                ss >> p(2);
                
                Transform3D<> t(p, r);
                result.push_back(t);
                count++;
            }
        }
    }
    
    cout << count << " transforms imported." << endl;
}

void print_xyzrpy(Transform3D<>& transform) {
    cout << "XYZRPY: \t\t\t{";
    
    cout << transform.P()[0] << ", " << transform.P()[1] << ", " << transform.P()[2];
    
    cout << "}" << endl;
}