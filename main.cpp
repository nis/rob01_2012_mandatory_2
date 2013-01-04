#include <iostream>
#include <fstream>
#include <sstream>
#include <rw/rw.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::kinematics;

// Struct to hold the individual steps
struct Step {
    double time;
    double shifted_time;
    Transform3D<> t_world_desired;
    Transform3D<> t_desired;
    Q linear_interpolated_joint_configuration;
    Q joint_configuration;
    double delta;
    Vector3D<> positional_velocity_to_next;
    Vector3D<> angular_velocity_to_next;
    Vector3D<> positional_velocity;
    Vector3D<> angular_velocity;
    Q qdot;
};

void ass_i();
void ass_ii();
void ass_iii();
void ass_iv(vector<Step>& isteps);
void ass_v(vector<Step>& isteps);
void ass_vi();
void ass_vii();
void ass_viii();
void ass_ix();
void ass_x();
void ass_xi();
void ass_xii();
void ass_xiv();
void ass_xv();
void ass_xvi();

// Utility functions
void print_xyzrpy(Transform3D<>& transform);
void import_transforms_from_file(string file);
Transform3D<> linear_interpolate(Step& from, Step& to, double t);
template <typename T> int sgn(T val);
Vector3D<> w_of_rot(Rotation3D<>& r);
Rotation3D<> reaa(Vector3D<>& v, double& theta);
Rotation3D<> reaa(Vector3D<>& x);
void calculate_delta_u(Transform3D<>& tq, Transform3D<>& td, VelocityScrew6D<double>& deltau);
Q cubic_spline(double t, double ts, double tf, Q qs, Q qf, Q vs, Q vf);
double velocity_scaling(Q& vmax, Step& previous, Step& current);

// Global data
WorkCell::Ptr wc;
Device::Ptr device;
Frame* tool;
State start_state;
vector<Step> steps;
vector<Step> finished_steps;
vector<Step> fast_steps;
vector<Step> final_steps;

// Defines:
#define SCENE_FILE "/Users/tamen/Documents/Archive/Skole/SDU/7Semester/ROB/Exercises/Mandatory2/KUKA_KR120_scene/Rob01MillingSceneKR120.wc.xml"
#define DEVICE_NAME "KukaKr120"
#define TOOL_FRAME "ENDMILL4"
#define TRANSFORM_FILE "/Users/tamen/Documents/Archive/Skole/SDU/7Semester/ROB/Exercises/Mandatory2/transforms_v4.dat"
#define V_MILLING 0.05
#define LINIEAR_INTERPOLATION_STEPS 26
#define WORKSPACE_SIZE 2.8
#define FINISHED_STEPS 20000
#define OUTPUT_FILE "/Users/tamen/Desktop/joint_configurations.dat"
#define OUTPUT_FILE_TWO "/Users/tamen/Desktop/joint_configurations_2.dat"
#define APPROACH_AND_EXIT_STEP_NUMBER 100
#define APPROACH_AND_EXIT_TIME 5

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
    import_transforms_from_file(TRANSFORM_FILE);
    
    // Calculate the desired transfrom relative to the base
    ass_ii();
    
    // Calculate the time for each step
    ass_iii();
    
    // Compute linear interpolation
    vector<Step> linear_interpolated_steps; // For saving the interpolated steps
    ass_iv(linear_interpolated_steps);
    
    // Use algoritm 3 to get the joint configurations
    ass_v(linear_interpolated_steps);
    
    // Computer Positional velocities
    ass_vi();
    
    // Computer Angular velocities
    ass_vii();
    
    // Correct Positional velocities
    ass_viii();
    
    // Correct Angular velocities
    ass_ix();
    
    // Convert passage velocities to joint velocities
    ass_x();
    
    // Show cubic splines
    ass_xi();
    
    // 20000 steps
    ass_xii();
    
    // Fasttrack the Q's
    ass_xiv();
    
    // Add approach and exit
    ass_xv();
    
    // Export the big final list
    ass_xvi();
    
	cout << "Program done." << endl;
	return 0;
}

void ass_i() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment I." << endl << endl;
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
    
    cout << "Finished running assignment I." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}

// Only position part needs modyfying.
// Add the position part of the transform from world >> base to each position part in t_worl_desired
void ass_ii() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment II." << endl << endl;
    Transform3D<> world_to_base = device->worldTbase(start_state);
    
    int count = 0;
    
    for (int i = 1; i < steps.size(); i++) {
        steps[i].t_desired = steps[i].t_world_desired;
        steps[i].t_desired.P() = steps[i].t_desired.P() - world_to_base.P();
        count++;
    }
    cout << count << " transforms shifted to base." << endl;
    cout << "Finished running assignment II." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}


void ass_iii() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment III." << endl << endl;
    steps[1].time = 0.0;
    
    for (int i = 2; i < steps.size(); i++) {
        steps[i].time = ((steps[i].t_desired.P() - steps[i - 1].t_desired.P()).norm2() / V_MILLING) + steps[i - 1].time;
    }
    
    cout << "t2: \t" << steps[2].time << "s." << endl;
    cout << "t10: \t" << steps[10].time << "s." << endl;
    
    cout << "Finished running assignment III." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}


void ass_iv(vector<Step>& isteps) {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment IV." << endl << endl;
    
    cout << "Splitting each step up into " << LINIEAR_INTERPOLATION_STEPS << " steps." << endl;
    for (int i = 1; i < steps.size() - 1; i++) {
        double time_inc = (steps[i + 1].time - steps[i].time) / LINIEAR_INTERPOLATION_STEPS;
        Step s;
        s.t_desired = steps[i].t_desired;
        s.time = steps[i].time;
        isteps.push_back(s);
        for (int j = 1; j < LINIEAR_INTERPOLATION_STEPS; j++) {
            Transform3D<> t = linear_interpolate(steps[i], steps[i + 1], steps[i].time + ((double)j * time_inc));
            s.t_desired = t;
            s.time = steps[i].time + ((double)j * time_inc);
            isteps.push_back(s);
        }
    }
    // Add the last step
    Step s;
    s.t_desired = steps.back().t_desired;
    s.time = steps.back().time;
    isteps.push_back(s);
    
    cout << "Done splitting. " << isteps.size() << " interpolated steps created." << endl << endl;
    
    cout << "First transform:" << endl;
    Transform3D<> t = linear_interpolate(steps[1], steps[2], steps[2].time/2.0);
    cout << t << endl;
    
    cout << "Second transform:" << endl;
    t = linear_interpolate(steps[9], steps[10], (steps[9].time/3.0)+((2.0 * steps[10].time)/3.0));
    cout << t << endl;
    
    cout << "Finished running assignment IV." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}

void ass_v(vector<Step>& isteps) {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment V." << endl << endl;
    
    // Start configuration
    Q q(6, 0.158, -1.18, 1.526, 1.48, -0.522, -1.256);
    
    cout << "Calcualting the deltas." << endl;
    
    // Computing deltas
    isteps[0].linear_interpolated_joint_configuration = q;
    for (int i = 1; i < isteps.size(); i++) {
        Rotation3D<> wrot2 = inverse(isteps[i - 1].t_desired.R()) * isteps[i].t_desired.R();
        isteps[i].delta = ((isteps[i].t_desired.P() - isteps[i - 1].t_desired.P()).norm2() / WORKSPACE_SIZE) + (w_of_rot(wrot2).norm2() / Pi);
        if (isteps[i].delta > 0.01) {
            cout << "Delta too large!" << endl;
        }
    }
    
    // Computing joint configuration
    cout << "Computing joint configuration." << endl;
    State state = start_state;
    device->setQ(q, state);
    VelocityScrew6D<double> deltau;
    double epsilon = 0.0000001;
    for (int i = 1; i < isteps.size(); i++) {
        calculate_delta_u(isteps[i - 1].t_desired, isteps[i].t_desired, deltau);
        while (deltau.norm2() > epsilon) {
            Q dq(prod(LinearAlgebra::inverse(device->baseJframe(tool, state).m()), deltau.m()));
            q = q + dq;
            device->setQ(q, state);
            Transform3D<> tnew = device->baseTframe(tool, state);
            calculate_delta_u(tnew, isteps[i].t_desired, deltau);
        }
        isteps[i].linear_interpolated_joint_configuration = q;
    }
    
    // Match configurations from the linear interpolated big list to the small list
    cout << "Mapping joint configurations to the smaller list of steps." << endl;
    int small_list_index = 1;
    for (int i = 0; i < isteps.size(); i++) {
        if (isteps[i].time == steps[small_list_index].time) {
            steps[small_list_index].linear_interpolated_joint_configuration = isteps[i].linear_interpolated_joint_configuration;
            small_list_index++;
        }
    }
    
    cout << "Results:" << endl;
    cout << "Q(2) \t" << steps[2].linear_interpolated_joint_configuration << endl;
    cout << "Q(10) \t" << steps[10].linear_interpolated_joint_configuration << endl;
    
    cout << "Finished running assignment V." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}

void ass_vi() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment VI." << endl << endl;
    
    cout << "Computing positional velocities." << endl;
    
    int number_of_steps = steps.size() - 1;
    for (int i = 1; i < number_of_steps; i++) {
        steps[i].positional_velocity_to_next = (steps[i + 1].t_desired.P() - steps[i].t_desired.P()) / (steps[i + 1].time - steps[i].time);
    }
    
    cout << "Results:" << endl;
    cout << "v(1,2)  \t" << steps[1].positional_velocity_to_next << endl;
    cout << "v(9,10) \t" << steps[9].positional_velocity_to_next << endl;
    
    cout << "Finished running assignment VI." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}

void ass_vii() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment VII." << endl << endl;
    
    cout << "Computing angular velocities." << endl;
    
    int number_of_steps = steps.size() - 1;
    for (int i = 1; i < number_of_steps; i++) {
        Rotation3D<> wrot = steps[i].t_desired.R().inverse() * steps[i + 1].t_desired.R();
        Vector3D<> wrotted = w_of_rot(wrot);
        steps[i].angular_velocity_to_next = (1.0 / (steps[i + 1].time - steps[i].time)) * wrotted;
    }
    
    cout << "Results:" << endl;
    cout << "w(1,2)  \t" << steps[1].angular_velocity_to_next << endl;
    cout << "w(9,10) \t" << steps[9].angular_velocity_to_next << endl;
    
    cout << "Finished running assignment VII." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}

void ass_viii() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment VIII." << endl << endl;
    
    cout << "Correcting positional velocities." << endl;
    
    steps[1].positional_velocity = steps[1].positional_velocity_to_next;
    steps.back().positional_velocity = steps.back().positional_velocity_to_next;
    
    int number_of_steps = steps.size() - 1;
    for (int i = 2; i < number_of_steps; i++) {
        steps[i].positional_velocity = (steps[i].positional_velocity_to_next + steps[i - 1].positional_velocity_to_next) / 2.0;
    }
    
    cout << "Results:" << endl;
    cout << "v(2)  \t" << steps[2].positional_velocity << endl;
    cout << "v(10) \t" << steps[10].positional_velocity << endl;
    
    cout << "Finished running assignment VIII." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}

void ass_ix() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment IX." << endl << endl;
    
    cout << "Correcting angular velocities." << endl;
    
    steps[1].angular_velocity = steps[1].angular_velocity_to_next;
    steps.back().angular_velocity = steps.back().angular_velocity_to_next;
    
    int number_of_steps = steps.size() - 1;
    for (int i = 2; i < number_of_steps; i++) {
        steps[i].angular_velocity = (steps[i].angular_velocity_to_next + steps[i - 1].angular_velocity_to_next) / 2.0;
    }
    
    cout << "Results:" << endl;
    cout << "w(2)  \t" << steps[2].angular_velocity << endl;
    cout << "w(10) \t" << steps[10].angular_velocity << endl;
    
    cout << "Finished running assignment IX." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}

void ass_x() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment X." << endl << endl;
    
    cout << "Converting passage velocities to joint velocities." << endl;
    
    State state = start_state;
    for (int i = 1; i < steps.size(); i++) {
        VelocityScrew6D<> u(steps[i].positional_velocity[0], steps[i].positional_velocity[1], steps[i].positional_velocity[2], steps[i].angular_velocity[0], steps[i].angular_velocity[1], steps[i].angular_velocity[2]);
        device->setQ(steps[i].linear_interpolated_joint_configuration, state);
        Q qdot(prod(LinearAlgebra::inverse(device->baseJframe(tool, state).m()), u.m()));
        steps[i].qdot = qdot;
    }
    
    cout << "Results:" << endl;
    cout << "qdot(2)  \t" << steps[2].qdot << endl;
    cout << "qdot(10) \t" << steps[10].qdot << endl;
    
    cout << "Finished running assignment X." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}

void ass_xi() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment XI." << endl << endl;
    
    cout << "Showing cubic splines values." << endl;
    
    cout << "Results:" << endl;
    cout << "First Q: \t" << cubic_spline(steps[2].time / 2.0, steps[1].time, steps[2].time, steps[1].linear_interpolated_joint_configuration, steps[2].linear_interpolated_joint_configuration, steps[1].qdot, steps[2].qdot) << endl;
    cout << "Second Q: \t" << cubic_spline(((steps[9].time / 3.0) + ((2.0 * steps[10].time) / 3)), steps[9].time, steps[10].time, steps[9].linear_interpolated_joint_configuration, steps[10].linear_interpolated_joint_configuration, steps[9].qdot, steps[10].qdot) << endl;
    
    cout << "Finished running assignment XI." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}

void ass_xii() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment XII." << endl << endl;
    
    cout << "Making a " << FINISHED_STEPS << " step path." << endl;
    
    double h = (steps.back().time - steps[1].time) / FINISHED_STEPS;
    Step s;
    finished_steps.push_back(s);
    int step_index = 1;
    
    s.joint_configuration = steps[1].linear_interpolated_joint_configuration;
    s.time = steps[1].time;
    finished_steps.push_back(s);
    
    double current_time = steps[1].time;
    for (int i = 1; i < FINISHED_STEPS; i++) {
        current_time = current_time + h;
        s.time = current_time;
        
        while (current_time > steps[step_index + 1].time) {
            step_index++;
        }
        
        s.joint_configuration = cubic_spline(current_time, steps[step_index].time, steps[step_index + 1].time, steps[step_index].linear_interpolated_joint_configuration, steps[step_index + 1].linear_interpolated_joint_configuration, steps[step_index].qdot, steps[step_index + 1].qdot);
        
        finished_steps.push_back(s);
    }
    
    cout << "Outputting the " << finished_steps.size() - 1 << " joint configurations to: " << OUTPUT_FILE << endl;
    ofstream out;
    out.open(OUTPUT_FILE, ios::out | ios::trunc);
    if (out.is_open()) {
        for (int i = 1; i < finished_steps.size(); i++) {
            if (finished_steps[i].joint_configuration.size() != 6) {
                cout << "Empty Q at index: " << i << endl;
            } else {
                out << finished_steps[i].joint_configuration(0) << " ";
                out << finished_steps[i].joint_configuration(1) << " ";
                out << finished_steps[i].joint_configuration(2) << " ";
                out << finished_steps[i].joint_configuration(3) << " ";
                out << finished_steps[i].joint_configuration(4) << " ";
                out << finished_steps[i].joint_configuration(5) << " ";
                out << endl;
            }
        }
    } else {
        cout << "Could not open file!" << endl;
    }
    out.close();
    
    cout << "Results:" << endl;
    cout << "Q(5): \t" << finished_steps[5].joint_configuration << endl;
    cout << "Q(200): " << finished_steps[200].joint_configuration << endl;
    
    cout << "Finished running assignment XII." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}

void ass_xiv() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment XIV." << endl << endl;
    
    cout << "Computing fastest time-profile" << endl;
    Q vmax = device->getVelocityLimits();
    
    Step s;
    fast_steps.push_back(s);
    
    s = finished_steps[1];
    s.shifted_time = 0;
    fast_steps.push_back(s);
    
    for (int i = 2; i < finished_steps.size() - 2; i++) {
        // Check for the unfortunate NaNs.
        if (!isnan(finished_steps[i].joint_configuration[0]) && !isnan(finished_steps[i - 1].joint_configuration[0]) && finished_steps[i].joint_configuration.size() == 6 && finished_steps[i - 1].joint_configuration.size() == 6) {
            s = finished_steps[i];
            s.shifted_time = velocity_scaling(vmax, finished_steps[i - 1], finished_steps[i]);
            fast_steps.push_back(s);
        }
    }
    
    cout << "Original execution-time: \t" << finished_steps.back().time - finished_steps[1].time << endl;
    cout << "Fastest execution-time: \t" << fast_steps[fast_steps.size() - 2].shifted_time - fast_steps[1].shifted_time << endl;
    cout << "These results are not correct!" << endl;
    
    cout << "Finished running assignment XIV." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}

void ass_xv() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment XV." << endl << endl;
    
    cout << "Adding approach and exit trajectories." << endl;
    
//    cout << finished_steps[1].qdot;
//    cout << finished_steps.back().qdot;
//    cout << finished_steps[finished_steps.size() - 1].joint_configuration << endl;
//    cout << finished_steps[finished_steps.size() - 1].time << endl;
//    cout << finished_steps[finished_steps.size() - 2].joint_configuration << endl;
//    cout << finished_steps[finished_steps.size() - 2].time << endl;
    
    vector<Step> approach;
    vector<Step> exit;
    
    Q q_approach_start(6, 0.0, -Pi/2.0, Pi/2.0, 0.0, -0.1745, 0.0);
    Q q_approach_end = finished_steps[1].joint_configuration;
    double approach_start_time = 0;
    double approach_end_time = APPROACH_AND_EXIT_TIME;
    
    Step s;
    s.time = 0.0;
    s.joint_configuration = q_approach_start;
    approach.push_back(s);

    
    double current_time = 0.0;
    double time_inc = ((double)APPROACH_AND_EXIT_TIME / (double)APPROACH_AND_EXIT_STEP_NUMBER);
    for (int i = 1; i < APPROACH_AND_EXIT_STEP_NUMBER; i++) {
        current_time = current_time + time_inc;
        s.time = current_time;
        s.joint_configuration = q_approach_start + ((current_time - approach_start_time) / (approach_start_time - approach_end_time)) * (q_approach_end - q_approach_start);
        approach.push_back(s);
    }
    
    Q q_exit_start = finished_steps.back().joint_configuration;
    Q q_exit_end(6, 0.0, -(Pi/4.0), Pi/8.0, 0.0, 1.0, 0.0);
    double exit_start_time = 0;
    double exit_end_time = APPROACH_AND_EXIT_TIME;
    
    s.time = 0.0;
    s.joint_configuration = q_exit_start;
    exit.push_back(s);
    
    current_time = 0.0;
    time_inc = ((double)APPROACH_AND_EXIT_TIME / (double)APPROACH_AND_EXIT_STEP_NUMBER);
    for (int i = 1; i < APPROACH_AND_EXIT_STEP_NUMBER; i++) {
        current_time = current_time + time_inc;
        s.time = current_time;
        s.joint_configuration = q_exit_start + ((current_time - exit_start_time) / (exit_start_time - exit_end_time)) * (q_exit_end - q_exit_start);
        approach.push_back(s);
    }
    
    s.time = (double)APPROACH_AND_EXIT_TIME;
    s.joint_configuration = q_exit_end;
    exit.push_back(s);
    
    double break_time = 0.0;
    int last_approach_index = 0;
    for (int i = 0; i < approach.size(); i++) {
        final_steps.push_back(approach[i]);
        break_time = final_steps.back().time;
        last_approach_index = i;
    }
    
    double second_break_time = 0.0;
    int last_main_index = 0;
    for (int i = 0; i < finished_steps.size() - 1; i++) {
        final_steps.push_back(finished_steps[i]);
        final_steps.back().time += break_time;
        second_break_time = final_steps.back().time;
        last_main_index = i;
    }
    
    for (int i = 0; i < exit.size(); i++) {
        final_steps.push_back(exit[i]);
        final_steps.back().time += second_break_time;
    }
    
    cout << "Finished running assignment XV." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}

void ass_xvi() {
    cout << "------------------------------------------------------------------------" << endl;
    cout << "Running assignment XVI." << endl << endl;
    
    cout << "Outputting the " << final_steps.size() - 1 << " joint configurations to: " << OUTPUT_FILE_TWO << endl;
    ofstream out;
    out.open(OUTPUT_FILE_TWO, ios::out | ios::trunc);
    if (out.is_open()) {
        for (int i = 0; i < final_steps.size(); i++) {
            if (final_steps[i].joint_configuration.size() != 6) {
                cout << "Empty Q at index: " << i << endl;
            } else {
                out << final_steps[i].joint_configuration(0) << " ";
                out << final_steps[i].joint_configuration(1) << " ";
                out << final_steps[i].joint_configuration(2) << " ";
                out << final_steps[i].joint_configuration(3) << " ";
                out << final_steps[i].joint_configuration(4) << " ";
                out << final_steps[i].joint_configuration(5) << " ";
                out << endl;
            }
        }
    } else {
        cout << "Could not open file!" << endl;
    }
    out.close();
    
    cout << "Finished running assignment XVI." << endl;
    cout << "------------------------------------------------------------------------" << endl << endl;
}

double velocity_scaling(Q& vmax, Step& previous, Step& current) {
    double result;
    
    double best_time = abs(current.joint_configuration[0] - previous.joint_configuration[0]) / (double)vmax[0];
    double temp_time = 0.0;
    
    for (int i = 1; i < current.joint_configuration.size(); i++) {
        temp_time = abs(current.joint_configuration[i] - previous.joint_configuration[i]) / (double)vmax[i];
        
        if (temp_time > best_time) {
            best_time = temp_time;
        }
    }
    
    result = best_time + previous.shifted_time;
    
    return result;
}

Q cubic_spline(double t, double ts, double tf, Q qs, Q qf, Q vs, Q vf) {
    Q result = (-2.0 * (qf - qs) + (tf - ts) * (vs + vf)) * pow((t - ts)/(tf - ts), 3.0) + (3.0 * (qf - qs) - (tf - ts) * (2.0 * vs + vf)) * pow((t - ts)/(tf - ts), 2.0) + vs * (t - ts) + qs;
    return result;
}

Transform3D<> linear_interpolate(Step& from, Step& to, double t) {
    Transform3D<> result;
    
    // P-part
    Vector3D<> p;
    double time = ((t - from.time)/(to.time - from.time));
    p = from.t_desired.P() +  time * (to.t_desired.P() - from.t_desired.P());
    
    // R-part
    Rotation3D<> wrot = inverse(from.t_desired.R()) * to.t_desired.R();
    Vector3D<> x = w_of_rot(wrot);
    Vector3D<> tx = time * x;
    Rotation3D<> r = from.t_desired.R() * reaa(tx);
    
    result.P() = p;
    result.R() = r;
    return result;
}

Rotation3D<> reaa(Vector3D<>& x) {
    double theta = x.norm2();
    Vector3D<> v = x / theta;
    return reaa(v, theta);
}

Rotation3D<> reaa(Vector3D<>& v, double& theta) {
    Rotation3D<> result;
    result(0, 0) = pow(v[0], 2.0) * (1 - cos(theta)) + cos(theta);
    result(0, 1) = v[0] * v[1] * (1 - cos(theta)) - v[2] * sin(theta);
    result(0, 2) = v[0] * v[2] * (1 - cos(theta)) + v[1] * sin(theta);
    result(1, 0) = v[0] * v[1] * (1 - cos(theta)) + v[2] * sin(theta);
    result(1, 1) = pow(v[1], 2.0) * (1 - cos(theta)) + cos(theta);
    result(1, 2) = v[1] * v[2] * (1 - cos(theta)) - v[0] * sin(theta);
    result(2, 0) = v[0] * v[2] * (1 - cos(theta)) - v[1] * sin(theta);
    result(2, 1) = v[1] * v[2] * (1 - cos(theta)) + v[0] * sin(theta);
    result(2, 2) = pow(v[2], 2.0) * (1 - cos(theta)) + cos(theta);
    
    return result;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Vector3D<> w_of_rot(Rotation3D<>& r) {
    // Check which formular we should use
    Vector3D<> to_norm(r(2,1)-r(1,2), r(0,2)-r(2,0), r(1,0)-r(0,1));
    double norm = to_norm.norm2();
    double x = 0.5 * norm;
    
    Vector3D<> result;
    
    if (x <= 1/1000000) {
        // eq 4.18
        result(0) = r(2,1) - r(1,2);
        result(1) = r(0,2) - r(2,0);
        result(2) = r(1,0) - r(0,1);
        result = result * 0.5;
    } else if (Pi - x <= 1/1000000) {
        // eq 4.19
        int a1 = sgn(r(2,1) - r(1,2));
        int a2 = sgn(r(0,2) - r(2,0));
        int a3 = sgn(r(1,0) - r(0,1));
        
        result(0) = (double)a1 * sqrt(1 + r(0, 0));
        result(1) = (double)a2 * sqrt(1 + r(1, 1));
        result(2) = (double)a3 * sqrt(1 + r(2, 2));
        
        result = ((2.0 * Pi - norm) / (2.0 * sqrt(3.0 + r(0, 0) + r(1, 1) + r(2, 2)))) * result;
    } else {
        // eq 4.17
        result(0) = r(2,1) - r(1,2);
        result(1) = r(0,2) - r(2,0);
        result(2) = r(1,0) - r(0,1);
        
        result = ((acos((r(0, 0) + r(1, 1) + r(2, 2) - 1) / 2)) / norm) * result;
    }
    return result;
}

void import_transforms_from_file(string file) {
    ifstream infile(file.c_str());
    string line;
    Vector3D<> p(0.0, 0.0, 0.0);
    Rotation3D<> r(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    int count = 0;
    
    Step step;
    steps.push_back(step);
    
    if (!infile) {
        cout << "Transforms file not found!" << endl;
    } else {
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
                Step step;
                step.t_world_desired = t;
                steps.push_back(step);
                count++;
            }
        }
    }
}

void print_xyzrpy(Transform3D<>& transform) {
    cout << "XYZRPY: \t\t\t{";
    
    cout << transform.P()[0] << ", " << transform.P()[1] << ", " << transform.P()[2];
    
    cout << "}" << endl;
}

void calculate_delta_u(Transform3D<>& tq, Transform3D<>& td, VelocityScrew6D<double>& deltau) {
	Rotation3D<> tempR = td.R() * inverse(tq.R());

    deltau = VelocityScrew6D<double>(
                                     td.P()[0] - tq.P()[0],
                                     td.P()[1] - tq.P()[1],
                                     td.P()[2] - tq.P()[2],
                                     0.5 * (tempR(2, 1) - tempR(1, 2)),
                                     0.5 * (tempR(0, 2) - tempR(2, 0)),
                                     0.5 * (tempR(1, 0) - tempR(0, 1))
                                     );
}