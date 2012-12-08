#include <iostream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <fstream>
#include <sstream>
#include <cstring>

using namespace std;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

struct RESULT {
	int path_length;
	float computation_time;
	float extend_length;
	bool valid_path;
} run_result;

struct RESULT find_path(struct RESULT res);
bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q);

#define MAXTIME 10000.

int main(int argc, char** argv) {
	Math::seed();
	struct RESULT res;
	
	fstream filestr;
	filestr.open ("data_temp.csv", fstream::in | fstream::out | fstream::app);
	
	int how_many_of_each_epsilon 	= 10000;
	float max_epsilon 				= 6;
	float min_epsilon 				= 0.01;
	float delta_epsilon				= 0.01;
	
	
	for(int i = 1; i <= how_many_of_each_epsilon; i++)
	{
		for(float le = max_epsilon; le >= min_epsilon; le = le - delta_epsilon)
			{
			res.extend_length = 2;
			res = find_path(res);
			if(res.valid_path)
			{
				filestr << setprecision(3) << res.extend_length << "," << res.path_length << "," << setprecision(5) << res.computation_time << endl;
				// cout << setprecision(3) << res.extend_length << "\t" << res.path_length << "\t" << setprecision(5) << res.computation_time << endl;
			} else {
				cout << "Not a valid path!" << endl;
			}
		}
	}

	std::cout << "Program done." << std::endl;
	return 0;
}

struct RESULT find_path(struct RESULT res) {
	res.valid_path = true;
	std::string wcFile = "../../../rob_scenes/KukaKr16Adv/Scene.wc.xml";
	std::string deviceName = "KukaKr16";

	// New points
	Q from(6, -3.1415, -0.82, -3, -3.1415, 0.1, -3.1415/2);
	Q to(6, 3.1415/2, 0.01, 0, 0.15, 0.8, 4.5);

	WorkCell::Ptr wc = WorkCellLoader::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {
		cout << "Device: " << deviceName << " not found!" << std::endl;
		res.valid_path = false;
	}
	
	State state = wc->getDefaultState();
	device->setQ(from, state);
	
	Frame* gripperframe = device->getEnd();
	if (gripperframe == NULL) {
		cout << "Frame: " << "PG70.TCP" << " not found!" << std::endl;
		res.valid_path = false;
	}
	
	MovableFrame* bottleframe = (MovableFrame*)wc->findFrame("Bottle");
	if (bottleframe == NULL) {
		cout << "Frame: " << "Bottle" << " not found!" << std::endl;
		res.valid_path = false;
	}
	
	Kinematics::gripFrame(bottleframe, gripperframe, state);
	
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	double extend = res.extend_length;
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

	if (!checkCollisions(device, state, detector, from)) {
		res.valid_path = false;
	}
	if (!checkCollisions(device, state, detector, to)){
		res.valid_path = false;
	}
		
	if(res.valid_path)
	{
		QPath path;
		Timer t;
		t.resetAndResume();
		planner->query(from,to,path,MAXTIME);
		t.pause();
		
		res.path_length = path.size();
		res.computation_time = t.getTime();
	}
	return res;
}

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		std::cerr << "Configuration in collision: " << q << std::endl;
		std::cerr << "Colliding frames: " << std::endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			std::cerr << (*it).first->getName() << " " << (*it).second->getName() << std::endl;
		}
		return false;
	}
	return true;
}
