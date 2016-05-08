#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <vector>
#include "ibex.h"
#include "ros/ros.h"
#include "dlnd/camMsg.h"
#include "dlnd/poseMsg.h"
#include "dlnd/localizer.h"
#define PARAMS 5

void callback_Pose(const dlnd::poseMsg::ConstPtr& msg);
void callback_Cam(const dlnd::camMsg::ConstPtr& msg);
ibex::IntervalVector SIVIA(ibex::CtcCompo& C, const ibex::IntervalVector& X0, const int eps);
ibex::IntervalVector getFirstBox(ibex::Function &f1, ibex::Function &f2, ibex::Function &f3, ibex::IntervalVector &prev, double &time);
void initPositions(ros::NodeHandle& n);
void initTables(ros::NodeHandle& n);
void initSubscribers(ros::NodeHandle& n, std::list<ros::Subscriber> &lstSub);
void initContractor(ibex::CtcCompo **C, std::vector<Localizer*> &lstCon, std::vector<ibex::CtcCompo*> &lstCtc);
void clearMem(ibex::CtcCompo **C, std::vector<Localizer*> &lstCon, std::vector<ibex::CtcCompo*> &lstCtc);
void evInt(ibex::Interval &interval, const double &value, const double &eps);
void drawBox(ibex::IntervalVector &res, ros::Publisher &pub);

#endif
