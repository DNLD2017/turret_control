#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "dlnd/localization.h"
#include "dlnd/vibes.h"
#include <string>
#define D_VCT std::vector<double>
#define D_TBL std::vector<std::vector<double>>

const double EANGLE1 =  2*3.1415926/180;
const double EANGLE2 =  5*3.1415926/180;
const double EPOS = 0.5;
const double EPS = 1;

static int NBTURRET;
static int NBVARS;
static D_TBL angles;
static D_TBL positions;

static double VX;
static double VY;
static double VZ;

int main(int argc, char **argv){
    ros::init(argc,argv,"localisation");
    ros::NodeHandle n;

    initTables(n);
    initPositions(n);

    std::list<ros::Subscriber> lstSub;

    initSubscribers(n, lstSub);
    ros::Publisher est_pub = n.advertise<visualization_msgs::Marker>("estimation", 1000);
    ros::Publisher dr_est_pub = n.advertise<visualization_msgs::Marker>("drone_estimation", 1000);
    ros::Rate loop_rate(100);

    std::vector<Localizer*>  lstCon;
    std::vector<ibex::CtcCompo*> lstCtc;
    ibex::CtcCompo *C = NULL;
    initContractor(&C, lstCon, lstCtc);
    ibex::Function f1("f","t","p","et","ep","cos(t)*cos(p)*cos(et)*cos(ep)+cos(et)*sin(ep)*(-cos(f)*sin(p)+sin(t)*cos(p)*sin(f))-sin(et)*(sin(p)*sin(f)+sin(t)*cos(p)*cos(f))");
    ibex::Function f2("f","t","p","et","ep","cos(t)*sin(p)*cos(et)*cos(ep)+cos(et)*sin(ep)*(cos(f)*cos(p)+sin(t)*sin(p)*sin(f))-sin(et)*(-cos(p)*sin(f)+sin(t)*sin(p)*cos(f))");
    ibex::Function f3("f","t","p","et","ep","-sin(t)*cos(et)*cos(ep)+cos(et)*sin(ep)*cos(t)*sin(f)-sin(et)*cos(t)*cos(f)");
    ibex::IntervalVector res(3);
    res[0] = ibex::Interval(-50, 50);
    res[1] = ibex::Interval(-50, 50);
    res[2] = ibex::Interval(-50, 50);
    double t1,t2,time = 0;

    while(ros::ok()){
        t1 = ros::Time::now().toSec();
        ibex::IntervalVector X0 = getFirstBox(f1, f2, f3, res, time);
        res = SIVIA(*C, X0, EPS);

        drawBox(res,est_pub);

        visualization_msgs::Marker msgp;
        msgp.header.frame_id = "/my_frame";
        msgp.header.stamp = ros::Time::now();
        msgp.ns = "drone_est";
        msgp.action = visualization_msgs::Marker::ADD;
        msgp.type = visualization_msgs::Marker::SPHERE;
        msgp.pose.position.x = res[0].mid();
        msgp.pose.position.y = res[1].mid();
        msgp.pose.position.z = res[2].mid();
        msgp.scale.x = 0.2;
        msgp.scale.y = 0.2;
        msgp.scale.z = 0.2;
        msgp.color.r = 1.0f;
        msgp.color.a = 1.0;
        dr_est_pub.publish(msgp);

        ros::spinOnce();
        loop_rate.sleep();
        t2 = ros::Time::now().toSec();
        time = t2-t1;
    }

    clearMem(&C, lstCon, lstCtc);
    return 0;
}

void callback_Pose(const dlnd::poseMsg::ConstPtr& msg){
    int id = msg->id;
    tf::Quaternion qg;
    tf::quaternionMsgToTF(msg->pos, qg);
    tf::Matrix3x3 m(qg);
    m.getRPY(angles[id-1][0],angles[id-1][1],angles[id-1][2]);
}

void callback_Cam(const dlnd::camMsg::ConstPtr& msg){
    int id = msg->id;
    angles[id-1][3] = msg->etheta;
    angles[id-1][4] = msg->epsi;
}

ibex::IntervalVector SIVIA(ibex::CtcCompo& C, const ibex::IntervalVector& X0, const int eps){
    std::list<ibex::IntervalVector> L;
    std::vector<ibex::IntervalVector> result;
    L.push_back (X0);
    while ( !L.empty() ){
        ibex::IntervalVector X = L.front(); L.pop_front();
        C.contract(X);
        if(!X.is_empty()){
            if (X.max_diam()>eps)
            {
                ibex::LargestFirst bisector(0.0,0.5);
                std::pair<ibex::IntervalVector, ibex::IntervalVector> p = bisector.bisect(X);
                L.push_back(p.first);  L.push_back(p.second);
            }
            else{
                result.push_back(X);
            }
        }
    }
    ibex::IntervalVector iv = ibex::IntervalVector::empty(3);
    for(int i=0; i<result.size(); i++){
        iv[0] |= result.at(i)[0];
        iv[1] |= result.at(i)[1];
        iv[2] |= result.at(i)[2];
    }
    return iv;
}

ibex::IntervalVector getFirstBox(ibex::Function &f1, ibex::Function &f2, ibex::Function &f3, ibex::IntervalVector &prev, double &time){
    ibex::IntervalVector X0(NBVARS);
    if(!prev.is_empty()){
        X0[0]=prev[0]+ibex::Interval(-time*VX, time*VX);
        X0[1]=prev[1]+ibex::Interval(-time*VY, time*VY);
        X0[2]=prev[2]+ibex::Interval(-time*VZ, time*VZ);
    }else{
        X0[0] = ibex::Interval(-50, 50);
        X0[1] = ibex::Interval(-50, 50);
        X0[2] = ibex::Interval(-50, 50);
    }
    for(int i = 0; i<NBTURRET; i++){
        ibex::IntervalVector var(PARAMS);
        evInt(var[0],angles[i][0],EANGLE1);
        evInt(var[1],angles[i][1],EANGLE1);
        evInt(var[2],angles[i][2],EANGLE1);
        evInt(var[3],0,EANGLE2);
        evInt(var[4],0,EANGLE2);
        X0[6*i+3] = positions[i][0];
        X0[6*i+4] = positions[i][1];
        X0[6*i+5] = positions[i][2];
        X0[6*i+6] = f1.eval(var);
        X0[6*i+7] = f2.eval(var);
        X0[6*i+8] = f3.eval(var);
    }
    return X0;
}

void evInt(ibex::Interval &interval, const double &value, const double &eps){
    interval = ibex::Interval(value-eps, value+eps);
}

void initPositions(ros::NodeHandle& n){
    for(int i = 0; i<NBTURRET; i++){
        std::string name = std::string("/turret")+std::to_string(i+1);
        n.getParam(name+std::string("/x"),positions[i][0]);
        n.getParam(name+std::string("/y"),positions[i][1]);
        n.getParam(name+std::string("/z"),positions[i][2]);
    }
}

void initTables(ros::NodeHandle& n){
    n.getParam("nbturret",NBTURRET);
    n.getParam("vx", VX);
    n.getParam("vy", VY);
    n.getParam("vz", VZ);
    NBVARS = 3+6*NBTURRET;
    D_TBL ang(NBTURRET, D_VCT(PARAMS,0));
    D_TBL pos(NBTURRET, D_VCT(3,0));
    D_TBL::iterator it1 = ang.begin();
    angles.assign(it1, ang.end());
    D_TBL::iterator it2 = pos.begin();
    positions.assign(it2, pos.end());
}

void initSubscribers(ros::NodeHandle& n, std::list<ros::Subscriber> &lstSub){
    for(int i = 0; i<NBTURRET; i++){
        std::string name = std::string("/turret");
        std::string namePos = name + std::to_string(i+1) + std::string("/posbr");
        std::string nameCam = name + std::to_string(i+1) + std::string("/cambr");
        ros::Subscriber pos_sub = n.subscribe<dlnd::poseMsg>(namePos,1000,callback_Pose);
        ros::Subscriber cam_sub = n.subscribe<dlnd::camMsg>(nameCam,1000,callback_Cam);
        lstSub.push_back(pos_sub);
        lstSub.push_back(cam_sub);
    }
}

void initContractor(ibex::CtcCompo **C, std::vector<Localizer*> &lstCon, std::vector<ibex::CtcCompo*> &lstCtc){
    for(int i = 0; i<NBTURRET; i++){
        Localizer *l = new Localizer(i, NBTURRET);
        lstCon.push_back(l);
        ibex::CtcCompo *c = l->getCtc();
        if(i == 0){
            *C = c;
            lstCtc.push_back(*C);
        }else{
            ibex::CtcCompo *c_ = new ibex::CtcCompo(**C, *c);
            *C = c_;
            lstCtc.push_back(*C);
        }
    }
}

void clearMem(ibex::CtcCompo **C, std::vector<Localizer*> &lstCon, std::vector<ibex::CtcCompo*> &lstCtc){
    for(int i = 0; i<NBTURRET; i++){
        delete(lstCon.at(i));
        lstCon.at(i) = NULL;
    }

    // The first pointer has been cleared with the first Localizer object.
    lstCtc.at(0) = NULL;
    for(int i = 1; i<lstCtc.size(); i++){
        delete(lstCtc.at(i));
        lstCtc.at(i) = NULL;
    }
    // C has been cleared with the last element of the above loop.
    *C = NULL;
}

void drawBox(ibex::IntervalVector &res, ros::Publisher &pub){
    visualization_msgs::Marker msg;
    msg.header.frame_id = "/my_frame";
    msg.header.stamp = ros::Time::now();
    msg.ns = "estimation";
    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::CUBE;
    msg.pose.position.x = res[0].mid();
    msg.pose.position.y = res[1].mid();
    msg.pose.position.z = res[2].mid();
    msg.scale.x = res[0].diam();
    msg.scale.y = res[1].diam();
    msg.scale.z = res[2].diam();
    msg.color.g = 1.0f;
    msg.color.r = 1.0f;
    msg.color.b = 1.0f;
    msg.color.a = 0.5;
    pub.publish(msg);
}

