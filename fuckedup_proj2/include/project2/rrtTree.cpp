#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#define PI 3.14159265358979323846

double max_alpha = 0.2;
double L = 0.325;

#define BOUND(a) ((a > PI) ? (a - 2* PI) : ((a < -PI) ? (a + 2*PI) : a))
#define DISTSQ(p, q) (std::pow(p.x - q.x, 2) + std::pow(p.y - q.y, 2))

rrtTree::rrtTree()
{
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal)
{
    this->x_init = x_init;
    this->x_goal = x_goal;

    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->alpha = 0;
    root->d = 0;
}

rrtTree::~rrtTree(){
    for (int i = 1; i <= count; i++)
        delete ptrTable[i];
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin)
{
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;
    std::srand(std::time(NULL));

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin)
{
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++)
        for (int j = 0; j < xSize; j++)
            if (map.at<uchar>(i, j) < 125)
                for (int k = i - margin; k <= i + margin; k++)
                    for (int l = j - margin; l <= j + margin; l++)
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize)
                            map_margin.at<uchar>(k, l) = 0;

    return map_margin;
}

void rrtTree::visualizeTree()
{
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
        for(int j = 0; j < 10; j++) {
            double alpha = this->ptrTable[i]->alpha;
            double d = this->ptrTable[i]->d;
            double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
            double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
            double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
        }
    }
    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::visualizeTree(std::vector<traj> path)
{
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(path[0].y/res + map_origin_y)), (int)(Res*(path[0].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(path[path.size()-1].y/res + map_origin_y)), (int)(Res*(path[path.size()-1].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
        for(int j = 0; j < 10; j++) {
            double alpha = this->ptrTable[i]->alpha;
            double d = this->ptrTable[i]->d;
            double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
            double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
            double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
        }
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
        for(int j = 0; j < 10; j++) {
            double alpha = path[i].alpha;
            double d = path[i].d;
            double p1_th = path[i-1].th + d*j/10*tan(alpha)/L; // R = L/tan(alpha)
            double p2_th = path[i-1].th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = path[i-1].x + L/tan(alpha)*(sin(p1_th) - sin(path[i-1].th));
            double p1_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p1_th));
            double p2_x = path[i-1].x + L/tan(alpha)*(sin(p2_th) - sin(path[i-1].th));
            double p2_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
        }
    }
    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha, double d)
{
    //TODO
    node *n = new node();
    n->idx = count;
    n->rand = x_rand;
    n->location = x_new;
    n->idx_parent = idx_near;
    n->alpha = alpha;
    n->d = d;
    ptrTable[count++] = n;
}


int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep)
{
    //TODO
    for (int i = 0 ; i < K ; i++) {
        point x_rand;
        int idx_near;
        do {
            x_rand = randomState(x_max, x_min, y_max, y_min);
            idx_near = nearestNeighbor(x_rand, MaxStep);
        } while (idx_near < 0);
        point x_near = ptrTable[idx_near]->location;
        double out[5];
        if (newState(out, x_near, x_rand, MaxStep)) {
            point x_new;
            x_new.x = out[0];
            x_new.y = out[1];
            x_new.th = out[2];
            addVertex(x_new, x_rand, idx_near, out[3], out[4]);
            double distsq = DISTSQ(x_new, x_goal);
            if (distsq < 0.04)
                return 1; //FOUND
        }
    }
    return 0;
}

point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min)
{
    //TODO
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_real_distribution<> dis1(x_min, x_max);
    std::uniform_real_distribution<> dis2(y_min, y_max);
    point resultP;
    resultP.x = dis1(gen);
    resultP.y = dis2(gen);
    return resultP;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep)
{
    //TODO
    const double thmax = 0.15;
    int minidx = -1;
    double mindistsq = 1000;
    for (int i = 0; i < count; i++) {
        double distsq = DISTSQ(x_rand, ptrTable[i]->location);
        double angle = std::atan2(x_rand.y - ptrTable[i]->location.y, x_rand.x - ptrTable[i]->location.x) - ptrTable[i]->location.th;
        BOUND(angle);
        if (distsq < mindistsq && std::abs(angle) < thmax) {
            minidx = i;
            mindistsq = distsq;
        }
    }
    return minidx;
}

int rrtTree::nearestNeighbor(point x_rand)
{
    //TODO
    int minidx = -1;
    double mindistsq = 200; //FIXME
    for (int i = 0; i < count; i++) {
        double distsq = DISTSQ(x_rand, ptrTable[i]->location);
        if (distsq < mindistsq) {
            minidx = i;
            mindistsq = distsq;
        }
    }
    return minidx;
}

int rrtTree::newState(double *out, point x_near, point x_rand, double MaxStep)
{
    //TODO
    double alpha = std::atan2(x_rand.y - x_near.y, x_rand.x - x_near.x) - x_near.th;
    BOUND(alpha);
    point x_new;
    double R = L/std::tan(alpha);

    if (isCollision(x_rand, x_near, 0, 0))
        return 0;

    double xc, yc;

    if (alpha >= 0) {
        xc = x_near.x - R*std::sin(x_near.th);
        yc = x_near.y + R*std::cos(x_near.th);
    }
    else {
        xc = x_near.x + R*std::sin(x_near.th);
        yc = x_near.y - R*std::cos(x_near.th);
    }
    double thTmp = std::atan2(x_near.y - yc , x_near.x - xc) - std::atan2(x_rand.y - yc , x_rand.x - xc);
    double thnew = x_near.th - ((thTmp > 0) ? 1.0 : -1.0) * MaxStep / R;
    BOUND(thTmp);
    BOUND(thnew);
    if (std::abs(R * thTmp) <= MaxStep)
        x_new = x_rand;
    else {
        x_new.x = xc + R * std::cos(thnew);
        x_new.y = yc + R * std::sin(thnew);
    }

    x_new.th = BOUND(thnew - PI/2 + (thTmp > 0) ? 0 : 1 * PI);

    out[0] = x_new.x;
    out[1] = x_new.y;
    out[2] = x_new.th;
    out[3] = alpha;
    out[4] = R * std::abs(thTmp);

    return 1;
}

bool rrtTree::isCollision(point x1, point x2, double d, double R)
{
    //TODO
    int i1 = x1.x/res + map_origin_x;
    int i2 = x2.x/res + map_origin_x;

    int j1 = x1.y/res + map_origin_y;
    int j2 = x2.y/res + map_origin_y;

    if (j1 > j2) {
        std::swap(j1, j2);
        std::swap(i1, i2);
    }

    if (j1 == j2) {
        for (int i = i1; i <= i2; ++i)
            if (map.at<uchar>(i, j1) == 0)
                return true;
        return false;
    }

    const double slope = (i2-i1) / (j2-j1);

    for (int j = j1; j < j2; j++) {
        double istart = slope * (j - j1) + i1;
        double iend = slope * (j + 1 - j1) + i1;
        if (iend > istart)
            std::swap(istart, iend);
        const int start = std::floor(istart);
        const int end = std::ceil(iend);
        for (int i = start; i <= end; i++)
            if (map.at<uchar>(i,j) == 0)
                return true; //collision
    }
    return false;

    /*
      double inv_tilt = abs((i2- i1)/(j2 - j1));
      bool resultB = false;

      int i_init = i1;
      if (i1 > i2)
      i_init = i2;
      int j_init = j1;
      if (j1 > j2)
      j_init = j2;

      for (int j = j_init; j < j_init + abs(j2 - j1) ; ) {
      for (int i = i_init; i < i_init + abs(i2 - i1) ; i++) {
      if (cv::map_margin.at<uchar>(i,j) == 0) {
      return true;
      }
      if ((i - i_init) % inv_tilt == 0) {
      j ++;
      break;
      }
      }
      }
      return false;
    */

}

std::vector<traj> rrtTree::backtracking_traj()
{
    //TODO
    const int ptrEnd = count-1;
    std::vector<traj> track;
    double distsq = DISTSQ(ptrTable[ptrEnd]->location, x_goal);
    if (distsq >= 0.04)
        return track;

    node *parent = ptrTable[ptrEnd];
    traj t;
    t.x = parent->location.x;
    t.y = parent->location.y;
    t.th = parent->location.th;
    t.alpha = parent->alpha;
    t.d = parent->d;
    track.push_back(t);
    while (parent != root) {
        parent = ptrTable[parent->idx_parent];
        traj t;
        t.x = parent->location.x;
        t.y = parent->location.y;
        t.th = parent->location.th;
        t.alpha = parent->alpha;
        t.d = parent->d;
        track.push_back(t);
    }
    return track;
}
