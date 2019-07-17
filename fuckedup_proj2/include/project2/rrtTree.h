#ifndef RRTTREE_H
#define RRTTREE_H

#include <iostream>
#include <cstdlib>
#include <climits>
#include <ctime>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <project2/point.h>
#include <project2/control.h>
#include <project2/traj.h>
#include <random>

class rrtTree
{
 private:
    struct node
    {
        int idx;
        point rand;
        point location;
        int idx_parent;
        double alpha;
        double d;
    } *root;

    int count;
    point x_init, x_goal;
    cv::Mat map;
    cv::Mat map_original;
    double map_origin_x, map_origin_y;
    double res;
    node *ptrTable[20000];

    cv::Mat addMargin(cv::Mat map, int margin);
    void addVertex(point x_new, point x_rand, int idx_near, double alpha, double d);
    int nearestNeighbor(point x_rand, double MaxStep);
    int nearestNeighbor(point x_rand);
    bool isCollision(point x1, point x2, double d, double R);
    point randomState(double x_max, double x_min, double y_max, double y_min);
    int newState(double *out, point x_near, point x_rand, double MaxStep);

 public:
    rrtTree();
    rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin);
    rrtTree(point x_init, point x_goal);
    ~rrtTree();

    void visualizeTree();
    void visualizeTree(std::vector<traj> path);
    int generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep);
    std::vector<traj> backtracking_traj();
};

#endif  /* RRTTREE_H */
