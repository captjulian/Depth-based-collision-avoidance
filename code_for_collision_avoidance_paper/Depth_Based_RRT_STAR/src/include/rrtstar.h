#ifndef RRTSTAR_H
#define RRTSTAR_H

//#include "obstacles.h"

#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "mtrand.h"
using namespace std;
using namespace Eigen;

#define BOT_TURN_RADIUS     2
#define END_DIST_THRESHOLD     0.5

struct Node {
    vector<Node *> children;
    Node *parent;

    Vector2d position;
    float orientation;
    double cost;

};

class RRTSTAR
{
public:


    RRTSTAR();
    void initialize();
    Node* getRandomNode();
    Node* nearest(Vector2d point);
    void near(Vector2d point, float radius, vector<Node *>& out_nodes);
    double distance(Vector2d &p, Vector2d &q);
    double Cost(Node *q);
    double PathCost(Node *qFrom, Node *qTo);
    Vector3d newConfig(Node *q, Node *qNearest);
    void add(Node *qNearest, Node *qNew);
    bool reached();
    void setStepSize(double step);
    void setMaxIterations(int iter);
    void setSTART(double x, double y);
    void setEND(double x, double y);
    void deleteNodes(Node *root);
    std::vector<double> RRTSteer(std::vector<double> x_nearest ,
                                 std::vector<double> x_rand, double eta);
    double RRTNorm(std::vector<double> x1,std::vector<double> x2);
    double RRTsign(double n);
   // Obstacles *obstacles;
    vector<Node *> nodes;
    vector<Node *> path;
    Node *root, *lastNode;
    Vector2d startPos, endPos;
    int max_iter;
    double step_size;
    MTRand drand;

};

#endif // RRTSTAR_H
