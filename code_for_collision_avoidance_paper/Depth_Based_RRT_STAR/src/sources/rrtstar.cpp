#include "rrtstar.h"

RRTSTAR::RRTSTAR()
{
    //obstacles = new Obstacles;
    startPos.x() = 1;
    startPos.y() = 1;
    endPos.x() = 8;
    endPos.y() = 8;
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    root->orientation = 0.785;
    root->cost = 0.0;
    lastNode = root;
    nodes.push_back(root);
    step_size = 18;
    max_iter = 3000;

  //cout<<"IS IT OK?"<<endl;
}

/**
 * @brief Initialize root node of RRTSTAR.
 */
void RRTSTAR::initialize()
{
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    root->orientation = 0.785;
    root->cost = 0.0;
    lastNode = root;
    nodes.push_back(root);

}

/**
 * @brief Generate a random node in the field.
 * @return
 */
Node* RRTSTAR::getRandomNode()
{

    Node* ret;

    //drand(time(NULL));
    Vector2d point(drand() * 20, drand() * 20);
    float orient = drand() * 2 * 3.142;
    //cout<<"point_x = "<<drand()<<endl;
    //cout<<"point_y = "<<point.y()<<endl;
    if (point.x() >= 0 && point.x() <= 20 && point.y() >= 0 && point.y() <= 20
            && orient > 0 && orient < 2*3.142) {
        ret = new Node;
        ret->position = point;
        ret->orientation = orient;
        return ret;
    }
    return NULL;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
double RRTSTAR::distance(Vector2d &p, Vector2d &q)
{
    Vector2d v = p - q;
    return sqrt(pow(v.x(), 2) + pow(v.y(), 2));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
Node* RRTSTAR::nearest(Vector2d point)
{
  //  cout<<nodes.size()<<endl;
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes.size(); i++) {
        double dist = distance(point, nodes[i]->position);
      //  cout<<dist<<endl;
        if (dist < minDist) {
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

/**
 * @brief Get neighborhood nodes of a given configuration/position.
 * @param point
 * @param radius
 * @param out_nodes
 * @return
 */
void RRTSTAR::near(Vector2d point, float radius, vector<Node *>& out_nodes)
{
    for(int i = 0; i < (int)nodes.size(); i++) {
        double dist = distance(point, nodes[i]->position);
        if (dist < radius) {
            out_nodes.push_back(nodes[i]);
        }
    }
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q
 * @param qNearest
 * @return
 */
Vector3d RRTSTAR::newConfig(Node *q, Node *qNearest)
{
    Vector2d to = q->position;
    Vector2d from = qNearest->position;
    Vector2d intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2d pos = from + step_size * intermediate;

    Vector3d ret(pos.x(), pos.y(), 0.0);
    return ret;
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node
 * followin dynamic constraints of a nonholonomic robot (Dubins motion model).
 * @param q
 * @param qNearest
 * @return
 */


/**
 * @brief Return trajectory cost.
 * @param q
 * @return
 */
double RRTSTAR::Cost(Node *q)
{
    return q->cost;
}

/**
 * @brief Compute path cost.
 * @param qFrom
 * @param qTo
 * @return
 */
double RRTSTAR::PathCost(Node *qFrom, Node *qTo)
{
    return distance(qTo->position, qFrom->position);
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRTSTAR::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNew->cost = qNearest->cost + PathCost(qNearest, qNew);
    qNearest->children.push_back(qNew);
    nodes.push_back(qNew);
    lastNode = qNew;
}

/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRTSTAR::reached()
{
   // cout<<"distance = "<<distance(lastNode->position, endPos)<<endl;
    if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD)

        return true;
    return false;
}

void RRTSTAR::setStepSize(double step)
{
    step_size = step;
}

void RRTSTAR::setMaxIterations(int iter)
{
    max_iter = iter;
}

void RRTSTAR::setSTART(double x, double y)
{
    startPos.x() = x;
    startPos.y() = y;
    root->position = startPos;
}

void RRTSTAR::setEND(double x, double y)
{
    endPos.x() = x;
    endPos.y() = y;
}

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void RRTSTAR::deleteNodes(Node *root)
{
    for(int i = 0; i < (int)root->children.size(); i++) {
        deleteNodes(root->children[i]);
    }
    delete root;
}

std::vector<double> RRTSTAR::RRTSteer(std::vector<double> x_nearest, std::vector<double> x_rand, double eta)
{
    std::vector<double> x_new;

    if (RRTNorm(x_nearest,x_rand)<=eta){
        x_new=x_rand;
    }
    else{
        Vector2d to, from;
        from.x() = x_nearest[0];
        from.y() = x_nearest[1];
        to.x() = x_rand[0];
        to.y() = x_rand[1];
        Vector2d intermediate = to - from;
        intermediate = intermediate / intermediate.norm();
        Vector2d pos = from + eta * intermediate;
        x_new.push_back(pos.x());
        x_new.push_back(pos.y());
    }
    return x_new;
}

double RRTSTAR::RRTNorm(std::vector<double> x1, std::vector<double> x2)
{
    return pow(	(pow(abs((x2[0]-x1[0])),2)+pow(abs((x2[1]-x1[1])),2))	,0.5);
}

double RRTSTAR::RRTsign(double n)
{
    if (n<0.0){return -1.0;}
    else{return 1.0;}
}
