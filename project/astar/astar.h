/*
@Function:Astar Algorithm Implementation
@Time:2022-01-10
@Author:Tang Gu Jie
@E-Mail:2822902808@qq.com
*/
#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <functional>

struct Point2D {
    int x = -1;
    int y = -1;

    Point2D() = default;
    Point2D(int _x, int _y) : x(_x), y(_y) {}
    bool operator==(const Point2D& rhs) const { return this->x == rhs.x && this->y == rhs.y; }
    bool operator!=(const Point2D& rhs) const { return this->x != rhs.x || this->y != rhs.y; }
};

//命名空间 ASTAR
namespace ASTAR
{

//节点结构体
struct Node {
    Point2D parent;         ///父节点位置
    Point2D pos;            ///节点位置
    int g = 0;              ///节点的g值，当前节点与起点的关系
    int h = 0;              ///节点的h值，当前节点与终点的关系
    //int f = 0;              ///节点的f值，f=g+h

    Node() = default;
    Node(Point2D _pos) : pos(_pos) {}

    int GetScore() const { return g + h; }
};

class Astar {
public:
    ///地图格点的类型
    enum PointType {
        PASSABLE = 0,   ///可通过点
        WALL = 1    ///障碍物
    };

    ///距离计算类型
    enum DistanceType {
        EUCLIDEAN   = 0,   ///欧式距离
        MANHATTAN  = 1     ///曼哈顿距离
    };

public:
    /**
     * @brief: 构造函数
     *  默认的权值为1，默认的距离类型为 欧式距离
     * @param mapData
     */
    Astar(std::vector<std::vector<int>> mapData) : mapData_(std::move(mapData)) {}

    /**
     * @brief: 更新地地图数据
     * @param mapData
     */
    void UpdateMap(std::vector<std::vector<int>> mapData);

    /***
     * @brief   Astar的核心函数
     *      其实有一个隐藏的输出closeList，最终函数完成后，会完善closeList，最后根据closeList来逆变路径点
     * @param       无
     * @return
     */
    std::vector<Point2D> FindPath(const Point2D& startPos, const Point2D& targetPos);

private:
    /**
     * @brief: 计算两点间的欧式距离
     * @param:
     *  - start    起始节点的位置
     *  - target   目标节点的位置
     * @return: 欧式距离值：根号√((x2-x1)^2+(y2-y1)^2)
     */
    static int EuclideanDistance(Point2D start, Point2D target);

    /***
     * @brief: 计算两点间的曼哈顿距离
     * @param:
     *  - start    起始节点的位置
     *  - target   目标节点的位置
     * @return  曼哈顿距离值：|(x2-x1)+(y2-y1)|
     */
    static int ManhattanDistance(Point2D start, Point2D target);

    /**
     * @brief 查看给定的位置点是否在地图中
     * @param pos
     * @return
     */
    bool PointInMap(const Point2D& pos);

    /**
     * @brief: 查看某个节点是否位于 节点列表中
     * @param node
     * @param openFlag - openlist or closelist
     * @return
     */
    Node* NodeInList(const Point2D& pos, std::vector<Node>& nodeList);


    /***
     * @brief:   扩展一个节点的周围邻节点
     * @param
     *  - curNode      要扩展邻居的节点
     * @return
     */
    void ProcNeighborNodes(const Node& node);

    /**
     * @brief 将当前节点插入到xx list
     * @param pos         当前节点位置
     * @param parent       父节点
     * @param openFlag     插入到openlist or closelist
     * @return none
     */
    void InsertNode2List(const Point2D& pos, const Point2D& parent, int parentG, std::vector<Node>& nodeList);
    void RemoveNodeFromList(const Point2D& pos, std::vector<Node>& nodeList);

private:
    float weightG_ = 1.0F;                      ///权重g, 默认为1
    float weightH_ = 1.0F;                      ///权重h, 默认为1

    Point2D startPos_;                          ///路径起点
    Point2D targetPos_;                         ///路径终点

    std::vector<Node> openList_;                ///开放列表
    std::vector<Node> closeList_;               ///闭合列表
    std::vector<std::vector<int>> mapData_;     ///地图数据，双vector类型

    std::function<float(Point2D, Point2D)> Distance = EuclideanDistance;    ///默认计算 g 距离的函数
};
}