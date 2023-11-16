/**
* Description:Astar Algorithm Implementation
* Time:
* Author:
* E-Mail:
*/

#include "astar/astar.h"
#include <cmath>

using namespace ASTAR;

void print(const std::vector<Node>& nodeList, bool open = true) {
    if(open)
        std::cout << "The open list: " << std::endl;
    else
        std::cout << "The close list: " << std::endl;

    for(auto point : nodeList) {
        std::cout << "pos: [" << point.pos.x << "," << point.pos.y << "] "
                  << "g=" << point.g << ",h=" << point.h << ",f=" << point.GetScore()
                  << " + parent: [" << point.parent.x << "," << point.parent.y << "] \n";
    }
}

int Astar::EuclideanDistance(Point2D start, Point2D target) {
    return static_cast<int>(10 * sqrt(pow(target.x - start.x, 2) + pow(target.y - start.y, 2)));
}

int Astar::ManhattanDistance(Point2D start, Point2D target) {
    return static_cast<int>(10 * (std::abs(target.x - start.x) + std::abs(target.y - start.y)));
}

void Astar::UpdateMap(std::vector<std::vector<int>> mapData) {
    mapData_ = std::move(mapData);
}

bool Astar::PointInMap(const Point2D& pos) {
    if(pos.x < 0 || pos.x >= mapData_.size()
        || pos.y < 0 || pos.y >= mapData_[0].size())
        return false;
    return true;
}

Node* Astar::NodeInList(const Point2D& pos, std::vector<Node>& nodeList) {
    for(auto& node : nodeList) {
        if(pos == node.pos)
            return const_cast<Node*>(&node);
    }

    return nullptr;
}

void Astar::InsertNode2List(const Point2D& pos, const Point2D& parent, int parentG, std::vector<Node>& nodeList) {
    Node _node;
    _node.pos = pos;
    _node.parent = parent;
    _node.g = parentG + EuclideanDistance(parent, pos);
    _node.h = ManhattanDistance(pos, targetPos_);

    nodeList.emplace_back(_node);
}

void Astar::RemoveNodeFromList(const Point2D& pos, std::vector<Node>& nodeList) {
    for(auto it = nodeList.begin(); it != nodeList.end(); ++it) {
        if(pos == it->pos)
            nodeList.erase(it);
    }
}

void Astar::ProcNeighborNodes(const Node& node) {
    Point2D neighborPos;

    ///节点的8领域，顺序为：右下、下、左下、右、左、右上、上、左上
    for(int i=1; i>=-1; i--) {
        for(int j=1; j>=-1; j--) {
            ///不检测当前点，当前点为(0,0)
            if(i == 0 && j == 0)
                continue;

            ///查看是否存在这个邻居节点
            neighborPos = {node.pos.x + i, node.pos.y + j};
            if(!PointInMap(neighborPos))
                continue;

            ///查看该邻居节点是否为墙
            if(mapData_[neighborPos.x][neighborPos.y] == WALL)
                continue;

            ///查看是否处于 close list 中
            if(NodeInList(neighborPos, closeList_))
                continue;

            ///查看是否处于Open List
            ///如果没有处于开放列表中，则将其加入到开放列表
            int neighborG = node.g + EuclideanDistance(node.pos, neighborPos);
            auto successor = NodeInList(neighborPos, openList_);

            if(!successor) {
                InsertNode2List(neighborPos, node.pos, node.g, openList_);
            } else {    ///已经在开放列表中了
                ///如果当前计算出来的G值小于原来的开放列表中的G值，则替换open list中的节点的父节点和G
                if(neighborG < successor->g) {
                    successor->parent = node.pos;
                    successor->g = neighborG;
                }
            }
        }
    }
}

std::vector<Point2D> Astar::FindPath(const Point2D& startPos, const Point2D& targetPos) {
    std::vector<Point2D> path;

    if(mapData_.empty()) {
        std::cout << "The map was empty !" << std::endl;
        return path;
    }

    ///判断起点和终点的合法性
    if(!PointInMap(startPos)) {
        std::cout << "The map [ H: " << mapData_.size() << " , W: " << mapData_[0].size() << " ]\n";
        std::cout << "The start pos: [" << startPos.x << "," << startPos.y << "] not in map !" << std::endl;
        return path;
    }

    if(!PointInMap(targetPos)) {
        std::cout << "The map [ H: " << mapData_.size() << " , W: " << mapData_[0].size() << " ]\n";
        std::cout << "The target pos: [" << targetPos.x << "," << targetPos.y <<"] not in map!" << std::endl;
        return path;
    }

    if(startPos == targetPos) {
        std::cout << "StartPos = TargetPos = [" << startPos.x << "," << startPos.y << "]\n";
        return path;
    }

    ///判断起点位置和终点位置是否为墙
    if(mapData_[startPos.x][startPos.y] == WALL) {
        std::cout << "The targetPos: [" << startPos.x << "," << startPos.y <<"] = "
                  << mapData_[startPos.x][startPos.y] <<  " was WALL!" << std::endl;
        return path;
    }

    if(mapData_[targetPos.x][targetPos.y] == WALL) {
        std::cout << "The targetPos: [" << targetPos.x << "," << targetPos.y <<"] = "
                << mapData_[targetPos.x][targetPos.y] <<  " was WALL!" << std::endl;
        return path;
    }

    startPos_ = startPos;
    targetPos_ = targetPos;

    ///将起点放入到openList中
    InsertNode2List(startPos, startPos, 0, openList_);

    ///如果开放列表空了，要么是找到了路径，要么就是寻找失败了
    Node curNode;
    while (!openList_.empty()) {
        auto curNodeIt = openList_.begin();

        ///取open list中f值最小的那个node
        for(auto it = openList_.begin(); it != openList_.end(); ++it) {
            if(it->GetScore() < curNodeIt->GetScore())
                curNodeIt = it;
        }

        ///将当前寻找到的代价最小的节点压入到close list，并从open list中删除
        closeList_.emplace_back(*curNodeIt);

        ///如果当前节点为目标点，则认为找到了
        if(curNodeIt->pos == targetPos) {
            std::cout << "Find the shortest path !!!" << std::endl;
            break;
        }

        ///处理邻居节点
        curNode = *curNodeIt;
        openList_.erase(curNodeIt);

        ProcNeighborNodes(curNode);
    }

    ///提取出来Point点
    std::vector<Point2D> pointList;
    auto node = NodeInList(targetPos, closeList_);
    while(node && node->pos != startPos) {
        pointList.emplace_back(node->pos);
        node = NodeInList(node->parent, closeList_);
    }

    return pointList;
}
