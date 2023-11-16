
#include "display.h"

#include<opencv2/opencv.hpp>

using namespace std;


const int32_t GridPixSize = 30;     ///相对于画布顶端的偏移量
const int32_t Menu = GridPixSize/2;
const int32_t Width = 1200;
const int32_t Height = 1200;

void Display(const std::vector<Point2D>& pathPoint,
             const std::vector<std::vector<int>>& mapData,
             const Point2D& startPos, const Point2D& targetPos,
             std::string pictureName, bool saveFlag) {
    ///地图是否为空，为空就没必要进行下去了
    if(mapData.empty()) {
        std::cout << "The map was empty !!!" << std::endl;
        return;
    }

    ///地图的宽(列，Y轴)、高(行，X轴)
    auto rows = mapData.size(); ///行
    auto cols = mapData[0].size();  ///列

    cv::Point left_up, right_bottom;
    cv::Point point_first, point_second;
    cv::Mat img(Height, Width, CV_8UC3, cv::Scalar(255, 255, 255));

    ///路径点所在格点的底色 -> 黄色
    for (auto& point : pathPoint) {
        left_up.x = point.y * GridPixSize;
        left_up.y = point.x * GridPixSize;
        right_bottom.x = left_up.x + GridPixSize;
        right_bottom.y = left_up.y + GridPixSize;
        cv::rectangle(img, left_up, right_bottom, cv::Scalar(0, 255, 255), CV_FILLED, 8, 0);
    }

    ///障碍物--->黑色,起点--->蓝色，终点--->红色
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            left_up.x = j * GridPixSize; ///存储数组的列(j)对应矩形的x轴
            left_up.y = i * GridPixSize;
            right_bottom.x = left_up.x + GridPixSize;
            right_bottom.y = left_up.y + GridPixSize;
            if (mapData[i][j] == ASTAR::Astar::WALL) {  ///看是不是墙，如果是，填充黑色
                cv::rectangle(img, left_up, right_bottom, cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
            } else {
                if (i == startPos.x && j == startPos.y)    ///判断起始点 - 蓝色
                    cv::rectangle(img, left_up, right_bottom, cv::Scalar(255, 0, 0), CV_FILLED, 8, 0);
                else if (i == targetPos.x &&j == targetPos.y)  ///判断目标点 - 红色
                    cv::rectangle(img, left_up, right_bottom, cv::Scalar(0, 0, 255), CV_FILLED, 8, 0);
            }
        }
    }

    ///中间线--->黄色
    for (int i = 1; i < cols; i++) {    ///行线
        point_first.x = i * GridPixSize;
        point_first.y = 1 * GridPixSize;
        point_second.x = i * GridPixSize;
        point_second.y = (rows - 1) * GridPixSize;
        cv::line(img, point_first, point_second, cv::Scalar(141,238,238), 2, 2);
    }

    for (int i = 1; i < rows; i++) {    ///列线
        point_first.x = 1 * GridPixSize;
        point_first.y = i * GridPixSize;
        point_second.x = (cols - 1) * GridPixSize;
        point_second.y = i * GridPixSize;
        cv::line(img, point_first, point_second, cv::Scalar(141,238,238), 2, 2);
    }

    ///路径线--->黑色
    point_first.x = targetPos.y * GridPixSize + Menu;
    point_first.y = targetPos.x * GridPixSize + Menu;
    for (auto& point : pathPoint) {
        left_up.x = point.y * GridPixSize;
        left_up.y = point.x * GridPixSize;
        point_second.x = left_up.x + Menu;
        point_second.y = left_up.y + Menu;
        cv::line(img, point_first, point_second, cv::Scalar(0, 0, 0), 2, 4);
        point_first = point_second;
    }
    ///把起点也加进去
    point_second.x = startPos.y * GridPixSize + Menu;
    point_second.y = startPos.x * GridPixSize + Menu;
    cv::line(img, point_first, point_second, cv::Scalar(0, 0, 0), 2, 4);


    if (saveFlag) {
        string str1 = ".png";
        pictureName.append(str1);
        cv::imwrite(pictureName, img);
        cout << "save png success" << endl;
    }

    cv::imshow(pictureName, img);
    cv::waitKey(0);
}
