#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

// 析构函数：释放二维网格节点内存
AStar::~AStar()
{
    if (GridNodeMap_ == nullptr) return;
    // 释放每个节点对象
    for (int i = 0; i < POOL_SIZE_(0); ++i)
    {
        if (GridNodeMap_[i] == nullptr) continue;
        for (int j = 0; j < POOL_SIZE_(1); ++j)
        {
            delete GridNodeMap_[i][j];  // 释放节点
        }
        delete[] GridNodeMap_[i];  // 释放y方向指针数组
    }
    delete[] GridNodeMap_;  // 释放x方向指针数组
    GridNodeMap_ = nullptr;
}

// 初始化二维网格地图
void AStar::initGridMap(shared_ptr<GridMap2D> occ_map, const Eigen::Vector2i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;  // 二维中心索引（x,y方向分别取半）

    // 分配二维网格节点指针数组：[x][y]
    GridNodeMap_ = new GridNodePtr*[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); ++i)
    {
        GridNodeMap_[i] = new GridNodePtr[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); ++j)
        {
            GridNodeMap_[i][j] = new GridNode;  // 初始化每个二维节点
        }
    }

    grid_map_ = occ_map;
}

// 二维对角线启发函数（已适配）
double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));  // x方向索引差
    double dy = abs(node1->index(1) - node2->index(1));  // y方向索引差

    int diag = min(dx, dy);               // 对角线移动步数
    double remaining = abs(dx - dy);      // 剩余轴方向移动步数

    return diag * sqrt(2.0) + remaining * 1.0;  // 对角线成本+轴方向成本
}

// 二维曼哈顿启发函数（移除z轴）
double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));  // x方向差
    double dy = abs(node1->index(1) - node2->index(1));  // y方向差
    return dx + dy;  // 仅x+y方向求和
}

// 二维欧式距离启发函数（移除z轴影响）
double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    // 仅计算x、y方向的欧式距离
    return (node2->index - node1->index).head(2).norm();
}

// 回溯路径（无需修改，与维度无关）
vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != nullptr)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

// 转换坐标到索引并调整起点终点（适配二维）
bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector2d start_pt, Vector2d end_pt, 
                                                  Vector2i &start_idx, Vector2i &end_idx)
{
    const Vector2d start_pt_raw = start_pt;
    const Vector2d end_pt_raw = end_pt;
    bool adjusted_start = false;
    bool adjusted_end = false;

    // 仅使用x、y坐标转换为二维索引（忽略z轴）
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    // 使用原始连续坐标判占用（而不是回投到栅格中心），避免“看起来不在障碍但被判在障碍”。
    // 同时限制最大调整步数，防止异常地图下长时间循环卡住。
    constexpr int kMaxAdjustSteps = 200;

    // 检查起点是否在障碍物内，若在则向终点方向调整（仅x、y方向）
    if (checkOccupancy(start_pt))
    {
        const Vector2d delta = end_pt - start_pt;
        if (delta.norm() < 1e-6) {
            std::cout << "[调整A*起点] 起终点过近，无法生成有效调整方向，直接返回失败。" << std::endl;
            return false;
        }
        Vector2d dir = delta.normalized();
        int adjust_steps = 0;
        while (checkOccupancy(start_pt))
        {
            start_pt += dir * step_size_;
            adjusted_start = true;
            if (!Coord2Index(start_pt, start_idx))
                return false;
            if (++adjust_steps >= kMaxAdjustSteps) {
                std::cout << "[调整A*起点] 超过最大调整步数(" << kMaxAdjustSteps << ")，放弃本次搜索。" << std::endl;
                return false;
            }
        }
    }

    // 检查终点是否在障碍物内，若在则向起点方向调整（仅x、y方向）
    if (checkOccupancy(end_pt))
    {
        const Vector2d delta = start_pt - end_pt;
        if (delta.norm() < 1e-6) {
            std::cout << "[调整A*终点] 起终点过近，无法生成有效调整方向，直接返回失败。" << std::endl;
            return false;
        }
        Vector2d dir = delta.normalized();
        int adjust_steps = 0;
        while (checkOccupancy(end_pt))
        {
            end_pt += dir * step_size_;
            adjusted_end = true;
            if (!Coord2Index(end_pt, end_idx))
                return false;
            if (++adjust_steps >= kMaxAdjustSteps) {
                std::cout << "[调整A*终点] 超过最大调整步数(" << kMaxAdjustSteps << ")，放弃本次搜索。" << std::endl;
                return false;
            }
        }
    }

    // 分情况打印调整结果，明确“因在障碍物内而被调整”的原因。
    if (adjusted_start && adjusted_end) {
        std::cout << "[调整A*起终点] 由于起点和终点在障碍物内，所以调整起点为("
                  << start_pt.x() << ", " << start_pt.y() << ")，终点为("
                  << end_pt.x() << ", " << end_pt.y() << ")。原始起点=("
                  << start_pt_raw.x() << ", " << start_pt_raw.y() << ")，原始终点=("
                  << end_pt_raw.x() << ", " << end_pt_raw.y() << ")" << std::endl;
    } else if (adjusted_start) {
        std::cout << "[调整A*起点] 由于起点在障碍物内，所以调整起点为("
                  << start_pt.x() << ", " << start_pt.y() << ")。原始起点=("
                  << start_pt_raw.x() << ", " << start_pt_raw.y() << ")" << std::endl;
    } else if (adjusted_end) {
        std::cout << "[调整A*终点] 由于终点在障碍物内，所以调整终点为("
                  << end_pt.x() << ", " << end_pt.y() << ")。原始终点=("
                  << end_pt_raw.x() << ", " << end_pt_raw.y() << ")" << std::endl;
    }

    return true;
}

// 二维A*搜索主函数
bool AStar::AstarSearch(const double step_size, Vector2d start_pt, Vector2d end_pt)
{
    ++rounds_;  // 搜索轮次计数（用于节点状态重置）

    step_size_ = step_size;
    inv_step_size_ = 1.0 / step_size;
    center_ = (start_pt + end_pt) / 2.0;  // 中心位置（保留z但不影响二维搜索）

    Vector2i start_idx, end_idx;  // 二维索引
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        printf("无法处理起点或终点，强制返回！\n");
        return false;
    }

    // 获取起点和终点的节点指针（二维索引）
    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)];

    // 清空开放集
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = nullptr;
    GridNodePtr current = nullptr;

    // 初始化起点节点
    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0.0;
    startPtr->fScore = getHeu(startPtr, endPtr);  // 启发函数（根据配置选择）
    startPtr->state = GridNode::OPENSET;
    startPtr->cameFrom = nullptr;
    openSet_.push(startPtr);

    endPtr->index = end_idx;  // 初始化终点索引

    double tentative_gScore;  // 临时g值（代价）
    int num_iter = 0;         // 迭代计数

    while (!openSet_.empty())
    {
        ++num_iter;
        current = openSet_.top();  // 取出f值最小的节点
        openSet_.pop();

        // 检查是否到达终点（仅x、y维度）
        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1))
        {
            gridPath_ = retrievePath(current);  // 回溯路径
            return true;
        }

        current->state = GridNode::CLOSEDSET;  // 移至关闭集

        // 生成二维邻居节点（x和y方向偏移：-1,0,1，排除自身）
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                if (dx == 0 && dy == 0)  // 跳过当前节点
                    continue;

                // 计算邻居节点索引（二维）
                Vector2i neighborIdx;
                neighborIdx(0) = current->index(0) + dx;
                neighborIdx(1) = current->index(1) + dy;

                // 检查邻居是否在网格边界内（仅x、y维度）
                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 ||
                    neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1)
                {
                    continue;
                }

                // 获取邻居节点指针
                neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)];
                neighborPtr->index = neighborIdx;

                // 检查是否已在当前轮次处理过
                bool flag_explored = (neighborPtr->rounds == rounds_);

                // 若在关闭集则跳过
                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    continue;

                // 标记当前轮次
                neighborPtr->rounds = rounds_;

                // 检查邻居是否为障碍物
                if (checkOccupancy(Index2Coord(neighborPtr->index)))
                    continue;

                // 计算移动代价（二维欧氏距离：dx和dy的平方和开方）
                double static_cost = sqrt(dx*dx + dy*dy);
                tentative_gScore = current->gScore + static_cost;

                if (!flag_explored)  // 新节点：加入开放集
                {
                    neighborPtr->state = GridNode::OPENSET;
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    openSet_.push(neighborPtr);
                }
                else if (tentative_gScore < neighborPtr->gScore)  // 已在开放集：更新代价
                {
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                }
            }
        }
    }

    // 开放集为空且未找到终点
    return false;
}

// 获取最终路径（转换为坐标）
vector<Vector2d> AStar::getPath()
{
    vector<Vector2d> path;

    for (auto ptr : gridPath_)
    {
        Vector2d coord = Index2Coord(ptr->index);  // 二维索引转坐标（z可设为0或保留原值）
        path.push_back(coord);
    }

    reverse(path.begin(), path.end());  // 反转路径（从起点到终点）
    return path;
}

vector<Vector3d> AStar::get3DPath()
{
    vector<Vector3d> path;
    Eigen::Vector3d pos;
    Eigen::Vector2d pos2d;
    for (auto ptr : gridPath_)
    {
        pos2d = Index2Coord(ptr->index);
        pos<<pos2d(0),pos2d(1),0,
        std::cout << "pos2d:" << pos2d << std::endl;
        path.push_back(pos);
    }
    reverse(path.begin(), path.end());
    return path;
}
