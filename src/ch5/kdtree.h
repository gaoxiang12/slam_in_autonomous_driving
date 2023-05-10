//
// Created by xiang on 2021/9/22.
//

#ifndef SLAM_IN_AUTO_DRIVING_KDTREE_H
#define SLAM_IN_AUTO_DRIVING_KDTREE_H

#include "common/eigen_types.h"
#include "common/point_types.h"

#include <glog/logging.h>
#include <map>
#include <queue>

namespace sad {

/// Kd树节点，二叉树结构，内部用祼指针，对外一个root的shared_ptr
struct KdTreeNode {
    int id_ = -1;
    int point_idx_ = 0;            // 点的索引
    int axis_index_ = 0;           // 分割轴
    float split_thresh_ = 0.0;     // 分割位置
    KdTreeNode* left_ = nullptr;   // 左子树
    KdTreeNode* right_ = nullptr;  // 右子树

    bool IsLeaf() const { return left_ == nullptr && right_ == nullptr; }  // 是否为叶子
};

/// 用于记录knn结果
struct NodeAndDistance {
    NodeAndDistance(KdTreeNode* node, float dis2) : node_(node), distance2_(dis2) {}
    KdTreeNode* node_ = nullptr;
    float distance2_ = 0;  // 平方距离，用于比较

    bool operator<(const NodeAndDistance& other) const { return distance2_ < other.distance2_; }
};

/**
 * 手写kd树
 * 测试这个kd树的召回!
 */
class KdTree {
   public:
    explicit KdTree() = default;
    ~KdTree() { Clear(); }

    bool BuildTree(const CloudPtr& cloud);

    /// 获取k最近邻
    bool GetClosestPoint(const PointType& pt, std::vector<int>& closest_idx, int k = 5);

    /// 并行为点云寻找最近邻
    bool GetClosestPointMT(const CloudPtr& cloud, std::vector<std::pair<size_t, size_t>>& matches, int k = 5);

    /// 这个被用于计算最近邻的倍数
    void SetEnableANN(bool use_ann = true, float alpha = 0.1) {
        approximate_ = use_ann;
        alpha_ = alpha;
    }

    /// 返回节点数量
    size_t size() const { return size_; }

    /// 清理数据
    void Clear();

    /// 打印所有节点信息
    void PrintAll();

   private:
    /// kdtree 构建相关
    /**
     * 在node处插入点
     * @param points
     * @param node
     */
    void Insert(const IndexVec& points, KdTreeNode* node);

    /**
     * 计算点集的分割面
     * @param points 输入点云
     * @param axis   轴
     * @param th     阈值
     * @param left   左子树
     * @param right  右子树
     * @return
     */
    bool FindSplitAxisAndThresh(const IndexVec& point_idx, int& axis, float& th, IndexVec& left, IndexVec& right);

    void Reset();

    /// 两个点的平方距离
    static inline float Dis2(const Vec3f& p1, const Vec3f& p2) { return (p1 - p2).squaredNorm(); }

    // Knn 相关
    /**
     * 检查给定点在kdtree node上的knn，可以递归调用
     * @param pt     查询点
     * @param node   kdtree 节点
     */
    void Knn(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;

    /**
     * 对叶子节点，计算它和查询点的距离，尝试放入结果中
     * @param pt    查询点
     * @param node  Kdtree 节点
     */
    void ComputeDisForLeaf(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& result) const;

    /**
     * 检查node下是否需要展开
     * @param pt   查询点
     * @param node Kdtree 节点
     * @return true if 需要展开
     */
    bool NeedExpand(const Vec3f& pt, KdTreeNode* node, std::priority_queue<NodeAndDistance>& knn_result) const;

    int k_ = 5;                                   // knn最近邻数量
    std::shared_ptr<KdTreeNode> root_ = nullptr;  // 根节点
    std::vector<Vec3f> cloud_;                    // 输入点云
    std::unordered_map<int, KdTreeNode*> nodes_;  // for bookkeeping

    size_t size_ = 0;       // 叶子节点数量
    int tree_node_id_ = 0;  // 为kdtree node 分配id

    // 近似最近邻
    bool approximate_ = true;
    float alpha_ = 0.1;
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_KDTREE_H
