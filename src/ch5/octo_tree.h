//
// Created by xiang on 2021/9/27.
//

#ifndef SLAM_IN_AUTO_DRIVING_OCTO_TREE_H
#define SLAM_IN_AUTO_DRIVING_OCTO_TREE_H

#include "common/eigen_types.h"
#include "common/point_types.h"

#include <glog/logging.h>
#include <map>
#include <queue>

namespace sad {

// 3D Box 记录各轴上的最大最小值
struct Box3D {
    Box3D() = default;
    Box3D(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z)
        : min_{min_x, min_y, min_z}, max_{max_x, max_y, max_z} {}

    /// 判断pt是否在内部
    bool Inside(const Vec3f& pt) const {
        return pt[0] <= max_[0] && pt[0] >= min_[0] && pt[1] <= max_[1] && pt[1] >= min_[1] && pt[2] <= max_[2] &&
               pt[2] >= min_[2];
    }

    /// 点到3D Box距离
    /// 我们取外侧点到边界的最大值
    float Dis(const Vec3f& pt) const {
        float ret = 0;
        for (int i = 0; i < 3; ++i) {
            if (pt[i] < min_[i]) {
                float d = min_[i] - pt[i];
                ret = d > ret ? d : ret;
            } else if (pt[i] > max_[i]) {
                float d = pt[i] - max_[i];
                ret = d > ret ? d : ret;
            }
        }

        assert(ret >= 0);
        return ret;
    }

    float min_[3] = {0};
    float max_[3] = {0};
};

/// octo tree 节点
struct OctoTreeNode {
    int id_ = -1;
    int point_idx_ = -1;                    // 点的索引，-1为无效
    Box3D box_;                             // 边界框
    OctoTreeNode* children[8] = {nullptr};  // 子节点

    bool IsLeaf() const {
        for (const OctoTreeNode* n : children) {
            if (n != nullptr) {
                return false;
            }
        }
        return true;
    }
};

/// 用于记录knn结果
struct NodeAndDistanceOcto {
    NodeAndDistanceOcto(OctoTreeNode* node, float dis2) : node_(node), distance_(dis2) {}
    OctoTreeNode* node_ = nullptr;
    float distance_ = 0;  // 平方距离，用于比较

    bool operator<(const NodeAndDistanceOcto& other) const { return distance_ < other.distance_; }
};

class OctoTree {
   public:
    explicit OctoTree() = default;
    ~OctoTree() { Clear(); }

    bool BuildTree(const CloudPtr& cloud);

    /// 获取k最近邻
    bool GetClosestPoint(const PointType& pt, std::vector<int>& closest_idx, int k = 5) const;

    /// 并行为点云寻找最近邻
    bool GetClosestPointMT(const CloudPtr& cloud, std::vector<std::pair<size_t, size_t>>& match, int k = 5);

    /// 设置近似最近邻参数
    void SetApproximate(bool use_ann = true, float alpha = 0.1) {
        approximate_ = use_ann;
        alpha_ = alpha;
    }

    /// 返回节点数量
    size_t size() const { return size_; }

    /// 清理数据
    void Clear();

   private:
    /// kdtree 构建相关
    /**
     * 在node处插入点
     * @param points
     * @param node
     */
    void Insert(const IndexVec& points, OctoTreeNode* node);

    /// 为全局点云生成边界框
    Box3D ComputeBoundingBox();

    /**
     * 展开一个节点
     * @param [in] node 被展开的节点
     * @param [in] parent_idx 父节点的点云索引
     * @param [out] children_idx 子节点的点云索引
     */
    void ExpandNode(OctoTreeNode* node, const IndexVec& parent_idx, std::vector<IndexVec>& children_idx);

    void Reset();

    /// 两个点的平方距离
    static inline float Dis2(const Vec3f& p1, const Vec3f& p2) { return (p1 - p2).squaredNorm(); }

    // Knn 相关
    /**
     * 检查给定点在kdtree node上的knn，可以递归调用
     * @param pt     查询点
     * @param node   kdtree 节点
     */
    void Knn(const Vec3f& pt, OctoTreeNode* node, std::priority_queue<NodeAndDistanceOcto>& result) const;

    /**
     * 对叶子节点，计算它和查询点的距离，尝试放入结果中
     * @param pt    查询点
     * @param node  Kdtree 节点
     */
    void ComputeDisForLeaf(const Vec3f& pt, OctoTreeNode* node, std::priority_queue<NodeAndDistanceOcto>& result) const;

    /**
     * 检查node下是否需要展开
     * @param pt   查询点
     * @param node Kdtree 节点
     * @return true if 需要展开
     */
    bool NeedExpand(const Vec3f& pt, OctoTreeNode* node, std::priority_queue<NodeAndDistanceOcto>& knn_result) const;

    int k_ = 5;                                     // knn最近邻数量
    std::shared_ptr<OctoTreeNode> root_ = nullptr;  // 叶子节点
    std::vector<Vec3f> cloud_;                      // 输入点云
    std::map<int, OctoTreeNode*> nodes_;            // for bookkeeping
    size_t size_ = 0;                               // 叶子节点数量
    int tree_node_id_ = 0;                          // 为kdtree node 分配id

    // flann
    bool approximate_ = false;
    float alpha_ = 1.0;  // flann的距离倍数
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_OCTO_TREE_H
