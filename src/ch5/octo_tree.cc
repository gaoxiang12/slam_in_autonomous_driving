//
// Created by xiang on 2021/9/27.
//

#include "octo_tree.h"
#include "common/math_utils.h"

#include <execution>

namespace sad {

bool OctoTree::BuildTree(const CloudPtr &cloud) {
    if (cloud->empty()) {
        return false;
    }

    cloud_.clear();
    cloud_.resize(cloud->size());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud_[i] = ToVec3f(cloud->points[i]);
    }

    Clear();
    Reset();

    IndexVec idx(cloud->size());
    for (int i = 0; i < cloud->points.size(); ++i) {
        idx[i] = i;
    }

    // 生成根节点的边界框
    root_->box_ = ComputeBoundingBox();
    Insert(idx, root_.get());

    return true;
}

void OctoTree::Insert(const IndexVec &points, OctoTreeNode *node) {
    nodes_.insert({node->id_, node});

    if (points.empty()) {
        return;
    }

    if (points.size() == 1) {
        size_++;
        node->point_idx_ = points[0];
        return;
    }

    /// 只要点数不为1,就继续展开这个节点
    std::vector<IndexVec> children_points;
    ExpandNode(node, points, children_points);

    /// 对子节点进行插入操作
    for (size_t i = 0; i < 8; ++i) {
        Insert(children_points[i], node->children[i]);
    }
}

void OctoTree::ExpandNode(OctoTreeNode *node, const IndexVec &parent_idx, std::vector<IndexVec> &children_idx) {
    children_idx.resize(8);
    for (int i = 0; i < 8; ++i) {
        node->children[i] = new OctoTreeNode();
        node->children[i]->id_ = tree_node_id_++;
    }

    const Box3D &b = node->box_;  // 本节点的box
    // 中心点
    float c_x = 0.5 * (node->box_.min_[0] + node->box_.max_[0]);
    float c_y = 0.5 * (node->box_.min_[1] + node->box_.max_[1]);
    float c_z = 0.5 * (node->box_.min_[2] + node->box_.max_[2]);

    // 8个外框示意图
    // clang-format off
    // 第一层：左上1 右上2 左下3 右下4
    // 第二层：左上5 右上6 左下7 右下8
    //     ---> x    /-------/-------/|
    //    /|        /-------/-------/||
    //   / |       /-------/-------/ ||
    //  y  |z      |       |       | /|
    //             |_______|_______|/|/
    //             |       |       | /
    //             |_______|_______|/
    // clang-format on
    node->children[0]->box_ = {b.min_[0], c_x, b.min_[1], c_y, b.min_[2], c_z};
    node->children[1]->box_ = {c_x, b.max_[0], b.min_[1], c_y, b.min_[2], c_z};
    node->children[2]->box_ = {b.min_[0], c_x, c_y, b.max_[1], b.min_[2], c_z};
    node->children[3]->box_ = {c_x, b.max_[0], c_y, b.max_[1], b.min_[2], c_z};

    node->children[4]->box_ = {b.min_[0], c_x, b.min_[1], c_y, c_z, b.max_[2]};
    node->children[5]->box_ = {c_x, b.max_[0], b.min_[1], c_y, c_z, b.max_[2]};
    node->children[6]->box_ = {b.min_[0], c_x, c_y, b.max_[1], c_z, b.max_[2]};
    node->children[7]->box_ = {c_x, b.max_[0], c_y, b.max_[1], c_z, b.max_[2]};

    // 把点云归到子节点中
    for (int idx : parent_idx) {
        const auto pt = cloud_[idx];
        for (int i = 0; i < 8; ++i) {
            if (node->children[i]->box_.Inside(pt)) {
                children_idx[i].emplace_back(idx);
                break;
            }
        }
    }
}

Box3D OctoTree::ComputeBoundingBox() {
    float min_values[3] = {std::numeric_limits<float>::max()};
    float max_values[3] = {-std::numeric_limits<float>::max()};

    for (const auto &p : cloud_) {
        for (int i = 0; i < 3; ++i) {
            max_values[i] = p[i] > max_values[i] ? p[i] : max_values[i];
            min_values[i] = p[i] < min_values[i] ? p[i] : min_values[i];
        }
    }

    return {min_values[0], max_values[0], min_values[1], max_values[1], min_values[2], max_values[2]};
}

bool OctoTree::GetClosestPoint(const PointType &pt, std::vector<int> &closest_idx, int k) const {
    if (k > size_) {
        LOG(ERROR) << "cannot set k larger than cloud size: " << k << ", " << size_;
        return false;
    }

    std::priority_queue<NodeAndDistanceOcto> knn_result;
    Knn(ToVec3f(pt), root_.get(), knn_result);

    // 排序并返回结果
    closest_idx.resize(knn_result.size());
    for (int i = closest_idx.size() - 1; i >= 0; --i) {
        // 倒序插入
        closest_idx[i] = knn_result.top().node_->point_idx_;
        knn_result.pop();
    }
    return true;
}

bool OctoTree::GetClosestPointMT(const CloudPtr &cloud, std::vector<std::pair<size_t, size_t>> &matches, int k) {
    k_ = k;
    matches.resize(cloud->size() * k);
    // 索引
    std::vector<int> index(cloud->size());
    for (int i = 0; i < cloud->points.size(); ++i) {
        index[i] = i;
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [this, &cloud, &matches, &k](int idx) {
        std::vector<int> closest_idx;
        GetClosestPoint(cloud->points[idx], closest_idx, k);

        for (int i = 0; i < k; ++i) {
            matches[idx * k + i].second = idx;
            if (i < closest_idx.size()) {
                matches[idx * k + i].first = closest_idx[i];
            } else {
                matches[idx * k + i].first = math::kINVALID_ID;
            }
        }
    });

    return true;
}

void OctoTree::Clear() {
    for (const auto &np : nodes_) {
        if (np.second != root_.get()) {
            delete np.second;
        }
    }

    root_ = nullptr;
    size_ = 0;
    tree_node_id_ = 0;
}

void OctoTree::Reset() {
    tree_node_id_ = 0;
    root_.reset(new OctoTreeNode());
    root_->id_ = tree_node_id_++;
    size_ = 0;
}

void OctoTree::Knn(const Vec3f &pt, OctoTreeNode *node, std::priority_queue<NodeAndDistanceOcto> &knn_result) const {
    if (node->IsLeaf()) {
        if (node->point_idx_ != -1) {
            // 如果是叶子，看该点是否为最近邻
            ComputeDisForLeaf(pt, node, knn_result);
            return;
        }
        return;
    }

    // 看pt落在哪一格，优先搜索pt所在的子树
    // 然后再看其他子树是否需要搜索
    // 如果pt在外边，优先搜索最近的子树
    int idx_child = -1;
    float min_dis = std::numeric_limits<float>::max();
    for (int i = 0; i < 8; ++i) {
        if (node->children[i]->box_.Inside(pt)) {
            idx_child = i;
            break;
        } else {
            float d = node->children[i]->box_.Dis(pt);
            if (d < min_dis) {
                idx_child = i;
                min_dis = d;
            }
        }
    }

    // 先检查idx_child
    Knn(pt, node->children[idx_child], knn_result);

    // 再检查其他的
    for (int i = 0; i < 8; ++i) {
        if (i == idx_child) {
            continue;
        }

        if (NeedExpand(pt, node->children[i], knn_result)) {
            Knn(pt, node->children[i], knn_result);
        }
    }
}

void OctoTree::ComputeDisForLeaf(const Vec3f &pt, OctoTreeNode *node,
                                 std::priority_queue<NodeAndDistanceOcto> &knn_result) const {
    // 比较与结果队列的差异，如果优于最远距离，则插入
    float dis2 = Dis2(pt, cloud_[node->point_idx_]);
    if (knn_result.size() < k_) {
        // results 不足k
        knn_result.push({node, dis2});
    } else {
        // results等于k，比较current与max_dis_iter之间的差异
        if (dis2 < knn_result.top().distance_) {
            knn_result.push({node, dis2});
            knn_result.pop();
        }
    }
}

bool OctoTree::NeedExpand(const Vec3f &pt, OctoTreeNode *node,
                          std::priority_queue<NodeAndDistanceOcto> &knn_result) const {
    if (knn_result.size() < k_) {
        return true;
    }

    if (approximate_) {
        float d = node->box_.Dis(pt);
        if ((d * d) < knn_result.top().distance_ * alpha_) {
            return true;
        } else {
            return false;
        }
    } else {
        // 不用flann时，按通常情况查找
        float d = node->box_.Dis(pt);
        if ((d * d) < knn_result.top().distance_) {
            return true;
        } else {
            return false;
        }
    }
}

}  // namespace sad