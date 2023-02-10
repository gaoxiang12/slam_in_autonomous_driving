#ifndef TFUSION_UI_IMAGE_H
#define TFUSION_UI_IMAGE_H

#include <glog/logging.h>
#include <array>

namespace sad::ui {

struct UiColorRgb {
    uint8_t r, g, b;
    UiColorRgb(const uint8_t &rr, const uint8_t &gg, const uint8_t &bb) : r(rr), g(gg), b(bb) {}

    static inline const UiColorRgb RED() { return {255, 0, 0}; }
    static inline const UiColorRgb GREEN() { return {0, 255, 0}; }
    static inline const UiColorRgb BLUE() { return {0, 0, 255}; }
    static inline const UiColorRgb WHITE() { return {255, 255, 255}; }
};

struct UiPoint {
    int u, v;
    UiPoint(const int &uu, const int &vv) : u(uu), v(vv) {}
};

template <int resolution, int ui_rows_, int ui_cols_>
class UiImageRgb {
   public:
    static constexpr int ui_rows = ui_rows_;
    static constexpr int ui_cols = ui_cols_;
    static constexpr int img_rows = resolution * ui_rows_;
    static constexpr int img_cols = resolution * ui_cols_;
    static constexpr int img_channels = 3;
    static constexpr int img_size = img_channels * img_cols * img_rows;
    std::array<uint8_t, 3 * resolution * ui_rows_ * resolution * ui_cols_> array_;

    /**
     * @brief 画雷达/视觉定位时，栅格搜索的得分和最佳得分位置
     * @param loc_scores 搜索得分
     * @param best_index 最佳得分的索引
     * @param img_y_up 是否上下翻转
     */
    void DrawLocScore(const std::vector<double> &loc_scores, const std::size_t &best_index, const bool &img_y_up) {
        if (loc_scores.size() == 0) {
            return;
        }
        assert(loc_scores.size() % ui_cols == 0);
        const std::size_t step = ui_cols;

        // draw rectangle for each score
        const auto max_score = *std::max_element(loc_scores.begin(), loc_scores.end());
        for (int i = 0; i < loc_scores.size(); ++i) {
            int x = i % step;
            int y = i / step;
            if (!img_y_up) {
                y = step - 1 - y;  // opengl坐标是从下往上的，与opencv相反
            }

            double sc = 1 - loc_scores[i] / max_score;
            sc = sc < 0 ? 0 : sc;
            sc = sc > 1 ? 1 : sc;

            DrawRect(x, y, UiColorRgb{0, 0, (unsigned char)(sc * 255)}, true);
        }

        // draw rectangle for best grid
        if (best_index >= 0 && best_index < loc_scores.size()) {
            int x = best_index % step;
            int y = best_index / step;
            if (!img_y_up) {
                y = step - 1 - y;  // opengl坐标是从下往上的，与opencv相反
            }

            DrawRect(x, y, UiColorRgb::WHITE(), false);
        } else {
            LOG(ERROR) << "[ui] bad best score!";
        }
    }

    /// @brief 更新pgo图优化中的图
    void DrawPgoGraph(const Eigen::MatrixXf &vertexes, const bool &img_y_up) {
        // rgb
        static const UiColorRgb color_opti{250, 128, 114};    // 粉红
        static const UiColorRgb color_lidar{255, 250, 205};   // 米红
        static const UiColorRgb color_gps{30, 144, 255};      // 蓝
        static const UiColorRgb color_visual{255, 250, 205};  // 米红（与雷达颜色一致）

        enum class VertexStatus { Invalid = -1, NotSet = 0, Valid = 1 };

        array_.fill((unsigned char)50);  // 重置成灰色

        // 更新pgo的图
        const int cols = vertexes.cols();

        Vec2f center = vertexes.block<2, 1>(0, cols - 1).eval();
        for (int i = 0; i < cols; ++i) {
            Vec2f xy = vertexes.block<2, 1>(0, i) - center;
            Vec2f xy_gps = vertexes.block<2, 1>(4, i) - center;
            Vec2f xy_lidar = vertexes.block<2, 1>(8, i) - center;
            Vec2f xy_visual = vertexes.block<2, 1>(12, i) - center;
            VertexStatus gps_status = static_cast<VertexStatus>(vertexes(7, i));
            VertexStatus lidar_status = static_cast<VertexStatus>(vertexes(11, i));
            VertexStatus visual_status = static_cast<VertexStatus>(vertexes(15, i));

            // // opengl和opencv的y轴朝向相反
            if (!img_y_up) {
                xy(1) *= -1.0;
                xy_gps(1) *= -1.0;
                xy_lidar(1) *= -1.0;
            }

            DrawRect(xy(0), xy(1), color_opti, false, 10);
            if (gps_status == VertexStatus::Valid) {
                DrawRect(xy_gps(0), xy_gps(1), color_gps, false, 10);
            } else if (gps_status == VertexStatus::Invalid) {
                DrawX(xy_gps(0), xy_gps(1), color_gps, 10);
            } else {
                // vertex not set, nothing to do
            }

            if (lidar_status == VertexStatus::Valid) {
                DrawRect(xy_lidar(0), xy_lidar(1), color_lidar, false, 10);
            } else if (lidar_status == VertexStatus::Invalid) {
                DrawX(xy_lidar(0), xy_lidar(1), color_lidar, 10);
            } else {
                // vertex not set, nothing to do
            }

            if (visual_status == VertexStatus::Valid) {
                DrawRect(xy_visual(0), xy_visual(1), color_visual, false, 10);
            } else if (visual_status == VertexStatus::Invalid) {
                DrawX(xy_visual(0), xy_visual(1), color_visual, 10);
            } else {
                // vertex not set, nothing to do
            }

            if (i < cols - 1) {
                Vec2f xy_next = vertexes.block<2, 1>(0, i + 1) - center;
                if (!img_y_up) {
                    xy_next(1) *= -1.0;  // opengl和opencv的y轴朝向相反
                }
                DrawLine(xy(0), xy(1), xy_next(0), xy_next(1), color_opti);
            }
        }
    }

   private:
    /// 左下角为坐标原点，画实心矩形或者矩形边框
    /// @note 矩形大小固定为resolution*resolution
    void DrawRect(const int &x, const int &y, const UiColorRgb &rgb, const bool fill) {
        UiPoint p1{x * resolution, y * resolution};
        UiPoint p2{(x + 1) * resolution, (y + 1) * resolution};
        DrawRect(p1, p2, rgb, fill);
    }

    /// 图像中央为坐标原点，画实心矩形或者矩形边框
    void DrawRect(const float &x, const float &y, const UiColorRgb &rgb, const bool fill, const int &size) {
        float center_u = 0.5f * img_cols + x * resolution;
        float center_v = 0.5f * img_rows + y * resolution;
        UiPoint p1{static_cast<int>(center_u - 0.5f * size), static_cast<int>(center_v - 0.5f * size)};
        UiPoint p2{static_cast<int>(center_u + 0.5f * size), static_cast<int>(center_v + 0.5f * size)};
        DrawRect(p1, p2, rgb, fill);
    }

    // 图像中央为坐标原点，画一把X
    void DrawX(const float &x, const float &y, const UiColorRgb &rgb, const int &size) {
        float center_u = 0.5f * img_cols + x * resolution;
        float center_v = 0.5f * img_rows + y * resolution;
        UiPoint p1{static_cast<int>(center_u - 0.5f * size), static_cast<int>(center_v - 0.5f * size)};
        UiPoint p2{static_cast<int>(center_u + 0.5f * size), static_cast<int>(center_v + 0.5f * size)};
        DrawLine(p1, p2, rgb);
        DrawLine(UiPoint(p1.u, p2.v), UiPoint(p2.u, p1.v), rgb);
    }

    // 图像中央为坐标原点，画一条线
    void DrawLine(const float &x1, const float &y1, const float &x2, const float &y2, const UiColorRgb &rgb) {
        float u1 = 0.5 * img_cols + x1 * resolution;
        float v1 = 0.5 * img_rows + y1 * resolution;
        float u2 = 0.5 * img_cols + x2 * resolution;
        float v2 = 0.5 * img_rows + y2 * resolution;
        UiPoint p1{static_cast<int>(u1), static_cast<int>(v1)};
        UiPoint p2{static_cast<int>(u2), static_cast<int>(v2)};
        DrawLine(p1, p2, rgb);
    }

   private:
    int ClipU(const int &u) {
        int res = u;
        res = res < 0 ? 0 : res;
        res = res >= img_cols ? img_cols : res;
        return res;
    }

    int ClipV(const float &v) {
        int res = v;
        res = res < 0 ? 0 : res;
        res = res >= img_rows ? img_rows : res;
        return res;
    }

    /// 点的坐标，是否在图像范围内
    bool IsPointInside(const UiPoint &p) { return p.u >= 0 && p.u < img_cols && p.v >= 0 && p.v < img_rows; }

    /// 给定左下角、右上角坐标，画矩形/矩形框
    void DrawRect(const UiPoint &p1, const UiPoint &p2, const UiColorRgb &rgb, const bool &fill) {
        if (!IsPointInside(p1) && !IsPointInside(p2)) {
            return;
        }
        if (fill) {
            for (int u = ClipU(p1.u); u <= ClipU(p2.u); ++u) {
                for (int v = ClipV(p1.v); v <= ClipV(p2.v); ++v) {
                    SetColor(u, v, rgb);
                }
            }
        } else {
            for (int u = ClipU(p1.u); u <= ClipU(p2.u); ++u) {
                SetColor(u, p1.v, rgb);
                SetColor(u, p2.v, rgb);
            }
            for (int v = ClipV(p1.v); v <= ClipV(p2.v); ++v) {
                SetColor(p1.u, v, rgb);
                SetColor(p2.u, v, rgb);
            }
        }
    }

    // 画一条线
    void DrawLine(const UiPoint &p1, const UiPoint &p2, const UiColorRgb &rgb) {
        if (!IsPointInside(p1) && !IsPointInside(p1)) {
            return;
        }

        if (p1.u == p2.u && p1.v == p2.v) {
            SetColor(p1.u, p1.v, rgb);
            return;
        }

        UiPoint q1 = p1, q2 = p2;
        if (std::abs(1.0 * q2.v - q1.v) >= std::abs(1.0 * q2.u - q1.u)) {
            if (q1.v > q2.v) {
                std::swap(q1, q2);
            }
            float k = (1.0 * q2.u - q1.u) / (1.0 * q2.v - q1.v);
            for (int v = ClipV(q1.v); v <= ClipV(q2.v); ++v) {
                int u = q1.u + k * (1.0 * v - q1.v);
                SetColor(u, v, rgb);
            }
        } else {
            if (q1.u > q2.u) {
                std::swap(q1, q2);
            }
            float k = (1.0 * q2.v - q1.v) / (1.0 * q2.u - q1.u);
            for (int u = ClipU(q1.u); u <= ClipU(q2.u); ++u) {
                int v = q1.v + k * (1.0 * u - q1.u);
                SetColor(u, v, rgb);
            }
        }
    }

    void SetColor(const int &u, const int &v, const UiColorRgb &rgb) { SetColor(u, v, rgb.r, rgb.g, rgb.b); }

    void SetColor(const int &u, const int &v, const char &r, const char &g, const char &b) {
        if (u < 0 || u >= img_cols || v < 0 || v >= img_rows) {
            return;
        }
        const int pos = img_channels * ((img_cols * v) + u);
        array_[pos + 0] = r;
        array_[pos + 1] = g;
        array_[pos + 2] = b;
    }
};

}  // namespace sad::ui

#endif