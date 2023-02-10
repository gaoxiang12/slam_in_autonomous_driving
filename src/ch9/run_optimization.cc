//
// Created by xiang on 22-12-7.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "optimization.h"

// 测试优化的工作情况

DEFINE_string(config_yaml, "./config/mapping.yaml", "配置文件");
DEFINE_int64(stage, 1, "运行第1阶段或第2阶段优化");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    assert(FLAGS_stage == 1 || FLAGS_stage == 2);

    LOG(INFO) << "testing optimization";
    sad::Optimization opti(FLAGS_config_yaml);
    if (!opti.Init(FLAGS_stage)) {
        LOG(ERROR) << "failed to init frontend.";
        return -1;
    }

    opti.Run();
    return 0;
}