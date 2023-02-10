//
// Created by xiang on 22-12-7.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "frontend.h"

// 测试前端的工作情况

DEFINE_string(config_yaml, "./config/mapping.yaml", "配置文件");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    LOG(INFO) << "testing frontend";
    sad::Frontend frontend(FLAGS_config_yaml);
    if (!frontend.Init()) {
        LOG(ERROR) << "failed to init frontend.";
        return -1;
    }

    frontend.Run();
    return 0;
}