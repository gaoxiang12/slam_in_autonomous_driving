//
// Created by xiang on 22-12-19.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include "loopclosure.h"

DEFINE_string(config_yaml, "./config/mapping.yaml", "配置文件");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::LoopClosure lc(FLAGS_config_yaml);
    lc.Init();
    lc.Run();

    return 0;
}