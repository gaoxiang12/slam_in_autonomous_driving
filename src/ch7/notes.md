## ch7 问题列表

- 从Db里导出来的wxb数据是关键帧的，作LO并不是很平滑，还是建议用bag里的
- bag 的话，需要加入packets转换的部分
- wxb 的bag是真的麻烦。。引入一堆自定的头文件和msg，velodyne scan也是自定的...
- 整个LO对旋转并不稳定（在没有IMU预测的情况下）
- ULHK 目前看来最简单（比较好的方法是从一个本来就可以跑的程序开始简化)
- NCLT 是个两轮平衡车采的，自转非常快，NDT LO有时候收敛不过去，没IMU时对LO不友好
- UTBM scan 自带outlier，貌似是转换过程中出了点问题
- 旋转未作去畸变处理。。本身就容易偏
- 综合看起来目前ULHK最容易跑
- loosely lio对各数据集都还行，初始化时不太稳定，个别退化场景可能飞掉，也可能吃IMU异常（再加点if-else会更好一些）

## LO 及各数据集运行状态

| 算法            | WXB  | ULHK | NCLT | UTBM |
|---------------|------|------|------|------|
| Direct NDL LO |      | Good | BAD  | GOOD |
| Inc NDT LO    | BAD  | Good |      | GOOD |
| LOAM-Like     |      |      |      |
| Loosely LIO   | Good | Good | Good | Good | 

## TODO

- done 滤波器状态的实时可视化(移植libfusion::ui的部分内容)
- done 去畸变
