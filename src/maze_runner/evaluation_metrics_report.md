# Final Navigation & Detection Evaluation Report

## 5.1.1 Path Quality Metrics
* **Path Length**: 288.25 meters (Euclidean trajectory tracking)
* **Smoothness**: 1327.00 radians (Total turning angles)
* **Safety**: 0.12 meters (Minimum clearance from obstacles)

## 5.1.2 Algorithm Performance Metrics
* **Computational Efficiency**: [请手动填写计算时间] seconds/plan ([请手动填写搜索节点数] search nodes explored)
* **Success Rate**: 100%
* **Suboptimality Ratio**: 288.25 (Estimated vs Direct distance)
* **Robustness**: High (Local minima recoveries: 32, Stuck count: 1)

## 5.1.3 Dynamic Environment Metrics
* **Replanning Frequency**: 0.000 Hz (Total 0 replans)
* **Adaptability**: Avoided 0 obstacles via Wall-Following
* **Real-time Performance**: Avg Planning Latency: [请手动填写延迟] ms (Constraint < 100ms)

## 5.2 Detection Success Rate
* **Rate**: 375.0% (Found 15 / 4 expected targets)

## 5.3 Average Response Time
* **Avg Time per Target**: 273.56 seconds
* **Total Mission Time**: 4103.37 seconds
