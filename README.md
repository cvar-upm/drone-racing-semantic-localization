# Semantic Localization for Autonomous Drone Racing

This repository contains the code for the paper "Dual Pose-Graph Semantic Localization for Vision-Based Autonomous Drone Racing".
<!-- https://arxiv.org/abs/2604.15168 -->

## 🎥 Videos

### Abu Dhabi Autonomous Racing League (A2RL) 2025 Results

[![A2RL competition results](docs/images/video_thumbnail_a2rl.jpg)](https://vimeo.com/1079143067)

## 📖 Paper <a id="published-papers"></a>

<details>
<summary><a href="https://arxiv.org/abs/2604.15168">
Dual Pose-Graph Semantic Localization for Vision-Based Autonomous Drone Racing
</a></summary>

```bibtex
@misc{perezsaura2026dualposegraphsemanticlocalization,
  title={Dual Pose-Graph Semantic Localization for Vision-Based Autonomous Drone Racing}, 
  author={David Perez-Saura and Miguel Fernandez-Cortizas and Alvaro J. Gaona and Pascual Campoy},
  year={2026},
  eprint={2604.15168},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2604.15168},
}
```

</details>
<!-- This paper has been accepted to the IEEE International Workshop on Metrology for Aerospace (MetroAeroSpace) 2026. -->

## 🔗 Links

- **Pose-graph SLAM library (`dual_pose_graph`):**  
  https://github.com/alvgaona/dual-pose-graph (conda: https://prefix.dev/channels/dual-pose-graph)

- **ROS 2 wrapper (`SemanticSlam`):**  
  https://github.com/alvgaona/SemanticSlam

- **Benchmark Dataset (TII RATM ROS 2 Bags):**  
  https://huggingface.co/datasets/alvgaona/tii-ratm-rosbag2

## 🧩 Components

The system spans three repositories; this one is the umbrella that ties them together.

- **[`dual_pose_graph`](https://github.com/alvgaona/dual-pose-graph)** — generic C++ dual pose-graph
  SLAM library (no ROS), published as a conda package on the `dual-pose-graph` prefix.dev channel.
- **[`SemanticSlam`](https://github.com/alvgaona/SemanticSlam)** — ROS 2 wrapper exposing the library
  as the `dual_pose_graph_node`.
- **this repo** — experiments, evaluation and reproducibility tooling.

## 🗂️ Repository layout

- `scripts/` — dataset/rosbag conversion, calibration, trajectory + gate-map evaluation, plotting.
- `docs/` — figures and notes.
- `config/` — experiment configuration.
- `data/` — datasets and rosbags (gitignored).

## 🚀 Getting started

```sh
pixi install

# Full experiment pipeline — SLAM node (+ RViz) replaying a recorded bag:
pixi run experiment bag:=/path/to/rosbag use_sim_time:=true

# The node writes slam_*.csv + a copy of the config into runs/<bag-name>/. Extract GT and evaluate:
python scripts/extract_ground_truth.py --bag /path/to/rosbag --output runs/<bag>/slam_ground_truth.csv
python scripts/evaluate_slam.py --log-dir runs/<bag> --align --origin-align    # trajectory RMSE
python scripts/extract_gates_gt.py --bag /path/to/rosbag --output runs/<bag>/gates_ground_truth.csv
python scripts/evaluate_gates.py --estimated runs/<bag>/slam_estimated_objects.csv \
    --true-csv runs/<bag>/gates_ground_truth.csv                                # gate-pose error vs GT
```
