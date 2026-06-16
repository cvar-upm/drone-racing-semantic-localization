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

- `scripts/` — dataset/rosbag conversion, calibration, trajectory evaluation (evo, RPE/APE), plotting.
- `docs/` — figures and notes.
- `config/` — experiment configuration.
- `data/` — datasets and rosbags (gitignored).

## 🚀 Getting started

```sh
pixi install

# Full experiment pipeline — SLAM node (+ RViz) replaying a recorded bag:
pixi run experiment bag:=/path/to/rosbag use_sim_time:=true

# The node writes slam_*.csv to the working directory; convert + evaluate:
python scripts/csv_to_tum.py ...   # SLAM CSV -> TUM trajectory
bash scripts/run_evo.sh ...        # APE / RPE via evo
```
