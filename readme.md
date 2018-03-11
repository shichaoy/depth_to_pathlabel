This package is used for automatically selecting best offline path based on depth image. The path label can be further used for deep network training to avoid the tedious human demonstration.


**Authors:** [Shichao Yang](http://www.frc.ri.cmu.edu/~syang/),  shichaoy@andrew.cmu.edu

**Related Paper:**

* **Obstacle Avoidance through Deep Networks based Intermediate Perception**, S.Yang, S.Konam, C.Ma, S.Rosenthal, M.Veloso, S.Scherer  [**PDF**](https://arxiv.org/pdf/1704.08759.pdf)


### Usage
* **/saved_dataset_label** stores our generated path label for NYUv2 dataset and Ram-lab dataset mentioned in the paper.

* **demo_get_path_from_depth.m** is example code to generate path label.

* **/path_library** stores the five offline paths for selection.
