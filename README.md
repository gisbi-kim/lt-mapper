# LT-mapper

<!-- ------------------------------------------ -->
## News
- ``July 2021``
  - A preprint manuscript is available ([download the preprint](./doc/ltmapper-v1.pdf)).
  - LT-SLAM module is released.
- [``The future release schedule`` is here](#release-schedule) 

<!-- ------------------------------------------ -->
## What is LT-mapper?
- A Modular Framework for LiDAR-based Lifelong Mapping

### Why LT-mapper?
- For LiDAR-based long-term mapping, three challenges exist. 
  <p align="center"><img src="doc/whyltmapper.png" width=1000></p>

### Features 
- TBA


<!-- ------------------------------------------ -->
## How to use?

### Prerequisites 
- TBA

### build 
- TBA
 
<!-- ------------------------------------------ -->
## Tutorial and examples

### 0. Single-session Data Generation 
- Using [the saver](https://github.com/gisbi-kim/SC-LIO-SAM#applications) provided with [SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM) (also in [SC-A-LOAM](https://github.com/gisbi-kim/SC-A-LOAM)  or [FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM)), a user should generate the set of sesssion data (i.e., keyframe point cloud scans, keyframe scan context descriptors (SCDs), and an initial pose-graph text file) for each session. 

### 1. LT-SLAM 
- [Tutorial video](https://youtu.be/BXBTVurNToU)
 
- command
  ```
      # change the paths in ltslam/config/params.yaml
      roslaunch ltslam run.launch
  ```

### 2. LT-removert 
- TBA

### 3. LT-map
- TBA


<!-- ------------------------------------------ -->
## LT-mapper ParkingLot dataset 
- LT-mapper ParkingLot dataset contains six sequences during three days for the same spatial site, but different initial poses. 
- LT-SLAM automatically aligns them in a shared coordinate.
  <p align="center"><img src="doc/ltparkinglot.png" width=630></p>
- [Dataset Download Link](https://bit.ly/ltmapper_parkinglot_data). A sequence is replayable using [MulRan File Player](https://github.com/irapkaist/file_player_mulran). 
  - For the details of use, see [this tutorial video (TBA)](TBA). 

<!-- ------------------------------------------ -->
## Citation
```
@article{kim2021ltmapper,
  title={{LT-mapper: A Modular Framework for LiDAR-based Lifelong Mapping}},
  author={Kim, Giseop and Kim, Ayoung},
  journal={arXiv preprint arXiv:2107.07712},
  year={2021}
}
```

<!-- ------------------------------------------ -->
## Contact 
- Maintained by Giseop Kim and please contact the author via ``paulgkim@kaist.ac.kr``


<!-- ------------------------------------------ -->
## Release schedule
- [x] ``July 2021`` A pre-print manuscript and initial version of LT-SLAM is released.
- [ ] ``By Sep 2021`` About LT-SLAM: Currently, we support 2-session alignment. N-session alignment will be supported.  
- [ ] ``By Dec 2021`` LT-removert and LT-map modules will be released.


<!-- ------------------------------------------ -->
## Acknowledgements
- TBA

