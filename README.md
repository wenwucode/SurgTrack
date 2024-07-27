# SurgTrack: CAD-Free 3D Tracking of Real-world Surgical Instruments

<div align=center>
<img src="./docs/framework.png"> 
</div>

## Instrument3D
- We collected a data set of ultrasound instruments with aruco markers in a real operating room environment- [Instrument3D](https://cashkisi-my.sharepoint.com/:f:/g/personal/wenwu_guo_cair-cas_org_hk/EvoUjqpwUVpHql0YKIry9H0BKPEaAiiczA5jM-aZeUJ-KQ?e=wuapkx). You can use the toolkit below to create the data set.

## Object Dataset Tools
This tool is used to create object masks, bounding box labels, and pose annotations for object sequences captured by an RGB-D camera. You can following this [method.md](docs/method.md) to make your own 6DoF dataset.

## Installation
  ```
  cd docker/
  docker pull gww106/sugicaltrack:tagname
  bash docker/run_container.sh
  bash build_all.sh
  docker exec -it sugicaltrack bash
```

## Checkpoints
- Download pretrained [weights of segment and match network](https://cashkisi-my.sharepoint.com/:f:/g/personal/wenwu_guo_cair-cas_org_hk/Ev56IHMxC21AlWYaljNx8OcBrjiXhk8jknroISds10LM5A?e=OVzLpK), and put it under `./checkpoints/`


# Inference
- Prepare your RGBD image folder as below.
```
root
  ├──rgb/    (PNG files)
  ├──depth/  (PNG files, stored in mm, uint16 format.)
  ├──masks/       (PNG files, 0 is background.)
  └──cam_K.txt   (3x3)
```
- Run your RGBD video. There are 3 steps.
```
# 1) Tracking and reconstruction
python run_custom.py --mode run_video

# 2) Global refinement 
python run_custom.py --mode global_refine

# 3) (Optional) Visualization
python run_custom.py --mode draw_pose
```




## Citation

If you use our code or paper in your work, please cite our paper.
```
@inproceedings{
  title = {SurgTrack: 3D Tracking of Real-world Surgical Instruments},
}

```
