Image only method for data colleciton.
Require robot move continuously and smoothly.

## pack original data to tfrecords format

`cd python_visual_mpc/misc`
`python file_2_record.py <save tfrecords path> <original data path> 64 --traj_per_file 128
`

**Example**
`python file_2_record.py ../tf_records/ /home/steven/Project/graduate_design/vp_based_control/data_collection/sawyer_robot/autograsp_env/sudri/train/ 64 --traj_per_file 128
`
