wr@wrc:~/localization_alg-stable-end$ roslaunch localization_ndt debug_amcl_localization.launch 
... logging to /home/wr/.ros/log/15b4faee-1e77-11f1-9aed-ebc34aa96048/roslaunch-wrc-143524.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://wrc:46153/

SUMMARY
========

PARAMETERS
 * /robot_description: <robot name="sens...
 * /rosdistro: noetic
 * /rosversion: 1.17.4
 * /use_sim_time: True

NODES
  /
    amcl_localizer (localization_ndt/amcl_localizer_node)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)

ROS_MASTER_URI=http://localhost:11311

process[robot_state_publisher-1]: started with pid [143538]
process[amcl_localizer-2]: started with pid [143539]
[INFO] [1773386964.630476490]: [LOC] InitialPoseManager: fixed=/home/wr/localization_alg-stable-end/src/localization_ndt/config/initial_pose/fixed/abtr_ref.yaml (#poses=2), last(stable)=/home/wr/localization_alg-stable-end/src/localization_ndt/config/initial_pose/last/abtr_ref.yaml, last(shutdown)=/home/wr/localization_alg-stable-end/src/localization_ndt/config/initial_pose/last/abtr_ref.yaml
[local_reloc] loadMap OK: yaml=/home/wr/localization_alg-stable-end/src/localization_ndt/map/abtr_ref.yaml image=/home/wr/localization_alg-stable-end/src/localization_ndt/map/abtr_ref.pgm w=2027 h=2850 res=0.0200 origin=(-13.432,-33.161,0.000deg) score0(req_sigma=0.200,req_maxd=1.000 used_sigma=0.200,used_maxd=1.000) pyr_req_Lmax=6 pyr_built_Lmax=6
[local_reloc] loadMap OK: yaml=/home/wr/localization_alg-stable-end/src/localization_ndt/map/abtr_ref.yaml image=/home/wr/localization_alg-stable-end/src/localization_ndt/map/abtr_ref.pgm w=2027 h=2850 res=0.0200 origin=(-13.432,-33.161,0.000deg) score0(req_sigma=0.200,req_maxd=1.000 used_sigma=0.200,used_maxd=1.000) pyr_req_Lmax=6 pyr_built_Lmax=6
[INFO] [1773386967.924812295]: [AMCL] init seed activate idx=1/5 reason=startup_shutdown_pose type=pose
[INFO] [1773386967.925405014]: [AMCL] initialize at pose, reason=startup_shutdown_pose pose=(13.1193,9.15705,28.5222deg) cov=(0.25,0.25,0.0685389)
[INFO] [1773386967.925444888]: 
================ AMCL Localizer ================
run     : map=abtr_ref
config  : interface=/home/wr/localization_alg-stable-end/src/localization_ndt/config/interface/interface.yaml params=/home/wr/localization_alg-stable-end/src/localization_ndt/config/amcl/abtr_ref.yaml
map_yaml=/home/wr/localization_alg-stable-end/src/localization_ndt/map/abtr_ref.yaml
image   =/home/wr/localization_alg-stable-end/src/localization_ndt/map/abtr_ref.pgm
topics  : scan=/scan odom=/odom/wheel imu=/imu initialpose=/initialpose
pub     : odom=amcl_odom pose=amcl_pose particles=particlecloud
frames  : map=map base=base_footprint odom=odom yaml=map use_yaml=0 tf_mode=map->base
odom    : source_mode=topic history=(2.000s,400 msgs)
map     : size=2027x2850 res=0.020 free=1778233 occ=53119 unk=3945598
pf      : min=200 max=800 pf_err=0.050 pf_z=0.990 alpha_fast=0.000 alpha_slow=0.000 resample_interval=1 global_init=1
laser   : model=likelihood_field beams=40 min=0.500 max=35.000 sigma=0.200 max_occ=2.000 beamskip=0
dyn     : enable=1 grid=0.100 inflate=0.200 keep=0.800 min_keep=200 ray=1 front=1
motion  : alpha=(0.200,0.200,0.200,0.200,0.200) update_min=(0.050m,2.000deg)
manual  : local_reloc=1 coarse=(5.000m,180.000deg) win(cov)=max(0.300m,3.000*sigma_xy) <= 5.000m yaw=max(6.000deg,3.000*sigma_yaw) final=(0.500m,10.000deg@0.250deg) reseed_scale=0.050 settle=3
local   : score>=0.200 margin>=0.000 vf>=0.300 bnb=5 pyr=6 yaw_step=1.000deg pts<=1200 win=(5.000m,180.000deg)
startup : local_reloc=1 K=3
lost    : local_reloc=1 dt>=2.000s expand=1.500^1
lostfix : enable=1 dt>=2.000s K=3 stop>=2.000s
state   : good(best>=0.980 amb<=0.100 c<=1 ov_xy<=0.600 jump=(0.100m,1.000deg)) amb(c>=2 best>=0.200 ratio>=0.350) lost(best<=0.200 c>=20 ov_xy>=2.000) init(good=2 bad=3) last(v<=0.050 w_deg<=3.000 dt>=2.000)
================================================
[local_reloc] match: bnb_req_Lmax=5 built_Lmax=6 use_Lmax=5
[local_reloc][P1] ok=0 best_score=0.2881 second=0.2609 margin=0.0272 vf=0.776 gate(score=1 vf=1 margin=1) best_pose(x=14.473 y=10.701 yaw=-176.0deg) win(xy=5.00 yaw=180.0deg) step(yaw=1.00deg) bnb(req=5 used=5 built=6)
[INFO] [1773387019.500669959, 1772164025.481725832]: [AMCL] initialize at pose, reason=startup_local_reloc pose=(14.4728,10.7014,-175.978deg) cov=(1.5625,1.5625,0.61685)
[INFO] [1773387019.500727498, 1772164025.491787093]: [AMCL] startup local_reloc success: x=14.4728 y=10.7014 yaw=-175.978deg score=0.288095 second=0.26086 vf=0.77565 seed=startup_shutdown_pose
[WARN] [1773387019.506596066, 1772164025.491787093]: [AMCL] event=dyn_front_empty in=865 front=0 axis=x sign=1.0
[INFO] [1773387019.506628459, 1772164025.491787093]: [AMCL] dyn_sensor used=1 keep=596/865 ratio=0.69 p50=0.14 p90=1.34 ray=4/363 rmap50=0.20 rmap90=5.30 front=0/0 fdrop=0.00 frec=0 use_raw=0 insuff=0
[INFO] [1773387019.510154853, 1772164025.491787093]: [AMCL][q] t=1772164025.424 guard(valid=1 core=1 init=1 odom=1 scan_n=751 upd=1) pf(conv=0 n=800 c=38 best=0.902 second=0.011 amb=0.012 sig_xy=1.091/1.042 sig_yaw=45.32deg ov_xy=1.267 ov_yaw=40.47deg) state(raw=LOST stable=UNINITIALIZED streak=1 init=1) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.173 dYaw=3.41deg)
[INFO] [1773387019.510467564, 1772164025.491787093]: [AMCL] init validated success: reason=startup_local_reloc -> LOST
[INFO] [1773387019.510483100, 1772164025.491787093]: [AMCL] state edge: UNINITIALIZED -> LOST t=1772164025.464
[INFO] [1773387020.518301251, 1772164026.504122502]: [AMCL][q] t=1772164026.464 guard(valid=1 core=1 init=1 odom=1 scan_n=732 upd=1) pf(conv=0 n=800 c=38 best=0.902 second=0.011 amb=0.012 sig_xy=1.091/1.042 sig_yaw=45.32deg ov_xy=1.267 ov_yaw=40.47deg) state(raw=LOST stable=LOST streak=27 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.000 dYaw=0.00deg)
[INFO] [1773387021.522098950, 1772164027.506397275]: [AMCL][q] t=1772164027.464 guard(valid=1 core=1 init=1 odom=1 scan_n=737 upd=1) pf(conv=0 n=800 c=38 best=0.902 second=0.011 amb=0.012 sig_xy=1.091/1.042 sig_yaw=45.32deg ov_xy=1.267 ov_yaw=40.47deg) state(raw=LOST stable=LOST streak=52 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.000 dYaw=0.00deg)
[INFO] [1773387022.534217129, 1772164028.519789077]: [AMCL][q] t=1772164028.465 guard(valid=1 core=1 init=1 odom=1 scan_n=742 upd=1) pf(conv=0 n=800 c=38 best=0.902 second=0.011 amb=0.012 sig_xy=1.091/1.042 sig_yaw=45.32deg ov_xy=1.267 ov_yaw=40.47deg) state(raw=LOST stable=LOST streak=77 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.000 dYaw=0.00deg)
[INFO] [1773387023.547327584, 1772164029.532152978]: [AMCL][q] t=1772164029.465 guard(valid=1 core=1 init=1 odom=1 scan_n=739 upd=1) pf(conv=0 n=800 c=38 best=0.902 second=0.011 amb=0.012 sig_xy=1.091/1.042 sig_yaw=45.32deg ov_xy=1.267 ov_yaw=40.47deg) state(raw=LOST stable=LOST streak=102 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.000 dYaw=0.00deg)
[INFO] [1773387024.555655550, 1772164030.545080439]: [AMCL][q] t=1772164030.505 guard(valid=1 core=1 init=1 odom=1 scan_n=746 upd=1) pf(conv=0 n=800 c=38 best=0.902 second=0.011 amb=0.012 sig_xy=1.091/1.042 sig_yaw=45.32deg ov_xy=1.267 ov_yaw=40.47deg) state(raw=LOST stable=LOST streak=127 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.000 dYaw=0.00deg)
[INFO] [1773387024.850904414, 1772164030.838603683]: [AMCL] initialize at pose, reason=manual_initialpose pose=(9.35258,5.02905,-13.294deg) cov=(0.25,0.25,0.0685389)
[INFO] [1773387024.850968265, 1772164030.838603683]: [AMCL] manual local_reloc queued: center=(9.35258,5.02905,-13.294deg) win=(1.5m,45deg)
[local_reloc] match: bnb_req_Lmax=5 built_Lmax=6 use_Lmax=5
[local_reloc][P1] ok=0 best_score=0.2703 second=0.2603 margin=0.0099 vf=0.802 gate(score=1 vf=1 margin=1) best_pose(x=6.108 y=-0.496 yaw=84.2deg) win(xy=5.00 yaw=180.0deg) step(yaw=1.00deg) bnb(req=5 used=5 built=6)
[local_reloc] match: bnb_req_Lmax=5 built_Lmax=6 use_Lmax=5
[local_reloc][P1] ok=0 best_score=0.2757 second=0.2661 margin=0.0095 vf=0.822 gate(score=1 vf=1 margin=1) best_pose(x=5.883 y=-0.196 yaw=89.2deg) win(xy=1.50 yaw=45.0deg) step(yaw=1.00deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2753 second=0.2668 margin=0.0084 vf=0.829 gate(score=1 vf=1 margin=1) best_pose(x=5.880 y=-0.214 yaw=89.5deg) win(xy=0.50 yaw=10.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[INFO] [1773387025.257225678, 1772164031.243991063]: [AMCL] initialize at pose, reason=manual_local_reloc pose=(5.8803,-0.2136,89.4872deg) cov=(0.0025,0.0025,0.00121847)
[INFO] [1773387025.257254859, 1772164031.243991063]: [AMCL] reloc_refine begin: reason=manual_local_reloc win=(1m,18deg) frames=12
[INFO] [1773387025.257270896, 1772164031.243991063]: [AMCL] manual local_reloc success: x=5.8803 y=-0.2136 yaw=89.4872deg score=0.275276 second=0.266829 vf=0.828685 combined=0.2865 stage=final coarse_win=(5m,180deg) fine_win=(1.5m,45deg) final_win=(0.5m,10deg@0.25deg)
[local_reloc][P1] ok=0 best_score=0.2775 second=0.2663 margin=0.0112 vf=0.807 gate(score=0 vf=1 margin=0) best_pose(x=6.065 y=-0.524 yaw=83.9deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2709 second=0.2640 margin=0.0069 vf=0.812 gate(score=0 vf=1 margin=0) best_pose(x=6.160 y=-0.509 yaw=85.1deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2779 second=0.2640 margin=0.0139 vf=0.822 gate(score=0 vf=1 margin=0) best_pose(x=5.885 y=-0.199 yaw=89.2deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2674 second=0.2610 margin=0.0064 vf=0.804 gate(score=0 vf=1 margin=0) best_pose(x=6.130 y=-0.481 yaw=84.2deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2680 second=0.2608 margin=0.0072 vf=0.813 gate(score=0 vf=1 margin=0) best_pose(x=6.140 y=-0.481 yaw=87.1deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2701 second=0.2616 margin=0.0085 vf=0.807 gate(score=0 vf=1 margin=0) best_pose(x=6.128 y=-0.506 yaw=84.4deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2688 second=0.2640 margin=0.0048 vf=0.805 gate(score=0 vf=1 margin=0) best_pose(x=6.098 y=-0.519 yaw=86.5deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2683 second=0.2662 margin=0.0021 vf=0.815 gate(score=0 vf=1 margin=0) best_pose(x=6.168 y=-0.491 yaw=88.2deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2696 second=0.2653 margin=0.0043 vf=0.827 gate(score=0 vf=1 margin=0) best_pose(x=5.885 y=-0.174 yaw=89.2deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2776 second=0.2683 margin=0.0093 vf=0.824 gate(score=0 vf=1 margin=0) best_pose(x=5.880 y=-0.194 yaw=89.2deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2740 second=0.2596 margin=0.0144 vf=0.824 gate(score=0 vf=1 margin=0) best_pose(x=5.890 y=-0.206 yaw=89.3deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2757 second=0.2699 margin=0.0058 vf=0.819 gate(score=0 vf=1 margin=0) best_pose(x=6.158 y=-0.486 yaw=88.2deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[INFO] [1773387026.659510942, 1772164032.643571823]: [AMCL] initialize at pose, reason=manual_local_reloc_refine_best pose=(5.8803,-0.2136,89.4872deg) cov=(0.0009,0.0009,0.000304617)
[WARN] [1773387026.659554001, 1772164032.643571823]: [AMCL] reloc_refine finish: ok=0 reason=manual_local_reloc keep_best=1 best_score=nan best_vf=0.000 best_combined=nan best_hits=0
[WARN] [1773387026.684718621, 1772164032.673793076]: [AMCL] event=dyn_front_empty in=860 front=0 axis=x sign=1.0
[INFO] [1773387026.684768650, 1772164032.673793076]: [AMCL] dyn_sensor used=1 keep=432/860 ratio=0.50 p50=0.00 p90=0.94 ray=173/363 rmap50=10.80 rmap90=35.00 front=0/0 fdrop=0.00 frec=0 use_raw=0 insuff=0
[INFO] [1773387026.685772600, 1772164032.673793076]: [AMCL] state edge: LOST -> UNINITIALIZED t=1772164032.585
[INFO] [1773387026.685801428, 1772164032.673793076]: [AMCL][q] t=1772164032.585 guard(valid=1 core=1 init=1 odom=1 scan_n=744 upd=1) pf(conv=1 n=201 c=1 best=1.000 second=0.000 amb=0.000 sig_xy=0.029/0.026 sig_yaw=1.00deg ov_xy=0.029 ov_yaw=1.00deg) state(raw=GOOD stable=UNINITIALIZED streak=1 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.004 dYaw=0.01deg)
[INFO] [1773387026.687212784, 1772164032.673793076]: [AMCL] state edge: UNINITIALIZED -> GOOD t=1772164032.625
[INFO] [1773387027.698034997, 1772164033.680539931]: [AMCL][q] t=1772164033.625 guard(valid=1 core=1 init=1 odom=1 scan_n=743 upd=1) pf(conv=1 n=201 c=1 best=1.000 second=0.000 amb=0.000 sig_xy=0.029/0.026 sig_yaw=1.00deg ov_xy=0.029 ov_yaw=1.00deg) state(raw=GOOD stable=GOOD streak=27 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.000 dYaw=0.00deg)
[INFO] [1773387028.702655599, 1772164034.691507219]: [AMCL][q] t=1772164034.625 guard(valid=1 core=1 init=1 odom=1 scan_n=744 upd=1) pf(conv=1 n=201 c=1 best=1.000 second=0.000 amb=0.000 sig_xy=0.029/0.026 sig_yaw=1.00deg ov_xy=0.029 ov_yaw=1.00deg) state(raw=GOOD stable=GOOD streak=52 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.000 dYaw=0.00deg)
[INFO] [1773387028.744423872, 1772164034.732011806]: [LOC] InitialPoseManager: last pose updated: x=5.87508 y=-0.212616 yaw=1.56174 -> /home/wr/localization_alg-stable-end/src/localization_ndt/config/initial_pose/last/abtr_ref.yaml
[INFO] [1773387029.707648722, 1772164035.695528193]: [AMCL][q] t=1772164035.665 guard(valid=1 core=1 init=1 odom=1 scan_n=744 upd=1) pf(conv=1 n=201 c=1 best=1.000 second=0.000 amb=0.000 sig_xy=0.029/0.026 sig_yaw=1.00deg ov_xy=0.029 ov_yaw=1.00deg) state(raw=GOOD stable=GOOD streak=78 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.000 dYaw=0.00deg)
[INFO] [1773387029.753504223, 1772164035.735872978]: [AMCL] initialize at pose, reason=manual_initialpose pose=(9.77772,6.03733,-33.9482deg) cov=(0.25,0.25,0.0685389)
[INFO] [1773387029.753559959, 1772164035.735872978]: [AMCL] manual local_reloc queued: center=(9.77772,6.03733,-33.9482deg) win=(1.5m,45deg)
[local_reloc][P1] ok=0 best_score=0.3008 second=0.2681 margin=0.0327 vf=0.795 gate(score=1 vf=1 margin=1) best_pose(x=14.283 y=10.574 yaw=-175.9deg) win(xy=5.00 yaw=180.0deg) step(yaw=1.00deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3008 second=0.2681 margin=0.0327 vf=0.795 gate(score=1 vf=1 margin=1) best_pose(x=14.283 y=10.574 yaw=-175.9deg) win(xy=1.50 yaw=45.0deg) step(yaw=1.00deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2994 second=0.2945 margin=0.0049 vf=0.784 gate(score=1 vf=1 margin=1) best_pose(x=14.423 y=10.639 yaw=-175.6deg) win(xy=0.50 yaw=10.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[INFO] [1773387030.170078977, 1772164036.160718512]: [AMCL] initialize at pose, reason=manual_local_reloc pose=(14.4228,10.6389,-175.605deg) cov=(0.0025,0.0025,0.00121847)
[INFO] [1773387030.170107813, 1772164036.160718512]: [AMCL] reloc_refine begin: reason=manual_local_reloc win=(1m,18deg) frames=12
[INFO] [1773387030.170121601, 1772164036.160718512]: [AMCL] manual local_reloc success: x=14.4228 y=10.6389 yaw=-175.605deg score=0.299422 second=0.294497 vf=0.784153 combined=0.2978 stage=final coarse_win=(5m,180deg) fine_win=(1.5m,45deg) final_win=(0.5m,10deg@0.25deg)
[local_reloc][P1] ok=0 best_score=0.3015 second=0.2851 margin=0.0164 vf=0.780 gate(score=0 vf=1 margin=0) best_pose(x=14.408 y=10.659 yaw=-175.9deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3192 second=0.2886 margin=0.0306 vf=0.795 gate(score=0 vf=1 margin=1) best_pose(x=14.415 y=10.624 yaw=-175.9deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2973 second=0.2689 margin=0.0283 vf=0.781 gate(score=0 vf=1 margin=1) best_pose(x=14.400 y=10.659 yaw=-175.5deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3026 second=0.2795 margin=0.0231 vf=0.790 gate(score=0 vf=1 margin=1) best_pose(x=14.418 y=10.629 yaw=-175.7deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3062 second=0.2807 margin=0.0255 vf=0.788 gate(score=0 vf=1 margin=1) best_pose(x=14.438 y=10.639 yaw=-175.7deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3064 second=0.2765 margin=0.0299 vf=0.785 gate(score=0 vf=1 margin=1) best_pose(x=14.418 y=10.661 yaw=-175.6deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3014 second=0.2875 margin=0.0139 vf=0.787 gate(score=0 vf=1 margin=0) best_pose(x=14.403 y=10.621 yaw=-175.7deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3099 second=0.2866 margin=0.0234 vf=0.796 gate(score=0 vf=1 margin=1) best_pose(x=14.438 y=10.636 yaw=-175.7deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2967 second=0.2823 margin=0.0145 vf=0.785 gate(score=0 vf=1 margin=0) best_pose(x=14.418 y=10.666 yaw=-175.9deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2933 second=0.2770 margin=0.0163 vf=0.786 gate(score=0 vf=1 margin=0) best_pose(x=14.430 y=10.549 yaw=-177.9deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3057 second=0.3032 margin=0.0025 vf=0.787 gate(score=0 vf=1 margin=0) best_pose(x=14.298 y=10.529 yaw=-177.6deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3119 second=0.2885 margin=0.0234 vf=0.800 gate(score=0 vf=1 margin=1) best_pose(x=14.405 y=10.619 yaw=-175.7deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[INFO] [1773387031.586164336, 1772164037.569309779]: [AMCL] initialize at pose, reason=manual_local_reloc_refine_best pose=(14.4228,10.6389,-175.605deg) cov=(0.0009,0.0009,0.000304617)
[WARN] [1773387031.586197166, 1772164037.569309779]: [AMCL] reloc_refine finish: ok=0 reason=manual_local_reloc keep_best=1 best_score=nan best_vf=0.000 best_combined=nan best_hits=0
[WARN] [1773387031.589891139, 1772164037.579407239]: [AMCL] event=dyn_front_empty in=861 front=0 axis=x sign=1.0
[INFO] [1773387031.589916435, 1772164037.579407239]: [AMCL] dyn_sensor used=1 keep=588/861 ratio=0.68 p50=0.14 p90=1.34 ray=4/360 rmap50=0.15 rmap90=2.40 front=0/0 fdrop=0.00 frec=0 use_raw=0 insuff=0
[INFO] [1773387031.590806915, 1772164037.579407239]: [AMCL] state edge: GOOD -> UNINITIALIZED t=1772164037.505
[INFO] [1773387031.590828983, 1772164037.579407239]: [AMCL][q] t=1772164037.505 guard(valid=1 core=1 init=1 odom=1 scan_n=742 upd=1) pf(conv=1 n=275 c=1 best=1.000 second=0.000 amb=0.000 sig_xy=0.032/0.028 sig_yaw=0.96deg ov_xy=0.032 ov_yaw=0.96deg) state(raw=GOOD stable=UNINITIALIZED streak=1 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.005 dYaw=0.07deg)
[INFO] [1773387031.601896482, 1772164037.589574279]: [AMCL] state edge: UNINITIALIZED -> GOOD t=1772164037.546
[INFO] [1773387032.619671376, 1772164038.603310160]: [AMCL][q] t=1772164038.546 guard(valid=1 core=1 init=1 odom=1 scan_n=741 upd=1) pf(conv=1 n=275 c=1 best=1.000 second=0.000 amb=0.000 sig_xy=0.032/0.028 sig_yaw=0.96deg ov_xy=0.032 ov_yaw=0.96deg) state(raw=GOOD stable=GOOD streak=27 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.000 dYaw=0.00deg)
[INFO] [1773387033.618792689, 1772164039.605824577]: [AMCL][q] t=1772164039.545 guard(valid=1 core=1 init=1 odom=1 scan_n=731 upd=1) pf(conv=1 n=275 c=1 best=1.000 second=0.000 amb=0.000 sig_xy=0.032/0.028 sig_yaw=0.96deg ov_xy=0.032 ov_yaw=0.96deg) state(raw=GOOD stable=GOOD streak=52 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.000 dYaw=0.00deg)
[INFO] [1773387033.661212785, 1772164039.646321983]: [LOC] InitialPoseManager: last pose updated: x=14.4197 y=10.6416 yaw=-3.06557 -> /home/wr/localization_alg-stable-end/src/localization_ndt/config/initial_pose/last/abtr_ref.yaml
[INFO] [1773387034.627092264, 1772164040.611108987]: [AMCL][q] t=1772164040.545 guard(valid=1 core=1 init=1 odom=1 scan_n=734 upd=1) pf(conv=1 n=275 c=1 best=1.000 second=0.000 amb=0.000 sig_xy=0.032/0.028 sig_yaw=0.96deg ov_xy=0.032 ov_yaw=0.96deg) state(raw=GOOD stable=GOOD streak=77 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.000 dYaw=0.00deg)
[INFO] [1773387035.628266077, 1772164041.617678476]: [AMCL][q] t=1772164041.586 guard(valid=1 core=1 init=1 odom=1 scan_n=743 upd=1) pf(conv=1 n=275 c=1 best=1.000 second=0.000 amb=0.000 sig_xy=0.032/0.028 sig_yaw=0.96deg ov_xy=0.032 ov_yaw=0.96deg) state(raw=GOOD stable=GOOD streak=102 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.000 dYaw=0.00deg)
[INFO] [1773387036.666978828, 1772164042.650494412]: [AMCL][q] t=1772164042.626 guard(valid=1 core=1 init=1 odom=1 scan_n=739 upd=1) pf(conv=1 n=275 c=1 best=1.000 second=0.000 amb=0.000 sig_xy=0.032/0.028 sig_yaw=0.96deg ov_xy=0.032 ov_yaw=0.96deg) state(raw=GOOD stable=GOOD streak=128 init=0) twist(v=0.000m/s w=0.0000rad/s w_src=odom_tmp) jump(dT=0.000 dYaw=0.00deg)
[INFO] [1773387037.241050018, 1772164043.229270210]: [AMCL] initialize at pose, reason=manual_initialpose pose=(9.11247,4.95066,-21.2148deg) cov=(0.25,0.25,0.0685389)
[INFO] [1773387037.241128695, 1772164043.229270210]: [AMCL] manual local_reloc queued: center=(9.11247,4.95066,-21.2148deg) win=(1.5m,45deg)
[local_reloc][P1] ok=0 best_score=0.2826 second=0.2776 margin=0.0049 vf=0.785 gate(score=1 vf=1 margin=1) best_pose(x=13.940 y=10.344 yaw=-176.3deg) win(xy=5.00 yaw=180.0deg) step(yaw=1.00deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3126 second=0.2913 margin=0.0213 vf=0.804 gate(score=1 vf=1 margin=1) best_pose(x=14.423 y=10.631 yaw=-175.6deg) win(xy=1.50 yaw=45.0deg) step(yaw=1.00deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3116 second=0.2923 margin=0.0193 vf=0.800 gate(score=1 vf=1 margin=1) best_pose(x=14.415 y=10.629 yaw=-175.8deg) win(xy=0.50 yaw=10.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[INFO] [1773387037.662885631, 1772164043.644655498]: [AMCL] initialize at pose, reason=manual_local_reloc pose=(14.4153,10.6289,-175.777deg) cov=(0.0025,0.0025,0.00121847)
[INFO] [1773387037.662912122, 1772164043.644655498]: [AMCL] reloc_refine begin: reason=manual_local_reloc win=(1m,18deg) frames=12
[INFO] [1773387037.662926832, 1772164043.644655498]: [AMCL] manual local_reloc success: x=14.4153 y=10.6289 yaw=-175.777deg score=0.311632 second=0.292309 vf=0.800274 combined=0.3067 stage=final coarse_win=(5m,180deg) fine_win=(1.5m,45deg) final_win=(0.5m,10deg@0.25deg)
[local_reloc][P1] ok=0 best_score=0.3070 second=0.2915 margin=0.0155 vf=0.792 gate(score=0 vf=1 margin=0) best_pose(x=14.425 y=10.631 yaw=-176.3deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2908 second=0.2828 margin=0.0080 vf=0.765 gate(score=0 vf=1 margin=0) best_pose(x=14.508 y=10.731 yaw=-175.5deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3123 second=0.2917 margin=0.0206 vf=0.808 gate(score=0 vf=1 margin=1) best_pose(x=14.433 y=10.636 yaw=-175.7deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2996 second=0.2924 margin=0.0072 vf=0.786 gate(score=0 vf=1 margin=0) best_pose(x=14.460 y=10.659 yaw=-175.8deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3075 second=0.2886 margin=0.0189 vf=0.788 gate(score=0 vf=1 margin=0) best_pose(x=14.258 y=10.561 yaw=-178.0deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3124 second=0.2962 margin=0.0162 vf=0.792 gate(score=0 vf=1 margin=0) best_pose(x=14.285 y=10.589 yaw=-177.3deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.50deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3070 second=0.2903 margin=0.0168 vf=0.793 gate(score=0 vf=1 margin=0) best_pose(x=14.418 y=10.664 yaw=-175.9deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2951 second=0.2894 margin=0.0057 vf=0.786 gate(score=0 vf=1 margin=0) best_pose(x=14.508 y=10.686 yaw=-175.9deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3150 second=0.2984 margin=0.0166 vf=0.807 gate(score=0 vf=1 margin=0) best_pose(x=14.420 y=10.629 yaw=-175.8deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.2962 second=0.2882 margin=0.0080 vf=0.778 gate(score=0 vf=1 margin=0) best_pose(x=14.438 y=10.681 yaw=-175.5deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3129 second=0.2940 margin=0.0189 vf=0.818 gate(score=0 vf=1 margin=0) best_pose(x=14.313 y=10.589 yaw=-174.8deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[local_reloc][P1] ok=0 best_score=0.3078 second=0.2828 margin=0.0250 vf=0.803 gate(score=0 vf=1 margin=1) best_pose(x=14.415 y=10.626 yaw=-175.8deg) win(xy=1.00 yaw=18.0deg) step(yaw=0.25deg) bnb(req=5 used=5 built=6)
[INFO] [1773387039.110784240, 1772164044.901827302]: [AMCL] initialize at pose, reason=manual_local_reloc_refine_best pose=(14.4153,10.6289,-175.777deg) cov=(0.0009,0.0009,0.000304617)
[WARN] [1773387039.110811709, 1772164044.901827302]: [AMCL] reloc_refine finish: ok=0 reason=manual_local_reloc keep_best=1 best_score=nan best_vf=0.000 best_combined=nan best_hits=0
