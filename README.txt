rm_phase3
RoboMaster EP Core, Ubuntu 24.04, ROS 2 Jazzy, Python 3.12
Version 1.1.0: x/y paths, prediction/correction audit, and error comparison.

START HERE
Use the same Python virtual environment and RoboMaster SDK/media codec used
for Phase 2. Only one program may own the robot connection. Close the Phase 2
driver, planner, ep_check, and RoboMaster app before connecting here.

1. Build the ROS 2 package.
   Open a desktop terminal, source ROS Jazzy, and activate the same Python 3.12
   virtual environment and RoboMaster SDK/media codec used for Phase 2.

   source /opt/ros/jazzy/setup.bash

   Activate your existing Phase 2 virtual environment now. Then extract the
   downloaded archive into the src directory of your ROS 2 workspace:

   mkdir -p ~/robomaster_ws/src
   unzip ~/Downloads/rm_phase3.zip -d ~/robomaster_ws/src
   cd ~/robomaster_ws

   Install colcon in the active virtual environment so generated executables use
   that environment's Python and RoboMaster SDK. Install numerical/vision/plot
   dependencies there too:

   python -m pip install colcon-common-extensions "setuptools<80" wheel
   python -m pip install -r src/rm_phase3/requirements.txt
   colcon build --symlink-install --packages-select rm_phase3
   source install/setup.bash
   export RM_PHASE3_HOME=~/robomaster_ws/src/rm_phase3

   Keep existing calibration and result files if replacing an earlier copy.
   The correct command is ros2 run, with a space. Check the installed executables:

   ros2 pkg executables rm_phase3
   ros2 run rm_phase3 check

   All commands below use installed executables. Calibration, config.json and
   results remain in RM_PHASE3_HOME. With --symlink-install the code can also
   locate its source package automatically when that variable is unset.
   Requirements use NumPy 1.26.4, desktop OpenCV 4.10, and Matplotlib. Continue
   using your existing Phase 2 RoboMaster SDK and codec. Use one OpenCV wheel
   provider and a desktop session for camera calibration.

   In EACH new terminal: source ROS Jazzy, activate the same Phase 2 environment,
   then source the workspace and set the project path before using ros2 run:

   source /opt/ros/jazzy/setup.bash
   # Activate your existing Phase 2 virtual environment here.
   source ~/robomaster_ws/install/setup.bash
   export RM_PHASE3_HOME=~/robomaster_ws/src/rm_phase3

   Rebuild and source install/setup.bash after packaging/entry-point changes.

2. Edit src/rm_phase3/config.json if needed. Defaults match Phase 2:
   connection=ap, AprilTag 36h11 ID=3, arm=(105,60) mm.
   For AP, connect Ubuntu Wi-Fi to the robot. For an existing working cable or
   router setup, use connection=rndis or sta, respectively.
   Set tag_size_m to the actual OUTSIDE BLACK-SQUARE width, excluding white paper.
   The default is 0.13 m. The original Tag Copy.pdf's 60 mm label is not a setting.
   Keep camera mounting fixed. Enter measured camera pitch/yaw and lateral offset
   if nonzero. Pitch is positive downward. Yaw and camera_left_m are positive left.

3. Fix the vertical tag to a rigid board. Keep its centre at a height visible
   from 1.50 m AND 0.20 m. Mark the floor projection of the front ToF sensor as
   the physical robot reference. Use this same point for calibration, starting
   distance, final estimate, and final tape measurement. The ToF reading is not
   used in this project. Fix this geometry for all trials.

4. Test the stationary robot, camera decoder, and sensor feedback:
   ros2 run rm_phase3 check --robot
   It prints detected IDs and saves a camera image. It does not move the arm.

5. Set and verify the Phase 2 arm position:
   ros2 run rm_phase3 arm
   This command moves only the arm to x=105 mm, y=60 mm.
   Calibration and each trial also set it and check its feedback.

6. Print src/rm_phase3/docs/checkerboard_25mm.svg at 100%, landscape, without fitting/scaling.
   Check a square with a ruler. The board has 10x7 squares, 9x6 INNER corners.
   Calibrate camera intrinsics with at least 15 sharp, varied checkerboard poses:
   ros2 run rm_phase3 calibrate camera
   Press S to capture each pose, Q to solve. Move/tilt the checkerboard and cover
   different image regions. The robot remains still. For a different board use:
   ros2 run rm_phase3 calibrate camera --cols 9 --rows 6 --square 0.025
   Values are inner-corner counts and square side in metres.

7. Put the vertical tag back in its fixed experimental position. Fit distance
   scale/offset using tape measurements from the marked robot reference:
   ros2 run rm_phase3 calibrate distance
   The program requests approximately 0.20, 0.50, 1.00, and 1.50 m.
   Physically reposition the stationary robot for each prompt and enter the
   actual tape distance in metres. Keep it facing the tag. Each position saves
   30 fresh detections and a photograph. Q cancels a capture.

8. Validate on NEW captures at approximately 0.20 and 1.50 m:
   ros2 run rm_phase3 calibrate verify
   Reposition and remeasure independently. Each mean error must be <=0.02 m.
   A motion trial refuses unvalidated or mismatched calibration.

9. Place the reference on the 1.50 m mark and align the robot with the tag.
   Keep the same floor, tag, lighting, arm, camera, and settings in every trial:
   ros2 run rm_phase3 trial
   Enter the actual starting tape distance, e.g. 1.500. The robot waits for eight
   consecutive centred detections, approaches, brakes, then records a stationary
   final estimate. Ctrl+C stops and retains the attempt's files.
   After it stops, do not reposition it. Measure the final distance independently
   with a tape/ruler, from the SAME marked reference to the tag's floor-projected
   centre, and enter that value in metres. Do not copy the camera reading.

   A timestamped results/trial_YYYYMMDD_HHMMSS_microseconds+offset folder appears
   for every attempt. Failed attempts have explicit status and do not count as
   completed trials. The nominal stopping distance is 0.200 m. The controller's
   threshold is 0.205 m; final physical accuracy is assessed from the tape data.
   A missing/rejected tag immediately commands zero speed. Three accepted frames
   allow resuming. A visual gap lasting 0.6 s fails the attempt.

   If you skipped final entry, record it later before losing the measurement:
   ros2 run rm_phase3 measure "$RM_PHASE3_HOME/results/trial_TIMESTAMP" 0.203
   Replace trial_TIMESTAMP with the actual saved folder name and 0.203 with your
   own measured value. This command never changes the stored final estimate.

10. Repeat step 9 until you have 8-10 completed trials with independent final
    measurements. Return the robot manually to the same 1.50 m reference mark.
    The saved trial.avi videos show tag acquisition, motion, braking, and settling.

11. Generate the report table, scatter graphs, paths, and correction diagnostics:
    ros2 run rm_phase3 plot
    To analyse one folder or a different results tree:
    ros2 run rm_phase3 plot "$RM_PHASE3_HOME/results/trial_TIMESTAMP"
    ros2 run rm_phase3 plot /absolute/path/to/results

    Open the newest results/analysis_TIMESTAMP/EXPERIMENT_ID directory:
    trials.csv, statistics.json, scatter.png, trial_XX_distance.png,
    trial_XX_trajectory.png, trial_XX_tracking_error.png,
    trial_XX_correction.png, trial_XX_metrics.json.
    Each graph also has a PDF copy for your report.
    Statistics use completed trials with an independent actual measurement.
    Different configurations, calibrations, or source versions form separate
    experiment groups. Missing measurements stay empty. No example data is added.

HOW TO READ THE NEW GRAPHS
trajectory: x versus plan time, y versus plan time, and x-y paths in three panels.
Dashed black = cubic reference. Grey = odometry only. Blue = corrected EKF.
An x on the x-y panel marks each path's last valid sample. The origin is the
initial front reference. Plan time starts when WAIT first changes to DRIVE and
continues through pauses and SETTLE. The reference holds at 1.30 m after T.

tracking_error: odometry/reference and EKF/reference deviations in x, y and xy.
Metrics report DRIVE-only tracking RMSE. These are deviations from the plan,
not independently measured localization errors.

correction: range/bearing residuals before and after each accepted tag update,
applied x/y changes from odometry and tag updates, NIS/rejections, and predicted
versus posterior model uncertainty. Residuals reuse the same camera sample.
A smaller residual or covariance alone does not prove higher physical accuracy.

updates.csv retains z, h, H, R, innovation, S, K, K*innovation, state and covariance
before/after every attempted update. Rejected tag updates have accepted=false,
K=null and zero applied correction. See guide sections 10-11 for the equations.

The odometry path integrates SDK forward speed and yaw-derived turn rate with
the Lecture 11A unicycle equations. It receives no tag corrections. It is an
estimate under the negligible-lateral-slip assumption, not an external motion
tracking measurement or the SDK chassis position service. sensors.csv retains
the original full velocity packets, including lateral components.

To assess final accuracy, trials.csv and scatter compare BOTH odometry-only and
corrected final distances against the independent final tape distance. Positive
absolute_error_reduction_m means correction reduced absolute final error.
statistics.json compares MAEs over the same completed, independently measured
subset. A negative reduction is retained when correction performs worse.

Old trial folders still produce their original distance/scatter outputs. New
path/correction figures require the added fields from version 1.1.0 trials.
Keep existing calibration and result folders when updating the package. Rebuild
with colcon build --symlink-install --packages-select rm_phase3, then source
install/setup.bash before collecting new trials.

SAVED DATA
summary.json: fixed final estimate, independent actual distance, error, status,
starting distance, reference, timing, final pose/covariance and protocol ID.
trace.csv: commands, wheel-derived forward velocity, yaw-derived turn rate,
2D odometry baseline, cubic x/y/speed references, prediction and pre-tag poses,
corrected pose, tracking deviations, corrections, model standard deviations,
camera range/bearing, innovations/residuals, NIS, acceptance, full posterior
state/covariance, timing, freshness flags, and frame numbers.
updates.csv: complete matrix/state audit for each odometry or tag update.
sensors.csv: original SDK velocity, attitude, and arm packets with host timestamps.
trial.avi, start.jpg, finish.jpg: annotated experiment evidence.
config.json, camera.json, distance.json, software.json: reproducibility records.
Calibration attempts keep their own timestamped image/sample folders.

EQUATIONS AND CONCEPT
Read src/rm_phase3/docs/rm_phase3_guide.pdf for the step-by-step concept, lecture-to-function
mapping, coordinate convention, calibration, control, file descriptions, and
report calculations. It also explains the wheel/IMU-assisted odometry and
why a measured tag size adds the along-track information bearing alone lacks.

ROS 2
The trial node is rm_phase3_trial. It sends SDK commands directly and publishes:
/rm_phase3/estimate  nav_msgs/Odometry
/rm_phase3/cmd_vel   geometry_msgs/Twist (a copy of commands, not an input)
Coordinates use forward x, left y, counterclockwise heading. /cmd_vel does not
control this experiment. No Phase 2 driver, cv_bridge, or floor homography is used.

OFFLINE VERIFICATION
python -m unittest discover -s "$RM_PHASE3_HOME/tests"
The included tests check lecture numbers, Jacobians, wrapping, covariance, and
reproduction of logged gain/state/covariance updates, including rejected tags.
The delivered code passed additional synthetic pose/control/logging checks.
No physical robot was connected during development. There are no robot results
or completed calibration files in this archive.

SOURCE BASELINE
https://github.com/khoutaltaqi/sceproject2
Commit c4b010d111867e9598958ed3a1789cc32d46bd2c
Required task: RoboMaster_Odometry_Project_Phase_3(1).pdf, page 1.
Lecture11A_2Nov.pdf: pages 13-15.
Lecture11B(1).pdf: pages 11, 13-14.
Lecture12A (2)(1).pdf: pages 15, 20-24, 26.
Lecture12B (2).pdf: page 15.
