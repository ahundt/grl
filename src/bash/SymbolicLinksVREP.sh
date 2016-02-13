VREPDIR=$HOME/V-REP_PRO_EDU_V3_2_2_64_Linux # Set to Vrep directory
# VREPDIR=/Applications/V-REP_PRO_EDU_V3_2_2_Mac/vrep.app/Contents/MacOS # OSX

GRLDIR=$HOME/grl
BUILDDIR=$GRLDIR/build
  
LIBDIR=/lib # Change if using eclipse or OSX
# LIBDIR=/lib/Debug/ # OSX

LIBEXT=.so
# LIBEXT=.dylib

for arg in "libagast" "libv_repExtGrlInverseKinematics" "libcamodocal_brisk" "libcamodocal_calib" "libcamodocal_camera_models" "libcamodocal_camera_systems" "libcamodocal_chessboard"  "libcamodocal_fivepoint" "libcamodocal_pugixml" "libcamodocal_DBoW2" "libcamodocal_gpl" "libcamodocal_sparse_graph" "libv_repExtHandEyeCalibration" "libv_repExtPivotCalibration" "libcamodocal_DUtilsCV" "libcamodocal_infrastr_calib" "libcamodocal_visual_odometry" "libfusionTrack64" "libv_repExtKukaLBRiiwa" "libcamodocal_DUtils" "libcamodocal_location_recognition" "libcamodocal_DVision" "libcamodocal_pose_estimation" "libceres" "libcamodocal_features2d" "libcamodocal_pose_graph" "libdevice64" "libv_repExtAtracsysFusionTrack"
do
    ln -s $BUILDDIR/$LIBDIR/$arg$LIBEXT $VREPDIR
done

ln -s $GRLDIR/src/lua/grl.lua $VREPDIR

# vrep won't pick up the correct libraries if everything is built with linuxbrew
# so change the dynamic library path loaded by vrep
#
# TODO: This solution isn't perfect, it may pick up versions conflicting with those
#       used to build vrep, resulting in mysterious crashes. Try to use the libraries
#       in the same directory as vrep whenever possible. 
if [ -d $HOME/.linuxbrew ] ; then
    ln -s $HOME/.linuxbrew/lib $VREPDIR
    patchelf --set-rpath '$ORIGIN:$ORIGIN/lib' $VREPDIR/vrep
fi