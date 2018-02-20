VREPDIR=$HOME/src/V-REP_PRO_EDU_V3_4_0_Linux/ # Set to Vrep directory
# VREPDIR=/Applications/V-REP_PRO_EDU_V3_2_2_Mac/vrep.app/Contents/MacOS # OSX

GRLDIR=$HOME/grl
BUILDDIR=$GRLDIR/build

LIBDIR=/lib # Change if using eclipse or OSX
# LIBDIR=/lib/Debug/ # OSX

LIBEXT=.so
# LIBEXT=.dylib

for arg in "libv_repExtGrlInverseKinematics" "libv_repExtKukaLBRiiwa" "libv_repExtPivotCalibration.so" "libv_repExtHandEyeCalibration" "libv_repExtPivotCalibration" "libdevice64" "libv_repExtAtracsysFusionTrack" "libfusionTrack64"
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