
// Library includes
#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE
#include "flatbuffers/util.h"
#include "grl/flatbuffer/readDatafromBinary.hpp"
#include <thirdparty/fbs_tk/fbs_tk.hpp>
#include "grl/flatbuffer/FusionTrack_generated.h"
#include "grl/flatbuffer/Time_generated.h"
#include "grl/flatbuffer/LogKUKAiiwaFusionTrack_generated.h"
#include "grl/time.hpp"


#include <vector>
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include <Eigen/Dense>

std::string foldname = "/home/chunting/src/V-REP_PRO_EDU_V3_4_0_Linux/";
std::string kukaBinaryfile = foldname + "2018_02_13_19_06_04_Kukaiiwa.iiwa";
std::string fusiontrackBinaryfile = foldname + "2018_02_13_19_05_49_FusionTrack.flik";
std::string FTKUKA_CSVfilename = foldname + current_date_and_time_string() + "_FTKUKA.csv";
std::string FT_CSVfilename = foldname + current_date_and_time_string() + "_FT.csv";
std::string KUKA_CSVfilename = foldname + current_date_and_time_string() + "_KUKA.csv";
std::string KUKA_CSVfilename_Joint = foldname + current_date_and_time_string() + "_KUKA_Joint.csv";


int main(int argc, char* argv[])
{
    grl::writetoCSVforFusionTrackKukaiiwa(fusiontrackBinaryfile, kukaBinaryfile, FTKUKA_CSVfilename, FT_CSVfilename, KUKA_CSVfilename);
    grl::writetoJointAngToCSV(kukaBinaryfile, KUKA_CSVfilename_Joint);
    return 0;
}







