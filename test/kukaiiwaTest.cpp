#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE grlTest

// system includes
#include <boost/test/unit_test.hpp>
#include <exception>
#include <iostream>
#include <vector>

//// local includes
//////#include "flatbuffers/util.h"
///#include "grl/sensor/FusionTrack.hpp"
///#include "grl/sensor/FusionTrackToEigen.hpp"
#include "grl/kuka/KukaToFlatbuffer.hpp"
#include <FRIMessages.pb.h>
#include "grl/kuka/KukaJAVAdriver.hpp"

BOOST_AUTO_TEST_SUITE(KukaTest)

BOOST_AUTO_TEST_CASE(runRepeatedly)
{
    bool debug = true;
    if (debug) std::cout << "starting KukaTest" << std::endl;
    // grl::robot::arm::KukaJAVAdriver::Parms javaDriverP = grl::robot::arm::KukaJAVAdriver::defaultParams();
    // rl::robot::arm::KukaJAVAdriver javaDriver(javaDriverP);
    // javaDriver.construct();
}

BOOST_AUTO_TEST_SUITE_END()
