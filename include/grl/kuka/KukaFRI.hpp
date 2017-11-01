/// This file simply wraps some includes provided by KUKA's FRI driver libraries, in the zip file KUKA should provide you.
#ifndef GRL_KUKA_FRI
#define GRL_KUKA_FRI


/// Kuka Fast Robot Inteface (FRI) C++ SDK interface include files
/// The FRI headers below depend on code that can't be shared
/// This header helps the Java API work when FRI is not available.
/// See grl documentation for details on how to activate FRI.
#include "friClientIf.h"
#include "friClientData.h"
#include "FRIMessages.pb.h"
#include "friCommandMessageEncoder.h"
#include "friMonitoringMessageDecoder.h"

#endif // GRL_KUKA_FRI
