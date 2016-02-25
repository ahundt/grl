#ifndef GRL_KUKA_FRI
#define GRL_KUKA_FRI


/// @todo the FRI headers below depend on code that can't be shared, find a way to remove them so the java interface can work without the FRI source.
#include "friClientIf.h"
#include "friClientData.h"
#include "FRIMessages.pb.h"
#include "friCommandMessageEncoder.h"
#include "friMonitoringMessageDecoder.h"
#endif // GRL_KUKA_FRI