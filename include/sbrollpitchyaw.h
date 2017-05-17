#pragma once

#include <sbtypes.h>

namespace Scenebuilder{;

/** 
   yaw-pitch-roll‰ñ“]‚Æquaternion‚Æ‚Ì‘ŠŒÝ•ÏŠ·
   q = Rz(yaw)*Ry(pitch)*Rx(roll)
 **/

vec3_t ToRollPitchYaw  (const quat_t& q     );
quat_t FromRollPitchYaw(const vec3_t& angles);

}
