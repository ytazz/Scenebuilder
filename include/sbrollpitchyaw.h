#pragma once

#include <sbtypes.h>

namespace Scenebuilder{;

/** 
   yaw-pitch-roll回転とquaternionとの相互変換
   q = Rz(yaw)*Ry(pitch)*Rx(roll)
 **/

vec3_t ToRollPitchYaw  (const quat_t& q     );
quat_t FromRollPitchYaw(const vec3_t& angles);

}
