#include <sbrollpitchyaw.h>

namespace Scenebuilder{;

static const real_t pi = M_PI;

vec3_t ToRollPitchYaw(const quat_t& q){
	vec3_t angles;

	mat3_t R;
	q.ToMatrix(R);
	vec3_t xdir = R.col(0);
	angles[2] = atan2(xdir.y, xdir.x);
	angles[1] = atan2(xdir.z, sqrt(xdir.x*xdir.x + xdir.y*xdir.y));
	
	quat_t qroll = quat_t::Rot(-angles[1], 'y') * quat_t::Rot(-angles[2], 'z') * q;
	angles[0] = 2.0f * atan2(qroll[1], qroll[0]);

	// yaw angle needs wrapping
	if(angles[0] >  pi) angles[0] -= 2.0*pi;
	if(angles[0] < -pi) angles[0] += 2.0*pi;

	return angles;
}

quat_t FromRollPitchYaw(const vec3_t& angles){
	return quat_t::Rot(angles[2], 'z') * quat_t::Rot(angles[1], 'y') * quat_t::Rot(angles[0], 'x');
}

}
