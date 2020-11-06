#pragma once
#include "main/scene_base/base.hpp"

#ifdef SCENE_SKINNING

// Helper quaternion structure with associated functions
struct quaternion : vcl::vec4 {
    quaternion();
    quaternion(const vcl::vec4& v);
    quaternion(float x_arg,float y_arg,float z_arg,float w_arg);

    // Build quaternion from axis/angle values
    static quaternion axis_angle(const vcl::vec3& axis, float angle);
    vcl::mat3 matrix() const;

    // Apply quaternion to
    vcl::vec3 apply(const vcl::vec3& p) const;
};

quaternion inverse(quaternion const& q);
quaternion operator*(float s, const quaternion& q);
quaternion operator*(const quaternion& q, const float s);
quaternion operator+(const quaternion& q1, const quaternion& q2);
quaternion operator*(const quaternion& q1,const quaternion& q2);
quaternion operator/(const quaternion& q, float s);
quaternion conjugate(const quaternion& q);
quaternion slerp(quaternion q1, const quaternion& q2, float t);



#endif
