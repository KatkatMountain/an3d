#include "quaternion.hpp"
#ifdef SCENE_SKINNING

using namespace vcl;

quaternion::quaternion(float x_arg,float y_arg,float z_arg,float w_arg)
    :vec<4>({x_arg,y_arg,z_arg,w_arg})
{}

quaternion::quaternion()
    :vec<4>(1,0,0,0)
{}
quaternion::quaternion(const vec4& v)
    :vec<4>(v)
{
}

quaternion operator*(const quaternion& q1,const quaternion& q2)
{
    return {q1.x*q2.w + q1.w*q2.x + q1.y*q2.z - q1.z*q2.y,
                q1.y*q2.w + q1.w*q2.y + q1.z*q2.x - q1.x*q2.z,
                q1.z*q2.w + q1.w*q2.z + q1.x*q2.y - q1.y*q2.x,
                q1.w*q2.w-q1.x*q2.x-q1.y*q2.y-q1.z*q2.z};
}

vcl::mat3 quaternion::matrix() const
{
    return mat3(1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y));
}

vcl::vec3 quaternion::apply(const vcl::vec3& p) const
{
    const quaternion q_p    = quaternion(p.x,p.y,p.z,0);
    const quaternion q_conj = {-x,-y,-z,w};

    const quaternion q_res  = (*this)*q_p*q_conj;
    return {q_res.x,q_res.y,q_res.z};
}
quaternion quaternion::axis_angle(const vcl::vec3& axis, float angle)
{
    const float c = std::cos(angle/2.0f);
    const float s = std::sin(angle/2.0f);
    return quaternion(axis.x*s, axis.y*s, axis.z*s, c);
}
quaternion conjugate(const quaternion& q)
{
    return {-q.x,-q.y,-q.z, q.w};
}
quaternion inverse(quaternion const& q)
{
    float const n = norm(q);
    return conjugate(q)/(n*n);
}

quaternion operator*(float s, const quaternion& q)
{
    return {s*q.x, s*q.y, s*q.z, s*q.w};
}
quaternion operator*(const quaternion& q, const float s)
{
    return s*q;
}
quaternion operator+(const quaternion& q1, const quaternion& q2)
{
    return {q1.x+q2.x, q1.y+q2.y, q1.z+q2.z, q1.w+q2.w};
}
quaternion operator/(const quaternion& q, float s)
{
    return {q.x/s, q.y/s, q.z/s, q.w/s};
}

quaternion slerp(quaternion q1, const quaternion& q2, float t)
{
    float cos_omega = dot(q1,q2);

    // perform linear interpolation for very small angle (avoid division by sin(omega) in this case)
    const float epsilon = 1e-5f;
    if( std::abs(cos_omega-1.0f)<epsilon )
    {
        quaternion q = (1-t)*q1+t*q2;
        q = normalize(q);
        return q;
    }

    // make sure we take the shortest interpolating path
    // (If you are using slerp interpolation, you may comment this part and look at the result on the running character)
    if( cos_omega<0 )
    {
        q1=-q1;
        cos_omega = -cos_omega;
    }


    const float omega = std::acos(cos_omega);
    quaternion q = std::sin( (1-t)*omega )/std::sin(omega)*q1 + std::sin(t*omega)/std::sin(omega)*q2;

    return q;
}







#endif
