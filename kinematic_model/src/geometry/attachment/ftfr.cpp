#include <kinematic_model/geometry/attachment/ftfr.hpp>

using namespace kinematic_model::geometry::attachment;

// CONSTRUCTORS
ftfr_t::ftfr_t()
    : attachment_t(attachment_t::type_t::FIXED)
{}

// FACTORY
std::shared_ptr<ftfr_t> ftfr_t::create(double x, double y, double z, double qw, double qx, double qy, double qz)
{
    // Create shared pointer instance.
    // NOTE: std::make_shared doesn't work with private constructors.
    std::shared_ptr<ftfr_t> ftfr(new ftfr_t());

    // Set fixed transform.
    ftfr->m_transform.translation.x() = x;
    ftfr->m_transform.translation.y() = y;
    ftfr->m_transform.translation.z() = z;
    ftfr->m_transform.rotation.w() = qw;
    ftfr->m_transform.rotation.x() = qx;
    ftfr->m_transform.rotation.y() = qy;
    ftfr->m_transform.rotation.z() = qz;

    // Return instance.
    return ftfr;
}
std::shared_ptr<ftfr_t> ftfr_t::create(double x, double y, double z, double roll, double pitch, double yaw)
{
    // Convert euler rotation to quaternion.
    Eigen::Quaterniond q = transform::transform_t::to_quaternion(Eigen::Vector3d(roll, pitch, yaw));

    // Use base factory method to create instance.
    return ftfr_t::create(x, y, z, q.w(), q.x(), q.y(), q.z());
}

// METHODS
const transform::transform_t& ftfr_t::get_transform(const Eigen::VectorXd& state_vector)
{
    return ftfr_t::m_transform;
}