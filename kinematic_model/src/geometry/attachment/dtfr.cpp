#include <kinematic_model/geometry/attachment/dtfr.hpp>

using namespace kinematic_model::geometry::attachment;

// CONSTRUCTORS
dtfr_t::dtfr_t()
    :attachment_t(attachment_t::type_t::DYNAMIC)
{}

// FACTORY
std::shared_ptr<dtfr_t> dtfr_t::create(uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, double_t qw, double_t qx, double_t qy, double_t qz)
{
    // Create shared pointer instance.
    // NOTE: std::make_shared doesn't work with private constructors.
    std::shared_ptr<dtfr_t> dtfr(new dtfr_t());

    // Set transform's fixed rotation.
    dtfr->m_transform.rotation.w() = qw;
    dtfr->m_transform.rotation.x() = qx;
    dtfr->m_transform.rotation.y() = qy;
    dtfr->m_transform.rotation.z() = qz;

    // Store translation state indices.
    dtfr->m_state_index_x = state_index_x;
    dtfr->m_state_index_y = state_index_y;
    dtfr->m_state_index_z = state_index_z;

    // Return instance.
    return dtfr;
}
std::shared_ptr<dtfr_t> dtfr_t::create(uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, double_t roll, double_t pitch, double_t yaw)
{
    // Convert euler rotation to quaternion.
    Eigen::Quaterniond q = transform::transform_t::to_quaternion(Eigen::Vector3d(roll, pitch, yaw));

    // Use base factory method to create instance.
    return dtfr_t::create(state_index_x, state_index_y, state_index_z, q.w(), q.x(), q.y(), q.z());
}

// METHODS
transform::transform_t dtfr_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    // Populate transform's translation from state vector.
    dtfr_t::m_transform.translation.x() = state_vector[dtfr_t::m_state_index_x];
    dtfr_t::m_transform.translation.y() = state_vector[dtfr_t::m_state_index_y];
    dtfr_t::m_transform.translation.z() = state_vector[dtfr_t::m_state_index_z];

    // Return transform.
    return dtfr_t::m_transform;
}