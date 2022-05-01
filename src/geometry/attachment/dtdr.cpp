#include <kinematic_model/geometry/attachment/dtdr.hpp>

using namespace kinematic_model::geometry::attachment;

// CONSTRUCTORS
dtdr_t::dtdr_t()
    : attachment_t(attachment_t::type_t::DYNAMIC)
{}

// FACTORY
std::shared_ptr<dtdr_t> dtdr_t::create(uint32_t state_index_x, uint32_t state_index_y, uint32_t state_index_z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz)
{
    // Create shared pointer instance.
    // NOTE: std::make_shared doesn't work with private constructors.
    std::shared_ptr<dtdr_t> dtdr(new dtdr_t());

    // Store translation indices.
    dtdr->m_state_index_x = state_index_x;
    dtdr->m_state_index_y = state_index_y;
    dtdr->m_state_index_z = state_index_z;

    // Store rotation indices.
    dtdr->m_state_index_qw = state_index_qw;
    dtdr->m_state_index_qx = state_index_qx;
    dtdr->m_state_index_qy = state_index_qy;
    dtdr->m_state_index_qz = state_index_qz;

    // Return instance.
    return dtdr;
}

// METHODS
transform::transform_t dtdr_t::get_transform(const Eigen::VectorXd& state_vector) const
{
    // Create output transform.
    transform::transform_t transform;

    // Set translation.
    transform.translation.x() = state_vector[dtdr_t::m_state_index_x];
    transform.translation.y() = state_vector[dtdr_t::m_state_index_y];
    transform.translation.z() = state_vector[dtdr_t::m_state_index_z];

    // Set rotation.
    transform.rotation.w() = state_vector[dtdr_t::m_state_index_qw];
    transform.rotation.x() = state_vector[dtdr_t::m_state_index_qx];
    transform.rotation.y() = state_vector[dtdr_t::m_state_index_qy];
    transform.rotation.z() = state_vector[dtdr_t::m_state_index_qz];

    // Return transform.
    return transform;
}