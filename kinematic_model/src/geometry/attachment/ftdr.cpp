#include <kinematic_model/geometry/attachment/ftdr.hpp>

using namespace kinematic_model::geometry::attachment;

// CONSTRUCTORS
ftdr_t::ftdr_t()
    : attachment_t(attachment_t::type_t::DYNAMIC)
{}

// FACTORY
std::shared_ptr<ftdr_t> ftdr_t::create(double x, double y, double z, uint32_t state_index_qw, uint32_t state_index_qx, uint32_t state_index_qy, uint32_t state_index_qz)
{
    // Create shared pointer instance.
    // NOTE: std::make_shared doesn't work with private constructors.
    std::shared_ptr<ftdr_t> ftdr(new ftdr_t());

    // Store transform's fixed translation.
    ftdr->m_transform.translation.x() = x;
    ftdr->m_transform.translation.y() = y;
    ftdr->m_transform.translation.z() = z;

    // Store rotation state indices.
    ftdr->m_state_index_qw = state_index_qw;
    ftdr->m_state_index_qx = state_index_qx;
    ftdr->m_state_index_qy = state_index_qy;
    ftdr->m_state_index_qz = state_index_qz;

    // Return instance.
    return ftdr;
}

// METHODS
const transform::transform_t& ftdr_t::get_transform(const Eigen::VectorXd& state_vector)
{
    // Populate transform's rotation from state vector.
    ftdr_t::m_transform.rotation.w() = state_vector[ftdr_t::m_state_index_qw];
    ftdr_t::m_transform.rotation.x() = state_vector[ftdr_t::m_state_index_qx];
    ftdr_t::m_transform.rotation.y() = state_vector[ftdr_t::m_state_index_qy];
    ftdr_t::m_transform.rotation.z() = state_vector[ftdr_t::m_state_index_qz];

    // Return transform.
    return ftdr_t::m_transform;
}