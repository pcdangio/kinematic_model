#include <kinematic_model/kinematic_model.hpp>

using namespace kinematic_model;

// CONSTRUCTORS
kinematic_model_t::kinematic_model_t(uint32_t n_state_variables, uint32_t n_sensors)
    : ukf_t(n_state_variables, n_sensors)
{}

// OPERATION
void kinematic_model_t::initialize()
{
    // First attempt to build geometry design.
    geometry::design_t design;
    try
    {
        build_geometry(design);
    }
    catch(const std::exception& error)
    {
        throw std::runtime_error("failed to build geometry (" + std::string(error.what()) + ")");
    }

    // Build graph.
    kinematic_model_t::m_graph.build(design);
}
void kinematic_model_t::iterate()
{
    // Iterate the underlying UKF.
    try
    {
        ukf_t::iterate();
    }
    catch(const std::exception& error)
    {
        throw std::runtime_error("state estimation failed (" + std::string(error.what()) + ")");
    }

    // Reset the transform cache.
    kinematic_model_t::m_transform_cache.clear();
}

// TRANSFORM
bool kinematic_model_t::get_transform(const std::string& source_frame, const std::string& target_frame, const Eigen::VectorXd& state_vector, transform::transform_t& transform) const
{
    // Get the transform path from the graph.
    // NOTE: graph internally uses caching on path solving.
    auto path = kinematic_model_t::m_graph.solve_path(source_frame, target_frame);

    // Check if path was found.
    if(!path)
    {
        return false;
    }

    // Iterate through the solved path.
    for(auto connection = path->cbegin(); connection != path->cend(); ++connection)
    {
        // Get the transform from the connection's attachment using the current state.
        auto attachment_transform = connection->attachment->get_transform(state_vector);

        // Invert the transform if needed.
        if(connection->direction == geometry::graph::connection_t::direction_t::PARENT_CHILD)
        {
            attachment_transform = attachment_transform.inverse();
        }

        // Chain the transform.
        attachment_transform.apply(transform);
    }

    return true;
}
bool kinematic_model_t::get_transform(const std::string& source_frame, const std::string& target_frame, transform::transform_t& transform)
{
    // Create flag for tracking success of transform retrieval.
    bool success = false;

    // Check the transform cache.
    std::string cache_key = source_frame + ":" + target_frame;
    auto cache_iterator = kinematic_model_t::m_transform_cache.find(cache_key);
    if(cache_iterator != kinematic_model_t::m_transform_cache.end())
    {
        // Transform found in cache.

        // Read transform from cache.
        transform = cache_iterator->second;

        // Flag successful transform.
        success = true;
    }
    else
    {
        // Transform not found in cache.

        // Calculate the transform with current state.
        if(kinematic_model_t::get_transform(source_frame, target_frame, kinematic_model_t::x, transform))
        {
            // Transform successfully calculated.

            // NOTE: transform stored in output via function.

            // Flag successful transform.
            success = true;

            // Add transform forward/reverse to cache.
            std::string cache_key_reverse = target_frame + ":" + source_frame;
            kinematic_model_t::m_transform_cache[cache_key] = transform;
            kinematic_model_t::m_transform_cache[cache_key_reverse] = transform.inverse();
        }
    }

    // Indicate if transform calculation was a success.
    return success;
}