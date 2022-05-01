/// \file kinematic_model/kinematic_model.hpp
/// \brief Defines the kinematic_model::kinematic_model_t class.
#ifndef KINEMATIC_MODEL___KINEMATIC_MODEL_H
#define KINEMATIC_MODEL___KINEMATIC_MODEL_H

#include <kinematic_model/geometry/design.hpp>
#include <kinematic_model/geometry/graph/graph.hpp>

#include <kalman_filter/ukf.hpp>

/// \brief Components for implementing the kinematic model.
namespace kinematic_model {

/// \brief Provides state estimation and transforms for a kinematic model.
class kinematic_model_t
    : protected kalman_filter::ukf_t
{
public:
    // CONSTRUCTORS
    /// \brief Instantiates a new kinematic_model object.
    /// \param n_state_variables The number of variables in the model's state vector.
    /// \param n_sensors The number of sensors providing observations to the model's state.
    kinematic_model_t(uint32_t n_state_variables, uint32_t n_sensors);

    // OPERATION
    /// \brief Initializes the kinematic model for operation.
    /// \exception std::runtime_error if initialization fails.
    void initialize();
    /// \brief Performs a single iteration of the kinematic model.
    void iterate() override;

    // TRANSFORM
    /// \brief Gets a transform between two frames.
    /// \param source_frame The desired source frame of the transform.
    /// \param target_frame The desired target frame of the transform.
    /// \param transform OUTPUT The calculated transform.
    /// \returns TRUE if the transform was able to be calculated, otherwise FALSE.
    bool get_transform(const std::string& source_frame, const std::string& target_frame, transform::transform_t& transform);

protected:
    // GEOMETRY
    /// \brief Specifies the design of the physical geometry to model.
    /// \param design The design object for adding instructions to.
    /// \details This method is where plugins shall specify links, joints, and frames.
    virtual void build_geometry(geometry::design_t& design) const = 0;

    // TRANSFORM
    /// \brief Gets a transform between two frames using a specified state vector.
    /// \param source_frame The desired source frame of the transform.
    /// \param target_frame The desired target frame of the transform.
    /// \param state_vector The state vector to base transforms on.
    /// \param transform OUTPUT The calculated transform.
    /// \returns TRUE if the transform was able to be calculated, otherwise FALSE.
    bool get_transform(const std::string& source_frame, const std::string& target_frame, const Eigen::VectorXd& state_vector, transform::transform_t& transform) const;

private:
    // GEOMETRY
    /// \brief The graph of geometry objects.
    geometry::graph::graph_t m_graph;
    /// \brief A cache of calculated transforms for the get_transform service.
    std::unordered_map<std::string, transform::transform_t> m_transform_cache;
};

}

#endif