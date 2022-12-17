#include <kinematic_model/geometry/design.hpp>

using namespace kinematic_model::geometry;

// ADDITION
void design_t::add_object(const std::shared_ptr<object::object_t>& object)
{
    // Call base add_object() with empty parent/attachment.
    return design_t::add_object(object, nullptr, nullptr);
}
void design_t::add_object(const std::shared_ptr<object::object_t>& object, const std::shared_ptr<object::object_t>& parent, const std::shared_ptr<attachment::attachment_t>& attachment)
{
    // Check that object exists.
    if(!object)
    {
        throw std::runtime_error("failed to add object to model design (object is nullptr)");
    }

    // If a parent is given, ensure that it exists in the design.
    if(parent)
    {
        bool parent_exists = false;
        for(auto instruction = design_t::m_instructions.cbegin(); instruction != design_t::m_instructions.cend(); ++instruction)
        {
            if(instruction->object == parent)
            {
                parent_exists = true;
                break;
            }
        }

        // If this point reached and parent_exists is still false, parent does not exist.
        if(!parent_exists)
        {
            throw std::runtime_error("failed to add object [" + object->name() + "] to model design (parent does not exist)");
        }
    }

    // If a parent is given, ensure that an attachment is given.
    if(parent && !attachment)
    {
        throw std::runtime_error("failed to add object [" + object->name() + "] to model design (given parent with nullptr attachment)");
    }
    else if(!parent && attachment)
    {
        throw std::runtime_error("failed to add object [" + object->name() + "] to model design (cannot use attachment on nullptr parent)");
    }

    // Add object to instructions.
    design_t::m_instructions.push_back({object, parent, attachment});

    // Lock the object so it can no longer be edited.
    object->lock();
}

// ACCESS
std::vector<design_t::instruction_t> design_t::instructions() const
{
    return design_t::m_instructions;
}