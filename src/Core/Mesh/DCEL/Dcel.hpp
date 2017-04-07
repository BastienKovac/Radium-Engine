#ifndef RADIUMENGINE_DCEL_HPP
#define RADIUMENGINE_DCEL_HPP

#include <vector>
#include <Core/Index/IndexMap.hpp>
#include <Core/Index/IndexedObject.hpp>
#include <Core/Mesh/DCEL/Definition.hpp>
#include <Core/Math/Ray.hpp>

namespace Ra {
namespace Core {

/**
* Class DCEL (a.k.a. Doubly-Connected Edge List).
* The DCEL is a data structure representing a mesh as a collection of
* vertices, halfedges and faces.
*
* For further references read:
* "Computational Geometry - Geometry and applications"
* [ Mark de Berg, Otfried Cheong, Mark van Kreveld, Mark Overmars ]
* Chapter 2, Paragraph 2.2, Page 29
*/
class RA_CORE_API Dcel : public IndexedObject {
public:
    /// CONSTRUCTOR
    Dcel( const Index& index = Index::INVALID_IDX() ); // Build an empty DCEL with index equal to "index"
    Dcel( const Dcel& dcel );                          // Copy constructor

    /// DESTRUCTOR
    ~Dcel();

    /// CLEAR
    inline void clear(); // Clear the data from the DCEL, making it empty

    /// QUERY
    inline bool empty() const;
    inline bool compact() const;

    /// Return the index of the triangle hit by the ray or -1 if there's no hit.
    struct RayCastResult { int m_hitTriangle; int m_nearestVertex; Scalar m_t; };
    RA_CORE_API RayCastResult castRay(const Ray& ray);

    /// VARIABLE
    IndexMap< Vertex_ptr >   m_vertex;   // Vertices  Data
    IndexMap< HalfEdge_ptr > m_halfedge; // HalfEdges Data
    IndexMap< FullEdge_ptr > m_fulledge; // FullEdge  Data
    IndexMap< Face_ptr >     m_face;     // Faces     Data
};

} // namespace Core
} // namespace Ra

#include <Core/Mesh/DCEL/Dcel.inl>

#endif // RADIUMENGINE_DCEL_HPP
