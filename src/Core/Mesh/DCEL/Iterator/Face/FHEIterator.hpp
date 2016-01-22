#ifndef RADIUMENGINE_DCEL_FACE_HALFEDGE_ITERATOR_HPP
#define RADIUMENGINE_DCEL_FACE_HALFEDGE_ITERATOR_HPP

#include <Core/Mesh/DCEL/Iterator/Face/FaceIterator.hpp>

namespace Ra {
namespace Core {

class FHEIterator : public FIterator< HalfEdge_ptr > {
public:
    /// CONSTRUCTOR
    inline FHEIterator( const Face_ptr& f );
    inline FHEIterator( const FHEIterator& it ) = default;

    /// DESTRUCTOR
    inline ~FHEIterator();

    /// LIST
    inline HalfEdgeList list() const override;

    /// OPERATOR
    inline HalfEdge_ptr operator->() const override;
    inline HalfEdge_ptr operator* () const override;
};

} // namespace Core
} // namespace Ra

#include <Core/Mesh/DCEL/Iterator/Face/FHEIterator.inl>

#endif // RADIUMENGINE_DCEL_FACE_HALFEDGE_ITERATOR_HPP
