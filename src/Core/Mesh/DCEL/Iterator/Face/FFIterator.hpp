#ifndef RADIUMENGINE_DCEL_FACE_FACE_ITERATOR_HPP
#define RADIUMENGINE_DCEL_FACE_FACE_ITERATOR_HPP

#include <Core/Mesh/DCEL/Iterator/Face/FaceIterator.hpp>

namespace Ra {
namespace Core {

class FFIterator : public FIterator< Face_ptr > {
public:
    /// CONSTRUCTOR
    inline FFIterator( const Face_ptr& f );
    inline FFIterator( const FFIterator& it ) = default;

    /// DESTRUCTOR
    inline ~FFIterator();

    /// LIST
    inline FaceList list() const override;

    /// OPERATOR
    inline Face_ptr operator->() const override;
    inline Face_ptr operator* () const override;
};

} // namespace Core
} // namespace Ra

#include <Core/Mesh/DCEL/Iterator/Face/FFIterator.inl>

#endif // RADIUMENGINE_DCEL_FACE_FACE_ITERATOR_HPP
