/*
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                                     *
 * The ACME project                                                    *
 *                                                                     *
 * Copyright (c) 2020, Davide Stocco and Enrico Bertolazzi.            *
 *                                                                     *
 * The ACME project and its components are supplied under the terms of *
 * the open source BSD 2-Clause License. The contents of the ACME      *
 * project and its components may not be copied or disclosed except in *
 * accordance with the terms of the BSD 2-Clause License.              *
 *                                                                     *
 * URL: https://opensource.org/licenses/BSD-2-Clause                   *
 *                                                                     *
 *    Davide Stocco                                                    *
 *    Department of Industrial Engineering                             *
 *    University of Trento                                             *
 *    e-mail: davide.stocco@unitn.it                                   *
 *                                                                     *
 *    Enrico Bertolazzi                                                *
 *    Department of Industrial Engineering                             *
 *    University of Trento                                             *
 *    e-mail: enrico.bertolazzi@unitn.it                               *
 *                                                                     *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*/

///
/// file: entity.hxx
///

#pragma once
#ifndef INCLUDE_ACME_ENTITY_HXX
#define INCLUDE_ACME_ENTITY_HXX

namespace acme
{

  /*\
   |              _   _ _
   |    ___ _ __ | |_(_) |_ _   _
   |   / _ \ '_ \| __| | __| | | |
   |  |  __/ | | | |_| | |_| |_| |
   |   \___|_| |_|\__|_|\__|\__, |
   |                        |___/
  \*/

  //! Geometrical entity class container
  /**
   * Geometrical entity in 3D space.
   */
  class entity
  {
  public:
    typedef std::shared_ptr<entity> ptr;    //!< Shared pointer to entity object
    typedef std::vector<ptr>        vecptr; //!< Vector of pointers to entity objects

    //! Entity copy constructor
    entity(entity const &) = default;

    //! Entity move constructor
    entity(entity &&) = default;

    //! Entity assignment operator
    entity & operator=(const entity &) = default;

    //! Entity move assignment operator
    entity & operator=(entity &&) = default;

    //! Entity class destructor
    virtual
    ~entity(void);

    //! Entity class destructor
    entity(void);

    //! Translate entity by vector
    virtual
    void
    translate(
      vec3 const & input //!< Input translation vector
    ) = 0;

    //! Transform entity with affine transformation matrix
    virtual void
    transform(
      affine const & matrix //!< 4x4 affine transformation matrix
    ) = 0;

    //! Rotate entity by a rotation angle around an arbitrary axis
    void
    rotate(
      real         angle, //!< Input angle [rad]
      vec3 const & axis   //!< Input axis
    );

    //! Check if entity is degenerated
    virtual
    inline
    bool
    isDegenerated(
      real tolerance = EPSILON //!< Tolerance
    ) const = 0;

    //! Return object hierarchical level
    virtual
    inline
    integer
    level(void)
    const = 0;

    //! Return object type as string
    virtual
    inline
    std::string
    type(void)
    const = 0;

    //! Check whether the object is an entity
    inline
    bool
    isEntity(void)
    const
    {
      return true;
    }

    //! Check whether the object is no entity
    virtual
    inline
    bool
    isNone(void)
    const = 0;

    //! Check whether the object is a point
    virtual
    inline
    bool
    isPoint(void)
    const = 0;

    //! Check whether the object is a line
    virtual
    inline
    bool
    isLine(void)
    const = 0;

    //! Check whether the object is a ray
    virtual
    inline
    bool
    isRay(void)
    const = 0;

    //! Check whether the object is a plane
    virtual
    inline
    bool
    isPlane(void)
    const = 0;

    //! Check whether the object is a segment
    virtual
    inline
    bool
    isSegment(void)
    const = 0;

    //! Check whether the object is a triangle
    virtual
    inline
    bool
    isTriangle(void)
    const = 0;

    //! Check whether the object is a disk
    virtual
    inline
    bool
    isDisk(void)
    const = 0;

    //! Check whether the object is a ball
    virtual
    inline
    bool
    isBall(void)
    const = 0;

    //! Check whether in the entity is clampable
    virtual
    inline
    bool
    isClampable(void)
    const = 0;

    //! Check whether in the entity is non clampable
    virtual
    inline
    bool
    isNonClampable(void)
    const = 0;

    //! Get minumum and maximum values along axes
    virtual bool
    clamp(
      vec3 & min, //!< Input minimum point
      vec3 & max  //!< Input maximum point
    ) const = 0;

    //! Get minumum and maximum values along axes
    virtual bool
    clamp(
      real & min_x, //!< Input x value of minimum point
      real & min_y, //!< Input y value of minimum point
      real & min_z, //!< Input z value of minimum point
      real & max_x, //!< Input x value of maximum point
      real & max_y, //!< Input y value of maximum point
      real & max_z  //!< Input z value of maximum point
    ) const = 0;

  }; // class entity

} // namespace acme

#endif

///
/// eof: entity.hxx
///
