/* 
 * Lsst Data Management System
 * Copyright 2008, 2009, 2010 LSST Corporation.
 * 
 * This product includes software developed by the
 * LSST Project (http://www.lsst.org/).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the LSST License Statement and 
 * the GNU General Public License along with this program.  If not, 
 * see <http://www.lsstcorp.org/LegalNotices/>.
 */
 
// -*- LSST-C++ -*-
#include <cmath>
#include "lsst/pex/exceptions.h"
#include "lsst/pex/logging/Trace.h"
#include "lsst/afw/image.h"
#include "lsst/meas/algorithms/Measure.h"


#include <stdio.h>

namespace pexPolicy = lsst::pex::policy;
namespace afwDet = lsst::afw::detection;
namespace afwCoord = lsst::afw::coord;
namespace afwImage = lsst::afw::image;
namespace afwGeom = lsst::afw::geom;
namespace measAlg = lsst::meas::algorithms;

namespace lsst {
namespace meas {
namespace extensions {
namespace rotAngle {

/// Reports the orientation of the CCD on the sky
///
/// This doesn't really belong in "Astrometry", but that's the closest of
/// Astrometry/Photometry/Shape.
class RotationAngle : public afwDet::Astrometry
{
protected:
    enum { EAST = Astrometry::NVALUE, NORTH, NVALUE };
public:
    typedef boost::shared_ptr<RotationAngle> Ptr;
    typedef boost::shared_ptr<RotationAngle const> ConstPtr;

    /// Ctor
    RotationAngle(double east, double north) : afwDet::Astrometry() {
        init();
        // Everything has to be set, even to a meaningless value
        double const NaN = std::numeric_limits<double>::quiet_NaN();
        set<X>(NaN);
        set<X_ERR>(NaN);
        set<Y>(NaN);
        set<Y_ERR>(NaN);

        set<EAST>(east);
        set<NORTH>(north);
    }
    RotationAngle() : afwDet::Astrometry() {
        init();
    }
    
    /// Add desired fields to the schema
    virtual void defineSchema(afwDet::Schema::Ptr schema) {
        schema->add(afwDet::SchemaEntry("east", EAST, afwDet::Schema::DOUBLE, 1, "radians"));
        schema->add(afwDet::SchemaEntry("north", NORTH, afwDet::Schema::DOUBLE, 1, "radians"));
    }

    double getEast() const { return afwDet::Astrometry::get<EAST, double>(); }
    double getNorth() const { return afwDet::Astrometry::get<NORTH, double>(); }

private:
    LSST_SERIALIZE_PARENT(lsst::afw::detection::Astrometry);
};
LSST_REGISTER_SERIALIZER(RotationAngle);

/**
 * @brief A class that calculates the rotation angles (angles between x axis of CCD and East,North)
 */
template<typename ExposureT>
class RotationAngleAlgorithm : public measAlg::Algorithm<afwDet::Astrometry, ExposureT>
{
public:
    typedef measAlg::Algorithm<afwDet::Astrometry, ExposureT> AlgorithmT;
    typedef boost::shared_ptr<RotationAngleAlgorithm> Ptr;
    typedef boost::shared_ptr<RotationAngleAlgorithm const> ConstPtr;

    /// Ctor
    RotationAngleAlgorithm() : AlgorithmT() {}

    virtual std::string getName() const { return "ROTANGLE"; }

    virtual PTR(AlgorithmT) clone() const {
        return boost::make_shared<RotationAngleAlgorithm<ExposureT> >();
    }

    virtual void configure(pexPolicy::Policy const& policy) {}

    virtual PTR(afwDet::Astrometry) measureNull(void) const {
        double const NaN = std::numeric_limits<double>::quiet_NaN();
        return boost::make_shared<RotationAngle>(NaN, NaN);
    }

    virtual PTR(afwDet::Astrometry) measureSingle(afwDet::Source const&, afwDet::Source const&,
                                                  measAlg::ExposurePatch<ExposureT> const&) const;
};

template<typename ExposureT>
PTR(afwDet::Astrometry) RotationAngleAlgorithm<ExposureT>::measureSingle(
    afwDet::Source const& target,
    afwDet::Source const& source,
    measAlg::ExposurePatch<ExposureT> const& patch
    ) const
{
    CONST_PTR(ExposureT) exp = patch.getExposure();
    afwImage::Wcs::ConstPtr wcs = exp->getWcs();
    if (!wcs) {
        double const NaN = std::numeric_limits<double>::quiet_NaN();
        return boost::make_shared<RotationAngle>(NaN, NaN);
    }

    afwGeom::AffineTransform const lin = wcs->linearizePixelToSky(patch.getCenter(), afwGeom::radians);
    afwGeom::AffineTransform::ParameterVector const param = lin.getParameterVector();
    double const east = ::atan2(param[lin.XY], param[lin.XX]);
    double const north = ::atan2(param[lin.YY], param[lin.YX]);
    return boost::make_shared<RotationAngle>(east, north);
}

LSST_DECLARE_ALGORITHM(RotationAngleAlgorithm, afwDet::Astrometry);

}}}}
