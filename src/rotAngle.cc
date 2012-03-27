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
#include "lsst/meas/algorithms/Algorithm.h"
#include "lsst/meas/extensions/rotAngle.h"


namespace lsst {
namespace meas {
namespace extensions {
namespace rotAngle {

namespace {

/// A class that knows how to calculate rotation angles
class RotAngle : public algorithms::Algorithm {
public:

    RotAngle(RotAngleControl const & ctrl, afw::table::Schema & schema) :
        algorithms::Algorithm(ctrl),
        _northKey(schema.addField<afw::geom::Angle>(ctrl.name + ".north",
                                                    "Angle of North measured from x axis towards y")),
        _eastKey(schema.addField<afw::geom::Angle>(ctrl.name + ".east",
                                                   "Angle of East measured from x axis towards y"))
        {}

private:
    
    template <typename PixelT>
    void _apply(
        afw::table::SourceRecord & source,
        afw::image::Exposure<PixelT> const & exposure,
        afw::geom::Point2D const & center
    ) const;

    LSST_MEAS_ALGORITHM_PRIVATE_INTERFACE(RotAngle);

    afw::table::Key<afw::table::Angle> _northKey;
    afw::table::Key<afw::table::Angle> _eastKey;
};

template <typename PixelT>
void RotAngle::_apply(
    afw::table::SourceRecord & source,
    afw::image::Exposure<PixelT> const& exposure,
    afw::geom::Point2D const & center
) const {
    afw::image::Wcs::ConstPtr wcs = exposure.getWcs();
    afw::table::Angle east, north;
    if (!wcs) {
        east = north = std::numeric_limits<double>::quiet_NaN() * afw::geom::radians;
    } else {
        afw::geom::AffineTransform const lin = wcs->linearizePixelToSky(center, afw::geom::radians);
        afw::geom::AffineTransform::ParameterVector const param = lin.getParameterVector();
        north = ::atan2(param[lin.XY], param[lin.XX]) * afw::geom::radians;
        east = ::atan2(param[lin.YY], param[lin.YX]) * afw::geom::radians;
    }

    source.set(_northKey, north);
    source.set(_eastKey, east);
}

LSST_MEAS_ALGORITHM_PRIVATE_IMPLEMENTATION(RotAngle);

} // anonymous namespace


PTR(algorithms::Algorithm) RotAngleControl::_makeAlgorithm(
    afw::table::Schema & schema,
    PTR(daf::base::PropertyList) const &
) const {
    return boost::make_shared<RotAngle>(*this, boost::ref(schema));
}


}}}} // namespace lsst::meas::extensions::rotAngle
