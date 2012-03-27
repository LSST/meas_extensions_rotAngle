// -*- lsst-c++ -*-
#ifndef LSST_MEAS_EXTENSIONS_ROTANGLE_H
#define LSST_MEAS_EXTENSIONS_ROTANGLE_H

#include "lsst/meas/algorithms/FluxControl.h"

namespace lsst { namespace meas { namespace extensions { namespace rotAngle {

/**
 *  @brief C++ control object for rotation angle.
 *
 *  @sa RotAngleConfig.
 */
class RotAngleControl : public algorithms::AlgorithmControl {
public:
    RotAngleControl() : algorithms::AlgorithmControl("rotAngle", 3.0) {}

private:
    virtual PTR(algorithms::AlgorithmControl) _clone() const {
        return boost::make_shared<RotAngleControl>(*this);
    }
    virtual PTR(algorithms::Algorithm) _makeAlgorithm(
        afw::table::Schema & schema, PTR(daf::base::PropertyList) const & metadata
    ) const;
};

}}}} // namespace lsst::meas::extensions::photometryKron

#endif // !LSST_MEAS_EXTENSIONS_PHOTOMETRY_KRON_H
