// Stand-in for the dynamic_reconfigure-generated <ekf_rio/EkfRioConfig.h>.
//
// `ekf_rio/ekf_rio_filter.h` includes this header but never names the type --
// `EkfRioFilter::configure()` is templated on the config container. The alias
// exists so glue code can spell `ekf_rio::EkfRioConfig`.

#pragma once

#include <pyekf_rio/config.h>

namespace ekf_rio
{
using EkfRioConfig = ::pyekf_rio::RioConfig;
}  // namespace ekf_rio
