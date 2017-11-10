#ifndef _RQT_MRTA_ARCHITECTURE_CONFIG_PARAM_FACTORY_H_
#define _RQT_MRTA_ARCHITECTURE_CONFIG_PARAM_FACTORY_H_

#include "rqt_mrta/config/architecture/param.h"
#include "rqt_mrta/config/architecture/params.h"
#include "rqt_mrta/config/architecture/params_array.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
class ParamFactory
{
public:
  static ParamInterface* newInstance(const QString& group_name)
  {
    ParamInterface* param = NULL;
    if (group_name == "param")
    {
      param = new Param();
    }
    else if (group_name == "params")
    {
      param = new Params();
    }
    else if (group_name == "array")
    {
      param = new ParamsArray();
    }
    return param;
  }
};
}
}
}

#endif // _RQT_MRTA_ARCHITECTURE_CONFIG_PARAM_FACTORY_H_
