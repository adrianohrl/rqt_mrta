#include "rqt_mrta/config/architecture/params_array.h"

namespace rqt_mrta
{
namespace config
{
namespace architecture
{
ParamsArray::ParamsArray(Params* parent) : Params("array", parent) {}

ParamsArray::~ParamsArray() {}

ParamInterface* ParamsArray::clone() const
{
  ParamsArray* array = new ParamsArray();
  *array = *this;
  return array;
}
}
}
}
