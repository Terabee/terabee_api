/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <memory>

#include "terabee/internal/factories/TerarangerFactory.hpp"
#include "terabee/ITerarangerFactory.hpp"

namespace terabee
{

std::unique_ptr<ITerarangerFactory> ITerarangerFactory::getFactory()
{
  return std::make_unique<internal::factories::TerarangerFactory>();
}

}  // namespace terabee
