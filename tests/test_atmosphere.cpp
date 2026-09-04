#include <gtest/gtest.h>
#include "Atmosphere.h"

TEST(AtmosphereTest, ReturnsSeaLevelDensity)
{
  sim::Atmosphere atmosphere;
  
  const auto sample = atmosphere.sample(0.0);

  EXPECT_NEAR(sample.density, 1.225, 0.001);
}
