#include <gtest/gtest.h>
#include "Integrator.h"

TEST(AeroModelTest, ZeroAlphaProducesZeroLiftForZeroControl)
{
    sim::Vehicle vehicle{};
    sim::State state{};
    sim::ControlInput control{};

    state.V = 100.0;
    state.theta = 0.0;
    state.gamma = 0.0;

    sim::Aero_model model;

    const auto forces =
        model.evaluate(vehicle, state, control, 1.225);

    EXPECT_NEAR(forces.lift, 0.0, 1e-9);
}
