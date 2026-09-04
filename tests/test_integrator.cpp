#include <gtest/gtest.h>
#include "Integrator.h"

TEST(IntegratorTest, ConstantDerivativeProducesExpectedState)
{
    sim::State initial_state{};

    initial_state.x = 0.0;

    // Constant system: dx/dt = 10 m/s.
    // All other state derivatives remain zero.
    const auto constant_derivative =
        [](double /*time*/, const sim::State& /*state*/)
        {
            sim::StateDerivative derivative{};
            derivative.x_dot = 10.0;

            return derivative;
        };

    const double t = 0.0;
    const double dt = 0.1;

    const auto next_state =
        sim::rk4_step(
            constant_derivative,
            initial_state,
            t,
            dt);

    EXPECT_NEAR(next_state.x, 1.0, 1e-9);
}
