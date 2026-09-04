add_executable(sim_tests
    test_atmosphere.cpp
    test_aero_model.cpp
    test_integrator.cpp
)

target_link_libraries(sim_tests
    PRIVATE
        sim_engine
        GTest::gtest_main
)

include(GoogleTest)

gtest_discover_tests(sim_tests)
