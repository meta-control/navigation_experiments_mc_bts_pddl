ament_add_gtest(get_order_test get_order_test.cpp)
ament_target_dependencies(get_order_test ${dependencies})

ament_add_gtest(navigate_to_barman_test navigate_to_barman_test.cpp)
ament_target_dependencies(navigate_to_barman_test ${dependencies})

ament_add_gtest(interact_with_barman_test interact_with_barman_test.cpp)
ament_target_dependencies(interact_with_barman_test ${dependencies})

ament_add_gtest(navigate_to_client_test navigate_to_client_test.cpp)
ament_target_dependencies(navigate_to_client_test ${dependencies})

ament_add_gtest(deliver_order_test deliver_order_test.cpp)
ament_target_dependencies(deliver_order_test ${dependencies})

ament_add_gtest(check_order_test check_order_test.cpp)
ament_target_dependencies(check_order_test ${dependencies})

ament_add_gtest(pilot_integration_test pilot_integration_test.cpp)
ament_target_dependencies(pilot_integration_test ${dependencies})
