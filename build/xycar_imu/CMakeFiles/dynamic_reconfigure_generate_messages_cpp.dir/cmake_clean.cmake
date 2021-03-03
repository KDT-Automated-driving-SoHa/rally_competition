file(REMOVE_RECURSE
  "xycar_imu_automoc.cpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/dynamic_reconfigure_generate_messages_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
