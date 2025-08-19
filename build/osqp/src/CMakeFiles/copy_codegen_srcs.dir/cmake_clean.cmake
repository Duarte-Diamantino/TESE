file(REMOVE_RECURSE
  "../codegen_src/src/auxil.c"
  "../codegen_src/src/error.c"
  "../codegen_src/src/osqp_api.c"
  "../codegen_src/src/scaling.c"
  "../codegen_src/src/util.c"
  "CMakeFiles/copy_codegen_srcs"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/copy_codegen_srcs.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
