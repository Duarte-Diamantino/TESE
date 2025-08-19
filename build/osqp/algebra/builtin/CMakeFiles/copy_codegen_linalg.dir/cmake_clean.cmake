file(REMOVE_RECURSE
  "../../codegen_src/inc/private/algebra_impl.h"
  "../../codegen_src/inc/private/csc_math.h"
  "../../codegen_src/inc/private/csc_utils.h"
  "../../codegen_src/inc/private/kkt.h"
  "../../codegen_src/inc/private/qdldl.h"
  "../../codegen_src/inc/private/qdldl_interface.h"
  "../../codegen_src/inc/private/qdldl_types.h"
  "../../codegen_src/inc/private/qdldl_version.h"
  "../../codegen_src/src/algebra_libs.c"
  "../../codegen_src/src/csc_math.c"
  "../../codegen_src/src/csc_utils.c"
  "../../codegen_src/src/kkt.c"
  "../../codegen_src/src/matrix.c"
  "../../codegen_src/src/qdldl.c"
  "../../codegen_src/src/qdldl_interface.c"
  "../../codegen_src/src/vector.c"
  "CMakeFiles/copy_codegen_linalg"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/copy_codegen_linalg.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
