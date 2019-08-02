# Create list with ARIA files for inclusion in Dune's compilation
file(GLOB DUNE_LIBARIA_FILES
  vendor/libraries/aria/src/*.cpp)

# Finding Aria library's include directory
find_path( ARIA_INCLUDE_DIR Aria.h 
  HINTS vendor/libraries/aria/include
  DOC "Aria lib include dir" )
IF( NOT ARIA_INCLUDE_DIR )
  MESSAGE( FATAL_ERROR "Aria.h not found!" )
ENDIF()

# Finding and removing windows-specific files from compilation
file(GLOB ARIA_UNWANTED_SOURCES vendor/libraries/aria/src/*_WIN.cpp)
FOREACH(ITEM ${ARIA_UNWANTED_SOURCES})
  list (REMOVE_ITEM DUNE_LIBARIA_FILES ${ITEM})
ENDFOREACH(ITEM ${ARIA_UNWANTED_SOURCES})

# Setting include flag so compiler will see the Aria.h folder
set (ARIA_INCLUDE_FLAGS "-I${ARIA_INCLUDE_DIR}")

# Suppressing some annoying compilation warnings
set (ARIA_WARNING_FLAGS "-Wno-sign-compare -Wno-unused-parameter -Wno-unused-variable -Wno-unused-parameter -Wno-shadow -Wno-pedantic -Wno-maybe-uninitialized")

# Include cpp files in Dune compilation
set_source_files_properties(${DUNE_LIBARIA_FILES}
  PROPERTIES COMPILE_FLAGS "${ARIA_INCLUDE_FLAGS} ${ARIA_WARNING_FLAGS} ${DUNE_CXX_FLAGS} ${DUNE_CXX_FLAGS_STRICT}")
list(APPEND DUNE_VENDOR_FILES ${DUNE_LIBARIA_FILES})