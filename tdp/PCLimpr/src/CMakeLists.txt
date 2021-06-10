include_directories(${PROJECT_SOURCE_DIR}/addons/include/preg6d)

include_directories(${SUITESPARSE_INCLUDE_DIRS})

include_directories(${NEWMAT_INCLUDE_DIRS})

add_definitions(-DWITH_LIBZIP)

set(PREG6D_SRCS
  planereg.cc
  ioplanes.cc
  planescan.cc
  optimizer.cc
  normalplane.cc
  ../../../src/slam6d/searchTree.cc
  ../../../src/scanio/framesreader.cc
  )

set(CLUSTER_SRCS
  cluster.cc
  planescan.cc
  ../../../src/slam6d/searchTree.cc
)

add_executable(preg6d ${PREG6D_SRCS})
add_executable(cluster ${CLUSTER_SRCS})

if(WIN32)
  target_link_libraries(preg6d shape scan scanio ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
  target_link_libraries(cluster shape scan scanio ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
  else()
  target_link_libraries(preg6d shape scan scanio ${NEWMAT_LIBRARIES_SHARED} ${CXSPARSE_LIBRARIES} ${ANN_LIBRARIES_SHARED} dl ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
  target_link_libraries(cluster shape scan scanio ${NEWMAT_LIBRARIES_SHARED} ${CXSPARSE_LIBRARIES} ${ANN_LIBRARIES_SHARED} dl ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
endif()