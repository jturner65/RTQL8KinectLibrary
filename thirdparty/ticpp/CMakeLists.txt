INCLUDE_DIRECTORIES(
	.
)

ADD_DEFINITIONS(-DTIXML_USE_TICPP)

set(headers
	tinyxml.h
	tinystr.h
	ticpprc.h
	ticpp.h
)

set(sources
	ticpp.cpp
	tinystr.cpp
	tinyxml.cpp
	tinyxmlerror.cpp
	tinyxmlparser.cpp
  ${headers}
)


if (${BUILD_SHARED_LIBS})
  ADD_LIBRARY(ticpp SHARED ${sources})
else ()
  ADD_LIBRARY(ticpp STATIC ${sources})
endif()

install(FILES ${headers} DESTINATION include/ticpp)
install (TARGETS ticpp DESTINATION lib)
