
# get sha1
execute_process(
	COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
	WORKING_DIRECTORY ${GIT_WORKING_DIR}
	RESULT_VARIABLE RESULT
	OUTPUT_VARIABLE GIT_SHA1
	OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT ${RESULT} EQUAL 0)
	message("failed to run 'git rev-parse'")
	set(REV "unknown")
endif()

# check for unclean index (untracked/modified files)
execute_process(
	COMMAND ${GIT_EXECUTABLE} status --porcelain
	WORKING_DIRECTORY ${GIT_WORKING_DIR}
	RESULT_VARIABLE RESULT
	OUTPUT_VARIABLE GIT_STATUS
	OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT ${RESULT} EQUAL 0)
	message("failed to run 'git status --porcelain'")
	set(GIT_SHA1 "unknown")
else()
	if (NOT "${GIT_STATUS}" STREQUAL "")
		set(GIT_MODIFIED "-dirty")
	endif()
endif()

# create output string
set(REV ${GIT_SHA1}${GIT_MODIFIED})

if(NOT BUILD_HOST) 
	site_name(SITE_NAME)
	set(BUILD_HOST ${SITE_NAME})
endif()

configure_file(
	${TEMPLATE_FILE}
	${OUTPUT_FILE}
)
