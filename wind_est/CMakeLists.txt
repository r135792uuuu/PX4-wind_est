set(MODULE_CFLAGS)
px4_add_module(
	MODULE modules__wind_est
	MAIN wind_est
	COMPILE_FLAGS
	STACK_MAIN 2000
	SRCS
		wind_est_main.cpp
	DEPENDS
		git_ecl
		ecl_geo_lookup
		px4_platform

	)

