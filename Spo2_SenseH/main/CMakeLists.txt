# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.c"
					"API_utility.c"
					"bpf.c"
					"ecg_config.c"
					"quick_test.c"
					"API_SPO2.c")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
